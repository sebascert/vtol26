"""Host checks for provisioning and image contracts; no Git, root or networking."""

import importlib.util
import ipaddress
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import sysconfig
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent.parent


class ImageTest(unittest.TestCase):
    def setUp(self):
        (ROOT / "build").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "build")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.image = self.work / "project/companion/image"
        shutil.copytree(
            ROOT, self.image, ignore=shutil.ignore_patterns("build", "__pycache__")
        )
        shutil.copytree(
            REPO / "companion/allocator",
            self.image.parent / "allocator",
            ignore=shutil.ignore_patterns("build", "__pycache__"),
        )
        shutil.copytree(REPO / "config", self.image.parent.parent / "config")
        self.poky, self.bsp = self.work / "poky", self.work / "meta-raspberrypi"
        for path, content in {
            self.poky / "oe-init-build-env": "",
            self.poky / "meta/conf/distro/include/yocto-uninative.inc": "",
            self.poky / "meta/conf/layer.conf": 'LAYERSERIES_CORENAMES = "scarthgap"',
            self.bsp
            / "conf/layer.conf": 'LAYERSERIES_COMPAT_raspberrypi = "scarthgap"',
            self.bsp / "conf/machine/raspberrypi4-64.conf": "",
        }.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        self.user_home = self.work / "user home"
        self.user_home.mkdir()
        self.environ = dict(os.environ, HOME=str(self.user_home))
        subprocess.run(
            ["bash", REPO / "config/install.sh"],
            env=self.environ,
            check=True,
            capture_output=True,
        )
        self.vehicle = self.user_home / ".local/share/vtol26/config/vehicle.json"
        self.home_patch = patch.dict(os.environ, {"HOME": str(self.user_home)})
        self.home_patch.start()
        self.addCleanup(self.home_patch.stop)
        configure = self.image / "configure.py"
        spec = importlib.util.spec_from_file_location("image_configure", configure)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_prepare_snapshot_and_refresh(self):
        build = self.module.prepare(self.poky, self.bsp)
        self.assertFalse((self.image / "build/ssh").exists())
        provision = self.image / "build/provision"
        self.assertFalse((provision / "authorized_keys").exists())
        with tarfile.open(provision / "vtol-source.tar.gz") as archive:
            names = archive.getnames()
            self.assertIn(
                "vtol-source/companion/allocator/src/pso/pso_runtime.hpp", names
            )
            self.assertIn(
                "vtol-source/companion/allocator/deploy/vtol_allocator@.service", names
            )
            self.assertIn("vtol-source/config/vehicle.json", names)
            self.assertEqual(len(names), len(set(names)))
            self.assertTrue(
                all(
                    name.startswith(
                        ("vtol-source/companion/allocator/", "vtol-source/config/")
                    )
                    for name in names
                )
            )
            self.assertFalse(
                any("id_ed25519" in name or "/build/" in name for name in names)
            )
            tuning = json.load(
                archive.extractfile(
                    "vtol-source/companion/allocator/src/pso/config.json"
                )
            )
            vehicle = json.load(archive.extractfile("vtol-source/config/vehicle.json"))
            initial = tuning["actuators"]["initial"]
            thrust = [
                initial[i] * vehicle["propulsion"]["k_t_n_per_command"][i]
                for i in range(3)
            ]
            self.assertAlmostEqual(
                sum(thrust),
                vehicle["rigid_body"]["mass_kg"]
                * vehicle["environment"]["gravity_m_s2"],
            )
            points = [
                vehicle["force_positions_body_m"][key]
                for key in ("motor_left", "motor_right", "motor_rear")
            ]
            for axis in (0, 1):
                self.assertAlmostEqual(
                    sum(p[axis] * t for p, t in zip(points, thrust)), 0
                )
            self.assertEqual(initial[3:], [0] * 6)
            self.assertEqual(tuning["cost_function"]["dt"], 0.25)
            self.assertEqual(tuning["pso"]["external_cycles"], 2)
        self.assertNotIn("@PROVISION@", (build / "conf/local.conf").read_text())
        source = self.image.parent / "allocator/src/main.cpp"
        source.write_text("// updated working tree\n")
        self.module.prepare(self.poky, self.bsp)
        with tarfile.open(provision / "vtol-source.tar.gz") as archive:
            self.assertEqual(
                archive.extractfile(
                    "vtol-source/companion/allocator/src/main.cpp"
                ).read(),
                source.read_bytes(),
            )

    def test_bad_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            self.module.prepare(self.work / "missing", self.bsp)
        with self.assertRaises(ValueError):
            self.module.bitbake_path(self.work / "bad path")
        (self.bsp / "conf/layer.conf").write_text(
            'LAYERSERIES_COMPAT_raspberrypi = "master"'
        )
        with self.assertRaises(ValueError):
            self.module.prepare(self.poky, self.bsp)

    def test_network_ssh_and_uart_contract(self):
        files = ROOT / "meta-vtol/recipes-core/vtol-config/files"
        network = (files / "10-vtol-ethernet.network").read_text()
        address = next(
            line.split("=", 1)[1]
            for line in network.splitlines()
            if line.startswith("Address=")
        )
        self.assertEqual(str(ipaddress.ip_interface(address)), "192.168.76.2/24")
        self.assertIn("Name=eth0", network)
        self.assertNotIn("Gateway=", network)
        ssh = (
            ROOT / "meta-vtol/recipes-connectivity/openssh/files/sshd_config"
        ).read_text()
        for value in (
            "PermitRootLogin no",
            "PasswordAuthentication yes",
            "PubkeyAuthentication yes",
            "AllowUsers vtol",
        ):
            self.assertIn(value, ssh)
        config = (ROOT / "local.conf.in").read_text()
        for value in (
            'ENABLE_UART = "1"',
            'CMDLINE_SERIAL = ""',
            'SERIAL_CONSOLES = ""',
            "dtoverlay=disable-bt",
        ):
            self.assertIn(value, config)

    def test_config_install_permissions(self):
        stage = self.work / "stage"
        files = ROOT / "meta-vtol/recipes-core/vtol-config/files"
        shutil.copytree(files, self.work / "config-input")
        inputs = self.work / "config-input"
        recipe = (files.parent / "vtol-config.bb").read_text()
        body = recipe.split("do_install() {", 1)[1].split("\n}", 1)[0]
        for key, value in {
            "D": str(stage),
            "S": str(REPO),
            "sysconfdir": "/etc",
            "WORKDIR": str(inputs),
        }.items():
            body = body.replace("${" + key + "}", value)
        # User ownership is applied by Yocto's fakeroot during the real install.
        subprocess.run(["sh", "-eu", "-c", "chown() { :; }\n" + body], check=True)
        for path, mode in {
            "etc/sudoers.d": 0o750,
            "etc/sudoers.d/vtol": 0o440,
            "home/vtol": 0o755,
        }.items():
            self.assertEqual(stat.S_IMODE((stage / path).stat().st_mode), mode, path)
        self.assertEqual(
            (stage / "etc/sudoers.d/vtol").read_bytes(),
            (files / "vtol-sudo").read_bytes(),
        )
        self.assertFalse((stage / "home/vtol/.ssh").exists())
        self.assertNotIn("authorized_keys", recipe)
        password_hash = (
            recipe.split('VTOL_PASSWORD_HASH = "', 1)[1]
            .split('"', 1)[0]
            .replace("\\$", "$")
        )
        generated = subprocess.run(
            ["openssl", "passwd", "-6", "-salt", "vtol26", "vtol"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        self.assertEqual(password_hash, generated)
        self.assertIn("--password '${VTOL_PASSWORD_HASH}' vtol", recipe)

    def test_application_install_paths(self):
        stage = self.work / "stage"
        template = stage / "usr/lib/systemd/system/vtol_allocator@.service"
        template.parent.mkdir(parents=True)
        shutil.copy(
            REPO / "companion/allocator/deploy/vtol_allocator@.service", template
        )
        config = stage / "usr/share/vtol26/companion/mavlink_uart/src/pso/config.json"
        config.parent.mkdir(parents=True)
        config.write_text('{"calibration":"example"}\n')
        recipe = (
            ROOT / "meta-vtol/recipes-apps/vtol-allocator/vtol-allocator.bb"
        ).read_text()
        for dependency in (
            "python3-core",
            "python3-crypt",
            "python3-json",
            "python3-jsonschema",
            "python3-venv",
            "python3-ensurepip",
            "python3-pip",
        ):
            self.assertIn(dependency, recipe)
        self.assertIn("chown -R vtol:vtol ${D}/home/vtol", recipe)
        body = recipe.split("do_install:append() {", 1)[1].split("\n}", 1)[0]
        # Ownership is checked above; the host tests have no vtol account or root.
        body = body.replace("chown -R vtol:vtol", "test -d")
        for key, value in {
            "D": str(stage),
            "S": str(REPO),
            "sysconfdir": "/etc",
            "datadir": "/usr/share",
            "libdir": "/usr/lib",
            "bindir": "/usr/bin",
            "systemd_system_unitdir": "/usr/lib/systemd/system",
        }.items():
            body = body.replace("${" + key + "}", value)
        subprocess.run(["sh", "-eu", "-c", body], check=True)
        installed = template.read_text()
        self.assertIn("ExecStart=/usr/bin/vtol_allocator", installed)
        self.assertIn("--pso-config /etc/vtol/pso.json", installed)
        self.assertNotIn("--vehicle-config", installed)
        self.assertIn("ProtectHome=read-only", installed)
        for option in (
            "--baud 230400",
            "--loop-ms 2",
            "--pso-period-ms 20",
            "--pso-max-particles 96",
            "--pso-max-iterations 15",
            "--pso-external-cycles 2",
        ):
            self.assertIn(option, installed)
        self.assertEqual(
            (stage / "home/vtol/.local/share/vtol26/config/vehicle.json").read_bytes(),
            (REPO / "config/vehicle.json").read_bytes(),
        )
        self.assertNotIn("/usr/local", installed)
        postinstall = recipe.split("pkg_postinst_ontarget:${PN}() {", 1)[1].split(
            "\n}", 1
        )[0]
        postinstall = postinstall.replace("${bindir}/python3", sys.executable)
        postinstall = postinstall.replace(
            "${PYTHON_SITEPACKAGES_DIR}", sysconfig.get_path("purelib")
        )
        postinstall = postinstall.replace("/home/vtol", str(stage / "home/vtol"))
        postinstall = postinstall.replace("chown -R vtol:vtol", "test -d")
        subprocess.run(["sh", "-eu", "-c", postinstall], check=True)
        image_python = stage / "home/vtol/.local/share/vtol26/config/venv/bin/python3"
        result = subprocess.run(
            [
                image_python,
                "-I",
                "-c",
                "import jsonschema, sys; print(sys.prefix); print(jsonschema.__file__)",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        environment = str(image_python.parent.parent)
        self.assertTrue(
            all(line.startswith(environment) for line in result.stdout.splitlines())
        )
        self.assertTrue((stage / "etc/vtol/pso.json").is_file())
        self.assertTrue(
            (
                stage / "home/vtol/.local/share/vtol26/config/validate_vehicle.py"
            ).is_file()
        )
        self.assertTrue(
            (
                stage / "home/vtol/.local/share/vtol26/config/vehicle.schema.json"
            ).is_file()
        )
        shared = stage / "usr/share/keep.txt"
        shared.write_text("preserve shared data\n")
        config.parent.mkdir(parents=True)
        config.write_text('{"calibration":"example"}\n')
        subprocess.run(["sh", "-eu", "-c", body], check=True)
        self.assertEqual(shared.read_text(), "preserve shared data\n")
        self.assertIn('SYSTEMD_SERVICE:${PN} = "vtol_allocator@vtol.service"', recipe)
        image = (
            ROOT / "meta-vtol/recipes-core/images/vtol-companion-image.bb"
        ).read_text()
        self.assertIn(" vtol-allocator ", image)

    def test_invalid_hover_configuration_does_not_replace_snapshot(self):
        self.module.prepare(self.poky, self.bsp)
        provision = self.image / "build/provision/vtol-source.tar.gz"
        original = provision.read_bytes()
        source = self.vehicle
        vehicle = json.loads(source.read_text())
        for invalid in ("asymmetric", "insufficient", "nonfinite"):
            candidate = json.loads(json.dumps(vehicle))
            if invalid == "asymmetric":
                candidate["force_positions_body_m"]["motor_right"][0] += 0.1
            elif invalid == "insufficient":
                candidate["rigid_body"]["mass_kg"] = 100
            else:
                candidate["rigid_body"]["mass_kg"] = math.nan
            source.write_text(json.dumps(candidate))
            with self.assertRaises((ValueError, subprocess.CalledProcessError)):
                self.module.prepare(self.poky, self.bsp)
            self.assertEqual(provision.read_bytes(), original)

    def test_hil_generator_cannot_overwrite_source_vehicle(self):
        repo = self.image.parent.parent
        vehicle = repo / "config/vehicle.json"
        original = vehicle.read_bytes()
        result = subprocess.run(
            [
                sys.executable,
                str(repo / "companion/allocator/prepare_hil.py"),
                str(repo / "companion/allocator/src/pso/config.json"),
                str(self.vehicle),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(vehicle.read_bytes(), original)
        self.assertIn("Choose a separate output file", result.stderr)

    def build_environment(self):
        bin_dir = self.work / "bin"
        bin_dir.mkdir()
        self.log = self.work / "bitbake.json"
        self.calls = self.work / "setup-calls.jsonl"
        self.server = self.work / "archives"
        self.server.mkdir()
        (self.poky / "oe-init-build-env").write_text(
            'mkdir -p "$1"\ncd "$1"\n: "$YOCTO_TEST_OPTIONAL"\n'
        )
        for name, source in (
            ("poky-scarthgap", self.poky),
            ("meta-raspberrypi-scarthgap", self.bsp),
        ):
            with tarfile.open(self.server / f"{name}.tar.gz", "w:gz") as archive:
                archive.add(source, arcname=name)
        scripts = {
            "unshare": """import os, sys
mode = os.environ.get("VTOL_TEST_NAMESPACES", "allowed")
sys.exit(0 if mode == "allowed" or (mode == "profile" and os.environ.get("VTOL_IMAGE_USERNS") == "1") else 1)
""",
            "sysctl": 'print("1")\n',
            "sudo": """import json, os, sys
with open(os.environ["VTOL_TEST_CALLS"], "a") as log: log.write(json.dumps(sys.argv) + "\\n")
""",
            "aa-exec": """import json, os, sys
with open(os.environ["VTOL_TEST_CALLS"], "a") as log: log.write(json.dumps(sys.argv) + "\\n")
assert sys.argv[1] == "-p" and sys.argv[3] == "--"
os.execvpe(sys.argv[4], sys.argv[4:], os.environ)
""",
            "wget": """import json, os, shutil, sys
with open(os.environ["VTOL_TEST_CALLS"], "a") as log: log.write(json.dumps(sys.argv) + "\\n")
if os.environ.get("VTOL_TEST_DOWNLOAD_FAIL"): sys.exit(23)
name = "poky-scarthgap" if "/yoctoproject/poky/" in sys.argv[-1] else "meta-raspberrypi-scarthgap"
shutil.copyfile(os.path.join(os.environ["VTOL_TEST_ARCHIVES"], name + ".tar.gz"), sys.argv[2])
""",
            "bitbake": """import json, os, sys
with open(os.environ["VTOL_TEST_BITBAKE"], "w") as log: json.dump([os.getcwd(), sys.argv[1:]], log)
sys.exit(int(os.environ.get("VTOL_TEST_BITBAKE_EXIT", "0")))
""",
        }
        for name, script in scripts.items():
            executable = bin_dir / name
            executable.write_text(f"#!{sys.executable}\n" + script)
            executable.chmod(0o755)
        env = dict(
            os.environ,
            PATH=f"{bin_dir}:{os.environ['PATH']}",
            POKY_DIR=str(self.poky),
            BSP_DIR=str(self.bsp),
            VTOL_TEST_CALLS=str(self.calls),
            VTOL_TEST_BITBAKE=str(self.log),
            VTOL_TEST_ARCHIVES=str(self.server),
        )
        env.pop("VTOL_IMAGE_USERNS", None)
        return env

    def run_build(self, env, *args):
        return subprocess.run(
            ["bash", str(self.image / "build.sh"), *args],
            check=False,
            cwd=self.work,
            env=env,
            capture_output=True,
            text=True,
        )

    def test_build_script_refreshes_source_and_invokes_bitbake(self):
        env = self.build_environment()
        result = self.run_build(env, "-n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            json.loads(self.log.read_text()),
            [str(self.image / "build/yocto"), ["vtol-companion-image", "-n"]],
        )
        self.assertTrue((self.image / "build/provision/vtol-source.tar.gz").is_file())
        # Cached keys from previous builds are ignored, without deleting them.
        old_key = self.image / "build/provision/authorized_keys"
        old_key.write_text("unused cached key\n")
        result = self.run_build(env, "-n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(old_key.read_text(), "unused cached key\n")
        env["POKY_DIR"] = str(self.work / "missing")
        self.log.unlink()
        result = self.run_build(env)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.log.exists())
        self.assertFalse(
            self.calls.exists()
        )  # No download of custom paths or privileged setup.

    def test_build_script_downloads_defaults_and_reuses_archives(self):
        env = self.build_environment()
        env.pop("POKY_DIR")
        env.pop("BSP_DIR")
        result = self.run_build(env, "-n")
        self.assertEqual(result.returncode, 0, result.stderr)
        sources = self.image / "build/sources"
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(Path(call[0]).name == "wget" for call in calls))
        self.calls.unlink()
        for name in ("poky-scarthgap", "meta-raspberrypi-scarthgap"):
            self.assertTrue((sources / f"{name}.tar.gz.sha256").is_file())
            shutil.rmtree(sources / name)
        result = self.run_build(env, "-n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.calls.exists())  # Cached archives avoid networking.
        shutil.rmtree(sources / "poky-scarthgap")
        with (sources / "poky-scarthgap.tar.gz").open("ab") as archive:
            archive.write(b"corrupt cache")
        self.log.unlink()
        result = self.run_build(env)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.log.exists())
        self.assertFalse((sources / "poky-scarthgap").exists())

    def test_download_failure_never_runs_bitbake(self):
        env = self.build_environment()
        env.pop("POKY_DIR")
        env.pop("BSP_DIR")
        env["VTOL_TEST_DOWNLOAD_FAIL"] = "1"
        result = self.run_build(env)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.log.exists())
        self.assertFalse((self.image / "build/sources/poky-scarthgap").exists())

    def test_temporary_apparmor_profile_is_cleaned_up(self):
        env = self.build_environment()
        env["VTOL_TEST_NAMESPACES"] = "profile"
        for code in (0, 17):
            with self.subTest(bitbake_exit=code):
                env["VTOL_TEST_BITBAKE_EXIT"] = str(code)
                self.calls.unlink(missing_ok=True)
                result = self.run_build(env, "-n")
                self.assertEqual(result.returncode, code, result.stderr)
                calls = [
                    json.loads(line) for line in self.calls.read_text().splitlines()
                ]
                self.assertEqual(
                    [Path(c[0]).name for c in calls], ["sudo", "aa-exec", "sudo"]
                )
                self.assertEqual(
                    calls[0][1:4], ["apparmor_parser", "--skip-cache", "-r"]
                )
                self.assertEqual(
                    calls[-1][1:4], ["apparmor_parser", "--skip-cache", "-R"]
                )
                self.assertEqual(calls[0][-1], calls[-1][-1])
                profile = Path(calls[0][-1]).read_text()
                self.assertIn("userns,", profile)
                self.assertIn(f"profile {calls[1][2]} flags=(unconfined)", profile)
                self.assertEqual(
                    json.loads(self.log.read_text())[1], ["vtol-companion-image", "-n"]
                )

    def test_namespace_failure_does_not_recurse(self):
        env = self.build_environment()
        env["VTOL_TEST_NAMESPACES"] = "blocked"
        result = self.run_build(env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("User namespaces are unavailable", result.stderr)
        self.assertFalse(self.log.exists())
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        self.assertEqual([Path(c[0]).name for c in calls], ["sudo", "aa-exec", "sudo"])

    def test_missing_locale_is_generated(self):
        env = self.build_environment()
        python = self.work / "bin/python3"
        python.write_text(
            f"#!{sys.executable}\n"
            "import os, sys\n"
            "if sys.argv[1] == '-c': sys.exit(1)\n"
            "os.execv(sys.executable, [sys.executable] + sys.argv[1:])\n"
        )
        python.chmod(0o755)
        result = self.run_build(env, "-n")
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][1:], ["locale-gen", "en_US.UTF-8"])


if __name__ == "__main__":
    unittest.main()
