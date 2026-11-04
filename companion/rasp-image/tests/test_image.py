"""Host checks for provisioning and image contracts; no Git, root or networking."""
import importlib.util
import ipaddress
import shutil
import stat
import subprocess
import tarfile
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent.parent


class ImageTest(unittest.TestCase):
    def setUp(self):
        (ROOT / "build").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "build")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.image = self.work / "project/companion/rasp-image"
        shutil.copytree(ROOT, self.image, ignore=shutil.ignore_patterns("build", "__pycache__"))
        shutil.copytree(REPO / "companion/mavlink_uart", self.image.parent / "mavlink_uart",
                        ignore=shutil.ignore_patterns("build", "__pycache__"))
        shutil.copytree(REPO / "simulation/pso", self.image.parent.parent / "simulation/pso",
                        ignore=shutil.ignore_patterns("build", "__pycache__"))
        self.poky, self.bsp = self.work / "poky", self.work / "meta-raspberrypi"
        for path, content in {
            self.poky / "oe-init-build-env": "",
            self.poky / "meta/conf/distro/include/yocto-uninative.inc": "",
            self.poky / "meta/conf/layer.conf": 'LAYERSERIES_CORENAMES = "scarthgap"',
            self.bsp / "conf/layer.conf": 'LAYERSERIES_COMPAT_raspberrypi = "scarthgap"',
            self.bsp / "conf/machine/raspberrypi4-64.conf": "",
        }.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        spec = importlib.util.spec_from_file_location("image_configure", self.image / "configure.py")
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_prepare_keys_snapshot_and_refresh(self):
        build = self.module.prepare(self.poky, self.bsp)
        key = self.image / "build/ssh/id_ed25519"
        self.assertEqual(stat.S_IMODE(key.stat().st_mode), 0o600)
        original_key = key.read_bytes()
        provision = self.image / "build/provision"
        self.assertEqual((provision / "authorized_keys").read_bytes(), key.with_suffix(".pub").read_bytes())
        with tarfile.open(provision / "vtol-source.tar.gz") as archive:
            names = archive.getnames()
            self.assertIn("vtol-source/simulation/pso/pso_runtime.hpp", names)
            self.assertIn("vtol-source/companion/mavlink_uart/deploy/mavlink-uart@.service", names)
            self.assertFalse(any("id_ed25519" in name or "/build/" in name for name in names))
        self.assertNotIn("@PROVISION@", (build / "conf/local.conf").read_text())
        source = self.image.parent / "mavlink_uart/src/main.cpp"
        source.write_text("// updated working tree\n")
        self.module.prepare(self.poky, self.bsp)
        self.assertEqual(original_key, key.read_bytes())
        with tarfile.open(provision / "vtol-source.tar.gz") as archive:
            self.assertEqual(archive.extractfile("vtol-source/companion/mavlink_uart/src/main.cpp").read(),
                             source.read_bytes())

    def test_bad_inputs_are_rejected(self):
        for value in ("", "-----BEGIN OPENSSH PRIVATE KEY-----\nsecret", "ssh-ed25519 invalid"):
            public_key = self.work / "key.pub"
            public_key.write_text(value)
            with self.assertRaises((ValueError, subprocess.CalledProcessError)):
                self.module.prepare(self.poky, self.bsp, public_key)
        with self.assertRaises(ValueError):
            self.module.prepare(self.work / "missing", self.bsp)
        with self.assertRaises(ValueError):
            self.module.bitbake_path(self.work / "bad path")
        (self.bsp / "conf/layer.conf").write_text('LAYERSERIES_COMPAT_raspberrypi = "master"')
        with self.assertRaises(ValueError):
            self.module.prepare(self.poky, self.bsp)

    def test_network_ssh_and_uart_contract(self):
        files = ROOT / "meta-vtol/recipes-core/vtol-config/files"
        network = (files / "10-vtol-ethernet.network").read_text()
        address = next(line.split("=", 1)[1] for line in network.splitlines() if line.startswith("Address="))
        self.assertEqual(str(ipaddress.ip_interface(address)), "192.168.76.2/24")
        self.assertIn("Name=eth0", network)
        self.assertNotIn("Gateway=", network)
        ssh = (ROOT / "meta-vtol/recipes-connectivity/openssh/files/sshd_config").read_text()
        for value in ("PermitRootLogin no", "PasswordAuthentication no", "AllowUsers vtol"):
            self.assertIn(value, ssh)
        config = (ROOT / "local.conf.in").read_text()
        for value in ('ENABLE_UART = "1"', 'CMDLINE_SERIAL = ""', 'SERIAL_CONSOLES = ""', 'dtoverlay=disable-bt'):
            self.assertIn(value, config)

    def test_application_install_paths(self):
        stage = self.work / "stage"
        template = stage / "usr/lib/systemd/system/mavlink-uart@.service"
        template.parent.mkdir(parents=True)
        shutil.copy(REPO / "companion/mavlink_uart/deploy/mavlink-uart@.service", template)
        config = stage / "usr/share/vtol26/companion/mavlink_uart/src/pso/config.json"
        config.parent.mkdir(parents=True)
        config.write_text('{"calibration":"example"}\n')
        recipe = (ROOT / "meta-vtol/recipes-apps/mavlink-uart/mavlink-uart.bb").read_text()
        body = recipe.split("do_install:append() {", 1)[1].rsplit("}", 1)[0]
        for key, value in {"D": str(stage), "sysconfdir": "/etc", "datadir": "/usr/share",
                           "libdir": "/usr/lib", "bindir": "/usr/bin", "systemd_system_unitdir": "/usr/lib/systemd/system"}.items():
            body = body.replace("${" + key + "}", value)
        subprocess.run(["sh", "-eu", "-c", body], check=True)
        installed = template.read_text()
        self.assertIn("ExecStart=/usr/bin/mavlink_uart", installed)
        self.assertIn("--pso-config /etc/vtol/pso.json", installed)
        self.assertNotIn("/usr/local", installed)
        self.assertTrue((stage / "etc/vtol/pso.json").is_file())
        self.assertIn('SYSTEMD_SERVICE:${PN} = "mavlink-uart@vtol.service"', recipe)


if __name__ == "__main__":
    unittest.main()
