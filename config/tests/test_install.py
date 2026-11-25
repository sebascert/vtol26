"""Exercise the installer in a temporary user home without changing host files."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILES = (
    "vehicle.json",
    "vehicle.schema.json",
    "validate_vehicle.py",
    "requirements.txt",
)


class InstallTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.home = Path(temporary.name)
        self.destination = self.home / ".local/share/vtol26/config"
        self.env = dict(os.environ, HOME=str(self.home))
        self.installer = ROOT / "install.sh"

    def install(self, *args):
        return subprocess.run(
            ["bash", self.installer, *args],
            env=self.env,
            capture_output=True,
            text=True,
            check=False,
        )

    def assert_reference_files(self):
        for name in FILES:
            self.assertEqual(
                (self.destination / name).read_bytes(), (ROOT / name).read_bytes()
            )

    def assert_isolated_environment(self):
        python = self.destination / "venv/bin/python3"
        result = subprocess.run(
            [
                python,
                "-I",
                "-c",
                "import jsonschema, sys; print(sys.prefix); print(jsonschema.__file__)",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        environment = str(python.parent.parent)
        self.assertTrue(
            all(line.startswith(environment) for line in result.stdout.splitlines())
        )
        self.assertIn(
            "include-system-site-packages = false",
            (python.parent.parent / "pyvenv.cfg").read_text(),
        )

    def test_existing_files_without_environment_are_upgraded(self):
        self.destination.mkdir(parents=True)
        for name in FILES:
            (self.destination / name).write_text(f"keep {name}")
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration installed\n")
        self.assert_isolated_environment()
        for name in FILES:
            self.assertEqual((self.destination / name).read_text(), f"keep {name}")

    def test_first_install_preservation_and_force(self):
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration installed\n")
        self.assert_reference_files()
        self.assert_isolated_environment()

        contents = (b"", b"invalid schema", b"invalid Python", b"invalid requirements")
        for name, data in zip(FILES, contents):
            (self.destination / name).write_bytes(data)
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration already installed\n")
        for name, data in zip(FILES, contents):
            self.assertEqual((self.destination / name).read_bytes(), data)

        result = self.install("-f")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration installed\n")
        self.assert_reference_files()

    def test_partial_install_preserves_dangling_symlink(self):
        self.destination.mkdir(parents=True)
        vehicle = self.destination / "vehicle.json"
        vehicle.write_bytes(b"")
        validator = self.destination / "validate_vehicle.py"
        validator.symlink_to("missing-validator.py")
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration installed\n")
        self.assertEqual(vehicle.read_bytes(), b"")
        self.assertTrue(validator.is_symlink())
        self.assertFalse(validator.exists())
        self.assertEqual(
            (self.destination / "vehicle.schema.json").read_bytes(),
            (ROOT / "vehicle.schema.json").read_bytes(),
        )

        result = self.install("-f")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(validator.is_symlink())
        self.assertFalse((self.destination / "missing-validator.py").exists())
        self.assert_reference_files()

    def test_empty_home_is_rejected(self):
        self.env["HOME"] = ""
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("HOME must identify", result.stderr)

    def test_validation_failure_prevents_all_copies(self):
        source = self.home / "source"
        source.mkdir()
        for name in (*FILES, "install.sh"):
            shutil.copy2(ROOT / name, source / name)
        self.installer = source / "install.sh"
        (source / "vehicle.json").write_text('{"schema_version":99}')

        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Invalid vehicle configuration", result.stderr)
        self.assertFalse((self.destination / "vehicle.json").exists())

        self.destination.mkdir(parents=True, exist_ok=True)
        for name in FILES:
            (self.destination / name).write_text(f"keep {name}")
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "Configuration already installed\n")

        self.assertNotEqual(self.install("-f").returncode, 0)
        for name in FILES:
            self.assertEqual((self.destination / name).read_text(), f"keep {name}")

        (self.destination / "vehicle.schema.json").unlink()
        self.assertNotEqual(self.install().returncode, 0)
        self.assertFalse((self.destination / "vehicle.schema.json").exists())
        self.assertEqual(
            (self.destination / "vehicle.json").read_text(), "keep vehicle.json"
        )

    def test_dependency_failure_prevents_configuration_copy(self):
        source = self.home / "source"
        source.mkdir()
        for name in (*FILES, "install.sh"):
            shutil.copy2(ROOT / name, source / name)
        self.installer = source / "install.sh"
        (source / "requirements.txt").write_text("no-such-vtol-validator-package==0\n")
        self.env["PIP_NO_INDEX"] = "1"
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("Configuration installed", result.stdout)
        for name in FILES:
            self.assertFalse((self.destination / name).exists())

    def test_unknown_arguments_are_rejected(self):
        for args in (("-x",), ("",), ("-f", "extra"), ("-f", "-f")):
            with self.subTest(args=args):
                self.assertEqual(self.install(*args).returncode, 2)
                self.assertFalse(self.destination.exists())


if __name__ == "__main__":
    unittest.main()
