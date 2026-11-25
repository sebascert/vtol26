"""Check C++ delegation using a disposable installation, without host changes."""

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOADER = ROOT.parent / "companion/allocator/src/pso"


class CppLoaderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.work = Path(cls.temp.name)
        subprocess.run(
            ["bash", ROOT / "install.sh"],
            env=dict(os.environ, HOME=str(cls.work)),
            check=True,
            capture_output=True,
        )
        cls.validator = cls.work / ".local/share/vtol26/config/validate_vehicle.py"
        cls.schema = cls.validator.with_name("vehicle.schema.json")
        cls.vehicle = cls.work / ".local/share/vtol26/config/vehicle.json"
        cls.reference = cls.vehicle.read_text()
        source = LOADER / "config_loader.cpp"
        probe = cls.work / "probe.cpp"
        probe.write_text("""#include "config_loader.hpp"
#include <iostream>
int main(int argc, char** argv) {
    try {
        auto c = argc == 3 ? CargadorConfiguracion::cargarDesdeJson(argv[2], argv[1])
                           : argc == 2 ? CargadorConfiguracion::cargarVehiculo(argv[1])
                                       : CargadorConfiguracion::cargarVehiculo();
        std::cout << c.masa << " " << c.actuator_max[0];
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what(); return 1; }
}
""")
        flags = subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "json-c"], text=True
        ).split()
        cls.binary = cls.work / "probe"
        subprocess.run(
            [
                "c++",
                "-std=c++17",
                str(source),
                str(probe),
                "-I",
                str(LOADER),
                "-o",
                str(cls.binary),
                *flags,
            ],
            check=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.vehicle.write_text(self.reference)
        shutil.copy2(ROOT / "vehicle.schema.json", self.schema)

    def run_probe(self, *args):
        return subprocess.run(
            [self.binary, *(args or [self.vehicle])],
            capture_output=True,
            text=True,
            timeout=10,
            cwd=self.work,
            env=dict(os.environ, HOME=str(self.work)),
            check=False,
        )

    def test_default_paths_follow_the_runtime_user_home(self):
        another_home = self.work / "another user"
        environment = dict(os.environ, HOME=str(another_home))
        subprocess.run(
            ["bash", ROOT / "install.sh"],
            env=environment,
            check=True,
            capture_output=True,
        )
        vehicle = another_home / ".local/share/vtol26/config/vehicle.json"
        config = json.loads(self.reference)
        config["rigid_body"]["mass_kg"] = 3
        vehicle.write_text(json.dumps(config))
        result = subprocess.run(
            [self.binary], env=environment, capture_output=True, text=True, check=True
        )
        self.assertEqual(result.stdout, "3 2000")

    def test_valid_and_invalid_configuration(self):
        self.assertEqual(self.run_probe().stdout, "4 2000")
        self.vehicle.write_text('{"schema_version":99}')
        result = self.run_probe()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertIn("vehicle validation failed", result.stderr)
        self.vehicle.unlink()
        self.assertNotEqual(self.run_probe().returncode, 0)

    def test_schema_change_is_enforced_without_rebuilding_cpp(self):
        schema = json.loads(self.schema.read_text())
        schema["properties"]["rigid_body"]["properties"]["mass_kg"]["const"] = 3
        self.schema.write_text(json.dumps(schema))
        self.assertNotEqual(self.run_probe().returncode, 0)
        config = json.loads(self.reference)
        config["rigid_body"]["mass_kg"] = 3
        self.vehicle.write_text(json.dumps(config))
        self.assertEqual(self.run_probe().stdout, "3 2000")

    def test_paths_are_arguments_not_shell_commands(self):
        source = self.work / "vehicle ' $(touch unexpected).json"
        source.write_text(self.reference)
        self.assertEqual(self.run_probe(source).returncode, 0)
        self.assertFalse((self.work / "unexpected").exists())

    def test_missing_environment_stops_loading(self):
        python = self.validator.parent / "venv/bin/python3"
        saved = python.with_name("saved-python3")
        python.rename(saved)
        try:
            self.assertNotEqual(self.run_probe().returncode, 0)
        finally:
            saved.rename(python)

    def test_missing_validator_stops_loading(self):
        original = self.validator.read_bytes()
        self.validator.unlink()
        try:
            self.assertNotEqual(self.run_probe().returncode, 0)
        finally:
            self.validator.write_bytes(original)

    def test_tuning_fields_stay_separate(self):
        tuning = json.loads((LOADER / "config.json").read_text())
        tuning["physics"] = {"mass": 900}
        path = self.work / "tuning.json"
        path.write_text(json.dumps(tuning))
        result = self.run_probe(self.vehicle, path)
        self.assertIn("unknown field physics", result.stderr)


if __name__ == "__main__":
    unittest.main()
