"""Public allocator CLI no longer accepts vehicle-path overrides."""

import subprocess
import sys
import unittest

BINARY = sys.argv.pop(1)


class ConfigAccessTest(unittest.TestCase):
    def test_help_has_no_vehicle_override(self):
        result = subprocess.run(
            [BINARY, "--help"], capture_output=True, text=True, check=True
        )
        self.assertNotIn("--vehicle-config", result.stdout)

    def test_vehicle_override_is_rejected(self):
        result = subprocess.run(
            [BINARY, "--vehicle-config", "other.json"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Unknown argument", result.stderr)


if __name__ == "__main__":
    unittest.main()
