"""Validate the shared contract and synthetic vehicle without running PX4."""

import copy
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "vehicle.json"
spec = importlib.util.spec_from_file_location(
    "vehicle_config", ROOT / "validate_vehicle.py"
)
vehicle_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vehicle_config)


class VehicleConfigTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = vehicle_config.load(REFERENCE)

    def test_schema_and_reference(self):
        Draft202012Validator.check_schema(json.loads(vehicle_config.SCHEMA.read_text()))
        self.assertEqual(self.vehicle["metadata"]["calibration_status"], "placeholder")
        self.assertEqual(len(self.vehicle["actuators"]), 9)

    def test_strict_shape_frames_units_and_order(self):
        changes = [
            lambda v: v.pop("environment"),
            lambda v: v.update(unknown=1),
            lambda v: v["interface"].update(body_frame="FLU"),
            lambda v: v["interface"].update(world_frame="ENU"),
            lambda v: v["interface"].update(quaternion="xyzw"),
            lambda v: v["actuators"][3].update(input_unit="deg"),
            lambda v: v["actuators"][0].update(input_unit="RPM"),
            lambda v: v["actuators"].reverse(),
            lambda v: v["actuators"].pop(),
            lambda v: v["force_positions_body_m"].update(wing=[0, 0]),
            lambda v: v["rigid_body"].update(mass_kg=True),
            lambda v: v["propulsion"].update(law="quadratic"),
        ]
        for i, change in enumerate(changes):
            with self.subTest(case=i):
                candidate = copy.deepcopy(self.vehicle)
                change(candidate)
                with self.assertRaises(ValidationError):
                    vehicle_config.validate(candidate)

    def test_physical_and_output_constraints(self):
        changes = [
            lambda v: v["rigid_body"].update(mass_kg=-1),
            lambda v: v["rigid_body"].update(inertia_kg_m2=[0, 1, 1]),
            lambda v: v["rigid_body"].update(inertia_kg_m2=[1, 1, 3]),
            lambda v: v["propulsion"].update(k_t_n_per_command=[0, 1, 1]),
            lambda v: v["propulsion"].update(k_t_n_per_command=[0.00024] * 3),
            lambda v: v["actuators"][0].update(max=0),
            lambda v: v["actuators"][0].update(disarmed=1),
            lambda v: v["actuators"][0].update(neutral=3000),
            lambda v: v["actuators"][3].update(min=0.1, neutral=0.1, disarmed=0.1),
            lambda v: v["actuators"][1].update(hil_channel=1),
            lambda v: v["actuators"][0].update(hil_channel=17),
            lambda v: v["aerodynamics"]["wing"].update(cd0=-1),
            lambda v: v["rigid_body"].update(mass_kg=math.nan),
            lambda v: v["force_positions_body_m"].update(wing=[math.inf, 0, 0]),
            lambda v: v["actuators"][0].update(max=1e40),
            lambda v: v["actuators"][5].update(
                min=1, max=1 + 1e-12, neutral=1, disarmed=1
            ),
        ]
        for i, change in enumerate(changes):
            with self.subTest(case=i):
                candidate = copy.deepcopy(self.vehicle)
                change(candidate)
                with self.assertRaises((ValueError, ValidationError)):
                    vehicle_config.validate(candidate)

    def test_oversized_input_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "oversized.json"
            source.write_text(json.dumps(self.vehicle) + " " * (1024 * 1024))
            with self.assertRaisesRegex(ValueError, "exceeds 1 MiB"):
                vehicle_config.load(source)

    def test_reference_has_balanced_hover_trim(self):
        # Independent static equilibrium calculation for this symmetric fixture.
        v = self.vehicle
        positions = v["force_positions_body_m"]
        points = [
            positions[name] for name in ("motor_left", "motor_right", "motor_rear")
        ]
        self.assertEqual(points[0][0], points[1][0])
        self.assertEqual(points[0][1], -points[1][1])
        self.assertEqual(points[2][1], 0)
        self.assertEqual(v["propulsion"]["reaction_torque_per_thrust_m"], [0, 0, 0])
        weight = v["rigid_body"]["mass_kg"] * v["environment"]["gravity_m_s2"]
        rear = weight * points[0][0] / (points[0][0] - points[2][0])
        thrusts = [(weight - rear) / 2, (weight - rear) / 2, rear]
        self.assertAlmostEqual(sum(thrusts), weight)
        self.assertAlmostEqual(
            sum(-r[1] * thrust for r, thrust in zip(points, thrusts)), 0
        )
        self.assertAlmostEqual(
            sum(r[0] * thrust for r, thrust in zip(points, thrusts)), 0
        )
        for i, thrust in enumerate(thrusts):
            command = thrust / v["propulsion"]["k_t_n_per_command"][i]
            self.assertGreaterEqual(command, v["actuators"][i]["min"])
            self.assertLessEqual(command, v["actuators"][i]["max"])

    def test_normalization_contract_endpoints_and_roundtrip(self):
        for i, actuator in enumerate(self.vehicle["actuators"]):
            lower, upper = actuator["min"], actuator["max"]
            for command in (lower, actuator["neutral"], upper):
                fraction = (command - lower) / (upper - lower)
                output = fraction if i < 3 else 2 * fraction - 1
                inverse_fraction = output if i < 3 else (output + 1) / 2
                self.assertAlmostEqual(
                    lower + inverse_fraction * (upper - lower), command
                )
                self.assertGreaterEqual(output, 0 if i < 3 else -1)
                self.assertLessEqual(output, 1)

    def test_invalid_json_does_not_overwrite_snapshot(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            digest = vehicle_config.stage(self.vehicle, root / "snapshot")
            snapshot = root / "snapshot/vehicle.json"
            self.assertEqual(hashlib.sha256(snapshot.read_bytes()).hexdigest(), digest)
            original = snapshot.read_bytes()
            malformed = [
                '{"schema_version": 1, "schema_version": 1}',
                '{"mass": NaN}',
                '{"mass": 1e999}',
                "{broken",
            ]
            for value in malformed:
                source = root / "bad.json"
                source.write_text(value)
                result = subprocess.run(
                    [
                        sys.executable,
                        str(ROOT / "validate_vehicle.py"),
                        str(source),
                        "--output-dir",
                        str(snapshot.parent),
                    ],
                    capture_output=True,
                    check=False,
                    text=True,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(snapshot.read_bytes(), original)

    def test_staged_configuration_digest_is_deterministic(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            first = vehicle_config.stage(self.vehicle, root)
            second = vehicle_config.stage(
                dict(reversed(list(self.vehicle.items()))), root
            )
            self.assertEqual(first, second)
            self.assertEqual(
                (root / "vehicle.sha256").read_text(), f"{first}  vehicle.json\n"
            )

    def test_generated_limits_compile_and_normalize_configured_values(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            vehicle_config.stage(self.vehicle, root)
            source = root / "check.cpp"
            source.write_text("""#include "vehicle_actuators.hpp"
#include "policy.hpp"
#include <cassert>
int main() {
    using namespace vtol_vehicle;
    assert(maximum_thrust > 39.24f);
    for (unsigned i=0; i<9; ++i) {
        assert(minimum[i]<maximum[i]);
        assert(vtol_pso::normalize(disarmed[i],i,minimum,maximum)==(i<5 ? (i<3 ? 0.f : -1.f) : 0.f));
    }
}
""")
            subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    str(source),
                    "-I",
                    str(root),
                    "-I",
                    str(ROOT.parent / "controller/src/modules/pso_control"),
                    "-o",
                    str(root / "check"),
                ],
                check=True,
            )
            subprocess.run([root / "check"], check=True)


if __name__ == "__main__":
    unittest.main()
