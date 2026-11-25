"""Prepare the tuning shared by the HIL image and desktop hover regression."""

import copy
import json
import math
import subprocess
from pathlib import Path


def configuration(vehicle, tuning):
    points = vehicle["force_positions_body_m"]
    front, right, rear = (
        points[key] for key in ("motor_left", "motor_right", "motor_rear")
    )
    # shortcut: symmetric, vertical-rotor hover; extend when validating asymmetric aircraft.
    if (
        front[0] != right[0]
        or front[1] != -right[1]
        or rear[1] != 0
        or front[0] == rear[0]
        or vehicle["propulsion"]["reaction_torque_per_thrust_m"] != [0, 0, 0]
    ):
        raise ValueError(
            "HIL hover trim requires symmetric front motors, a centered rear motor and zero reaction torque"
        )
    weight = vehicle["rigid_body"]["mass_kg"] * vehicle["environment"]["gravity_m_s2"]
    aft = weight * front[0] / (front[0] - rear[0])
    thrust = [(weight - aft) / 2, (weight - aft) / 2, aft]
    coefficients = vehicle["propulsion"]["k_t_n_per_command"]
    if len(coefficients) != 3 or any(
        not math.isfinite(k) or k <= 0 for k in coefficients
    ):
        raise ValueError("HIL thrust coefficients must be three positive finite values")
    trim = [force / k for force, k in zip(thrust, coefficients)] + [0] * 6
    actuators = vehicle["actuators"]
    if len(actuators) != 9 or any(
        not math.isfinite(value) or not actuator["min"] <= value <= actuator["max"]
        for value, actuator in zip(trim, actuators)
    ):
        raise ValueError(
            "HIL hover trim exceeds the selected vehicle's actuator limits"
        )
    result = copy.deepcopy(tuning)
    result["actuators"]["initial"] = trim
    result["cost_function"]["dt"] = 0.25
    result["pso"].update(
        num_particles=96,
        max_iterations=15,
        external_cycles=2,
        k_am=100,
        convergence_threshold=0.1,
    )
    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tuning", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.resolve() in (
        (Path.home() / ".local/share/vtol26/config/vehicle.json"),
        args.tuning.resolve(),
    ):
        parser.error(
            "Choose a separate output file; source configuration must be preserved"
        )
    result = configuration(
        json.loads(
            subprocess.check_output(
                [
                    str(Path.home() / ".local/share/vtol26/config/venv/bin/python3"),
                    "-I",
                    str(Path.home() / ".local/share/vtol26/config/validate_vehicle.py"),
                    "--print-json",
                ]
            )
        ),
        json.loads(args.tuning.read_text()),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
