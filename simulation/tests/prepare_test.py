"""Check that SDF generation follows the selected JSON and preserves sources."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

prepare, source, vehicle, build = map(Path, sys.argv[1:])
templates = [source / "models/stallion_vtol/model.sdf", source / "worlds/pvtol.sdf"]
original = [path.read_bytes() for path in templates]
with tempfile.TemporaryDirectory(prefix="prepare test ", dir=build) as temporary:
    root = Path(temporary)
    config = json.loads(vehicle.read_text())
    config["rigid_body"] = {
        "mass_kg": 3,
        "inertia_kg_m2": [0.2, 0.25, 0.3],
        "cg_model_m": [0.1, -0.2, 0.3],
    }
    config["environment"]["gravity_m_s2"] = 8
    selected = root / "selected vehicle.json"
    selected.write_text(json.dumps(config))
    output = root / "generated world"

    def run(path, destination):
        return subprocess.run(
            [prepare, path, source, destination], capture_output=True, text=True
        )

    result = run(selected, output)
    assert result.returncode == 0, result.stderr
    model = ET.parse(output / "models/stallion_vtol/model.sdf")
    inertial = model.find("model/link/inertial")
    assert float(inertial.findtext("mass")) == 3
    assert list(map(float, inertial.findtext("pose").split())) == [
        0.1,
        0.2,
        -0.3,
        0,
        0,
        0,
    ]
    link = model.find("model/link")
    assert link.attrib["name"] == "base_link"
    assert {s.attrib["name"] for s in link.findall("sensor")} == {
        "imu_sensor",
        "air_pressure_sensor",
        "navsat_sensor",
    }
    for sensor in link.findall("sensor"):
        assert list(map(float, sensor.findtext("pose").split())) == [
            0.1,
            0.2,
            -0.3,
            0,
            0,
            0,
        ]
    assert [
        float(inertial.findtext(f"inertia/{axis}")) for axis in ("ixx", "iyy", "izz")
    ] == [0.2, 0.25, 0.3]
    assert [
        float(inertial.findtext(f"inertia/{axis}")) for axis in ("ixy", "ixz", "iyz")
    ] == [0, 0, 0]
    assert model.findtext("model/plugin/vehicle_config") == str(output / "vehicle.json")
    assert json.loads((output / "vehicle.json").read_text()) == config
    assert ET.parse(output / "pvtol.sdf").findtext("world/gravity") == "0 0 -8"
    assert (output / "models/stallion_vtol/meshes").resolve() == (
        source / "models/stallion_vtol/meshes"
    ).resolve()
    assert run(output / "vehicle.json", output).returncode == 0

    # Reject bad input before creating or replacing generated artifacts.
    config["rigid_body"]["mass_kg"] = -1
    selected.write_text(json.dumps(config))
    previous = (output / "vehicle.json").read_bytes()
    assert run(selected, output).returncode != 0
    assert (output / "vehicle.json").read_bytes() == previous
    assert run(selected, root / "invalid").returncode != 0
    assert not (root / "invalid").exists()
    assert run(root / "missing.json", output).returncode != 0

assert [path.read_bytes() for path in templates] == original
print(
    "PASS selected configuration, CG frames, snapshots, paths with spaces and invalid inputs"
)
