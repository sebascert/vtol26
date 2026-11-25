"""Check assembled visual geometry, independently of the aircraft force model."""

from math import cos, sin
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1]) / "models/stallion_vtol"
model = ET.parse(root / "model.sdf").find("model")
frame = model.find("frame[@name='cad']")
assert frame.attrib["attached_to"] == "base_link"
tx, ty, tz, roll, pitch, yaw = map(float, frame.findtext("pose").split())


def body(point):
    x, y, z = point
    y, z = cos(roll) * y - sin(roll) * z, sin(roll) * y + cos(roll) * z
    x, z = cos(pitch) * x + sin(pitch) * z, -sin(pitch) * x + cos(pitch) * z
    return (cos(yaw) * x - sin(yaw) * y + tx, sin(yaw) * x + cos(yaw) * y + ty, z + tz)


bounds = {}
for visual in model.findall("link/visual"):
    mesh = visual.find("geometry/mesh")
    if mesh is None:
        continue
    assert visual.find("pose").attrib["relative_to"] == "cad"
    px, py, pz, rx, ry, rz = map(float, visual.findtext("pose").split())
    assert (rx, ry, rz) == (0, 0, 0)
    scale = tuple(map(float, mesh.findtext("scale").split()))
    data = (
        root / mesh.findtext("uri").removeprefix("model://stallion_vtol/")
    ).read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    assert len(data) == 84 + count * 50
    lo, hi = [float("inf")] * 3, [float("-inf")] * 3
    for triangle in struct.iter_unpack("<12fH", data[84:]):
        for start in (3, 6, 9):
            point = body(
                (
                    triangle[start] * scale[0] + px,
                    triangle[start + 1] * scale[1] + py,
                    triangle[start + 2] * scale[2] + pz,
                )
            )
            lo = list(map(min, lo, point))
            hi = list(map(max, hi, point))
    bounds[visual.attrib["name"]] = (lo, hi)


def combine(names):
    pairs = [bounds[name] for name in names]
    return (
        [min(p[0][i] for p in pairs) for i in range(3)],
        [max(p[1][i] for p in pairs) for i in range(3)],
    )


lo, hi = combine(bounds)
assert 0.98 < hi[0] - lo[0] < 1.02, (lo, hi)
assert 1.3 < hi[1] - lo[1] < 1.4, (lo, hi)
assert hi[2] - lo[2] < 0.3, (lo, hi)  # Aircraft lies horizontally.
assert bounds["nose"][0][0] > bounds["tail_1"][1][0]

# Fuselage cross sections meet in order instead of intersecting or floating.
sections = [combine(("fuselage_left", "fuselage_right"))]
sections += [combine((f"fus_{i}_l", f"fus_{i}_r")) for i in range(2, 6)]
for lo, hi in sections:
    assert abs(lo[1] + hi[1]) < 5e-6  # Centered halves.
for front, rear in zip(sections, sections[1:]):
    assert abs(front[0][0] - rear[1][0]) < 1e-5, (front, rear)
for i in range(1, 4):
    left, right = bounds[f"wing_{i}_left"], bounds[f"wing_{i}_right"]
    assert left[0][1] > 0 and right[1][1] < 0
    assert abs(left[0][1] + right[1][1]) < 1e-5
    assert abs(left[1][1] + right[0][1]) < 1e-5
    assert abs(left[0][0] - right[0][0]) < 1e-5

boom = model.find("link/visual[@name='tail_boom']")
assert boom is not None and float(boom.findtext("geometry/cylinder/length")) == 0.435
print(
    "PASS horizontal aircraft, joined fuselage, symmetric wings, tail and metre scale"
)
