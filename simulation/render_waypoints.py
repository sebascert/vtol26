"""Render a completed SIL flight from its measured Gazebo positions and attitudes."""

import argparse
from bisect import bisect_right
import csv
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import shutil
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image as CameraImage
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from PIL import Image, ImageDraw, ImageFont

FPS = 25
MODEL = "controlled_drone"
CAMERA = Path(__file__).resolve().parent / "worlds/waypoint_camera.sdf"


def waypoint_camera(arrivals):
    model = ET.parse(CAMERA).find("model")
    link = model.find("link")
    goals = {entry["phase"]: entry["target_ned_m"] for entry in arrivals}
    # The route is stored as launch-relative NED. The scene uses Gazebo ENU.
    pads = {
        name: (goals[phase][1], goals[phase][0])
        for name, phase in (("home", "hover"), ("north", "north"), ("east", "east"))
    }
    for name, (x, y) in pads.items():
        pose = link.find(f"visual[@name='{name}']/pose")
        _, _, z, *angles = pose.text.split()
        pose.text = " ".join((str(x), str(y), z, *angles))
    for name, start, end in (
        ("north_leg", "home", "north"),
        ("east_leg", "north", "east"),
        ("return_leg", "east", "home"),
    ):
        ax, ay = pads[start]
        bx, by = pads[end]
        stripe = link.find(f"visual[@name='{name}']")
        pose = stripe.find("pose")
        z = pose.text.split()[2]
        pose.text = (
            f"{(ax + bx) / 2} {(ay + by) / 2} {z} 0 0 {math.atan2(by - ay, bx - ax)}"
        )
        size = stripe.find("geometry/box/size")
        sx, sy, depth = map(float, size.text.split())
        size.text = f"{math.hypot(bx - ax, by - ay)} {min(sx, sy)} {depth}"

    # Frame the route plus two metres for the wingspan and margins at both ends.
    xs, ys = zip(*pads.values())
    width = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) + 2.0
    sensor = link.find("sensor")
    fov = float(sensor.findtext("camera/horizontal_fov"))
    distance = width / (2 * math.tan(fov / 2))
    pose = sensor.find("pose")
    roll, pitch, yaw = map(float, pose.text.split()[3:])
    horizontal = distance * math.cos(pitch)
    x = (min(xs) + max(xs)) / 2 - horizontal * math.cos(yaw)
    y = (min(ys) + max(ys)) / 2 - horizontal * math.sin(yaw)
    z = -goals["hover"][2] + distance * math.sin(pitch)
    pose.text = f"{x} {y} {z} {roll} {pitch} {yaw}"
    return model


def replay_world(directory, arrivals):
    # Only the measured aircraft pose drives the replay. Keep the run's exact
    # assembly, material definitions, lighting and model resource directory.
    tree = ET.parse(directory / "world/pvtol.sdf")
    world = tree.find("world")
    for plugin in world.findall("plugin"):
        if plugin.get("name") != "gz::sim::systems::Physics":
            world.remove(plugin)
    for system, filename in (
        ("Sensors", "sensors"),
        ("UserCommands", "user-commands"),
        ("SceneBroadcaster", "scene-broadcaster"),
    ):
        plugin = ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{system}",
        )
        if system == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    include = world.find("include")
    aircraft = ET.parse(directory / "world/models/stallion_vtol/model.sdf").find(
        "model"
    )
    aircraft.set("name", MODEL)
    aircraft.append(include.find("pose"))
    world.remove(include)
    for plugin in aircraft.findall("plugin"):
        aircraft.remove(plugin)
    for link in aircraft.findall("link"):
        for item in list(link):
            if item.tag in {"sensor", "collision"}:
                link.remove(item)
        ET.SubElement(link, "kinematic").text = "true"
        ET.SubElement(link, "gravity").text = "false"
    world.extend((aircraft, waypoint_camera(arrivals)))
    path = directory / "waypoint-replay.sdf"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def render(directory):
    result = json.loads((directory / "result.json").read_text())
    if not result["passed"] or not result.get("arrivals"):
        raise ValueError("Expected a completed waypoint flight")
    with (directory / "world/pose.csv").open() as file:
        rows = csv.reader(file)
        next(rows)
        poses = [tuple(map(float, row)) for row in rows]
    stamps = [row[0] for row in poses]
    start, end = result["start_monotonic_s"], result["end_monotonic_s"]
    if not stamps[0] <= start < end <= stamps[-1] + 0.2:
        raise ValueError("Pose trace does not span the flight")
    with (directory / "waypoint-trace.csv").open() as file:
        phases = [
            (float(row["elapsed_s"]), row["phase"]) for row in csv.DictReader(file)
        ]
    phase_times = [t for t, _ in phases]
    encoder = shutil.which("ffmpeg")
    if encoder is None:
        from imageio_ffmpeg import get_ffmpeg_exe

        encoder = get_ffmpeg_exe()
    os.environ.update(GZ_PARTITION=f"waypoint-replay-{os.getpid()}", GZ_IP="127.0.0.1")
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(directory / "world/models"))
    env.setdefault("LP_NUM_THREADS", "3")
    node = Node()
    frames = Queue()
    lock = threading.Condition()
    observed = None

    def observe(message):
        nonlocal observed
        for pose in message.pose:
            if pose.name == MODEL:
                with lock:
                    observed = (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    )
                    lock.notify_all()
                break

    node.subscribe(Pose_V, "/world/pvtol_world/pose/info", observe)
    node.subscribe(CameraImage, "/waypoints/image", frames.put)
    shutter = node.advertise("/waypoints/shutter", Boolean)
    font = ImageFont.truetype("DejaVuSans.ttf", 20)
    temp = directory / "waypoints.tmp.mp4"
    replay = replay_world(directory, result["arrivals"])
    with (directory / "waypoint-render.log").open("w") as log:
        server = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", str(replay)],
            env=env,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 45
            while not shutter.has_connections() or not node.service_info(
                "/world/pvtol_world/set_pose"
            ):
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        "Camera startup failed; inspect waypoint-render.log"
                    )
                time.sleep(0.1)
            origin = poses[max(0, bisect_right(stamps, start) - 1)]
            size = ET.parse(CAMERA).find(".//camera/image")
            width, height = int(size.findtext("width")), int(size.findtext("height"))
            frames_total = math.ceil((end - start) * FPS)
            args = [
                encoder,
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{width}x{height}",
                "-r",
                str(FPS),
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-crf",
                "19",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(temp),
            ]
            with subprocess.Popen(args, stdin=subprocess.PIPE, stderr=log) as movie:
                for i in range(frames_total):
                    elapsed = i / FPS
                    _, n, e, d, qx, qy, qz, qw = poses[
                        bisect_right(stamps, start + elapsed) - 1
                    ]
                    target = (e, n, -d, qx, qy, qz, qw)
                    pose = Pose(name=MODEL)
                    pose.position.x, pose.position.y, pose.position.z = target[:3]
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ) = target[3:]
                    ok, reply = node.request(
                        "/world/pvtol_world/set_pose", pose, Pose, Boolean, 3000
                    )
                    with lock:
                        if (
                            not ok
                            or not reply.data
                            or not lock.wait_for(
                                lambda: (
                                    observed is not None
                                    and math.dist(observed, target) < 1e-6
                                ),
                                5,
                            )
                        ):
                            raise RuntimeError(f"Gazebo did not apply frame {i} pose")
                    shutter.publish(Boolean(data=True))
                    try:
                        image = frames.get(timeout=5)
                    except Empty as err:
                        raise RuntimeError(f"No camera image for frame {i}") from err
                    if (
                        image.width,
                        image.height,
                        image.step,
                        image.pixel_format_type,
                    ) != (width, height, width * 3, 3):
                        raise ValueError("Expected a packed RGB8 camera image")
                    rgb = Image.frombytes("RGB", (width, height), image.data)
                    draw = ImageDraw.Draw(rgb)
                    phase_index = bisect_right(phase_times, elapsed) - 1
                    phase = phases[phase_index][1] if phase_index >= 0 else "on pad"
                    text = f"{phase.capitalize()}     {elapsed:04.1f} s     N {n - origin[1]:+.2f} m     E {e - origin[2]:+.2f} m     Height {origin[3] - d:.2f} m"
                    draw.rectangle((0, height - 41, width, height), fill=(22, 32, 43))
                    draw.text((18, height - 34), text, font=font, fill="white")
                    movie.stdin.write(rgb.tobytes())
                movie.stdin.close()
                if movie.wait():
                    raise RuntimeError(
                        "Video encoding failed; inspect waypoint-render.log"
                    )
            temp.replace(directory / "waypoints.mp4")
            print(
                f"{directory / 'waypoints.mp4'} ({frames_total} frames at {FPS} fps)",
                flush=True,
            )
        finally:
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGTERM)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
            temp.unlink(missing_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="waypoint flight output directory")
    render(parser.parse_args().run.resolve())
