"""Run Gazebo and the shared HIL transport, with optional GCS and target traffic."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo_sensor_model import GazeboSensors
from gz.msgs10.actuators_pb2 import Actuators
from gz.transport13 import Node
from hil_bridge import Bridge
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

SIMULATION = Path(__file__).resolve().parent


class Scenario:
    def __init__(self, target):
        self.target = target
        self.last_target = 0
        self.pacing = None

    def tick(self, bridge, now):
        if not bridge.ready_to_forward(now):
            self.pacing = None
            return
        if self.target is not None and now - self.last_target >= 0.02:
            bridge.link.mav.set_position_target_local_ned_send(
                0,
                bridge.system,
                1,
                mav.MAV_FRAME_LOCAL_NED,
                2552,
                *self.target,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.last_target = now
        if self.pacing is None:
            self.pacing = (now, bridge.sim_time)
        elif now - self.pacing[0] >= 1:
            factor = (bridge.sim_time - self.pacing[1]) / (now - self.pacing[0])
            if not 0.8 <= factor <= 1.2:
                raise RuntimeError(
                    f"Gazebo is not keeping real time ({factor:.2f}); restart the experiment"
                )
            self.pacing = (now, bridge.sim_time)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", required=True, help="dedicated Pixhawk USB device or test transport"
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--gcs", default="127.0.0.1:14550", help="local GCS UDP destination"
    )
    parser.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("N", "E", "D"),
        help="optional constant NED position target; never arms the board",
    )
    parser.add_argument("--output-dir", type=Path, default=SIMULATION / "build/hil")
    parser.add_argument(
        "--seed", type=int, help="random seed for Gazebo and sensor noise"
    )
    args = parser.parse_args()
    if args.target and not all(
        math.isfinite(v) and abs(v) <= 10000 for v in args.target
    ):
        parser.error("Target must contain finite local NED coordinates within 10 km")
    output = args.output_dir.resolve()
    subprocess.run(
        [
            SIMULATION / "build/vtol_prepare",
            (Path.home() / ".local/share/vtol26/config/vehicle.json"),
            SIMULATION,
            output,
        ],
        check=True,
    )
    config = json.loads((output / "vehicle.json").read_text())
    world = ET.parse(output / "pvtol.sdf")
    element = world.find("world")
    # HIL waits on the ground for its real controller to initialize.
    model_path = output / "models/stallion_vtol/model.sdf"
    model = ET.parse(model_path)
    # The HIL accelerometer needs every physics step before downsampling.
    model.find("model/link/sensor[@name='imu_sensor']/update_rate").text = "0"
    model.write(model_path, encoding="utf-8", xml_declaration=True)
    radius = float(model.findtext("model/link/collision/geometry/sphere/radius"))
    pose = element.find("include/pose")
    values = pose.text.split()
    values[2] = str(radius)
    pose.text = " ".join(values)
    field = [float(v) for v in element.findtext("magnetic_field").split()]
    world.write(output / "pvtol.sdf", encoding="utf-8", xml_declaration=True)
    os.environ.setdefault("GZ_PARTITION", f"vtol-hil-{os.getpid()}")
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    env = dict(
        os.environ,
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(SIMULATION / "build"),
        GZ_SIM_RESOURCE_PATH=str(output / "models"),
    )
    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=254, source_component=51
    )
    gcs = mavutil.mavlink_connection("udpout:" + args.gcs)
    node = Node()
    model_name = element.findtext("include/name")
    sensors = GazeboSensors(node, element.attrib["name"], model_name, field, args.seed)
    publisher = node.advertise(f"/model/{model_name}/actuators_normalized", Actuators)
    bridge = Bridge(link, config, publisher)
    scenario = Scenario(args.target)
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    stats_at = 0
    with (output / "gazebo.log").open("w") as log:
        gazebo = subprocess.Popen(
            [
                "gz",
                "sim",
                "-r",
                *(["-s"] if args.headless else []),
                *(["--seed", str(args.seed)] if args.seed is not None else []),
                str(output / "pvtol.sdf"),
            ],
            env=env,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        try:
            print(
                f"HIL USB: {args.port}; waiting for profile 22001, HIL heartbeat and clock sync",
                flush=True,
            )
            while running:
                now = time.monotonic()
                for _ in range(100):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    internal_sync = message.get_type() == "TIMESYNC" and (
                        message.tc1 == 0 or message.ts1 in bridge.requests
                    )
                    bridge.receive(message, now)
                    if (
                        message.get_type() not in ("BAD_DATA", "HIL_ACTUATOR_CONTROLS")
                        and not internal_sync
                    ):
                        gcs.write(message.get_msgbuf())
                for _ in range(100):
                    message = gcs.recv_match(blocking=False)
                    if message is None:
                        break
                    if (
                        message.get_type() != "BAD_DATA"
                        and not message.get_type().startswith("HIL_")
                    ):
                        link.write(message.get_msgbuf())
                bridge.tick(now, sensors.snapshot())
                scenario.tick(bridge, now)
                if now - stats_at >= 2:
                    result = dict(
                        bridge.counts,
                        ready=bridge.ready_to_forward(now),
                    )
                    pending = output / "status.json.tmp"
                    pending.write_text(json.dumps(result, indent=2) + "\n")
                    pending.replace(output / "status.json")
                    print(json.dumps(result), flush=True)
                    stats_at = now
                if gazebo.poll() is not None:
                    raise RuntimeError(
                        "Gazebo exited; inspect " + str(output / "gazebo.log")
                    )
                time.sleep(0.002)
        finally:
            link.close()
            gcs.close()
            if gazebo.poll() is None:
                os.killpg(gazebo.pid, signal.SIGTERM)
                try:
                    gazebo.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(gazebo.pid, signal.SIGKILL)
                    gazebo.wait()


if __name__ == "__main__":
    main()
