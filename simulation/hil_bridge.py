"""Gazebo Harmonic <-> PX4 external HIL over a dedicated MAVLink connection."""

import argparse
import json
import math
import os
import random
import signal
import struct
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

SIMULATION = Path(__file__).resolve().parent


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nsec / 1e9


def inverse_rotate(q, vector):
    """World vector -> body for a body-to-world quaternion [w,x,y,z]."""
    w, x, y, z = q
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError("Invalid IMU orientation")
    # The transpose of the unit-quaternion rotation matrix.
    return [
        (1 - 2 * (y * y + z * z)) * vector[0]
        + 2 * (x * y + w * z) * vector[1]
        + 2 * (x * z - w * y) * vector[2],
        2 * (x * y - w * z) * vector[0]
        + (1 - 2 * (x * x + z * z)) * vector[1]
        + 2 * (y * z + w * x) * vector[2],
        2 * (x * z + w * y) * vector[0]
        + 2 * (y * z - w * x) * vector[1]
        + (1 - 2 * (x * x + y * y)) * vector[2],
    ]


class Bridge:
    def __init__(
        self,
        connection,
        vehicle,
        world,
        model,
        magnetic_field,
        temperature,
        target=None,
    ):
        self.link = connection
        self.mapping = [a["hil_channel"] - 1 for a in vehicle["actuators"]]
        self.safe = []
        for i, a in enumerate(vehicle["actuators"]):
            fraction = (a["disarmed"] - a["min"]) / (a["max"] - a["min"])
            self.safe.append(fraction if i < 3 else 2 * fraction - 1)
        self.field = magnetic_field
        self.temperature = temperature
        self.target = target
        self.latest = {}
        self.lock = threading.Lock()
        self.node = Node()
        prefix = f"/world/{world}/model/{model}/link/base_link/sensor/"
        for kind, topic, message in (
            ("imu", prefix + "imu_sensor/imu", IMU),
            ("pressure", prefix + "air_pressure_sensor/air_pressure", FluidPressure),
            ("gps", prefix + "navsat_sensor/navsat", NavSat),
            ("clock", f"/world/{world}/clock", Clock),
            ("pose", f"/world/{world}/pose/info", Pose_V),
        ):
            if not self.node.subscribe(
                message,
                topic,
                lambda value, kind=kind: self.receive_sensor(kind, value),
            ):
                raise RuntimeError("Cannot subscribe to " + topic)
        self.publisher = self.node.advertise(
            f"/model/{model}/actuators_normalized", Actuators
        )
        self.system = None
        self.model = model
        self.heartbeat = 0
        self.parameters = {}
        self.offset = None
        self.sync_received = 0
        self.requests = set()
        self.last_control_stamp = 0
        self.control_received = 0
        self.control = list(self.safe)
        self.sent = {}
        self.timers = {}
        self.sim_time = None
        self.clock_advanced = 0
        self.pacing = None
        self.counts = {"imu": 0, "gps": 0, "controls": 0, "rejected": 0}

    def receive_sensor(self, kind, message):
        with self.lock:
            self.latest[kind] = (message, time.monotonic())

    def receive(self, message, now):
        kind = message.get_type()
        if (
            kind == "HEARTBEAT"
            and message.get_srcComponent() == 1
            and message.autopilot == mav.MAV_AUTOPILOT_PX4
        ):
            if self.system is None:
                self.system = message.get_srcSystem()
            if message.get_srcSystem() == self.system:
                self.heartbeat = (
                    now if message.base_mode & mav.MAV_MODE_FLAG_HIL_ENABLED else 0
                )
        if message.get_srcSystem() != self.system or message.get_srcComponent() != 1:
            return
        if kind == "PARAM_VALUE":
            key = message.param_id.rstrip("\0")
            if key in ("SYS_AUTOSTART", "SYS_HITL"):
                expected = 22001 if key == "SYS_AUTOSTART" else 1
                value = struct.unpack("<i", struct.pack("<f", message.param_value))[0]
                if message.param_type != mav.MAV_PARAM_TYPE_INT32 or value != expected:
                    raise RuntimeError(
                        f"Expected {key}={expected}; configure the HIL profile and reboot"
                    )
                self.parameters[key] = expected
        elif kind == "TIMESYNC":
            if message.tc1 == 0:
                self.link.mav.timesync_send(int(now * 1e9), message.ts1)
            elif message.ts1 in self.requests:
                self.requests.remove(message.ts1)
                rtt = now - message.ts1 / 1e9
                if 0 <= rtt < 0.05:
                    self.offset = (now + message.ts1 / 1e9) / 2 - message.tc1 / 1e9
                    self.sync_received = now
        elif kind == "HIL_ACTUATOR_CONTROLS":
            age = (
                None
                if self.offset is None
                else now - (message.time_usec / 1e6 + self.offset)
            )
            values = [message.controls[i] for i in self.mapping]
            valid = (
                self.ready(now)
                and age is not None
                and -0.03 <= age < 0.1
                and message.time_usec > self.last_control_stamp
                and message.mode & mav.MAV_MODE_FLAG_HIL_ENABLED
                and all(
                    math.isfinite(v) and (0 if i < 3 else -1) <= v <= 1
                    for i, v in enumerate(values)
                )
            )
            if not valid:
                self.control = list(self.safe)
                self.counts["rejected"] += 1
                return
            self.last_control_stamp = message.time_usec
            self.control_received = now
            self.control = (
                values
                if message.mode & mav.MAV_MODE_FLAG_SAFETY_ARMED
                else list(self.safe)
            )
            self.counts["controls"] += 1

    def ready(self, now):
        return (
            self.heartbeat > 0
            and now - self.heartbeat < 3
            and len(self.parameters) == 2
            and self.offset is not None
            and now - self.sync_received < 1
        )

    def tick(self, now):
        if now - self.timers.get("heartbeat", 0) >= 1:
            self.link.mav.heartbeat_send(
                mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
            self.timers["heartbeat"] = now
            if self.system is not None:
                for name in ("SYS_AUTOSTART", "SYS_HITL"):
                    self.link.mav.param_request_read_send(
                        self.system, 1, name.encode(), -1
                    )
        if self.system is not None and now - self.timers.get("sync", 0) >= 0.25:
            request = int(now * 1e9)
            self.requests = {s for s in self.requests if now - s / 1e9 < 1}
            self.requests.add(request)
            self.link.mav.timesync_send(0, request)
            self.timers["sync"] = now
        with self.lock:
            latest = dict(self.latest)
        if "clock" in latest:
            clock = latest["clock"][0].sim
            sim_time = clock.sec + clock.nsec / 1e9
            if self.sim_time is not None and sim_time < self.sim_time:
                raise RuntimeError("Gazebo time reset; restart the board and bridge")
            if sim_time != self.sim_time:
                self.sim_time, self.clock_advanced = sim_time, now
        clock_ok = self.sim_time is not None and now - self.clock_advanced < 0.1
        ready = self.ready(now) and clock_ok
        if ready:
            if self.target is not None and now - self.timers.get("target", 0) >= 0.02:
                self.link.mav.set_position_target_local_ned_send(
                    0,
                    self.system,
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
                self.timers["target"] = now
            if self.pacing is None:
                self.pacing = (now, self.sim_time)
            elif now - self.pacing[0] >= 1:
                factor = (self.sim_time - self.pacing[1]) / (now - self.pacing[0])
                if not 0.8 <= factor <= 1.2:
                    raise RuntimeError(
                        f"Gazebo is not keeping real time ({factor:.2f}); restart the experiment"
                    )
                self.pacing = (now, self.sim_time)
            if "imu" in latest and "pose" in latest and now - latest["pose"][1] < 0.1:
                imu, received = latest["imu"]
                sample = stamp(imu)
                if now - received < 0.1 and sample > self.sent.get("imu", -1):
                    accel, gyro = imu.linear_acceleration, imu.angular_velocity
                    pose = next(
                        (p for p in latest["pose"][0].pose if p.name == self.model),
                        None,
                    )
                    if pose is None:
                        raise ValueError("Aircraft pose missing from world")
                    q = pose.orientation
                    field = inverse_rotate([q.w, q.x, q.y, q.z], self.field)
                    fields = 0x3F | 0x1C0
                    pressure = latest.get("pressure")
                    baro = pressure[0].pressure / 100 if pressure else 0
                    if (
                        pressure
                        and now - pressure[1] < 0.1
                        and stamp(pressure[0]) > self.sent.get("pressure", -1)
                    ):
                        fields |= 0x1A00
                        self.sent["pressure"] = stamp(pressure[0])
                    altitude = (
                        44330 * (1 - (baro / 1013.25) ** 0.190295) if baro > 0 else 0
                    )
                    # Match PX4 sensor_mag_sim noise so stationary samples aren't marked stuck.
                    data = [
                        accel.x,
                        -accel.y,
                        -accel.z,
                        gyro.x,
                        -gyro.y,
                        -gyro.z,
                        field[0] * 1e4 + random.gauss(0, 0.02),
                        -field[1] * 1e4 + random.gauss(0, 0.02),
                        -field[2] * 1e4 + random.gauss(0, 0.03),
                        baro,
                        0,
                        altitude,
                        self.temperature,
                    ]
                    if not all(math.isfinite(v) for v in data):
                        raise ValueError("Nonfinite simulated sensor")
                    self.link.mav.hil_sensor_send(int(sample * 1e6), *data, fields)
                    self.sent["imu"] = sample
                    self.counts["imu"] += 1
            if "gps" in latest:
                gps, received = latest["gps"]
                sample = stamp(gps)
                if now - received < 0.2 and sample - self.sent.get("gps", -1) >= 0.1:
                    vn, ve, vd = gps.velocity_north, gps.velocity_east, -gps.velocity_up
                    speed = math.hypot(vn, ve, vd)
                    values = [
                        gps.latitude_deg,
                        gps.longitude_deg,
                        gps.altitude,
                        vn,
                        ve,
                        vd,
                    ]
                    if (
                        not all(math.isfinite(v) for v in values)
                        or speed > 300
                        or abs(gps.latitude_deg) > 90
                        or abs(gps.longitude_deg) > 180
                        or abs(gps.altitude) > 100000
                    ):
                        raise ValueError("Invalid simulated GPS")
                    course = round(math.degrees(math.atan2(ve, vn)) % 360 * 100) % 36000
                    self.link.mav.hil_gps_send(
                        int(sample * 1e6),
                        3,
                        round(gps.latitude_deg * 1e7),
                        round(gps.longitude_deg * 1e7),
                        round(gps.altitude * 1000),
                        100,
                        100,
                        round(speed * 100),
                        round(vn * 100),
                        round(ve * 100),
                        round(vd * 100),
                        course,
                        12,
                    )
                    self.sent["gps"] = sample
                    self.counts["gps"] += 1
        else:
            self.pacing = None
        if self.sim_time is not None and now - self.timers.get("outputs", 0) >= 0.01:
            controls = (
                self.control
                if ready and now - self.control_received < 0.1
                else self.safe
            )
            packet = Actuators()
            ns = round(self.sim_time * 1e9)
            packet.header.stamp.sec, packet.header.stamp.nsec = divmod(ns, 1000000000)
            packet.normalized.extend(controls)
            self.publisher.publish(packet)
            self.timers["outputs"] = now


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
    args = parser.parse_args()
    if args.target and not all(
        math.isfinite(v) and abs(v) <= 10000 for v in args.target
    ):
        parser.error("Target must contain finite local NED coordinates within 10 km")
    output = SIMULATION / "build/hil"
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
    model = ET.parse(output / "models/stallion_vtol/model.sdf")
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
    bridge = Bridge(
        link,
        config,
        element.attrib["name"],
        element.findtext("include/name"),
        field,
        15,
        args.target,
    )
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
                bridge.tick(now)
                if now - stats_at >= 2:
                    result = dict(
                        bridge.counts,
                        ready=bool(
                            bridge.ready(now)
                            and bridge.sim_time is not None
                            and now - bridge.clock_advanced < 0.1
                        ),
                    )
                    (output / "status.json").write_text(
                        json.dumps(result, indent=2) + "\n"
                    )
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
