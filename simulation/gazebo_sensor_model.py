"""Gazebo observations and the simulated body-frame magnetometer for SIL and HIL."""

import math
import random
import threading
import time
from collections import deque

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.pose_v_pb2 import Pose_V


# 250 Hz on the serial link. Average four output periods so a one-step ground
# contact impulse is represented across several PX4 reception timestamps.
IMU_PERIOD_NS = 4_000_000
ACCEL_WINDOW_NS = 4 * IMU_PERIOD_NS


def inverse_rotate(q, vector):
    """World vector -> body for a body-to-world quaternion [w,x,y,z]."""
    w, x, y, z = q
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError("Invalid Gazebo orientation")
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


class GazeboSensors:
    def __init__(self, node, world, model, magnetic_field, seed=None):
        self.model = model
        self.field = magnetic_field
        self.rng = random.Random(seed)
        self.latest = {}
        self.pose = None
        self.pending_imu = None
        self.accel_window = deque()
        self.lock = threading.Lock()
        prefix = f"/world/{world}/model/{model}/link/base_link/sensor/"
        for kind, topic, message in (
            ("imu", prefix + "imu_sensor/imu", IMU),
            ("pressure", prefix + "air_pressure_sensor/air_pressure", FluidPressure),
            ("gps", prefix + "navsat_sensor/navsat", NavSat),
            ("clock", f"/world/{world}/clock", Clock),
            ("pose", f"/world/{world}/pose/info", Pose_V),
        ):
            if not node.subscribe(
                message,
                topic,
                lambda value, kind=kind: self.receive(kind, value, time.monotonic()),
            ):
                raise RuntimeError("Cannot subscribe to " + topic)

    def receive(self, kind, message, now):
        with self.lock:
            if kind == "pose":
                pose = next((p for p in message.pose if p.name == self.model), None)
                self.pose = (pose, now) if pose is not None else None
                return
            if kind == "imu":
                if self.pose is None or now - self.pose[1] >= 0.1:
                    self.pending_imu = None
                    self.latest.pop("imu", None)
                    self.accel_window.clear()
                    return
                imu = self.sample_imu(message)
                if imu is not None:
                    self.pending_imu = (imu, self.pose[0].orientation, now)
                return
            self.latest[kind] = (message, now)

    def sample_imu(self, message):
        stamp = message.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nsec
        previous = self.accel_window[-1][0] if self.accel_window else None
        if previous is not None and ns <= previous:
            return None
        while self.accel_window and ns - self.accel_window[0][0] >= ACCEL_WINDOW_NS:
            self.accel_window.popleft()
        a = message.linear_acceleration
        self.accel_window.append((ns, (a.x, a.y, a.z)))
        if previous is not None and ns // IMU_PERIOD_NS == previous // IMU_PERIOD_NS:
            return None
        imu = IMU()
        imu.CopyFrom(message)
        axes = zip(*(v for _, v in self.accel_window))
        accel = imu.linear_acceleration
        accel.x, accel.y, accel.z = (sum(axis) / len(axis) for axis in axes)
        return imu

    def snapshot(self):
        with self.lock:
            if self.pending_imu is not None:
                imu, q, received = self.pending_imu
                field = inverse_rotate([q.w, q.x, q.y, q.z], self.field)
                # Add the magnetometer's body-frame noise in tesla, once per sample.
                noisy = tuple(
                    v + self.rng.gauss(0, sigma)
                    for v, sigma in zip(field, (2e-6, 2e-6, 3e-6))
                )
                self.latest["imu"] = ((imu, noisy), received)
                self.pending_imu = None
            return self.latest.copy()
