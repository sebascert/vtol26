"""Translate Gazebo sensor samples and MAVLink HIL actuator packets."""

import math
import struct

from gz.msgs10.actuators_pb2 import Actuators
from pymavlink.dialects.v20 import common as mav


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nsec / 1e9


class Bridge:
    def __init__(self, connection, vehicle, publisher, temperature=15):
        self.link = connection
        self.mapping = [a["hil_channel"] - 1 for a in vehicle["actuators"]]
        self.safe = []
        for i, a in enumerate(vehicle["actuators"]):
            fraction = (a["disarmed"] - a["min"]) / (a["max"] - a["min"])
            self.safe.append(fraction if i < 3 else 2 * fraction - 1)
        self.temperature = temperature
        self.publisher = publisher
        self.system = None
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
        self.counts = {"imu": 0, "gps": 0, "controls": 0, "rejected": 0}

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

    def ready_to_forward(self, now):
        return (
            self.ready(now)
            and self.sim_time is not None
            and now - self.clock_advanced < 0.1
        )

    def tick(self, now, latest):
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
        if "clock" in latest:
            clock = latest["clock"][0].sim
            sim_time = clock.sec + clock.nsec / 1e9
            if self.sim_time is not None and sim_time < self.sim_time:
                raise RuntimeError("Gazebo time reset; restart the board and bridge")
            if sim_time != self.sim_time:
                self.sim_time, self.clock_advanced = sim_time, now
        ready = self.ready_to_forward(now)
        if ready:
            if "imu" in latest:
                (imu, field), received = latest["imu"]
                sample = stamp(imu)
                if now - received < 0.1 and sample > self.sent.get("imu", -1):
                    accel, gyro = imu.linear_acceleration, imu.angular_velocity
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
                    data = [
                        accel.x,
                        -accel.y,
                        -accel.z,
                        gyro.x,
                        -gyro.y,
                        -gyro.z,
                        field[0] * 1e4,
                        -field[1] * 1e4,
                        -field[2] * 1e4,
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


if __name__ == "__main__":
    from run_hil import main

    main()
