"""HIL wire conversions and gates without a physical board or Gazebo server."""

import json
import math
import struct
import sys
import unittest
from itertools import pairwise
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gazebo_sensor_model as sensors
from gz.msgs10.pose_v_pb2 import Pose_V
from hil_bridge import Bridge
from pymavlink.dialects.v20 import common as mav
from run_hil import SIMULATION, Scenario


class FakePublisher:
    def publish(self, message):
        self.last = message


class FakeNode:
    def subscribe(self, *args):
        return True


class FakeLink:
    def __init__(self):
        self.mav = self
        self.sent = []

    def __getattr__(self, name):
        return lambda *args: self.sent.append((name, args))


class HilTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = json.loads(
            (SIMULATION.parent / "config/vehicle.json").read_text()
        )
        self.link = FakeLink()
        node = FakeNode()
        self.inputs = sensors.GazeboSensors(
            node, "world", "aircraft", [1e-6, 2e-5, -4e-5]
        )
        self.publisher = FakePublisher()
        self.bridge = Bridge(self.link, self.vehicle, self.publisher)
        self.scenario = Scenario(None)
        self.encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.decoder = mav.MAVLink(None)

    def tick(self, now):
        self.bridge.tick(now, self.inputs.snapshot())
        self.scenario.tick(self.bridge, now)

    def receive(self, message, now=100):
        self.bridge.receive(self.decoder.parse_char(message.pack(self.encoder)), now)

    def connect(self):
        self.receive(
            mav.MAVLink_heartbeat_message(
                mav.MAV_TYPE_QUADROTOR,
                mav.MAV_AUTOPILOT_PX4,
                mav.MAV_MODE_FLAG_HIL_ENABLED,
                0,
                mav.MAV_STATE_STANDBY,
                3,
            )
        )
        for key, value in [("SYS_AUTOSTART", 22001), ("SYS_HITL", 1)]:
            encoded = struct.unpack("<f", struct.pack("<i", value))[0]
            self.receive(
                mav.MAVLink_param_value_message(
                    key.encode(), encoded, mav.MAV_PARAM_TYPE_INT32, 2, 0
                )
            )
        self.bridge.requests.add(99998000000)
        self.receive(mav.MAVLink_timesync_message(2000000000, 99998000000))
        self.assertTrue(self.bridge.ready(100))

    def sensors(self):
        imu = sensors.IMU()
        imu.header.stamp.sec = 10
        imu.linear_acceleration.z = 9.81
        imu.angular_velocity.x = 1
        imu.angular_velocity.y = 2
        imu.angular_velocity.z = 3
        pressure = sensors.FluidPressure()
        pressure.header.stamp.sec = 10
        pressure.pressure = 101325
        gps = sensors.NavSat()
        gps.header.stamp.sec = 10
        gps.latitude_deg = 47
        gps.longitude_deg = 8
        gps.altitude = 100
        gps.velocity_north = 1
        gps.velocity_east = 2
        gps.velocity_up = 3
        pose = Pose_V()
        p = pose.pose.add()
        p.name = "aircraft"
        p.orientation.w = p.orientation.z = math.sqrt(0.5)
        clock = sensors.Clock()
        clock.sim.sec = 10
        self.messages = {
            "pose": pose,
            "pressure": pressure,
            "gps": gps,
            "clock": clock,
            "imu": imu,
        }
        for kind, message in self.messages.items():
            self.inputs.receive(kind, message, 100)

    def test_sensor_frames_units_flags_and_no_repetition(self):
        self.connect()
        with patch.object(self.inputs.rng, "gauss", return_value=0):
            self.sensors()
            self.tick(100)
        imu = next(a for n, a in self.link.sent if n == "hil_sensor_send")
        self.assertAlmostEqual(imu[3], -9.81)
        self.assertEqual(imu[4:7], (1, -2, -3))
        for actual, expected in zip(imu[7:10], [0.2, 0.01, 0.4]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(imu[10], 1013.25)
        self.assertEqual(imu[-1], 0x1BFF)
        gps = next(a for n, a in self.link.sent if n == "hil_gps_send")
        self.assertEqual(gps[2:5], (470000000, 80000000, 100000))
        self.assertEqual(gps[8:11], (100, 200, -300))
        self.tick(100.01)
        self.assertEqual(sum(n == "hil_sensor_send" for n, a in self.link.sent), 1)

    def test_touchdown_impulse_survives_250_hz_sampling(self):
        # A 2.32 m/s stop in one 1 ms physics step, at each phase of the HIL period.
        self.connect()
        self.sensors()
        imu = self.messages["imu"]
        clock = self.messages["clock"]
        for phase in range(4):
            with self.subTest(phase=phase):
                self.link.sent.clear()
                for step in range(80):
                    ms = 80 * phase + step
                    now = 100 + ms / 1000
                    imu.header.stamp.nsec = clock.sim.nsec = ms * 1_000_000
                    imu.linear_acceleration.z = 9.81 + (
                        2320 if step == 32 + phase else 0
                    )
                    self.inputs.receive("pose", self.messages["pose"], now)
                    self.inputs.receive("imu", imu, now)
                    self.inputs.receive("clock", clock, now)
                    if step % 4 == 0:
                        self.tick(now)
                packets = [
                    args for name, args in self.link.sent if name == "hil_sensor_send"
                ]
                self.assertEqual(len(packets), 20)
                delta_v = sum((-packet[3] - 9.81) * 0.004 for packet in packets)
                self.assertAlmostEqual(delta_v, 2.32)
                self.assertLessEqual(max(-p[3] - 9.81 for p in packets), 150)

    def test_stationary_magnetometer_remains_live_on_wire(self):
        self.connect()
        self.sensors()
        self.inputs.rng.seed(0)
        for sample in range(1200):
            now = 100 + sample / 250
            self.bridge.heartbeat = self.bridge.sync_received = now
            for key, message in self.messages.items():
                stamp = message.sim if key == "clock" else message.header.stamp
                stamp.sec = 10 + sample // 250
                stamp.nsec = (sample % 250) * 4000000
                self.inputs.receive(key, message, now)
            self.tick(now)
        samples = [
            self.decoder.parse_char(
                mav.MAVLink_hil_sensor_message(*args).pack(self.encoder)
            )
            for name, args in self.link.sent
            if name == "hil_sensor_send"
        ]
        self.assertEqual(len(samples), 1200)
        vectors = [(s.xmag, s.ymag, s.zmag) for s in samples]
        for previous, current in pairwise(vectors):
            self.assertGreater(max(abs(a - b) for a, b in zip(previous, current)), 1e-6)
        for axis, expected, sigma in zip(
            zip(*vectors), (0.2, 0.01, 0.4), (0.02, 0.02, 0.03)
        ):
            mean = sum(axis) / len(axis)
            self.assertAlmostEqual(mean, expected, delta=0.003)
            rms = math.sqrt(sum((v - mean) ** 2 for v in axis) / len(axis))
            self.assertGreater(rms, 0.7 * sigma)
            self.assertLess(rms, 1.3 * sigma)

    def test_imu_requires_a_recent_vehicle_pose(self):
        self.sensors()
        self.assertIn("imu", self.inputs.snapshot())
        self.inputs.receive("imu", self.messages["imu"], 100.2)
        self.assertNotIn("imu", self.inputs.snapshot())
        self.messages["pose"].pose[0].orientation.Clear()
        self.inputs.receive("pose", self.messages["pose"], 100.21)
        self.inputs.receive("imu", self.messages["imu"], 100.21)
        with self.assertRaisesRegex(ValueError, "Gazebo orientation"):
            self.inputs.snapshot()

    def test_profile_and_hil_required(self):
        self.sensors()
        self.tick(100)
        self.assertFalse(any(n == "hil_sensor_send" for n, a in self.link.sent))
        self.receive(
            mav.MAVLink_heartbeat_message(2, mav.MAV_AUTOPILOT_PX4, 32, 0, 3, 3)
        )
        with self.assertRaises(RuntimeError):
            self.receive(
                mav.MAVLink_param_value_message(
                    b"SYS_AUTOSTART", 1001, mav.MAV_PARAM_TYPE_INT32, 2, 0
                )
            )

    def controls(self, stamp=2001000, armed=True, values=None):
        return mav.MAVLink_hil_actuator_controls_message(
            stamp,
            values or [0.5, 0.6, 0.7, -1, -0.8, 0, 0.1, 0.2, 0.3] + [0] * 7,
            32 + (128 if armed else 0),
            0,
        )

    def test_controls_mapping_age_replay_bounds_and_disarm(self):
        self.connect()
        self.sensors()
        self.receive(self.controls())
        self.tick(100)
        self.assertAlmostEqual(self.publisher.last.normalized[0], 0.5)
        self.receive(self.controls())
        self.assertEqual(self.bridge.control, self.bridge.safe)
        self.receive(self.controls(stamp=2002000, armed=False))
        self.assertEqual(self.bridge.control, self.bridge.safe)
        self.receive(self.controls(stamp=1500000))
        self.assertEqual(self.bridge.control, self.bridge.safe)
        self.receive(self.controls(stamp=3000000))
        self.assertEqual(self.bridge.control, self.bridge.safe)
        bad = [math.nan] + [0] * 15
        self.receive(self.controls(stamp=2003000, values=bad))
        self.assertEqual(self.bridge.control, self.bridge.safe)
        self.receive(self.controls(stamp=2004000))
        self.tick(100.2)
        self.assertEqual(list(self.publisher.last.normalized), self.bridge.safe)

    def test_mapping_uses_configured_slots(self):
        self.bridge.mapping = [15, 0, 8, 1, 9, 2, 10, 3, 11]
        self.connect()
        self.sensors()
        values = [0] * 16
        for i, slot in enumerate(self.bridge.mapping):
            values[slot] = 0.1 * i
        self.receive(self.controls(values=values))
        for i, value in enumerate(self.bridge.control):
            self.assertAlmostEqual(value, 0.1 * i)

    def test_pause_reset_and_clock_sync_loss(self):
        self.connect()
        self.sensors()
        self.tick(100)
        self.messages["clock"].sim.sec = 9
        with self.assertRaises(RuntimeError):
            self.tick(100.01)
        self.assertFalse(self.bridge.ready(102))

    def test_optional_target_never_sends_arming_command(self):
        self.connect()
        self.sensors()
        self.scenario.target = [1, 2, -0.3]
        self.tick(100)
        target = next(
            a for n, a in self.link.sent if n == "set_position_target_local_ned_send"
        )
        self.assertEqual(target[5:8], (1, 2, -0.3))
        self.assertFalse(any(n == "command_long_send" for n, a in self.link.sent))

    def test_substantial_slowdown_stops_experiment(self):
        self.connect()
        self.sensors()
        self.tick(100)
        self.bridge.heartbeat = self.bridge.sync_received = 101.1
        self.messages["clock"].sim.nsec = 200000000
        with self.assertRaisesRegex(RuntimeError, "not keeping real time"):
            self.tick(101.1)


if __name__ == "__main__":
    unittest.main()
