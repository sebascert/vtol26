"""Real Gazebo + serial PTY, with a simulated PX4 MAVLink endpoint (no board)."""

import json
import math
import os
import pty
import select
import signal
import struct
import subprocess
import time
import tty
from pathlib import Path

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "build/hil-transport-test"


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = f"vtol-hil-transport-{os.getpid()}"
    os.environ["GZ_IP"] = "127.0.0.1"
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)

    class Writer:
        def write(self, data):
            return os.write(master, data)

    board = mav.MAVLink(Writer(), srcSystem=1, srcComponent=1)
    parser = mav.MAVLink(None)
    gcs = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14550", source_system=255, source_component=190
    )
    gcs_seen = {}
    node = Node()
    observed = {}
    counts = {}
    armed = False
    send_controls = True

    def pose_callback(message):
        for pose in message.pose:
            if pose.name == "controlled_drone":
                observed["z"] = pose.position.z

    def outputs_callback(message):
        observed["outputs"] = list(message.normalized)

    assert node.subscribe(Pose_V, "/world/pvtol_world/pose/info", pose_callback)
    assert node.subscribe(
        Actuators, "/model/controlled_drone/actuators_normalized", outputs_callback
    )
    timers = {}
    process = None
    result = {
        "passed": False,
        "scope": "MAVLink serial PTY and real Gazebo; simulated board, no firmware execution",
    }
    with (OUTPUT / "bridge.log").open("w") as log:
        try:
            process = subprocess.Popen(
                [
                    ROOT / "run-hil.sh",
                    "--port",
                    os.ttyname(slave),
                    "--headless",
                    "--target",
                    "0",
                    "0",
                    "-0.3",
                ],
                stdout=log,
                stderr=log,
                start_new_session=True,
            )

            def pump(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    now = time.monotonic()
                    board_time = int((now + 100) * 1e6)
                    if now - timers.get("hb", 0) > 0.2:
                        board.heartbeat_send(
                            2, mav.MAV_AUTOPILOT_PX4, 32 + (128 if armed else 0), 0, 4
                        )
                        timers["hb"] = now
                    if send_controls and now - timers.get("controls", 0) > 0.01:
                        board.hil_actuator_controls_send(
                            board_time,
                            [0.6, 0.6, 1, -1, -1, 0, 0, 0, 0] + [0] * 7,
                            32 + (128 if armed else 0),
                            0,
                        )
                        timers["controls"] = now
                    if select.select([master], [], [], 0.002)[0]:
                        for byte in os.read(master, 65536):
                            message = parser.parse_char(bytes([byte]))
                            if message is None:
                                continue
                            kind = message.get_type()
                            counts[kind] = counts.get(kind, 0) + 1
                            if kind == "TIMESYNC" and message.tc1 == 0:
                                board.timesync_send(
                                    int((time.monotonic() + 100) * 1e9), message.ts1
                                )
                            elif kind == "PARAM_REQUEST_READ":
                                if message.get_srcSystem() == 255:
                                    counts["GCS_REQUESTS"] = (
                                        counts.get("GCS_REQUESTS", 0) + 1
                                    )
                                value = {"SYS_AUTOSTART": 22001, "SYS_HITL": 1}[
                                    message.param_id
                                ]
                                encoded = struct.unpack("<f", struct.pack("<i", value))[
                                    0
                                ]
                                board.param_value_send(
                                    message.param_id.encode(),
                                    encoded,
                                    mav.MAV_PARAM_TYPE_INT32,
                                    2,
                                    0,
                                )
                    while (message := gcs.recv_match(blocking=False)) is not None:
                        gcs_seen[message.get_type()] = message
                    if process.poll() is not None:
                        raise RuntimeError("Bridge exited; inspect bridge.log")

            deadline = time.monotonic() + 25
            while (
                counts.get("HIL_SENSOR", 0) < 300
                or counts.get("HIL_GPS", 0) < 10
                or "z" not in observed
            ):
                pump(0.1)
                if time.monotonic() > deadline:
                    raise TimeoutError("HIL sensors/clock/profile handshake")
            assert observed["outputs"][:3] == [0, 0, 0], "Disarmed output gate"
            assert counts.get("SET_POSITION_TARGET_LOCAL_NED", 0) > 0, (
                "Optional target stream"
            )
            assert "HEARTBEAT" in gcs_seen, "GCS forwarding"
            gcs.mav.param_request_read_send(1, 1, b"SYS_HITL", -1)
            gcs.mav.timesync_send(0, time.monotonic_ns() + 123)
            pump(0.2)
            assert (
                counts.get("GCS_REQUESTS", 0) > 0
                and "PARAM_VALUE" in gcs_seen
                and "TIMESYNC" in gcs_seen
            ), "GCS return path"
            initial = observed["z"]
            armed = True
            pump(1)
            rise = observed["z"] - initial
            assert rise > 0.1, ("Actuator response", rise)
            assert all(
                math.isclose(a, b, abs_tol=1e-6)
                for a, b in zip(observed["outputs"][:3], [0.6, 0.6, 1])
            )
            send_controls = False
            pump(0.25)
            assert observed["outputs"][:5] == [0, 0, 0, -1, -1], "Command loss gate"
            # An arriving packet with old board time must not regain authority.
            board.hil_actuator_controls_send(
                int((time.monotonic() + 99) * 1e6), [1] * 16, 160, 0
            )
            pump(0.05)
            assert observed["outputs"][:3] == [0, 0, 0], "Delayed command gate"
            result.update(
                passed=True,
                sensor_packets=counts["HIL_SENSOR"],
                gps_packets=counts["HIL_GPS"],
                actual_rise_m=rise,
                checks=[
                    "serial round trip",
                    "HIL profile bytewise parameters",
                    "clock synchronization",
                    "sensors",
                    "disarmed controls",
                    "physical motor response",
                    "command loss",
                    "delayed command",
                    "GCS bidirectional forwarding",
                    "optional target stream",
                ],
            )
            print(json.dumps(result, indent=2))
        except BaseException as error:
            result["error"] = repr(error)
            raise
        finally:
            (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            os.close(master)
            os.close(slave)
            gcs.close()


if __name__ == "__main__":
    main()
