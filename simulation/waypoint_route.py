"""Host-side Stallion waypoint targets for the Pixhawk HIL and desktop SIL."""

import math

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

PAD = (0.0, 0.0, 0.0)
ROUTE = (
    ("hover", (0.0, 0.0, -0.30)),
    ("north", (2.0, 0.0, -0.30)),
    ("east", (2.0, 2.0, -0.30)),
    ("return", (0.0, 0.0, -0.30)),
    ("touchdown", PAD),
)


class WaypointRoute:
    def __init__(self):
        self.phase = "waiting"
        self.local = self.mode = self.landed = None
        self.origin = self.truth_origin = None
        self.offset = PAD
        self.arrivals = []
        self.started = self.ended = self.begin = self.last_ready = None
        self.last_pose = self.held_since = None
        self.worst = 0.0
        self.error = None

    def receive(self, message, now):
        kind = message.get_type()
        if kind == "HEARTBEAT" and message.autopilot == mav.MAV_AUTOPILOT_PX4:
            armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            main_mode = (message.custom_mode >> 16) & 0xFF
            self.mode = (armed, main_mode == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        elif kind == "LOCAL_POSITION_NED":
            position = (message.x, message.y, message.z)
            self.local = (now, position) if all(map(math.isfinite, position)) else None
        elif kind == "EXTENDED_SYS_STATE":
            self.landed = (now, message.landed_state == mav.MAV_LANDED_STATE_ON_GROUND)

    def abort(self, reason):
        self.phase = "aborted"
        self.error = reason
        return None

    def update(self, now, ready, observation):
        if self.phase in ("disarmed", "aborted"):
            return None
        if not ready:
            if self.started is not None and now - self.last_ready > 1:
                return self.abort("Lost HIL readiness during the route")
            return None
        self.last_ready = now
        if self.mode is None:
            return None
        armed, offboard = self.mode
        fresh_pose = observation is not None and now - observation[1] < 0.2
        if self.started is None:
            if not armed:
                if self.local is None or now - self.local[0] >= 1 or not fresh_pose:
                    self.phase = "waiting"
                    self.origin = None
                    return None
                self.origin = self.local[1]
                self.truth_origin = self.ned(observation)
                self.phase = "on pad"
                return self.origin, 0.0
            if (
                not offboard
                or self.origin is None
                or self.local is None
                or now - self.local[0] >= 1
                or not fresh_pose
            ):
                return self.abort(
                    "Arm from the pad in Offboard with fresh PX4 and Gazebo positions"
                )
            self.started = self.begin = now
            self.phase = ROUTE[0][0]

        if (
            self.phase == "landing"
            and self.landed
            and self.landed[0] >= self.begin
            and self.landed[1]
        ):
            self.phase = "landed"
        if not armed:
            if self.phase != "landed":
                return self.abort("PX4 disarmed before confirming landing")
            self.phase = "disarmed"
            self.ended = now
            return None
        if not offboard:
            return self.abort("PX4 left Offboard during the route")
        if self.phase == "landing":
            elapsed = now - self.begin
            if elapsed > 18:
                return self.abort("Timed out waiting for PX4 landing confirmation")
            self.offset = (0.0, 0.0, min(1.5, math.floor(elapsed / 0.5) * 0.09))
        elif self.phase != "landed":
            self.update_leg(now, observation if fresh_pose else None)
            if self.error:
                return None
        target = tuple(a + b for a, b in zip(self.origin, self.offset))
        return target, 0.5 if self.phase in ("touchdown", "landing", "landed") else 0.0

    @staticmethod
    def ned(observation):
        p = observation[0].position
        return (p.y, p.x, -p.z)

    def measured(self, observation):
        return tuple(a - b for a, b in zip(self.ned(observation), self.truth_origin))

    def update_leg(self, now, observation):
        index = len(self.arrivals)
        destination = ROUTE[index][1]
        previous = ROUTE[index - 1][1] if index else PAD
        speed = 0.10 if self.phase == "touchdown" else 0.16
        duration = math.dist(previous, destination) / speed
        elapsed = now - self.begin
        if elapsed > duration + 20:
            self.abort(f"Gazebo did not hold waypoint {self.phase} within 0.055 m")
            return
        # Keep each reference stable while the companion computes its allocation.
        fraction = min(1.0, math.floor(elapsed / 0.5) * 0.5 / duration)
        self.offset = tuple(
            a + fraction * (b - a) for a, b in zip(previous, destination)
        )
        if observation is None or fraction < 1:
            self.held_since = None
            return
        stamp = observation[1]
        if stamp == self.last_pose:
            return
        measured = self.measured(observation)
        error = math.dist(measured, destination)
        if self.last_pose is None or stamp - self.last_pose > 0.20:
            self.held_since = None
        self.last_pose = stamp
        if not math.isfinite(error) or error > 0.055:
            self.held_since = None
            return
        if self.held_since is None:
            self.held_since = stamp
            self.worst = 0.0
        self.worst = max(self.worst, error)
        if stamp - self.held_since >= 1.0:
            self.arrivals.append(
                dict(
                    phase=self.phase,
                    target_ned_m=destination,
                    gazebo_ned_m=measured,
                    max_hold_error_m=self.worst,
                    hold_s=stamp - self.held_since,
                    arrival_s=stamp - self.started,
                )
            )
            self.phase = ROUTE[index + 1][0] if index + 1 < len(ROUTE) else "landing"
            self.begin = now
            self.held_since = None

    def status(self):
        return dict(
            phase=self.phase,
            passed=self.phase == "disarmed",
            landed=self.phase in ("landed", "disarmed"),
            disarmed=self.phase == "disarmed",
            arrivals=self.arrivals,
            start_monotonic_s=self.started,
            end_monotonic_s=self.ended,
            error=self.error,
        )
