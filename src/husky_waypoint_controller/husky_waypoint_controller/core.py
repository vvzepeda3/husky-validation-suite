"""Pure-Python core of the Husky waypoint controller.

Everything that decides what the robot does lives here: waypoint validation, the
proportional heading control law, velocity clamping, the watchdog and the state
machine. This module must not import ``rclpy`` or any ROS message package
(REQ-ARCH-001), so it runs under plain pytest (REQ-ARCH-004).

Time only enters through the ``Watchdog``'s injected clock and the fixed control
period, so identical inputs always give identical outputs (REQ-ARCH-002).
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, fields
from enum import StrEnum

_LOG = logging.getLogger(__name__)

# A200 diff-drive controller caps (docs/requirements.md, fact A5).
PLATFORM_MAX_LINEAR_SPEED_MPS = 1.0
PLATFORM_MAX_ANGULAR_SPEED_RADPS = 1.0
# Clearpath twist_mux / diff-drive command timeout (fact A4).
PLATFORM_CMD_TIMEOUT_S = 0.5


class State(StrEnum):
    """Controller states (docs/design.md §4)."""

    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"


class Event(StrEnum):
    """Inputs that can change the controller state."""

    GOAL = "GOAL"
    GOAL_COMPLETE = "GOAL_COMPLETE"
    STOP = "STOP"
    ESTOP = "ESTOP"
    RESUME = "RESUME"
    RESET = "RESET"
    WATCHDOG_TRIP = "WATCHDOG_TRIP"
    INVALID_COMMAND = "INVALID_COMMAND"


class Reason(StrEnum):
    """Why a transition happened or a request was refused."""

    STARTUP = "STARTUP"
    GOAL_ACCEPTED = "GOAL_ACCEPTED"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    STOP_REQUESTED = "STOP_REQUESTED"
    ESTOP = "ESTOP"
    RESUMED = "RESUMED"
    RESET = "RESET"
    ODOM_STALE = "ODOM_STALE"
    NON_FINITE_COMMAND = "NON_FINITE_COMMAND"
    ALREADY_STOPPED = "ALREADY_STOPPED"
    # Request rejections
    EMPTY = "EMPTY"
    NON_FINITE = "NON_FINITE"
    TOO_MANY = "TOO_MANY"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    WRONG_FRAME = "WRONG_FRAME"
    IN_FAULT = "IN_FAULT"
    NO_ODOM = "NO_ODOM"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    NOT_STOPPED = "NOT_STOPPED"
    NOT_IN_FAULT = "NOT_IN_FAULT"


# Allowed (state, event) -> next state. Anything missing is refused (REQ-SM-010).
TRANSITIONS: dict[tuple[State, Event], State] = {
    (State.IDLE, Event.GOAL): State.NAVIGATING,  # REQ-SM-002
    (State.STOPPED, Event.GOAL): State.NAVIGATING,  # REQ-SM-002
    (State.NAVIGATING, Event.GOAL): State.NAVIGATING,  # REQ-SM-002 (replace mission)
    (State.NAVIGATING, Event.GOAL_COMPLETE): State.IDLE,  # REQ-SM-003
    (State.NAVIGATING, Event.STOP): State.STOPPED,  # REQ-SM-004
    (State.NAVIGATING, Event.ESTOP): State.STOPPED,  # REQ-SAFE-001
    (State.STOPPED, Event.RESUME): State.NAVIGATING,  # REQ-SM-005
    (State.NAVIGATING, Event.WATCHDOG_TRIP): State.FAULT,  # REQ-SAFE-003
    (State.STOPPED, Event.WATCHDOG_TRIP): State.FAULT,  # REQ-SAFE-003
    (State.NAVIGATING, Event.INVALID_COMMAND): State.FAULT,  # REQ-CTL-006
    (State.FAULT, Event.RESET): State.IDLE,  # REQ-SM-006
}


@dataclass(frozen=True)
class Waypoint:
    """A 2D target in the odometry frame (meters)."""

    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in the odometry frame (meters, radians)."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    """Body-frame velocity: forward speed (m/s) and yaw rate (rad/s)."""

    linear_mps: float = 0.0
    angular_radps: float = 0.0

    def is_finite(self) -> bool:
        """Return True if both components are finite numbers."""
        return math.isfinite(self.linear_mps) and math.isfinite(self.angular_radps)


ZERO = VelocityCommand()


@dataclass(frozen=True)
class Limits:
    """Velocity limits enforced by ``clamp_velocity`` (REQ-CTL-001..003)."""

    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float


@dataclass(frozen=True)
class ControllerParams:
    """Every tunable number the core uses. Values come from config/params.yaml."""

    goal_tolerance_m: float
    rotate_in_place_threshold_rad: float
    slowdown_radius_m: float
    heading_gain: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float
    control_rate_hz: float
    max_waypoints: int
    max_waypoint_range_m: float
    odom_timeout_s: float

    @property
    def limits(self) -> Limits:
        """Velocity limits for ``clamp_velocity``."""
        return Limits(self.max_linear_speed_mps, self.max_angular_speed_radps, self.max_linear_accel_mps2)

    @property
    def control_period_s(self) -> float:
        """Seconds per control cycle."""
        return 1.0 / self.control_rate_hz


@dataclass(frozen=True)
class Range:
    """Numeric interval used to validate a parameter."""

    low: float
    high: float
    low_open: bool = False
    high_open: bool = False

    def contains(self, value: float) -> bool:
        """Return True if ``value`` lies inside the interval."""
        above = value > self.low if self.low_open else value >= self.low
        below = value < self.high if self.high_open else value <= self.high
        return above and below

    def __str__(self) -> str:
        """Render as interval notation, e.g. ``(0, 1.0]``."""
        return f"{'(' if self.low_open else '['}{self.low}, {self.high}{')' if self.high_open else ']'}"


# Allowed ranges from docs/requirements.md §5 (REQ-IF-006).
PARAM_RANGES: dict[str, Range] = {
    "goal_tolerance_m": Range(0.0, 2.0, low_open=True),
    "rotate_in_place_threshold_rad": Range(0.0, math.pi, low_open=True),
    "slowdown_radius_m": Range(0.0, 10.0),
    "heading_gain": Range(0.0, 10.0, low_open=True),
    "max_linear_speed_mps": Range(0.0, PLATFORM_MAX_LINEAR_SPEED_MPS, low_open=True),  # REQ-CTL-001
    "max_angular_speed_radps": Range(0.0, PLATFORM_MAX_ANGULAR_SPEED_RADPS, low_open=True),  # REQ-CTL-002
    "max_linear_accel_mps2": Range(0.0, 2.0, low_open=True),
    "control_rate_hz": Range(5.0, 100.0),
    "max_waypoints": Range(1, 1000),
    "max_waypoint_range_m": Range(0.0, 500.0, low_open=True),
    "odom_timeout_s": Range(0.0, PLATFORM_CMD_TIMEOUT_S, low_open=True, high_open=True),  # REQ-SAFE-006
}


def validate_params(params: ControllerParams) -> list[str]:
    """Return every parameter problem, or an empty list if the config is valid.

    Implements REQ-IF-006 and enforces the limits behind REQ-CTL-001, REQ-CTL-002
    and REQ-SAFE-006.
    """
    errors: list[str] = []
    for f in fields(params):
        value = getattr(params, f.name)
        allowed = PARAM_RANGES[f.name]
        if f.name == "max_waypoints" and (isinstance(value, bool) or not isinstance(value, int)):
            errors.append(f"{f.name}={value!r} must be an integer")
        elif not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            errors.append(f"{f.name}={value!r} must be a finite number")
        elif not allowed.contains(value):
            errors.append(f"{f.name}={value} is outside the allowed range {allowed}")
    return errors


@dataclass(frozen=True)
class Result:
    """Outcome of a request (goal, stop, resume, reset)."""

    accepted: bool
    reason: Reason


@dataclass(frozen=True)
class Status:
    """Snapshot published on the status topic (REQ-SM-009)."""

    state: State
    reason: Reason
    waypoint_index: int
    waypoint_count: int
    estop_active: bool
    last_goal: Result | None

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-serializable representation."""
        return {
            "state": self.state.value,
            "reason": self.reason.value,
            "waypoint_index": self.waypoint_index,
            "waypoint_count": self.waypoint_count,
            "estop_active": self.estop_active,
            "last_goal": (
                None
                if self.last_goal is None
                else {"accepted": self.last_goal.accepted, "reason": self.last_goal.reason.value}
            ),
        }


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi) (REQ-NAV-004)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about z) from a unit quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def validate_waypoint(waypoint: Waypoint, robot: Pose2D, max_range_m: float) -> Reason | None:
    """Check one waypoint. Returns the rejection reason, or None if valid.

    REQ-WPT-002 (finite coordinates) and REQ-WPT-004 (within range of the robot).
    """
    if not (math.isfinite(waypoint.x) and math.isfinite(waypoint.y)):
        return Reason.NON_FINITE
    if math.hypot(waypoint.x - robot.x, waypoint.y - robot.y) > max_range_m:
        return Reason.OUT_OF_RANGE
    return None


def validate_waypoints(
    waypoints: Sequence[Waypoint],
    frame_id: str,
    odom_frame_id: str,
    robot: Pose2D,
    params: ControllerParams,
) -> Reason | None:
    """Check a whole goal. Returns the first rejection reason, or None if valid.

    REQ-WPT-001 (empty), REQ-WPT-003 (too many), REQ-WPT-005 (frame), and the
    per-waypoint checks in ``validate_waypoint``. Non-finite values are reported
    before range problems so the reason points at the real defect.
    """
    if not waypoints:
        return Reason.EMPTY
    if len(waypoints) > params.max_waypoints:
        return Reason.TOO_MANY
    if frame_id != odom_frame_id:
        return Reason.WRONG_FRAME
    if any(not (math.isfinite(w.x) and math.isfinite(w.y)) for w in waypoints):
        return Reason.NON_FINITE
    for waypoint in waypoints:
        reason = validate_waypoint(waypoint, robot, params.max_waypoint_range_m)
        if reason is not None:
            return reason
    return None


def compute_command(
    pose: Pose2D, target: Waypoint, is_final: bool, params: ControllerParams
) -> VelocityCommand:
    """Proportional heading control toward ``target``. The result is unclamped.

    - Angular rate is ``heading_gain`` times the wrapped heading error (REQ-NAV-004).
    - Linear speed is zero while the heading error exceeds the rotate-in-place
      threshold (REQ-NAV-005), otherwise ``max_linear_speed * cos(error)``.
    - On the final waypoint, linear speed scales down inside ``slowdown_radius_m``
      (REQ-NAV-006).
    """
    dx = target.x - pose.x
    dy = target.y - pose.y
    distance_m = math.hypot(dx, dy)
    heading_error = wrap_angle(math.atan2(dy, dx) - pose.yaw)

    angular = params.heading_gain * heading_error
    if abs(heading_error) > params.rotate_in_place_threshold_rad:
        linear = 0.0
    else:
        linear = params.max_linear_speed_mps * math.cos(heading_error)
        if is_final and params.slowdown_radius_m > 0.0:
            linear *= min(1.0, distance_m / params.slowdown_radius_m)
    return VelocityCommand(linear, angular)


def clamp_velocity(
    command: VelocityCommand, limits: Limits, previous_linear_mps: float, dt_s: float
) -> VelocityCommand:
    """Apply the final safety gate to every command before it is published.

    - Non-finite commands become zero (REQ-CTL-006).
    - Linear speed is clamped to [0, max] with no reverse motion (REQ-CTL-001, REQ-CTL-004).
    - Angular rate is clamped to [-max, max] (REQ-CTL-002).
    - Linear speed changes by at most ``max_linear_accel * dt`` per cycle, except a
      command of exactly zero, which always passes so stops are immediate (REQ-CTL-003).
    """
    if not command.is_finite():
        return ZERO

    linear = min(max(command.linear_mps, 0.0), limits.max_linear_speed_mps)
    angular = min(max(command.angular_radps, -limits.max_angular_speed_radps), limits.max_angular_speed_radps)

    if linear != 0.0:
        previous = previous_linear_mps if math.isfinite(previous_linear_mps) else 0.0
        max_step = limits.max_linear_accel_mps2 * dt_s
        linear = min(max(linear, previous - max_step), previous + max_step)
        linear = min(max(linear, 0.0), limits.max_linear_speed_mps)

    return VelocityCommand(linear, angular)


class Watchdog:
    """Tracks odometry freshness using an injected clock.

    ``clock`` returns seconds as a float. The node passes simulation time when
    ``use_sim_time`` is set and a steady clock otherwise. Tests pass a fake
    (REQ-SAFE-003, REQ-SAFE-004, REQ-SAFE-007).
    """

    def __init__(self, timeout_s: float, clock: Callable[[], float]) -> None:
        """Create a watchdog that has not been fed yet (so it starts stale)."""
        self._timeout_s = timeout_s
        self._clock = clock
        self._last_feed_s: float | None = None

    def feed(self) -> None:
        """Record that a fresh message arrived now."""
        self._last_feed_s = self._clock()

    def is_fresh(self) -> bool:
        """Return True if the last feed is at most ``timeout_s`` old.

        Never fed, or a clock that jumped backwards, counts as stale (REQ-SAFE-007).
        """
        if self._last_feed_s is None:
            return False
        now = self._clock()
        if now < self._last_feed_s:
            return False
        return (now - self._last_feed_s) <= self._timeout_s


class WaypointController:
    """State machine plus mission bookkeeping. No ROS, no clock reads of its own."""

    def __init__(self, params: ControllerParams, clock: Callable[[], float]) -> None:
        """Validate ``params`` and start in IDLE (REQ-SM-001, REQ-IF-006).

        Raises:
            ValueError: if any parameter is outside its allowed range.

        """
        errors = validate_params(params)
        if errors:
            raise ValueError("; ".join(errors))
        self._params = params
        self._watchdog = Watchdog(params.odom_timeout_s, clock)
        self._state = State.IDLE
        self._reason = Reason.STARTUP
        self._waypoints: tuple[Waypoint, ...] = ()
        self._index = 0
        self._pose: Pose2D | None = None
        self._odom_frame_id: str | None = None
        self._estop = False
        self._last_goal: Result | None = None

    @property
    def state(self) -> State:
        """Current state."""
        return self._state

    @property
    def params(self) -> ControllerParams:
        """Validated parameters."""
        return self._params

    def status(self) -> Status:
        """Snapshot for the status topic (REQ-SM-009)."""
        return Status(
            state=self._state,
            reason=self._reason,
            waypoint_index=self._index,
            waypoint_count=len(self._waypoints),
            estop_active=self._estop,
            last_goal=self._last_goal,
        )

    # Inputs ---------------------------------------------------------------

    def update_odom(self, pose: Pose2D, frame_id: str) -> None:
        """Store the latest pose and feed the watchdog.

        Non-finite poses are ignored, so bad odometry eventually trips the watchdog (REQ-SAFE-003).
        """
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            _LOG.warning("Ignoring non-finite odometry pose %s", pose)
            return
        self._pose = pose
        self._odom_frame_id = frame_id
        self._watchdog.feed()

    def set_estop(self, active: bool) -> None:
        """Track the platform E-stop. Asserting it stops navigation (REQ-SAFE-001).

        Releasing it never resumes motion by itself (REQ-SAFE-002).
        """
        self._estop = active
        if active and self._state is State.NAVIGATING:
            self._transition(Event.ESTOP, Reason.ESTOP)

    def submit_goal(self, waypoints: Sequence[Waypoint], frame_id: str) -> Result:
        """Start (or replace) a mission (REQ-SM-002).

        Rejected in FAULT (REQ-SM-007), without fresh odometry (REQ-SAFE-005), while
        the E-stop is asserted, or if validation fails (REQ-WPT-001..005). A
        rejection leaves state and mission untouched (REQ-WPT-006).
        """
        result = self._check_goal(waypoints, frame_id)
        self._last_goal = result
        if result.accepted:
            self._waypoints = tuple(waypoints)
            self._index = 0
            self._transition(Event.GOAL, Reason.GOAL_ACCEPTED)
        return result

    def stop(self) -> Result:
        """Pause the mission, keeping remaining waypoints (REQ-SM-004).

        Stopping when already IDLE or STOPPED succeeds without a transition.
        """
        if self._state is State.NAVIGATING:
            self._transition(Event.STOP, Reason.STOP_REQUESTED)
            return Result(True, Reason.STOP_REQUESTED)
        if self._state in (State.IDLE, State.STOPPED):
            return Result(True, Reason.ALREADY_STOPPED)
        return Result(False, Reason.IN_FAULT)

    def resume(self) -> Result:
        """Continue a stopped mission if E-stop is released and odometry is fresh (REQ-SM-005)."""
        if self._state is not State.STOPPED:
            return Result(False, Reason.NOT_STOPPED)
        if self._estop:
            return Result(False, Reason.ESTOP_ACTIVE)
        if not self._watchdog.is_fresh():
            return Result(False, Reason.ODOM_STALE)
        self._transition(Event.RESUME, Reason.RESUMED)
        return Result(True, Reason.RESUMED)

    def reset(self) -> Result:
        """Leave FAULT for IDLE once odometry is fresh again, clearing the mission (REQ-SM-006)."""
        if self._state is not State.FAULT:
            return Result(False, Reason.NOT_IN_FAULT)
        if not self._watchdog.is_fresh():
            return Result(False, Reason.ODOM_STALE)
        self._clear_mission()
        self._transition(Event.RESET, Reason.RESET)
        return Result(True, Reason.RESET)

    # Control cycle ----------------------------------------------------------

    def step(self) -> VelocityCommand:
        """Run one control cycle and return the raw (unclamped) command.

        The caller must pass the result through ``clamp_velocity`` before publishing.
        - Watchdog: stale odometry in NAVIGATING or STOPPED -> FAULT (REQ-SAFE-003, REQ-SAFE-004).
        - Outside NAVIGATING the command is always zero (REQ-SM-008).
        - Reached waypoints are consumed in order (REQ-NAV-001, REQ-NAV-002); after the
          last one the mission completes (REQ-NAV-003, REQ-SM-003).
        - A non-finite command faults the controller (REQ-CTL-006).
        """
        if self._state in (State.NAVIGATING, State.STOPPED) and not self._watchdog.is_fresh():
            self._transition(Event.WATCHDOG_TRIP, Reason.ODOM_STALE)
            return ZERO
        if self._state is not State.NAVIGATING or self._pose is None:
            return ZERO

        pose = self._pose
        while self._index < len(self._waypoints) and self._reached(pose, self._waypoints[self._index]):
            self._index += 1
        if self._index >= len(self._waypoints):
            self._clear_mission()
            self._transition(Event.GOAL_COMPLETE, Reason.MISSION_COMPLETE)
            return ZERO

        is_final = self._index == len(self._waypoints) - 1
        command = compute_command(pose, self._waypoints[self._index], is_final, self._params)
        if not command.is_finite():
            self._transition(Event.INVALID_COMMAND, Reason.NON_FINITE_COMMAND)
            return ZERO
        return command

    # Internals --------------------------------------------------------------

    def _check_goal(self, waypoints: Sequence[Waypoint], frame_id: str) -> Result:
        if self._state is State.FAULT:
            return Result(False, Reason.IN_FAULT)
        if self._pose is None or self._odom_frame_id is None or not self._watchdog.is_fresh():
            return Result(False, Reason.NO_ODOM)
        if self._estop:
            return Result(False, Reason.ESTOP_ACTIVE)
        reason = validate_waypoints(waypoints, frame_id, self._odom_frame_id, self._pose, self._params)
        if reason is not None:
            return Result(False, reason)
        return Result(True, Reason.GOAL_ACCEPTED)

    def _reached(self, pose: Pose2D, waypoint: Waypoint) -> bool:
        return math.hypot(waypoint.x - pose.x, waypoint.y - pose.y) <= self._params.goal_tolerance_m

    def _clear_mission(self) -> None:
        self._waypoints = ()
        self._index = 0

    def _transition(self, event: Event, reason: Reason) -> bool:
        """Apply ``event`` using the transition table. Unknown pairs are refused (REQ-SM-010)."""
        target = TRANSITIONS.get((self._state, event))
        if target is None:
            _LOG.warning("Refused event %s in state %s", event.value, self._state.value)
            return False
        _LOG.info("%s --%s--> %s (%s)", self._state.value, event.value, target.value, reason.value)
        self._state = target
        self._reason = reason
        return True
