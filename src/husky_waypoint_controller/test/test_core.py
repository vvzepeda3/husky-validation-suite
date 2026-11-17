"""Unit tests for the pure-Python core. No ROS required."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from husky_waypoint_controller.core import (
    TRANSITIONS,
    ZERO,
    ControllerParams,
    Event,
    Pose2D,
    Reason,
    State,
    VelocityCommand,
    Watchdog,
    Waypoint,
    WaypointController,
    clamp_velocity,
    compute_command,
    validate_params,
    validate_waypoint,
    validate_waypoints,
    wrap_angle,
    yaw_from_quaternion,
)

from husky_waypoint_controller import core

PARAMS = ControllerParams(
    goal_tolerance_m=0.20,
    rotate_in_place_threshold_rad=0.785,
    slowdown_radius_m=1.0,
    heading_gain=1.5,
    max_linear_speed_mps=0.5,
    max_angular_speed_radps=0.8,
    max_linear_accel_mps2=0.5,
    control_rate_hz=20.0,
    max_waypoints=100,
    max_waypoint_range_m=50.0,
    odom_timeout_s=0.2,
)
LIMITS = PARAMS.limits
DT = PARAMS.control_period_s
ORIGIN = Pose2D(0.0, 0.0, 0.0)
FRAME = "odom"


class FakeClock:
    """Manually advanced clock in seconds."""

    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def make_controller(params: ControllerParams = PARAMS) -> tuple[WaypointController, FakeClock]:
    clock = FakeClock()
    ctrl = WaypointController(params, clock)
    ctrl.update_odom(ORIGIN, FRAME)
    return ctrl, clock


def navigating(waypoints: list[Waypoint] | None = None) -> tuple[WaypointController, FakeClock]:
    ctrl, clock = make_controller()
    result = ctrl.submit_goal(waypoints or [Waypoint(3.0, 0.0), Waypoint(3.0, 3.0)], FRAME)
    assert result.accepted
    return ctrl, clock


# --- Navigation ------------------------------------------------------------------


@pytest.mark.req("REQ-NAV-001", "REQ-NAV-002")
def test_advances_only_after_current_waypoint_is_reached() -> None:
    ctrl, _ = navigating([Waypoint(3.0, 0.0), Waypoint(3.0, 3.0)])
    ctrl.update_odom(Pose2D(2.75, 0.0, 0.0), FRAME)  # 0.25 m away: not reached
    ctrl.step()
    assert ctrl.status().waypoint_index == 0
    ctrl.update_odom(Pose2D(2.85, 0.0, 0.0), FRAME)  # 0.15 m away: reached
    ctrl.step()
    assert ctrl.status().waypoint_index == 1


@pytest.mark.req("REQ-NAV-001")
def test_waypoint_exactly_at_tolerance_counts_as_reached() -> None:
    ctrl, _ = navigating([Waypoint(1.0, 0.0), Waypoint(5.0, 0.0)])
    ctrl.update_odom(Pose2D(1.0 - PARAMS.goal_tolerance_m, 0.0, 0.0), FRAME)
    ctrl.step()
    assert ctrl.status().waypoint_index == 1


@pytest.mark.req("REQ-NAV-003", "REQ-SM-003")
def test_final_waypoint_completes_mission_with_zero_command() -> None:
    ctrl, _ = navigating([Waypoint(1.0, 0.0)])
    ctrl.update_odom(Pose2D(0.95, 0.0, 0.0), FRAME)
    assert ctrl.step() == ZERO
    status = ctrl.status()
    assert status.state is State.IDLE
    assert status.reason is Reason.MISSION_COMPLETE
    assert status.waypoint_count == 0


@pytest.mark.req("REQ-NAV-004")
@pytest.mark.parametrize(
    ("yaw", "target", "expected_sign"),
    [
        (0.0, Waypoint(1.0, 1.0), 1),  # target to the left
        (0.0, Waypoint(1.0, -1.0), -1),  # target to the right
        (3.0, Waypoint(-1.0, -0.2), 1),  # crossing +pi -> turn left the short way
        (-3.0, Waypoint(-1.0, 0.2), -1),  # crossing -pi -> turn right the short way
    ],
)
def test_angular_command_turns_toward_target_the_short_way(
    yaw: float, target: Waypoint, expected_sign: int
) -> None:
    cmd = compute_command(Pose2D(0.0, 0.0, yaw), target, is_final=False, params=PARAMS)
    assert math.copysign(1, cmd.angular_radps) == expected_sign


@pytest.mark.req("REQ-NAV-004")
@pytest.mark.parametrize("angle", [0.0, math.pi, -math.pi, 3 * math.pi, -7.5, 7.5])
def test_wrap_angle_range(angle: float) -> None:
    wrapped = wrap_angle(angle)
    assert -math.pi <= wrapped < math.pi
    assert math.isclose(math.sin(wrapped), math.sin(angle), abs_tol=1e-9)
    assert math.isclose(math.cos(wrapped), math.cos(angle), abs_tol=1e-9)


@pytest.mark.req("REQ-NAV-004")
def test_yaw_from_quaternion() -> None:
    half = math.pi / 4  # quaternion for yaw = pi/2
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)), math.pi / 2)


@pytest.mark.req("REQ-NAV-005")
def test_rotates_in_place_when_heading_error_is_large() -> None:
    cmd = compute_command(ORIGIN, Waypoint(0.0, 2.0), is_final=False, params=PARAMS)  # 90 deg error
    assert cmd.linear_mps == 0.0
    assert cmd.angular_radps > 0.0


@pytest.mark.req("REQ-NAV-006")
def test_slows_down_only_near_final_waypoint() -> None:
    target = Waypoint(0.5, 0.0)  # half the slowdown radius, straight ahead
    final = compute_command(ORIGIN, target, is_final=True, params=PARAMS)
    intermediate = compute_command(ORIGIN, target, is_final=False, params=PARAMS)
    assert math.isclose(intermediate.linear_mps, PARAMS.max_linear_speed_mps)
    assert math.isclose(final.linear_mps, PARAMS.max_linear_speed_mps * 0.5)


# --- Waypoint validation -------------------------------------------------------------


@pytest.mark.req("REQ-WPT-001")
def test_rejects_empty_goal() -> None:
    assert validate_waypoints([], FRAME, FRAME, ORIGIN, PARAMS) is Reason.EMPTY


@pytest.mark.req("REQ-WPT-002")
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_rejects_non_finite_coordinates(bad: float) -> None:
    assert validate_waypoint(Waypoint(bad, 0.0), ORIGIN, 50.0) is Reason.NON_FINITE
    goal = [Waypoint(100.0, 0.0), Waypoint(0.0, bad)]  # non-finite reported even with a range problem
    assert validate_waypoints(goal, FRAME, FRAME, ORIGIN, PARAMS) is Reason.NON_FINITE


@pytest.mark.req("REQ-WPT-003")
def test_rejects_too_many_waypoints() -> None:
    goal = [Waypoint(1.0, 0.0)] * (PARAMS.max_waypoints + 1)
    assert validate_waypoints(goal, FRAME, FRAME, ORIGIN, PARAMS) is Reason.TOO_MANY
    assert validate_waypoints(goal[:-1], FRAME, FRAME, ORIGIN, PARAMS) is None


@pytest.mark.req("REQ-WPT-004")
def test_rejects_waypoint_out_of_range() -> None:
    assert validate_waypoint(Waypoint(50.1, 0.0), ORIGIN, 50.0) is Reason.OUT_OF_RANGE
    assert validate_waypoint(Waypoint(50.0, 0.0), ORIGIN, 50.0) is None


@pytest.mark.req("REQ-WPT-005")
def test_rejects_wrong_frame() -> None:
    assert validate_waypoints([Waypoint(1.0, 0.0)], "map", FRAME, ORIGIN, PARAMS) is Reason.WRONG_FRAME


@pytest.mark.req("REQ-WPT-006")
def test_rejected_goal_leaves_state_and_mission_unchanged() -> None:
    ctrl, _ = navigating([Waypoint(3.0, 0.0), Waypoint(3.0, 3.0)])
    before = ctrl.status()
    result = ctrl.submit_goal([Waypoint(math.nan, 0.0)], FRAME)
    after = ctrl.status()
    assert not result.accepted and result.reason is Reason.NON_FINITE
    assert (after.state, after.waypoint_index, after.waypoint_count) == (
        before.state,
        before.waypoint_index,
        before.waypoint_count,
    )
    assert after.last_goal == result


# --- Command limits ------------------------------------------------------------------


@pytest.mark.req("REQ-CTL-001", "REQ-CTL-004")
@pytest.mark.parametrize("linear", [-2.0, -0.1, 0.0, 0.3, 0.5, 0.9, 50.0])
def test_linear_speed_clamped_to_zero_and_max(linear: float) -> None:
    out = clamp_velocity(VelocityCommand(linear, 0.0), LIMITS, previous_linear_mps=0.5, dt_s=10.0)
    assert 0.0 <= out.linear_mps <= LIMITS.max_linear_speed_mps


@pytest.mark.req("REQ-CTL-002")
@pytest.mark.parametrize("angular", [-50.0, -0.8, -0.1, 0.0, 0.5, 0.81, 50.0])
def test_angular_speed_clamped(angular: float) -> None:
    out = clamp_velocity(VelocityCommand(0.0, angular), LIMITS, previous_linear_mps=0.0, dt_s=DT)
    assert abs(out.angular_radps) <= LIMITS.max_angular_speed_radps
    assert math.copysign(1, out.angular_radps) == math.copysign(1, angular)


@pytest.mark.req("REQ-CTL-003")
def test_linear_acceleration_limited_except_for_stops() -> None:
    step = LIMITS.max_linear_accel_mps2 * DT
    speeding_up = clamp_velocity(VelocityCommand(0.5, 0.0), LIMITS, previous_linear_mps=0.1, dt_s=DT)
    slowing_down = clamp_velocity(VelocityCommand(0.05, 0.0), LIMITS, previous_linear_mps=0.4, dt_s=DT)
    stopping = clamp_velocity(ZERO, LIMITS, previous_linear_mps=0.5, dt_s=DT)
    assert math.isclose(speeding_up.linear_mps, 0.1 + step)
    assert math.isclose(slowing_down.linear_mps, 0.4 - step)
    assert stopping.linear_mps == 0.0


@pytest.mark.req("REQ-CTL-006")
@pytest.mark.parametrize(
    "command",
    [VelocityCommand(math.nan, 0.0), VelocityCommand(0.1, math.inf), VelocityCommand(-math.inf, 0.0)],
)
def test_clamp_turns_non_finite_into_zero(command: VelocityCommand) -> None:
    assert clamp_velocity(command, LIMITS, previous_linear_mps=0.2, dt_s=DT) == ZERO


@pytest.mark.req("REQ-CTL-006")
def test_non_finite_control_output_faults(monkeypatch: pytest.MonkeyPatch) -> None:
    ctrl, _ = navigating()
    monkeypatch.setattr(core, "compute_command", lambda *_args, **_kw: VelocityCommand(math.nan, 0.0))
    assert ctrl.step() == ZERO
    assert ctrl.state is State.FAULT
    assert ctrl.status().reason is Reason.NON_FINITE_COMMAND


# --- State machine ---------------------------------------------------------------------


@pytest.mark.req("REQ-SM-001")
def test_starts_idle() -> None:
    assert WaypointController(PARAMS, FakeClock()).state is State.IDLE


@pytest.mark.req("REQ-SM-002")
@pytest.mark.parametrize("start", [State.IDLE, State.STOPPED, State.NAVIGATING])
def test_valid_goal_starts_or_replaces_mission(start: State) -> None:
    ctrl, _ = navigating([Waypoint(3.0, 0.0), Waypoint(3.0, 3.0)])
    if start is State.IDLE:
        ctrl, _ = make_controller()
    elif start is State.STOPPED:
        ctrl.stop()
    assert ctrl.state is start
    assert ctrl.submit_goal([Waypoint(1.0, 1.0)], FRAME).accepted
    assert ctrl.state is State.NAVIGATING
    assert ctrl.status().waypoint_count == 1


@pytest.mark.req("REQ-SM-004")
def test_stop_keeps_remaining_waypoints() -> None:
    ctrl, _ = navigating([Waypoint(3.0, 0.0), Waypoint(3.0, 3.0)])
    assert ctrl.stop().accepted
    assert ctrl.state is State.STOPPED
    assert ctrl.status().waypoint_count == 2


@pytest.mark.req("REQ-SM-005")
def test_resume_requires_released_estop_and_fresh_odom() -> None:
    ctrl, clock = navigating()
    ctrl.set_estop(True)
    assert ctrl.resume().reason is Reason.ESTOP_ACTIVE
    ctrl.set_estop(False)
    clock.t += PARAMS.odom_timeout_s + 0.01
    assert ctrl.resume().reason is Reason.ODOM_STALE
    ctrl.update_odom(ORIGIN, FRAME)
    assert ctrl.resume().accepted
    assert ctrl.state is State.NAVIGATING


@pytest.mark.req("REQ-SM-006")
def test_reset_only_leaves_fault_when_odom_is_fresh() -> None:
    ctrl, clock = navigating()
    clock.t += 1.0
    ctrl.step()
    assert ctrl.state is State.FAULT
    assert ctrl.reset().reason is Reason.ODOM_STALE
    assert ctrl.state is State.FAULT
    ctrl.update_odom(ORIGIN, FRAME)
    assert ctrl.reset().accepted
    assert ctrl.state is State.IDLE
    assert ctrl.status().waypoint_count == 0


@pytest.mark.req("REQ-SM-007")
def test_goals_rejected_in_fault() -> None:
    ctrl, clock = navigating()
    clock.t += 1.0
    ctrl.step()
    ctrl.update_odom(ORIGIN, FRAME)
    result = ctrl.submit_goal([Waypoint(1.0, 0.0)], FRAME)
    assert result.reason is Reason.IN_FAULT
    assert ctrl.state is State.FAULT


@pytest.mark.req("REQ-SM-008")
def test_zero_command_outside_navigating() -> None:
    idle, _ = make_controller()
    stopped, _ = navigating()
    stopped.stop()
    faulted, clock = navigating()
    clock.t += 1.0
    faulted.step()
    for ctrl in (idle, stopped, faulted):
        assert ctrl.state is not State.NAVIGATING
        assert ctrl.step() == ZERO


@pytest.mark.req("REQ-SM-009")
def test_status_snapshot_is_serializable() -> None:
    ctrl, _ = navigating()
    status = ctrl.status().to_dict()
    assert status["state"] == "NAVIGATING"
    assert status["reason"] == "GOAL_ACCEPTED"
    assert status["last_goal"] == {"accepted": True, "reason": "GOAL_ACCEPTED"}


@pytest.mark.req("REQ-SM-010")
def test_events_outside_transition_table_are_refused() -> None:
    ctrl, _ = make_controller()
    for state in State:
        for event in Event:
            target = TRANSITIONS.get((state, event))
            assert target is None or isinstance(target, State)
    assert ctrl.resume().reason is Reason.NOT_STOPPED  # (IDLE, RESUME) is not in the table
    assert ctrl.reset().reason is Reason.NOT_IN_FAULT  # (IDLE, RESET) is not in the table
    assert ctrl.state is State.IDLE


# --- Safety and watchdog ---------------------------------------------------------------


@pytest.mark.req("REQ-SAFE-001")
def test_estop_stops_navigation_immediately() -> None:
    ctrl, _ = navigating()
    ctrl.set_estop(True)
    assert ctrl.state is State.STOPPED
    assert ctrl.status().reason is Reason.ESTOP
    assert ctrl.step() == ZERO


@pytest.mark.req("REQ-SAFE-001")
def test_estop_in_fault_stays_in_fault() -> None:
    ctrl, clock = navigating()
    clock.t += 1.0
    ctrl.step()
    ctrl.set_estop(True)
    assert ctrl.state is State.FAULT


@pytest.mark.req("REQ-SAFE-002")
def test_estop_release_does_not_resume() -> None:
    ctrl, _ = navigating()
    ctrl.set_estop(True)
    ctrl.set_estop(False)
    assert ctrl.state is State.STOPPED
    assert ctrl.step() == ZERO


@pytest.mark.req("REQ-SAFE-003", "REQ-SAFE-004")
def test_stale_odometry_trips_watchdog_into_fault() -> None:
    ctrl, clock = navigating()
    clock.t += PARAMS.odom_timeout_s - 1e-6  # just inside the timeout: still fresh
    assert ctrl.step() != ZERO
    assert ctrl.state is State.NAVIGATING
    clock.t += PARAMS.control_period_s  # detected within one more control period
    assert ctrl.step() == ZERO
    assert ctrl.state is State.FAULT
    assert ctrl.status().reason is Reason.ODOM_STALE


@pytest.mark.req("REQ-SAFE-003")
def test_watchdog_also_guards_stopped_state() -> None:
    ctrl, clock = navigating()
    ctrl.stop()
    clock.t += 1.0
    ctrl.step()
    assert ctrl.state is State.FAULT


@pytest.mark.req("REQ-SAFE-005")
def test_goal_rejected_without_fresh_odometry() -> None:
    fresh_start = WaypointController(PARAMS, FakeClock())
    assert fresh_start.submit_goal([Waypoint(1.0, 0.0)], FRAME).reason is Reason.NO_ODOM
    ctrl, clock = make_controller()
    clock.t += 1.0
    assert ctrl.submit_goal([Waypoint(1.0, 0.0)], FRAME).reason is Reason.NO_ODOM
    assert ctrl.state is State.IDLE


@pytest.mark.req("REQ-SAFE-006")
@pytest.mark.parametrize("timeout", [0.5, 0.6, 0.0])
def test_odom_timeout_must_stay_below_platform_timeout(timeout: float) -> None:
    errors = validate_params(replace(PARAMS, odom_timeout_s=timeout))
    assert any("odom_timeout_s" in e for e in errors)


@pytest.mark.req("REQ-SAFE-007")
def test_clock_jumping_backwards_counts_as_stale() -> None:
    clock = FakeClock(50.0)
    dog = Watchdog(0.2, clock)
    assert not dog.is_fresh()  # never fed
    dog.feed()
    assert dog.is_fresh()
    clock.t = 10.0  # e.g. simulation restarted
    assert not dog.is_fresh()


@pytest.mark.req("REQ-SAFE-003")
def test_non_finite_odometry_is_ignored_and_goes_stale() -> None:
    ctrl, clock = navigating()
    clock.t += 0.15
    ctrl.update_odom(Pose2D(math.nan, 0.0, 0.0), FRAME)
    clock.t += 0.1
    ctrl.step()
    assert ctrl.state is State.FAULT


# --- Configuration -------------------------------------------------------------------------


@pytest.mark.req("REQ-IF-006", "REQ-CTL-001", "REQ-CTL-002")
def test_validate_params_reports_every_violation() -> None:
    bad = replace(PARAMS, max_linear_speed_mps=1.5, max_angular_speed_radps=-1.0, control_rate_hz=math.nan)
    errors = validate_params(bad)
    assert len(errors) == 3
    assert validate_params(PARAMS) == []
    with pytest.raises(ValueError, match="max_linear_speed_mps"):
        WaypointController(bad, FakeClock())


@pytest.mark.req("REQ-IF-006")
def test_max_waypoints_must_be_an_integer() -> None:
    assert validate_params(replace(PARAMS, max_waypoints=10.5)) != []  # type: ignore[arg-type]


# --- Architecture -------------------------------------------------------------------------


@pytest.mark.req("REQ-ARCH-002")
def test_identical_inputs_give_identical_outputs() -> None:
    def run() -> list[VelocityCommand]:
        ctrl, clock = navigating([Waypoint(2.0, 1.0), Waypoint(-1.0, 2.0)])
        out = []
        for i in range(40):
            clock.t += DT
            ctrl.update_odom(Pose2D(0.05 * i, 0.02 * i, 0.01 * i), FRAME)
            out.append(ctrl.step())
        return out

    assert run() == run()


@pytest.mark.req("REQ-CTL-001", "REQ-CTL-002", "REQ-CTL-004")
def test_closed_loop_commands_respect_limits_after_clamp() -> None:
    """Drive a crude kinematic model and check every clamped command stays in bounds."""
    ctrl, clock = navigating([Waypoint(5.0, 0.0), Waypoint(5.0, 5.0), Waypoint(0.0, 5.0), Waypoint(0.0, 0.0)])
    pose, last_v = ORIGIN, 0.0
    for _ in range(2000):
        cmd = clamp_velocity(ctrl.step(), LIMITS, last_v, DT)
        last_v = cmd.linear_mps
        assert 0.0 <= cmd.linear_mps <= LIMITS.max_linear_speed_mps
        assert abs(cmd.angular_radps) <= LIMITS.max_angular_speed_radps
        yaw = pose.yaw + cmd.angular_radps * DT
        pose = Pose2D(
            pose.x + cmd.linear_mps * math.cos(yaw) * DT, pose.y + cmd.linear_mps * math.sin(yaw) * DT, yaw
        )
        clock.t += DT
        ctrl.update_odom(pose, FRAME)
        if ctrl.state is State.IDLE:
            break
    assert ctrl.state is State.IDLE
    assert ctrl.status().reason is Reason.MISSION_COMPLETE
