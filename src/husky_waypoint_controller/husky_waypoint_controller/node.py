"""ROS 2 adapter for the waypoint controller (REQ-ARCH-003).

This node only converts messages, holds parameters and timers, and publishes.
Every decision is made by ``core.WaypointController``. Every velocity command
goes through ``_publish_cmd``, which applies ``clamp_velocity`` as its last step.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import fields
from typing import TypeVar

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from husky_waypoint_controller.core import (
    ZERO,
    ControllerParams,
    Pose2D,
    Result,
    State,
    VelocityCommand,
    Waypoint,
    WaypointController,
    clamp_velocity,
    yaw_from_quaternion,
)

NODE_NAME = "waypoint_controller"
_NS_PER_S = 1e9
_T = TypeVar("_T")

# ROS-side parameters (not used by the core). All values come from config/params.yaml.
_STRING_PARAMS = ("namespace", "odom_topic", "base_frame_id")
_BOOL_PARAMS = ("cmd_vel_stamped",)


class InvalidConfigError(RuntimeError):
    """Raised when parameters are missing or out of range (REQ-IF-006)."""


class WaypointControllerNode(Node):
    """Thin rclpy wrapper around ``WaypointController``."""

    def __init__(self) -> None:
        """Load and validate parameters, then wire up topics, services and the control timer."""
        super().__init__(NODE_NAME)
        ros_params, core_params = self._load_params()

        clock = (
            self.get_clock()
            if self.get_parameter("use_sim_time").value
            else Clock(clock_type=ClockType.STEADY_TIME)
        )
        try:
            self._core = WaypointController(core_params, lambda: clock.now().nanoseconds / _NS_PER_S)
        except ValueError as exc:
            for problem in str(exc).split("; "):
                self.get_logger().error(f"Invalid parameter: {problem}")
            raise InvalidConfigError(str(exc)) from exc

        self._limits = core_params.limits
        self._period_s = core_params.control_period_s
        self._last_linear_mps = 0.0
        self._stamped = bool(ros_params["cmd_vel_stamped"])
        self._base_frame_id = str(ros_params["base_frame_id"])
        self._last_status: dict[str, object] | None = None

        robot_ns = str(ros_params["namespace"]).strip("/")

        def husky(topic: str) -> str:
            # Husky topics are absolute under the robot namespace (REQ-IF-001).
            return f"/{robot_ns}/{topic}"

        # REQ-IF-002: TwistStamped on Jazzy, Twist on Humble. Never platform/cmd_vel (REQ-IF-007).
        self._cmd_pub = self.create_publisher(TwistStamped if self._stamped else Twist, husky("cmd_vel"), 10)

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._status_pub = self.create_publisher(String, "status", latched)  # REQ-SM-009

        self.create_subscription(
            Odometry, husky(str(ros_params["odom_topic"])), self._on_odom, qos_profile_sensor_data
        )  # REQ-IF-003
        self.create_subscription(Bool, husky("platform/emergency_stop"), self._on_estop, 10)  # REQ-IF-004
        self.create_subscription(Path, "waypoints", self._on_waypoints, 10)  # REQ-IF-005

        self.create_service(Trigger, "stop", self._on_stop)  # REQ-IF-005
        self.create_service(Trigger, "resume", self._on_resume)
        self.create_service(Trigger, "reset", self._on_reset)

        self.create_timer(self._period_s, self._on_control_tick)  # REQ-CTL-005

        self.get_logger().info(
            f"Ready: cmd_vel={husky('cmd_vel')} ({'TwistStamped' if self._stamped else 'Twist'}), "
            f"odom={husky(str(ros_params['odom_topic']))}, rate={core_params.control_rate_hz} Hz"
        )
        self._publish_status()

    # Parameters ---------------------------------------------------------------

    def _load_params(self) -> tuple[dict[str, object], ControllerParams]:
        """Declare every parameter without a default and read it from the params file (REQ-IF-006)."""
        missing: list[str] = []

        def read(name: str, kind: Parameter.Type) -> object:
            self.declare_parameter(name, kind)
            value = self.get_parameter(name).value
            if value is None or (kind is Parameter.Type.STRING and not str(value).strip("/")):
                missing.append(name)
            return value

        ros_params: dict[str, object] = {name: read(name, Parameter.Type.STRING) for name in _STRING_PARAMS}
        ros_params.update({name: read(name, Parameter.Type.BOOL) for name in _BOOL_PARAMS})
        core_values = {
            f.name: read(f.name, Parameter.Type.INTEGER if f.type in (int, "int") else Parameter.Type.DOUBLE)
            for f in fields(ControllerParams)
        }

        if missing:
            for name in missing:
                self.get_logger().error(f"Missing required parameter: {name}")
            raise InvalidConfigError(f"missing parameters: {', '.join(missing)}")
        return ros_params, ControllerParams(**core_values)  # type: ignore[arg-type]

    # Callbacks ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = Pose2D(
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)
        )
        self._core.update_odom(pose, msg.header.frame_id)

    def _on_estop(self, msg: Bool) -> None:
        self._apply(lambda: self._core.set_estop(bool(msg.data)))

    def _on_waypoints(self, msg: Path) -> None:
        waypoints = [Waypoint(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        result = self._apply(lambda: self._core.submit_goal(waypoints, msg.header.frame_id))
        # Separate call sites: rclpy raises if one call site logs at two severities.
        summary = f"Goal with {len(waypoints)} waypoint(s): {result.reason.value}"
        if result.accepted:
            self.get_logger().info(summary)
        else:
            self.get_logger().warning(summary)

    def _on_stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._respond(self._apply(self._core.stop), response)

    def _on_resume(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._respond(self._apply(self._core.resume), response)

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._respond(self._apply(self._core.reset), response)

    def _on_control_tick(self) -> None:
        was = self._core.state
        command = self._core.step()
        # Publish every cycle while navigating, plus the zero on the cycle we leave NAVIGATING (REQ-SM-008).
        if was is State.NAVIGATING or self._core.state is State.NAVIGATING:
            self._publish_cmd(command)
        self._publish_status()

    # Helpers --------------------------------------------------------------------

    def _apply(self, action: Callable[[], _T]) -> _T:
        """Run a core action, then publish an immediate zero if it ended navigation (REQ-SAFE-001)."""
        was = self._core.state
        outcome = action()
        if was is State.NAVIGATING and self._core.state is not State.NAVIGATING:
            self._publish_cmd(ZERO)
        self._publish_status()
        return outcome

    @staticmethod
    def _respond(result: Result, response: Trigger.Response) -> Trigger.Response:
        response.success = result.accepted
        response.message = result.reason.value
        return response

    def _publish_cmd(self, command: VelocityCommand) -> None:
        """Publish to cmd_vel (the only path); ``clamp_velocity`` is always the final step.

        Enforces REQ-CTL-001, REQ-CTL-002, REQ-CTL-003, REQ-CTL-004 and REQ-CTL-006
        no matter what the caller passes in.
        """
        safe = clamp_velocity(command, self._limits, self._last_linear_mps, self._period_s)
        self._last_linear_mps = safe.linear_mps

        twist = Twist()
        twist.linear.x = safe.linear_mps
        twist.angular.z = safe.angular_radps
        if self._stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame_id
            msg.twist = twist
        else:
            msg = twist
        self._cmd_pub.publish(msg)

    def _publish_status(self) -> None:
        """Publish the status snapshot when it changes (REQ-SM-009)."""
        status = self._core.status().to_dict()
        if status != self._last_status:
            self._last_status = status
            self._status_pub.publish(String(data=json.dumps(status)))


def main(args: list[str] | None = None) -> None:
    """Entry point for ``ros2 run husky_waypoint_controller waypoint_controller``."""
    rclpy.init(args=args)
    try:
        node = WaypointControllerNode()
    except InvalidConfigError:
        rclpy.try_shutdown()
        raise SystemExit(1) from None
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
