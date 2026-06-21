#!/usr/bin/env python3
"""Publish a display-only twist stream resilient to terminal key repeat gaps."""

from copy import deepcopy
import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from teleop_msgs.msg import TeleopCommand, TeleopStatus

from .display_twist_state import DisplayAction, DisplayTwistState


class TwistDisplayConditionerROS2Unity(Node):
    """Condition observed twists for Unity without writing to control topics."""

    def __init__(self) -> None:
        super().__init__("twist_display_conditioner")

        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("status_topic", "/teleop/status")
        self.declare_parameter("display_topic", "/unity/cmd_display/twist")
        self.declare_parameter("display_rate_hz", 30.0)
        self.declare_parameter("initial_hold_s", 0.65)
        self.declare_parameter("active_timeout_s", 0.25)
        self.declare_parameter("linear_zero_epsilon", 1e-6)
        self.declare_parameter("angular_zero_epsilon", 1e-6)

        self.linear_zero_epsilon = abs(
            float(self.get_parameter("linear_zero_epsilon").value)
        )
        self.angular_zero_epsilon = abs(
            float(self.get_parameter("angular_zero_epsilon").value)
        )
        self.state = DisplayTwistState(
            initial_hold_s=float(self.get_parameter("initial_hold_s").value),
            active_timeout_s=float(self.get_parameter("active_timeout_s").value),
        )
        self.last_nonzero: TwistStamped | None = None

        self.display_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("display_topic").value,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            self.on_twist,
            10,
        )
        self.create_subscription(
            TeleopCommand,
            self.get_parameter("command_topic").value,
            self.on_command,
            10,
        )
        self.create_subscription(
            TeleopStatus,
            self.get_parameter("status_topic").value,
            self.on_status,
            10,
        )

        display_rate_hz = max(
            float(self.get_parameter("display_rate_hz").value),
            1.0,
        )
        self.create_timer(1.0 / display_rate_hz, self.publish_display)
        self.get_logger().info(
            "Conditioning display-only twist -> %s"
            % self.get_parameter("display_topic").value
        )

    def on_twist(self, msg: TwistStamped) -> None:
        """Cache accepted nonzero twists; zeros are handled by display timeouts."""
        if not self._is_nonzero(msg):
            return
        self.last_nonzero = deepcopy(msg)
        self.state.observe_nonzero(self._now_s())

    def on_command(self, msg: TeleopCommand) -> None:
        """Apply explicit command-side stops immediately."""
        hard_stop = (
            msg.emergency_stop
            or not msg.enable
            or msg.control_mode != TeleopCommand.CARTESIAN_TWIST
        )
        if not hard_stop and not self._is_nonzero(msg.target_twist):
            hard_stop = True
        if hard_stop:
            self._force_stop(msg.target_twist.header.frame_id)

    def on_status(self, msg: TeleopStatus) -> None:
        """Never hold a command across a safety rejection or disabled state."""
        if msg.emergency_stop or not msg.enabled or not msg.safety_ok:
            self._force_stop()

    def publish_display(self) -> None:
        """Republish the held display value or emit one zero on expiry."""
        action = self.state.tick(self._now_s())
        if action == DisplayAction.PUBLISH and self.last_nonzero is not None:
            msg = deepcopy(self.last_nonzero)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.display_pub.publish(msg)
        elif action == DisplayAction.STOP:
            self._publish_zero()
            self.last_nonzero = None

    def _force_stop(self, frame_id: str = "") -> None:
        if self.state.force_stop():
            self._publish_zero(frame_id)
        self.last_nonzero = None

    def _publish_zero(self, frame_id: str = "") -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if frame_id:
            msg.header.frame_id = frame_id
        elif self.last_nonzero is not None:
            msg.header.frame_id = self.last_nonzero.header.frame_id
        self.display_pub.publish(msg)

    def _is_nonzero(self, msg: TwistStamped) -> bool:
        linear = msg.twist.linear
        angular = msg.twist.angular
        linear_norm = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        angular_norm = math.sqrt(
            angular.x ** 2 + angular.y ** 2 + angular.z ** 2
        )
        return (
            linear_norm > self.linear_zero_epsilon
            or angular_norm > self.angular_zero_epsilon
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    """Run the display twist conditioner node."""
    rclpy.init(args=args)
    node = TwistDisplayConditionerROS2Unity()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
