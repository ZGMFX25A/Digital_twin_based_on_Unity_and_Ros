#!/usr/bin/env python3
"""Watchdog that resends the UR robot program when it drops, while teleop is active."""

# The UR headless external-control program
# (/io_and_status_controller/robot_program_running) sometimes drops to false on
# its own; once it does, motion commands no longer reach the robot and it must be
# revived with /io_and_status_controller/resend_robot_program. This node watches
# that state and resends the program automatically.
#
# Control gating: resending restarts the external-control program (briefly
# interrupting/activating control), so it must not happen unsolicited. The
# watchdog only acts while the control stack is online, detected as
# count_publishers(/teleop/command) >= gate_min_publishers (keyboard_unity_servo
# or xbox_servo publishing commands). When control is inactive it only monitors
# and logs.
#
# Anti-storm: a resend fires only when the program has been false for
# false_debounce_s, no sooner than resend_cooldown_s after the previous resend,
# and at most max_consecutive_resends times before backing off to periodic
# warnings (cleared once the program returns to true).

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class RobotProgramWatchdogROS2Unity(Node):
    """Resend the UR robot program when it drops while teleop control is active."""

    def __init__(self) -> None:
        super().__init__("robot_program_watchdog")

        self.declare_parameter(
            "program_running_topic",
            "/io_and_status_controller/robot_program_running",
        )
        self.declare_parameter(
            "resend_service",
            "/io_and_status_controller/resend_robot_program",
        )
        self.declare_parameter("control_gate_topic", "/teleop/command")
        self.declare_parameter("gate_min_publishers", 1)
        self.declare_parameter("gate_debounce_s", 1.0)
        self.declare_parameter("require_control_active", True)
        self.declare_parameter("false_debounce_s", 1.5)
        self.declare_parameter("resend_cooldown_s", 5.0)
        self.declare_parameter("max_consecutive_resends", 5)
        self.declare_parameter("check_period_s", 1.0)
        self.declare_parameter("service_wait_timeout_s", 0.5)
        self.declare_parameter("enable", True)

        self.program_running_topic = self.get_parameter(
            "program_running_topic"
        ).value
        self.resend_service = self.get_parameter("resend_service").value
        self.control_gate_topic = self.get_parameter(
            "control_gate_topic"
        ).value
        self.gate_min_publishers = int(
            self.get_parameter("gate_min_publishers").value
        )
        self.gate_debounce_s = float(
            self.get_parameter("gate_debounce_s").value
        )
        self.require_control_active = bool(
            self.get_parameter("require_control_active").value
        )
        self.false_debounce_s = float(
            self.get_parameter("false_debounce_s").value
        )
        self.resend_cooldown_s = float(
            self.get_parameter("resend_cooldown_s").value
        )
        self.max_consecutive_resends = int(
            self.get_parameter("max_consecutive_resends").value
        )
        self.check_period_s = max(
            float(self.get_parameter("check_period_s").value), 0.1
        )
        self.service_wait_timeout_s = float(
            self.get_parameter("service_wait_timeout_s").value
        )
        self.enable = bool(self.get_parameter("enable").value)

        # Program state, latched from the broadcaster.
        self.last_running: bool | None = None
        self.false_since = None
        # Resend bookkeeping.
        self.last_resend_time = None
        self.consecutive_resends = 0
        self._resend_in_flight = False
        # Control gate state (debounced rising edge).
        self.control_active = False
        self._gate_active_since = None
        self._prev_control_active = False
        self._backoff_warned = False

        # robot_program_running is RELIABLE + TRANSIENT_LOCAL (latched); match it
        # so the latest value arrives on startup even without a fresh change.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool,
            self.program_running_topic,
            self.on_program_running,
            latched_qos,
        )
        self.resend_client = self.create_client(Trigger, self.resend_service)
        self.create_timer(self.check_period_s, self.check)

        self.get_logger().info(
            "watching %s; resending via %s"
            % (self.program_running_topic, self.resend_service)
        )
        self.get_logger().info(
            "control gate: %s (>= %d publishers), require_control_active=%s"
            % (
                self.control_gate_topic,
                self.gate_min_publishers,
                self.require_control_active,
            )
        )
        if not self.enable:
            self.get_logger().warning(
                "auto-resend disabled (enable=false); monitoring only"
            )

    def on_program_running(self, msg: Bool) -> None:
        """Cache the latest program-running state and track when it went false."""
        running = bool(msg.data)
        if running != self.last_running:
            self.get_logger().info(
                "robot_program_running -> %s" % running
            )
        if running:
            self.false_since = None
            if self.consecutive_resends:
                self.get_logger().info(
                    "robot program is back; clearing resend backoff"
                )
            self.consecutive_resends = 0
            self._backoff_warned = False
        elif self.false_since is None:
            self.false_since = self.get_clock().now()
        self.last_running = running

    def check(self) -> None:
        """Evaluate the control gate and resend the program if it has dropped."""
        control_active = self._evaluate_control_gate()
        if control_active and not self._prev_control_active:
            self.get_logger().info(
                "control stack online; watchdog active"
            )
        elif not control_active and self._prev_control_active:
            self.get_logger().info(
                "control stack offline; watchdog standing by"
            )
        self._prev_control_active = control_active
        self.control_active = control_active

        if not self.enable:
            return

        if self.require_control_active and not control_active:
            # Monitor only: do not revive external control unsolicited.
            return

        if self.last_running is None:
            return
        if self.last_running:
            return
        if self._resend_in_flight:
            return

        now = self.get_clock().now()
        if self.false_since is None:
            self.false_since = now
            return
        if (now - self.false_since) < Duration(seconds=self.false_debounce_s):
            return
        if self.last_resend_time is not None and (
            now - self.last_resend_time
        ) < Duration(seconds=self.resend_cooldown_s):
            return
        if self.consecutive_resends >= self.max_consecutive_resends:
            if not self._backoff_warned:
                self.get_logger().error(
                    "robot program still down after %d resends; backing off, "
                    "check robot power/safety/remote-control"
                    % self.consecutive_resends
                )
                self._backoff_warned = True
            return

        self._send_resend(now)

    def _evaluate_control_gate(self) -> bool:
        """Return whether the control stack is online (debounced rising edge)."""
        raw_active = (
            self.count_publishers(self.control_gate_topic)
            >= self.gate_min_publishers
        )
        now = self.get_clock().now()
        if not raw_active:
            self._gate_active_since = None
            return False
        if self._gate_active_since is None:
            self._gate_active_since = now
        return (now - self._gate_active_since) >= Duration(
            seconds=self.gate_debounce_s
        )

    def _send_resend(self, now) -> None:
        """Call resend_robot_program asynchronously and record the attempt."""
        if not self.resend_client.wait_for_service(
            timeout_sec=self.service_wait_timeout_s
        ):
            self.get_logger().warning(
                "%s service is not available" % self.resend_service
            )
            return

        self.last_resend_time = now
        self.consecutive_resends += 1
        self._resend_in_flight = True
        self.get_logger().warning(
            "robot program down while control active; resending (#%d)"
            % self.consecutive_resends
        )
        future = self.resend_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_resend_response)

    def _on_resend_response(self, future) -> None:
        self._resend_in_flight = False
        response = future.result()
        if response is None:
            self.get_logger().warning("resend_robot_program call failed")
            return
        if not response.success:
            self.get_logger().warning(
                "resend_robot_program returned false: %s" % response.message
            )
            return
        self.get_logger().info(
            "resend_robot_program ok: %s" % response.message
        )


def main(args=None) -> None:
    """Run the robot program watchdog node."""
    rclpy.init(args=args)
    node = RobotProgramWatchdogROS2Unity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
