#!/usr/bin/env python3
"""Forward a Unity enable toggle to the teleop set_teleop_enable service."""

# Subscribes to /unity/teleop/enable (std_msgs/Bool) and calls
# /teleop/set_teleop_enable (teleop_msgs/SetTeleopEnable) so the Unity control
# switch lands an enable state in the teleop manager without Unity having to
# depend on teleop_msgs/srv. This bridge only forwards the toggle; the
# immediate-stop guarantee is provided by Unity's stop handshake on the input
# sources (see the plan's safety section).

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from teleop_msgs.srv import SetTeleopEnable


class TeleopEnableBridgeROS2Unity(Node):
    """Bridge /unity/teleop/enable (Bool) -> /teleop/set_teleop_enable."""

    def __init__(self) -> None:
        super().__init__("teleop_enable_bridge")

        self.declare_parameter("enable_topic", "/unity/teleop/enable")
        self.declare_parameter(
            "set_teleop_enable_service", "/teleop/set_teleop_enable"
        )
        self.declare_parameter("service_wait_timeout_s", 0.5)

        self.enable_topic = self.get_parameter("enable_topic").value
        self.service_name = self.get_parameter(
            "set_teleop_enable_service"
        ).value
        self.service_wait_timeout_s = float(
            self.get_parameter("service_wait_timeout_s").value
        )

        self.client = self.create_client(SetTeleopEnable, self.service_name)
        self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)
        self.get_logger().info(
            "forwarding %s -> %s" % (self.enable_topic, self.service_name)
        )

    def on_enable(self, msg: Bool) -> None:
        """Call set_teleop_enable with the requested enable state."""
        if not self.client.wait_for_service(
            timeout_sec=self.service_wait_timeout_s
        ):
            self.get_logger().warning(
                "%s service is not available" % self.service_name
            )
            return

        request = SetTeleopEnable.Request()
        request.enable = bool(msg.data)
        future = self.client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        response = future.result()
        if response is None:
            self.get_logger().warning("set_teleop_enable call failed")
            return
        if not response.success:
            self.get_logger().warning(
                "set_teleop_enable returned false: %s" % response.message
            )
            return
        self.get_logger().info(
            "set_teleop_enable ok: %s" % response.message
        )


def main(args=None) -> None:
    """Run the teleop enable bridge node."""
    rclpy.init(args=args)
    node = TeleopEnableBridgeROS2Unity()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
