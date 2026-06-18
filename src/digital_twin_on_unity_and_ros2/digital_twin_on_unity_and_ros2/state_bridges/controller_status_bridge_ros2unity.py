#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from digital_twin_interfaces.msg import ControllerStatusUnity


class ControllerStatusBridgeROS2Unity(Node):
    """Unity forwarding layer for ros2_control controller states.

    Periodically calls /controller_manager/list_controllers and republishes
    the controller names and lifecycle states as ControllerStatusUnity on
    /unity/controller_status, so Unity can show which controllers are active.
    Read-only: it queries, it never switches controllers.
    """

    def __init__(self):
        super().__init__("controller_status_bridge")

        self.declare_parameter(
            "service_name", "/controller_manager/list_controllers"
        )
        self.declare_parameter("output_topic", "/unity/controller_status")
        self.declare_parameter("poll_rate_hz", 2.0)

        service_name = self.get_parameter("service_name").value
        output_topic = self.get_parameter("output_topic").value
        poll_rate = float(self.get_parameter("poll_rate_hz").value)

        self.client = self.create_client(ListControllers, service_name)
        self.pub = self.create_publisher(
            ControllerStatusUnity, output_topic, 10
        )
        self._pending = False
        self.timer = self.create_timer(
            1.0 / max(poll_rate, 0.1), self.poll_controllers
        )

        self.get_logger().info(f"Calling   : {service_name}")
        self.get_logger().info(f"Publishing : {output_topic}")

    def poll_controllers(self):
        if self._pending:
            return
        if not self.client.service_is_ready():
            self.get_logger().warn(
                "controller_manager list_controllers not available yet",
                throttle_duration_sec=5.0,
            )
            return
        self._pending = True
        future = self.client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"list_controllers call failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        out = ControllerStatusUnity()
        out.stamp = self.get_clock().now().to_msg()
        out.names = [c.name for c in response.controller]
        out.states = [c.state for c in response.controller]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerStatusBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
