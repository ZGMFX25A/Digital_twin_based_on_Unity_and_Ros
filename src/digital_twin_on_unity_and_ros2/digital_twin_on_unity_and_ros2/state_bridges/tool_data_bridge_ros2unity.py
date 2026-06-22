#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_msgs.msg import ToolDataMsg
from digital_twin_interfaces.msg import ToolDataUnity


class ToolDataBridgeROS2Unity(Node):
    """Unity forwarding layer for UR tool flange data."""

    # Subscribes to /io_and_status_controller/tool_data (ur_msgs/ToolDataMsg)
    # and republishes voltage/current/temperature/mode as ToolDataUnity on
    # /unity/tool_data, mapping tool_mode to a human label. Read-only.

    TOOL_MODE_LABELS = {
        249: "BOOTLOADER",
        253: "RUNNING",
        255: "IDLE",
    }

    def __init__(self):
        super().__init__("tool_data_bridge")

        self.declare_parameter(
            "input_topic", "/io_and_status_controller/tool_data"
        )
        self.declare_parameter("output_topic", "/unity/tool_data")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.sub = self.create_subscription(
            ToolDataMsg,
            input_topic,
            self.tool_data_callback,
            10,
        )
        self.pub = self.create_publisher(ToolDataUnity, output_topic, 10)

        self.get_logger().info(f"Subscribing: {input_topic}")
        self.get_logger().info(f"Publishing : {output_topic}")

    def tool_data_callback(self, msg: ToolDataMsg):
        out = ToolDataUnity()
        out.stamp = self.get_clock().now().to_msg()
        out.tool_voltage_48v = msg.tool_voltage_48v
        out.tool_output_voltage = msg.tool_output_voltage
        out.tool_current = msg.tool_current
        out.tool_temperature = msg.tool_temperature
        out.tool_mode = msg.tool_mode
        out.tool_mode_label = self.TOOL_MODE_LABELS.get(msg.tool_mode, "UNKNOWN")
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ToolDataBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
