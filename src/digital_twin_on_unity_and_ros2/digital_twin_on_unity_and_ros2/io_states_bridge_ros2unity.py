#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_msgs.msg import IOStates
from digital_twin_interfaces.msg import IoStatesUnity


class IoStatesBridgeROS2Unity(Node):
    """Unity forwarding layer for UR digital/analog IO.

    Subscribes to the driver's /io_and_status_controller/io_states
    (ur_msgs/IOStates, reliable/volatile) and republishes it as the flat
    IoStatesUnity message on /unity/io_states for an IO panel. Read-only:
    this never writes IO, it only mirrors state.
    """

    def __init__(self):
        super().__init__("io_states_bridge")

        self.declare_parameter(
            "input_topic", "/io_and_status_controller/io_states"
        )
        self.declare_parameter("output_topic", "/unity/io_states")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.sub = self.create_subscription(
            IOStates,
            input_topic,
            self.io_states_callback,
            10,
        )
        self.pub = self.create_publisher(IoStatesUnity, output_topic, 10)

        self.get_logger().info(f"Subscribing: {input_topic}")
        self.get_logger().info(f"Publishing : {output_topic}")

    def io_states_callback(self, msg: IOStates):
        out = IoStatesUnity()
        out.stamp = self.get_clock().now().to_msg()

        out.digital_in_pins = [d.pin for d in msg.digital_in_states]
        out.digital_in_states = [d.state for d in msg.digital_in_states]
        out.digital_out_pins = [d.pin for d in msg.digital_out_states]
        out.digital_out_states = [d.state for d in msg.digital_out_states]

        out.analog_in_pins = [a.pin for a in msg.analog_in_states]
        out.analog_in_values = [a.state for a in msg.analog_in_states]
        out.analog_in_domains = [a.domain for a in msg.analog_in_states]
        out.analog_out_pins = [a.pin for a in msg.analog_out_states]
        out.analog_out_values = [a.state for a in msg.analog_out_states]
        out.analog_out_domains = [a.domain for a in msg.analog_out_states]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IoStatesBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
