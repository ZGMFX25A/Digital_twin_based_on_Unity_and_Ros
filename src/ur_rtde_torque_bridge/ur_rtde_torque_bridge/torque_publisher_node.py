#!/usr/bin/env python3
"""ROS node shell: publishes joint torques on the intermediate /joint_torques.

This is the only file in the package that touches ROS. It owns a TorqueSource
(the RTDE implementation by default), keeps it connected, and republishes the
torques as a standard sensor_msgs/JointState (effort = N.m) so any consumer can
subscribe with zero dependency on this project's custom interfaces.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .rtde_torque_source import RtdeTorqueSource


class TorquePublisherNode(Node):
    def __init__(self):
        super().__init__("ur_rtde_torque_publisher")

        self.declare_parameter("robot_ip", "127.0.0.1")
        self.declare_parameter("rtde_port", 30004)
        self.declare_parameter("rtde_field", "actual_current_as_torque")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("output_topic", "/joint_torques")

        robot_ip = self.get_parameter("robot_ip").value
        rtde_port = self.get_parameter("rtde_port").value
        rtde_field = self.get_parameter("rtde_field").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        output_topic = self.get_parameter("output_topic").value

        if publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz must be > 0. Falling back to 20 Hz."
            )
            publish_rate_hz = 20.0

        self.source = RtdeTorqueSource(
            robot_ip=robot_ip,
            rtde_field=rtde_field,
            rtde_port=rtde_port,
            frequency=publish_rate_hz,
            logger=self.get_logger(),
        )

        self.pub = self.create_publisher(JointState, output_topic, 10)

        self.get_logger().info(
            f"Publishing joint torques (N.m) on {output_topic} "
            f"from RTDE field '{rtde_field}' at {robot_ip}:{rtde_port}."
        )

        # Publishing runs at the requested rate; reconnection is attempted on a
        # slower, separate timer so a down robot does not spam connect attempts
        # (and their warnings) at the publish rate.
        self.create_timer(1.0 / publish_rate_hz, self.publish_torques)
        self.create_timer(2.0, self.ensure_connected)

    def ensure_connected(self):
        # connect() logs a specific reason on failure (port unreachable,
        # ur_rtde missing, ...); the 2 s reconnect timer keeps it from spamming.
        if not self.source.is_connected():
            self.source.connect()

    def publish_torques(self):
        names, torques = self.source.get_torques()
        if torques is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.effort = torques
        self.pub.publish(msg)

    def destroy_node(self):
        self.source.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TorquePublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
