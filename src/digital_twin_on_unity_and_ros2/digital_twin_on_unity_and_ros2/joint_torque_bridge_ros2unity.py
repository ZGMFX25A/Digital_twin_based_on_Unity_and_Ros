#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from digital_twin_interfaces.msg import JointTorqueUnity


class JointTorqueBridgeROS2Unity(Node):
    """Unity forwarding layer for joint torques.

    Subscribes to the intermediate /joint_torques topic (a standard
    sensor_msgs/JointState carrying effort in N.m, published by the
    pluggable data-source layer) and republishes it as JointTorqueUnity on
    /unity/joint_torques. This node does no abstraction on purpose: the whole
    message mapping lives in one callback, so adapting to a different upstream
    format only means editing joint_state_to_unity below.
    """

    def __init__(self):
        super().__init__("joint_torque_bridge")

        self.declare_parameter("input_topic", "/joint_torques")
        self.declare_parameter("output_topic", "/unity/joint_torques")

        self.joint_order = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.sub = self.create_subscription(
            JointState,
            input_topic,
            self.joint_torque_callback,
            10,
        )

        self.pub = self.create_publisher(
            JointTorqueUnity,
            output_topic,
            10,
        )

        self.get_logger().info(f"Subscribing: {input_topic}")
        self.get_logger().info(f"Publishing : {output_topic}")
        self.get_logger().info("Forwarding joint torques in N.m (effort field).")

    def joint_torque_callback(self, msg: JointState):
        # Single mapping point: sensor_msgs/JointState (effort = N.m) ->
        # JointTorqueUnity. If the upstream message format changes, only this
        # function needs editing.
        torque_map = {}
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.effort):
                torque_map[joint_name] = msg.effort[i]

        output_msg = JointTorqueUnity()
        output_msg.stamp = msg.header.stamp
        output_msg.names = []
        output_msg.torques_nm = []

        for joint_name in self.joint_order:
            if joint_name not in torque_map:
                self.get_logger().warn(
                    f"Missing joint torque: {joint_name}",
                    throttle_duration_sec=2.0,
                )
                return

            output_msg.names.append(joint_name)
            output_msg.torques_nm.append(torque_map[joint_name])

        self.pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTorqueBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
