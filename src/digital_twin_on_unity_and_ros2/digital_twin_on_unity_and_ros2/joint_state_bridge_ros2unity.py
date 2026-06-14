#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from digital_twin_interfaces.msg import JointStateUnity


class JointStateBridgeROS2Unity(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")

        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/unity/joint_states")

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
            self.joint_state_callback,
            10,
        )

        self.pub = self.create_publisher(
            JointStateUnity,
            output_topic,
            10,
        )

        self.get_logger().info(f"Subscribing: {input_topic}")
        self.get_logger().info(f"Publishing : {output_topic}")
        self.get_logger().info("Joint positions will be converted from radians to degrees.")

    def joint_state_callback(self, msg: JointState):
        position_map = {}
        velocity_map = {}
        effort_map = {}

        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                position_map[joint_name] = msg.position[i]

            if i < len(msg.velocity):
                velocity_map[joint_name] = msg.velocity[i]

            # effort is UR `actual_current` (amperes), passed through as-is.
            if i < len(msg.effort):
                effort_map[joint_name] = msg.effort[i]

        output_msg = JointStateUnity()
        output_msg.names = []
        output_msg.positions = []
        output_msg.velocities = []
        output_msg.efforts = []

        for joint_name in self.joint_order:
            if joint_name not in position_map:
                self.get_logger().warn(
                    f"Missing joint: {joint_name}",
                    throttle_duration_sec=2.0,
                )
                return

            position_rad = position_map[joint_name]
            position_deg = math.degrees(position_rad)

            velocity_rad = velocity_map.get(joint_name, 0.0)
            velocity_deg = math.degrees(velocity_rad)

            # Missing effort defaults to 0.0 (current in amperes, not torque).
            effort = effort_map.get(joint_name, 0.0)

            output_msg.names.append(joint_name)
            output_msg.positions.append(position_deg)
            output_msg.velocities.append(velocity_deg)
            output_msg.efforts.append(effort)

        self.pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
