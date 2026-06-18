#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped
from teleop_msgs.msg import TeleopCommand, TeleopStatus
from digital_twin_interfaces.msg import CmdObservedUnity, CmdStatusUnity


MODE_LABELS = {
    0: "IDLE",
    1: "JOINT_POSITION",
    2: "JOINT_VELOCITY",
    3: "CARTESIAN_POSE",
    4: "CARTESIAN_TWIST",
    5: "GRIPPER",
}


class CommandSupervisorROS2Unity(Node):
    """Read-only observer of the teleoperation command stream.

    Subscribes to the teleop package's command/status/validated topics and
    mirrors them under /unity/cmd_observed/* for command-intent visualization
    in the digital twin. STRICTLY READ-ONLY: this node publishes ONLY on
    /unity/cmd_observed/* and never writes to /teleop/*, /servo_node/* or any
    controller. teleop_msgs is converted to *Unity so Unity needs no teleop_msgs.
    """

    def __init__(self):
        super().__init__("command_supervisor")

        # Inputs (teleop outputs); parameterized so a topic rename only edits params.
        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("status_topic", "/teleop/status")
        self.declare_parameter(
            "joint_velocity_topic", "/teleop/validated/joint_velocity"
        )
        self.declare_parameter(
            "joint_position_topic", "/teleop/validated/joint_position"
        )
        self.declare_parameter(
            "cartesian_pose_topic", "/teleop/validated/cartesian_pose"
        )
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")

        # Subscriptions (all read-only).
        self.create_subscription(
            TeleopCommand,
            self.get_parameter("command_topic").value,
            self.command_callback,
            10,
        )
        self.create_subscription(
            TeleopStatus,
            self.get_parameter("status_topic").value,
            self.status_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_velocity_topic").value,
            lambda m: self.joint_velocity_pub.publish(m),
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_position_topic").value,
            lambda m: self.joint_position_pub.publish(m),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("cartesian_pose_topic").value,
            lambda m: self.cartesian_pose_pub.publish(m),
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            lambda m: self.twist_pub.publish(m),
            10,
        )

        # Publishers: ONLY /unity/cmd_observed/*.
        self.command_pub = self.create_publisher(
            CmdObservedUnity, "/unity/cmd_observed/command", 10
        )
        self.status_pub = self.create_publisher(
            CmdStatusUnity, "/unity/cmd_observed/status", 10
        )
        self.joint_velocity_pub = self.create_publisher(
            JointState, "/unity/cmd_observed/joint_velocity", 10
        )
        self.joint_position_pub = self.create_publisher(
            JointState, "/unity/cmd_observed/joint_position", 10
        )
        self.cartesian_pose_pub = self.create_publisher(
            PoseStamped, "/unity/cmd_observed/cartesian_pose", 10
        )
        self.twist_pub = self.create_publisher(
            TwistStamped, "/unity/cmd_observed/twist", 10
        )

        self.get_logger().info(
            "Observing teleop command stream (read-only) -> /unity/cmd_observed/*"
        )

    def command_callback(self, msg: TeleopCommand):
        out = CmdObservedUnity()
        out.stamp = self.get_clock().now().to_msg()
        out.control_mode = msg.control_mode
        out.control_mode_label = MODE_LABELS.get(msg.control_mode, "UNKNOWN")
        out.enable = msg.enable
        out.emergency_stop = msg.emergency_stop
        out.command_frame = msg.command_frame

        out.joint_names = list(msg.joint_command.name)
        if msg.control_mode == TeleopCommand.JOINT_VELOCITY:
            out.joint_values = list(msg.joint_command.velocity)
        else:
            out.joint_values = list(msg.joint_command.position)

        twist = msg.target_twist.twist
        out.twist_linear = [twist.linear.x, twist.linear.y, twist.linear.z]
        out.twist_angular = [twist.angular.x, twist.angular.y, twist.angular.z]
        self.command_pub.publish(out)

    def status_callback(self, msg: TeleopStatus):
        out = CmdStatusUnity()
        out.stamp = self.get_clock().now().to_msg()
        out.active_mode = msg.active_mode
        out.active_mode_label = MODE_LABELS.get(msg.active_mode, "UNKNOWN")
        out.enabled = msg.enabled
        out.emergency_stop = msg.emergency_stop
        out.safety_ok = msg.safety_ok
        out.input_alive = msg.input_alive
        out.command_timeout = msg.command_timeout
        out.command_age = msg.command_age
        out.error_code = msg.error_code
        out.last_stop_reason = msg.last_stop_reason
        out.message = msg.message
        self.status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CommandSupervisorROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
