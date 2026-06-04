#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from digital_twin_interfaces.msg import (
    RobotStatusUnity,
    TcpPoseUnity,
    WrenchUnity,
)


class URStateBridgeROS2Unity(Node):
    def __init__(self):
        super().__init__("ur_state_bridge")

        self.declare_parameter("tcp_pose_topic", "/tcp_pose_broadcaster/pose")
        self.declare_parameter(
            "robot_mode_topic",
            "/io_and_status_controller/robot_mode",
        )
        self.declare_parameter(
            "safety_mode_topic",
            "/io_and_status_controller/safety_mode",
        )
        self.declare_parameter(
            "robot_program_running_topic",
            "/io_and_status_controller/robot_program_running",
        )
        self.declare_parameter(
            "speed_scaling_topic",
            "/speed_scaling_state_broadcaster/speed_scaling",
        )
        self.declare_parameter(
            "wrench_topic",
            "/force_torque_sensor_broadcaster/wrench",
        )
        self.declare_parameter("tcp_pose_output_topic", "/unity/tcp_pose")
        self.declare_parameter(
            "robot_status_output_topic",
            "/unity/robot_status",
        )
        self.declare_parameter("wrench_output_topic", "/unity/wrench")
        self.declare_parameter("tcp_pose_publish_rate_hz", 30.0)
        self.declare_parameter("robot_status_publish_rate_hz", 10.0)
        self.declare_parameter("wrench_publish_rate_hz", 30.0)

        self.latest_tcp_pose = None
        self.latest_wrench = None
        self.robot_mode = RobotMode.NO_CONTROLLER
        self.safety_mode = 0
        self.robot_program_running = False
        self.robot_program_running_received = False
        self.speed_scaling = 0.0

        tcp_pose_topic = self.get_parameter("tcp_pose_topic").value
        robot_mode_topic = self.get_parameter("robot_mode_topic").value
        safety_mode_topic = self.get_parameter("safety_mode_topic").value
        robot_program_running_topic = self.get_parameter(
            "robot_program_running_topic"
        ).value
        speed_scaling_topic = self.get_parameter("speed_scaling_topic").value
        wrench_topic = self.get_parameter("wrench_topic").value

        tcp_pose_output_topic = self.get_parameter(
            "tcp_pose_output_topic"
        ).value
        robot_status_output_topic = self.get_parameter(
            "robot_status_output_topic"
        ).value
        wrench_output_topic = self.get_parameter("wrench_output_topic").value
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PoseStamped,
            tcp_pose_topic,
            self.tcp_pose_callback,
            state_qos,
        )
        self.create_subscription(
            RobotMode,
            robot_mode_topic,
            self.robot_mode_callback,
            state_qos,
        )
        self.create_subscription(
            SafetyMode,
            safety_mode_topic,
            self.safety_mode_callback,
            state_qos,
        )
        self.create_subscription(
            Bool,
            robot_program_running_topic,
            self.robot_program_running_callback,
            state_qos,
        )
        self.create_subscription(
            Float64,
            speed_scaling_topic,
            self.speed_scaling_callback,
            state_qos,
        )
        self.create_subscription(
            WrenchStamped,
            wrench_topic,
            self.wrench_callback,
            state_qos,
        )

        self.tcp_pose_pub = self.create_publisher(
            TcpPoseUnity,
            tcp_pose_output_topic,
            10,
        )
        self.robot_status_pub = self.create_publisher(
            RobotStatusUnity,
            robot_status_output_topic,
            10,
        )
        self.wrench_pub = self.create_publisher(
            WrenchUnity,
            wrench_output_topic,
            10,
        )

        self.create_timer(
            self._period_from_rate("tcp_pose_publish_rate_hz"),
            self.publish_tcp_pose,
        )
        self.create_timer(
            self._period_from_rate("robot_status_publish_rate_hz"),
            self.publish_robot_status,
        )
        self.create_timer(
            self._period_from_rate("wrench_publish_rate_hz"),
            self.publish_wrench,
        )

        self.get_logger().info(f"Subscribing TCP pose: {tcp_pose_topic}")
        self.get_logger().info(f"Subscribing robot mode: {robot_mode_topic}")
        self.get_logger().info(f"Subscribing safety mode: {safety_mode_topic}")
        self.get_logger().info(
            "Subscribing robot program running: "
            f"{robot_program_running_topic}"
        )
        self.get_logger().info(
            f"Subscribing speed scaling: {speed_scaling_topic}"
        )
        self.get_logger().info(f"Subscribing wrench: {wrench_topic}")
        self.get_logger().info(f"Publishing TCP pose: {tcp_pose_output_topic}")
        self.get_logger().info(
            f"Publishing robot status: {robot_status_output_topic}"
        )
        self.get_logger().info(f"Publishing wrench: {wrench_output_topic}")

    def _period_from_rate(self, parameter_name):
        rate = self.get_parameter(parameter_name).value
        if rate <= 0.0:
            self.get_logger().warn(
                f"{parameter_name} must be > 0. Falling back to 1 Hz."
            )
            rate = 1.0
        return 1.0 / rate

    def tcp_pose_callback(self, msg):
        self.latest_tcp_pose = msg

    def robot_mode_callback(self, msg):
        self.robot_mode = msg.mode

    def safety_mode_callback(self, msg):
        self.safety_mode = msg.mode

    def robot_program_running_callback(self, msg):
        self.robot_program_running = msg.data
        self.robot_program_running_received = True

    def speed_scaling_callback(self, msg):
        self.speed_scaling = msg.data

    def wrench_callback(self, msg):
        self.latest_wrench = msg

    def publish_tcp_pose(self):
        output_msg = TcpPoseUnity()

        if self.latest_tcp_pose is None:
            output_msg.stamp = self.get_clock().now().to_msg()
            output_msg.frame_id = ""
            output_msg.orientation_w = 1.0
            self.get_logger().warn(
                "No TCP pose received yet; publishing default TCP pose.",
                throttle_duration_sec=5.0,
            )
        else:
            pose_msg = self.latest_tcp_pose
            output_msg.stamp = pose_msg.header.stamp
            output_msg.frame_id = pose_msg.header.frame_id
            output_msg.position_x = pose_msg.pose.position.x
            output_msg.position_y = pose_msg.pose.position.y
            output_msg.position_z = pose_msg.pose.position.z
            output_msg.orientation_x = pose_msg.pose.orientation.x
            output_msg.orientation_y = pose_msg.pose.orientation.y
            output_msg.orientation_z = pose_msg.pose.orientation.z
            output_msg.orientation_w = pose_msg.pose.orientation.w

        self.tcp_pose_pub.publish(output_msg)

    def publish_robot_status(self):
        output_msg = RobotStatusUnity()
        output_msg.stamp = self.get_clock().now().to_msg()
        output_msg.robot_mode = self.robot_mode
        output_msg.robot_mode_label = self._robot_mode_label(self.robot_mode)
        output_msg.safety_mode = self.safety_mode
        output_msg.safety_mode_label = self._safety_mode_label(
            self.safety_mode
        )
        output_msg.robot_program_running = self.robot_program_running
        output_msg.robot_program_running_received = (
            self.robot_program_running_received
        )
        output_msg.speed_scaling = self.speed_scaling

        if not self.robot_program_running_received:
            self.get_logger().warn(
                "No robot_program_running message received yet; "
                "publishing default false.",
                throttle_duration_sec=5.0,
            )

        self.robot_status_pub.publish(output_msg)

    def publish_wrench(self):
        output_msg = WrenchUnity()

        if self.latest_wrench is None:
            output_msg.stamp = self.get_clock().now().to_msg()
            output_msg.frame_id = ""
            self.get_logger().warn(
                "No wrench received yet; publishing default zero wrench.",
                throttle_duration_sec=5.0,
            )
        else:
            wrench_msg = self.latest_wrench
            output_msg.stamp = wrench_msg.header.stamp
            output_msg.frame_id = wrench_msg.header.frame_id
            output_msg.force_x = wrench_msg.wrench.force.x
            output_msg.force_y = wrench_msg.wrench.force.y
            output_msg.force_z = wrench_msg.wrench.force.z
            output_msg.torque_x = wrench_msg.wrench.torque.x
            output_msg.torque_y = wrench_msg.wrench.torque.y
            output_msg.torque_z = wrench_msg.wrench.torque.z

        self.wrench_pub.publish(output_msg)

    @staticmethod
    def _robot_mode_label(mode):
        labels = {
            RobotMode.NO_CONTROLLER: "NO_CONTROLLER",
            RobotMode.DISCONNECTED: "DISCONNECTED",
            RobotMode.CONFIRM_SAFETY: "CONFIRM_SAFETY",
            RobotMode.BOOTING: "BOOTING",
            RobotMode.POWER_OFF: "POWER_OFF",
            RobotMode.POWER_ON: "POWER_ON",
            RobotMode.IDLE: "IDLE",
            RobotMode.BACKDRIVE: "BACKDRIVE",
            RobotMode.RUNNING: "RUNNING",
            RobotMode.UPDATING_FIRMWARE: "UPDATING_FIRMWARE",
        }
        return labels.get(mode, "UNKNOWN")

    @staticmethod
    def _safety_mode_label(mode):
        labels = {
            SafetyMode.NORMAL: "NORMAL",
            SafetyMode.REDUCED: "REDUCED",
            SafetyMode.PROTECTIVE_STOP: "PROTECTIVE_STOP",
            SafetyMode.RECOVERY: "RECOVERY",
            SafetyMode.SAFEGUARD_STOP: "SAFEGUARD_STOP",
            SafetyMode.SYSTEM_EMERGENCY_STOP: "SYSTEM_EMERGENCY_STOP",
            SafetyMode.ROBOT_EMERGENCY_STOP: "ROBOT_EMERGENCY_STOP",
            SafetyMode.VIOLATION: "VIOLATION",
            SafetyMode.FAULT: "FAULT",
            SafetyMode.VALIDATE_JOINT_ID: "VALIDATE_JOINT_ID",
            SafetyMode.UNDEFINED_SAFETY_MODE: "UNDEFINED_SAFETY_MODE",
            SafetyMode.AUTOMATIC_MODE_SAFEGUARD_STOP:
                "AUTOMATIC_MODE_SAFEGUARD_STOP",
            SafetyMode.SYSTEM_THREE_POSITION_ENABLING_STOP:
                "SYSTEM_THREE_POSITION_ENABLING_STOP",
        }
        return labels.get(mode, "UNKNOWN")


def main(args=None):
    rclpy.init(args=args)
    node = URStateBridgeROS2Unity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
