#!/usr/bin/env python3

import os
import signal
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.node import Node


@dataclass
class ManagedProcess:
    name: str
    command: List[str]
    process: subprocess.Popen = field(init=False, default=None)


class DigitalTwinManager(Node):
    """Supervisor node for the Unity digital twin ROS side processes."""

    def __init__(self):
        super().__init__("digital_twin_manager")

        # Bind all interfaces so Unity on the Windows host (WSL2 has a separate
        # NAT IP) can reach the endpoint. 127.0.0.1 would only accept local
        # connections. The launch file passes 0.0.0.0 too.
        self.declare_parameter("ros_ip", "0.0.0.0")
        self.declare_parameter("ros_tcp_port", 10000)
        self.declare_parameter("respawn_permanent_nodes", True)

        # RTDE target the torque publisher connects OUT to. This is a different
        # IP from ros_ip (the endpoint bind interface Unity connects IN to).
        self.declare_parameter("robot_ip", "127.0.0.1")
        self.declare_parameter("rtde_port", 30004)
        self.declare_parameter("rtde_field", "actual_current_as_torque")
        self.declare_parameter("joint_torque_publish_rate_hz", 20.0)
        self.declare_parameter("joint_torques_topic", "/joint_torques")

        self.ros_ip = (
            self.get_parameter("ros_ip").get_parameter_value().string_value
        )
        self.ros_tcp_port = (
            self.get_parameter("ros_tcp_port")
            .get_parameter_value()
            .integer_value
        )
        self.respawn_permanent_nodes = (
            self.get_parameter("respawn_permanent_nodes")
            .get_parameter_value()
            .bool_value
        )
        self.robot_ip = (
            self.get_parameter("robot_ip").get_parameter_value().string_value
        )
        self.rtde_port = (
            self.get_parameter("rtde_port")
            .get_parameter_value()
            .integer_value
        )
        self.rtde_field = (
            self.get_parameter("rtde_field")
            .get_parameter_value()
            .string_value
        )
        self.joint_torque_publish_rate_hz = (
            self.get_parameter("joint_torque_publish_rate_hz")
            .get_parameter_value()
            .double_value
        )
        self.joint_torques_topic = (
            self.get_parameter("joint_torques_topic")
            .get_parameter_value()
            .string_value
        )

        self.permanent_nodes = self._build_permanent_nodes()
        self.controlled_nodes: Dict[str, ManagedProcess] = {}

        self._shutting_down = False
        self._monitor_timer = self.create_timer(
            2.0,
            self._monitor_permanent_nodes,
        )

        self.get_logger().info("Starting permanent digital twin nodes.")
        self.start_permanent_nodes()

    def _build_permanent_nodes(self) -> Dict[str, ManagedProcess]:
        return {
            "joint_state_bridge_ros2unity": ManagedProcess(
                name="joint_state_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "joint_state_bridge_ros2unity",
                ],
            ),
            # Producer of /joint_torques; started before its consumer
            # (joint_torque_bridge_ros2unity). The publisher self-reconnects
            # every 2s when the robot is offline, so it stays up without
            # crashing the manager.
            "ur_rtde_torque_publisher": ManagedProcess(
                name="ur_rtde_torque_publisher",
                command=[
                    "ros2",
                    "run",
                    "ur_rtde_torque_bridge",
                    "torque_publisher",
                    "--ros-args",
                    "-p",
                    f"robot_ip:={self.robot_ip}",
                    "-p",
                    f"rtde_port:={self.rtde_port}",
                    "-p",
                    f"rtde_field:={self.rtde_field}",
                    "-p",
                    f"publish_rate_hz:={self.joint_torque_publish_rate_hz}",
                    "-p",
                    f"output_topic:={self.joint_torques_topic}",
                ],
            ),
            "joint_torque_bridge_ros2unity": ManagedProcess(
                name="joint_torque_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "joint_torque_bridge_ros2unity",
                ],
            ),
            "ur_state_bridge_ros2unity": ManagedProcess(
                name="ur_state_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "ur_state_bridge_ros2unity",
                ],
            ),
            "io_states_bridge_ros2unity": ManagedProcess(
                name="io_states_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "io_states_bridge_ros2unity",
                ],
            ),
            "tool_data_bridge_ros2unity": ManagedProcess(
                name="tool_data_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "tool_data_bridge_ros2unity",
                ],
            ),
            "controller_status_bridge_ros2unity": ManagedProcess(
                name="controller_status_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "controller_status_bridge_ros2unity",
                ],
            ),
            "robot_info_bridge_ros2unity": ManagedProcess(
                name="robot_info_bridge_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "robot_info_bridge_ros2unity",
                ],
            ),
            "command_supervisor_ros2unity": ManagedProcess(
                name="command_supervisor_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "command_supervisor_ros2unity",
                ],
            ),
            "twist_display_conditioner_ros2unity": ManagedProcess(
                name="twist_display_conditioner_ros2unity",
                command=[
                    "ros2",
                    "run",
                    "digital_twin_on_unity_and_ros2",
                    "twist_display_conditioner_ros2unity",
                ],
            ),
            "ros_tcp_endpoint": ManagedProcess(
                name="ros_tcp_endpoint",
                command=[
                    "ros2",
                    "run",
                    "ros_tcp_endpoint",
                    "default_server_endpoint",
                    "--ros-args",
                    "-p",
                    f"ROS_IP:={self.ros_ip}",
                    "-p",
                    f"ROS_TCP_PORT:={self.ros_tcp_port}",
                ],
            ),
        }

    def start_permanent_nodes(self):
        for managed_process in self.permanent_nodes.values():
            self._start_process(managed_process)

    def _start_process(self, managed_process: ManagedProcess):
        if managed_process.process and managed_process.process.poll() is None:
            self.get_logger().debug(
                f"{managed_process.name} is already running."
            )
            return

        env = os.environ.copy()
        self.get_logger().info(
            f"Starting {managed_process.name}: "
            f"{' '.join(managed_process.command)}"
        )
        managed_process.process = subprocess.Popen(
            managed_process.command,
            env=env,
            start_new_session=True,
        )

    def _monitor_permanent_nodes(self):
        if self._shutting_down:
            return

        for managed_process in self.permanent_nodes.values():
            process = managed_process.process
            if process is None:
                continue

            return_code = process.poll()
            if return_code is None:
                continue

            self.get_logger().warn(
                f"{managed_process.name} exited with code {return_code}."
            )
            managed_process.process = None

            if self.respawn_permanent_nodes:
                self.get_logger().info(f"Respawning {managed_process.name}.")
                self._start_process(managed_process)

    def shutdown_processes(self):
        self._shutting_down = True

        all_processes = {
            **self.controlled_nodes,
            **self.permanent_nodes,
        }
        for managed_process in all_processes.values():
            self._stop_process(managed_process)

    def _stop_process(self, managed_process: ManagedProcess):
        process = managed_process.process
        if process is None or process.poll() is not None:
            return

        self.get_logger().info(f"Stopping {managed_process.name}.")
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"{managed_process.name} did not stop cleanly; killing it."
            )
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_processes()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
