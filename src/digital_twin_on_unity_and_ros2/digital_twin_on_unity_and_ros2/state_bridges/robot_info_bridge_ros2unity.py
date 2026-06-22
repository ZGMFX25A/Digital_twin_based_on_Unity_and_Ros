#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_msgs.srv import GetRobotSoftwareVersion
from ur_dashboard_msgs.srv import GetLoadedProgram, GetProgramState
from digital_twin_interfaces.msg import RobotInfoUnity


class RobotInfoBridgeROS2Unity(Node):
    """Unity forwarding layer for low-rate UR informational state."""

    # Polls read-only UR services (software version, loaded program, program
    # state) at a low rate and republishes them as RobotInfoUnity on
    # /unity/robot_info. Kept separate from the high-rate /unity/robot_status.
    # Last-known values are retained on service timeout. Read-only.

    def __init__(self):
        super().__init__("robot_info_bridge")

        self.declare_parameter(
            "software_version_service",
            "/ur_configuration_controller/get_robot_software_version",
        )
        self.declare_parameter(
            "loaded_program_service", "/dashboard_client/get_loaded_program"
        )
        self.declare_parameter(
            "program_state_service", "/dashboard_client/program_state"
        )
        self.declare_parameter("output_topic", "/unity/robot_info")
        self.declare_parameter("poll_rate_hz", 1.0)

        output_topic = self.get_parameter("output_topic").value
        poll_rate = float(self.get_parameter("poll_rate_hz").value)

        self.software_version = "UNKNOWN"
        self.loaded_program = "UNKNOWN"
        self.program_state = "UNKNOWN"
        self._pending = {"version": False, "program": False, "state": False}

        self.version_client = self.create_client(
            GetRobotSoftwareVersion,
            self.get_parameter("software_version_service").value,
        )
        self.program_client = self.create_client(
            GetLoadedProgram,
            self.get_parameter("loaded_program_service").value,
        )
        self.state_client = self.create_client(
            GetProgramState, self.get_parameter("program_state_service").value
        )

        self.pub = self.create_publisher(RobotInfoUnity, output_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(poll_rate, 0.1), self.tick
        )

        self.get_logger().info(f"Publishing : {output_topic}")

    def tick(self):
        self.publish_current()
        self._refresh_version()
        self._refresh_program()
        self._refresh_state()

    def publish_current(self):
        msg = RobotInfoUnity()
        msg.stamp = self.get_clock().now().to_msg()
        msg.software_version = self.software_version
        msg.loaded_program = self.loaded_program
        msg.program_state = self.program_state
        self.pub.publish(msg)

    def _refresh_version(self):
        if self._pending["version"] or not self.version_client.service_is_ready():
            return
        self._pending["version"] = True
        future = self.version_client.call_async(
            GetRobotSoftwareVersion.Request()
        )

        def done(fut):
            self._pending["version"] = False
            try:
                r = fut.result()
                self.software_version = (
                    f"{r.major}.{r.minor}.{r.bugfix}-{r.build}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"version service failed: {exc}",
                    throttle_duration_sec=10.0,
                )

        future.add_done_callback(done)

    def _refresh_program(self):
        if self._pending["program"] or not self.program_client.service_is_ready():
            return
        self._pending["program"] = True
        future = self.program_client.call_async(GetLoadedProgram.Request())

        def done(fut):
            self._pending["program"] = False
            try:
                r = fut.result()
                self.loaded_program = r.program_name or "NONE"
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"loaded_program service failed: {exc}",
                    throttle_duration_sec=10.0,
                )

        future.add_done_callback(done)

    def _refresh_state(self):
        if self._pending["state"] or not self.state_client.service_is_ready():
            return
        self._pending["state"] = True
        future = self.state_client.call_async(GetProgramState.Request())

        def done(fut):
            self._pending["state"] = False
            try:
                r = fut.result()
                self.program_state = r.state.state or "UNKNOWN"
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"program_state service failed: {exc}",
                    throttle_duration_sec=10.0,
                )

        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoBridgeROS2Unity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
