from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="0.0.0.0",
        description="ROS_IP value passed to ros_tcp_endpoint.",
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port",
        default_value="10000",
        description="ROS_TCP_PORT value passed to ros_tcp_endpoint.",
    )
    # robot_ip is the RTDE target the torque publisher connects OUT to. It is a
    # different IP from ros_ip (the endpoint bind interface Unity connects IN
    # to); the two are commonly confused, so keep them as distinct arguments.
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="UR/URSim RTDE target IP that the ur_rtde torque publisher "
                    "connects OUT to. Distinct from ros_ip, which is the "
                    "endpoint bind interface Unity connects IN to.",
    )
    rtde_port_arg = DeclareLaunchArgument(
        "rtde_port",
        default_value="30004",
        description="RTDE port on the robot used by the torque publisher.",
    )
    rtde_field_arg = DeclareLaunchArgument(
        "rtde_field",
        default_value="actual_current_as_torque",
        description="RTDE field the torque publisher reads as joint torque.",
    )
    joint_torque_publish_rate_hz_arg = DeclareLaunchArgument(
        "joint_torque_publish_rate_hz",
        default_value="20.0",
        description="Publish rate (Hz) of the ur_rtde torque publisher.",
    )
    joint_torques_topic_arg = DeclareLaunchArgument(
        "joint_torques_topic",
        default_value="/joint_torques",
        description="Output topic of the ur_rtde torque publisher.",
    )

    manager_node = Node(
        package="digital_twin_on_unity_and_ros2",
        executable="digital_twin_manager",
        name="digital_twin_manager",
        output="screen",
        parameters=[
            {
                "ros_ip": LaunchConfiguration("ros_ip"),
                "ros_tcp_port": LaunchConfiguration("ros_tcp_port"),
                "respawn_permanent_nodes": True,
                "robot_ip": LaunchConfiguration("robot_ip"),
                "rtde_port": LaunchConfiguration("rtde_port"),
                "rtde_field": LaunchConfiguration("rtde_field"),
                "joint_torque_publish_rate_hz": LaunchConfiguration(
                    "joint_torque_publish_rate_hz"
                ),
                "joint_torques_topic": LaunchConfiguration(
                    "joint_torques_topic"
                ),
            }
        ],
    )

    return LaunchDescription([
        ros_ip_arg,
        ros_tcp_port_arg,
        robot_ip_arg,
        rtde_port_arg,
        rtde_field_arg,
        joint_torque_publish_rate_hz_arg,
        joint_torques_topic_arg,
        manager_node,
    ])
