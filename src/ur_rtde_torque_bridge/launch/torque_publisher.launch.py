from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="IP of the robot/URSim RTDE interface.",
    )
    rtde_port_arg = DeclareLaunchArgument(
        "rtde_port",
        default_value="30004",
        description="RTDE port (standard 30004).",
    )
    rtde_field_arg = DeclareLaunchArgument(
        "rtde_field",
        default_value="actual_current_as_torque",
        description="RTDE field to read: actual_current_as_torque (measured, "
                    "PolyScope >= 5.23/10.11), target_moment (commanded, all "
                    "versions), or actual_current (current in A, diagnostic).",
    )
    publish_rate_hz_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="20.0",
        description="Publish rate for /joint_torques in Hz.",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/joint_torques",
        description="Intermediate topic for joint torques "
                    "(sensor_msgs/JointState, effort = N.m).",
    )

    torque_publisher_node = Node(
        package="ur_rtde_torque_bridge",
        executable="torque_publisher",
        name="ur_rtde_torque_publisher",
        output="screen",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "rtde_port": LaunchConfiguration("rtde_port"),
                "rtde_field": LaunchConfiguration("rtde_field"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "output_topic": LaunchConfiguration("output_topic"),
            }
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        rtde_port_arg,
        rtde_field_arg,
        publish_rate_hz_arg,
        output_topic_arg,
        torque_publisher_node,
    ])
