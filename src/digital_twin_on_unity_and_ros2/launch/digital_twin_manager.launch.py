from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="127.0.0.1",
        description="ROS_IP value passed to ros_tcp_endpoint.",
    )

    manager_node = Node(
        package="digital_twin_on_unity_and_ros2",
        executable="digital_twin_manager",
        name="digital_twin_manager",
        output="screen",
        parameters=[
            {
                "ros_ip": LaunchConfiguration("ros_ip"),
                "respawn_permanent_nodes": True,
            }
        ],
    )

    return LaunchDescription([
        ros_ip_arg,
        manager_node,
    ])
