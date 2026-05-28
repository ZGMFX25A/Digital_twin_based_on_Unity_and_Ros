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
            }
        ],
    )

    return LaunchDescription([
        ros_ip_arg,
        ros_tcp_port_arg,
        manager_node,
    ])
