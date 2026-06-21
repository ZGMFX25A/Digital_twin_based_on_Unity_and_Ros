import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("digital_twin_on_unity_and_ros2")
    default_params = os.path.join(
        package_dir, "config", "keyboard_unity_servo.yaml"
    )

    # The control stack is intentionally separate from the always-on observation
    # stack (digital_twin_manager): it is the first place this ws WRITES into the
    # teleop command pipeline, so the operator launches it explicitly.
    keyboard_params_arg = DeclareLaunchArgument(
        "keyboard_unity_servo_params",
        default_value=default_params,
        description="Parameter file for keyboard_unity_servo.",
    )

    keyboard_unity_servo_node = Node(
        package="digital_twin_on_unity_and_ros2",
        executable="keyboard_unity_servo",
        name="keyboard_unity_servo",
        output="screen",
        parameters=[LaunchConfiguration("keyboard_unity_servo_params")],
    )
    teleop_enable_bridge_node = Node(
        package="digital_twin_on_unity_and_ros2",
        executable="teleop_enable_bridge",
        name="teleop_enable_bridge",
        output="screen",
    )

    return LaunchDescription([
        keyboard_params_arg,
        keyboard_unity_servo_node,
        teleop_enable_bridge_node,
    ])
