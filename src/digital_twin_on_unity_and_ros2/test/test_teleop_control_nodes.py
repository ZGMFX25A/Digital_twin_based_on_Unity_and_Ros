"""Node-level tests for keyboard action dispatch and the enable bridge."""

# These construct the nodes (no ROS graph required) and mock the heavy service/
# action methods, so dispatch and service-forwarding are checked without a
# robot.

from unittest.mock import MagicMock

from digital_twin_interfaces.msg import TeleopKeysUnity
from digital_twin_on_unity_and_ros2.command_bridges import (
    keyboard_unity_servo_ros2unity as kbd,
)
from digital_twin_on_unity_and_ros2.command_bridges import (
    teleop_enable_bridge_ros2unity as enable_bridge,
)
import pytest
import rclpy
from std_msgs.msg import Bool
from teleop_msgs.msg import TeleopCommand
from teleop_msgs.srv import SetTeleopEnable


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def keyboard_node(ros_context):
    node = kbd.KeyboardUnityServoROS2Unity()
    yield node
    node.destroy_node()


@pytest.fixture
def enable_node(ros_context):
    node = enable_bridge.TeleopEnableBridgeROS2Unity()
    yield node
    node.destroy_node()


def _run_keys(node, held_keys="", action_key=""):
    node.on_keys(TeleopKeysUnity(held_keys=held_keys, action_key=action_key))
    node.process_pending_actions()


def test_space_action_publishes_idle(keyboard_node):
    keyboard_node.publish_command = MagicMock()
    _run_keys(keyboard_node, action_key="space")
    keyboard_node.publish_command.assert_called_once()
    assert keyboard_node.publish_command.call_args.kwargs["mode"] == (
        TeleopCommand.IDLE
    )


def test_action_keys_dispatch_to_their_methods(keyboard_node):
    keyboard_node.publish_home_pose = MagicMock()
    keyboard_node.toggle_motion_controller = MagicMock()
    keyboard_node.record_current_as_home = MagicMock()
    keyboard_node.record_current_as_zero = MagicMock()

    _run_keys(keyboard_node, action_key="h")
    _run_keys(keyboard_node, action_key="c")
    _run_keys(keyboard_node, action_key="r")
    _run_keys(keyboard_node, action_key="z")

    keyboard_node.publish_home_pose.assert_called_once()
    keyboard_node.toggle_motion_controller.assert_called_once()
    keyboard_node.record_current_as_home.assert_called_once()
    keyboard_node.record_current_as_zero.assert_called_once()


def test_stop_and_clear_action_clears_held_keys(keyboard_node):
    keyboard_node.publish_command = MagicMock()
    _run_keys(keyboard_node, held_keys="w", action_key="x")
    assert keyboard_node.held_keys == ""
    assert keyboard_node.publish_command.call_args.kwargs["mode"] == (
        TeleopCommand.IDLE
    )


def test_held_keys_drive_publish_tick(keyboard_node):
    keyboard_node.publish_command = MagicMock()
    keyboard_node.on_keys(TeleopKeysUnity(held_keys="w", action_key=""))
    keyboard_node.publish_tick()
    kwargs = keyboard_node.publish_command.call_args.kwargs
    assert kwargs["linear"][0] == keyboard_node.linear_step
    assert kwargs["log"] is False


def test_toggle_from_unknown_activates_trajectory_without_deactivation(
    keyboard_node,
):
    keyboard_node.get_controller_mode = MagicMock(
        return_value=kbd.ControllerMode.UNKNOWN
    )
    keyboard_node.stop_moveit_servo = MagicMock()
    keyboard_node.switch_controllers = MagicMock(return_value=True)

    keyboard_node.toggle_motion_controller()

    keyboard_node.switch_controllers.assert_called_once_with(
        activate=[keyboard_node.trajectory_controller],
        deactivate=[],
    )


def test_switch_to_servo_from_unknown_does_not_deactivate_trajectory(
    keyboard_node,
):
    keyboard_node.get_controller_mode = MagicMock(
        return_value=kbd.ControllerMode.UNKNOWN
    )
    keyboard_node.stop_moveit_servo = MagicMock()
    keyboard_node.current_joint_positions_for_servo = MagicMock(
        return_value=None
    )
    keyboard_node.switch_controllers = MagicMock(return_value=True)
    keyboard_node.reset_moveit_servo_status = MagicMock()
    keyboard_node.start_moveit_servo = MagicMock()

    assert keyboard_node.switch_to_servo_controller() is True
    keyboard_node.switch_controllers.assert_called_once_with(
        activate=[keyboard_node.servo_controller],
        deactivate=[],
    )


def _mock_enable_client(node):
    node.client.wait_for_service = MagicMock(return_value=True)
    node.client.call_async = MagicMock(return_value=MagicMock())
    return node.client.call_async


def test_enable_bridge_forwards_true(enable_node):
    call_async = _mock_enable_client(enable_node)
    enable_node.on_enable(Bool(data=True))
    call_async.assert_called_once()
    request = call_async.call_args.args[0]
    assert isinstance(request, SetTeleopEnable.Request)
    assert request.enable is True


def test_enable_bridge_forwards_false(enable_node):
    call_async = _mock_enable_client(enable_node)
    enable_node.on_enable(Bool(data=False))
    call_async.assert_called_once()
    assert call_async.call_args.args[0].enable is False
