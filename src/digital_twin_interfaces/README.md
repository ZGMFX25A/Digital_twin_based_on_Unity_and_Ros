# digital_twin_interfaces

Custom ROS 2 message definitions for the `/unity/*` topics. The bridge nodes in
`digital_twin_on_unity_and_ros2` already convert units and reorder joints, so the
Unity side must **not** convert again.

Unity generates these under the `RosMessageTypes.DigitalTwinInterfaces`
namespace; regenerate them in Unity (ROS-TCP-Connector → *Generate ROS Messages*)
whenever a `.msg` changes. The topic ↔ type ↔ source ↔ rate mapping is in the
[workspace README](../../README.md).

## Field reference

**`JointStateUnity`** — `names[]`, `positions[]` (**deg**), `velocities[]`
(**deg/s**), `efforts[]`. Positions/velocities are converted from rad and reordered
to the fixed UR7e joint sequence. `efforts` is the driver's `actual_current`
(motor **current in A**, *not* torque); real torque is on `/unity/joint_torques`.

**`JointTorqueUnity`** — `stamp`, `names[]`, `torques_nm[]` (**N·m**), in fixed
UR7e order. Sourced from `ur_rtde_torque_bridge`, which the manager now starts as a
permanent node (targeting `robot_ip`); zero/idle until that RTDE link reaches the
robot.

**`TcpPoseUnity`** — `stamp`, `frame_id`, `position_x/y/z` (**m**),
`orientation_x/y/z/w` (quaternion). Frame is `base`; ROS→Unity coordinate
conversion is done on the Unity side.

**`RobotStatusUnity`** — `stamp`, `robot_mode` (int8) + `robot_mode_label`,
`safety_mode` (uint8) + `safety_mode_label`, `robot_program_running` (bool) +
`robot_program_running_received` (bool, distinguishes "false" from "not yet
received"), `speed_scaling` (**0–100**, the driver already ×100 — do not multiply
again).

**`WrenchUnity`** — `stamp`, `frame_id`, `force_x/y/z` (**N**), `torque_x/y/z`
(**N·m**). Frame is `tool0_controller`.

**`IoStatesUnity`** — `stamp`, parallel arrays `digital_in_pins[]`/
`digital_in_states[]`, `digital_out_pins[]`/`digital_out_states[]`, and analog
`*_pins[]`/`*_values[]`/`*_domains[]` (domain: 0 = current, 1 = voltage).

**`ToolDataUnity`** — `stamp`, `tool_voltage_48v` (V), `tool_output_voltage` (V),
`tool_current` (A), `tool_temperature` (°C), `tool_mode` + `tool_mode_label`
(249 BOOTLOADER / 253 RUNNING / 255 IDLE).

**`ControllerStatusUnity`** — `stamp`, `names[]`, `states[]` (`active` /
`inactive`).

**`RobotInfoUnity`** — `stamp`, `software_version`, `loaded_program`,
`program_state` (`UNKNOWN` until first read).

**`CmdObservedUnity`** — observed teleop intent: `stamp`, `control_mode` +
`control_mode_label`, `enable`, `emergency_stop`, `command_frame`,
`joint_names[]`/`joint_values[]` (position or velocity per mode),
`twist_linear[3]`/`twist_angular[3]`.

**`CmdStatusUnity`** — observed teleop manager status: `stamp`, `active_mode` +
`active_mode_label`, `enabled`, `emergency_stop`, `safety_ok`, `input_alive`,
`command_timeout`, `command_age`, `error_code`, `last_stop_reason`, `message`.

**`TeleopKeysUnity`** — the one **Unity → ROS** message (published by Unity, not a
`/unity/*` state topic): `held_keys` (jog keys currently held, e.g. `"wq"`) and
`action_key` (one momentary action this frame: `""`, `space`, `h`, `c`, `r`, `z`,
`x`). Consumed by `keyboard_unity_servo` in the optional control stack.

## Coordinate frames

ROS is **FLU right-handed**; Unity is **left-handed RUF**. Position/vector mapping
on the Unity side is `(x, y, z)_ros → (-y, z, x)_unity`. Quaternion orientation is
left unconverted by default in the reference scripts — calibrate per use.
