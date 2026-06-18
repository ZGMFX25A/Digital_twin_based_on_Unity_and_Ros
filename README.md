# Digital Twin on Unity and ROS 2

ROS 2 backend for a Unity-based digital twin of the Universal Robots **UR7e**
collaborative robot. It subscribes to the UR ROS 2 driver, converts robot state
into Unity-friendly messages, and exposes them to Unity over the
ROS-TCP-Endpoint.

- **Stage:** read-only state synchronisation (no commands are sent to the robot).
- **Target platform:** ROS 2 Humble on Ubuntu 22.04.
- **Robot:** Universal Robots UR7e (e-Series), real or URSim.

---

## Contents

- [Overview](#overview)
- [Compatibility](#compatibility)
- [Architecture](#architecture)
- [Repository layout](#repository-layout)
- [Message interfaces](#message-interfaces)
- [Runtime nodes](#runtime-nodes)
- [Dependencies](#dependencies)
- [Build](#build)
- [Running the system](#running-the-system)
- [Connecting Unity (networking)](#connecting-unity-networking)
- [Unity integration](#unity-integration)
- [Development](#development)
- [License](#license)

---

## Overview

This workspace is the ROS-side half of a two-part system: ROS 2 on Linux/WSL
produces state, Unity on the host renders the twin.

**In scope (this stage):**

- Forward joint state, TCP pose, robot/safety mode, program state, speed scaling,
  and force/torque wrench to Unity as custom `/unity/*` messages.
- Forward IO state, tool-flange data, controller status, and robot info.
- *Observe* (read-only) the command stream of a separate teleoperation package and
  mirror it to `/unity/cmd_observed/*` for visualisation.

**Out of scope (by design):**

- No control commands are sent from Unity or this workspace to the robot. The
  architecture is strictly read-only; command/teleoperation lives in a separate
  workspace (`teleoperation_general_ros2`) which this workspace only observes.

Unity-side integration (C# scripts, scene, UI) is tracked separately; reference
scripts are provided under `plan/shared/script/`.

---

## Compatibility

| Component | Version / value |
| --- | --- |
| ROS 2 | Humble Hawksbill |
| OS | Ubuntu 22.04 (Jammy) |
| Python | 3.10 |
| Robot | UR7e (e-Series) |
| UR ROS 2 Driver | `humble` branch |
| Simulator | URSim e-Series (Docker) |
| PolyScope | ≥ 5.23 for measured joint torque (tested on 5.25.1) |
| Unity bridge | ROS-TCP-Endpoint `main-ros2` + ROS-TCP-Connector |

---

## Architecture

```text
URSim (Docker) / Real UR7e
  └─► Universal Robots ROS 2 Driver
        ├─ joint_state_broadcaster          → /joint_states
        ├─ tcp_pose_broadcaster             → /tcp_pose_broadcaster/pose
        ├─ io_and_status_controller         → robot_mode / safety_mode / program_running
        │                                     io_states / tool_data
        ├─ speed_scaling_state_broadcaster  → /speed_scaling_state_broadcaster/speed_scaling
        └─ force_torque_sensor_broadcaster  → /force_torque_sensor_broadcaster/wrench

  └─► This workspace (digital_twin_on_unity_and_ros2_ws)
        ├─ joint_state_bridge_ros2unity       → /unity/joint_states      (on-change)
        ├─ joint_torque_bridge_ros2unity      → /unity/joint_torques     (follows input)
        ├─ ur_state_bridge_ros2unity          → /unity/tcp_pose          (30 Hz)
        │                                       /unity/robot_status      (10 Hz)
        │                                       /unity/wrench            (30 Hz)
        ├─ io_states_bridge_ros2unity         → /unity/io_states
        ├─ tool_data_bridge_ros2unity         → /unity/tool_data
        ├─ controller_status_bridge_ros2unity → /unity/controller_status (2 Hz)
        ├─ robot_info_bridge_ros2unity        → /unity/robot_info        (~1 Hz)
        ├─ command_supervisor_ros2unity       → /unity/cmd_observed/*    (read-only)
        └─ ros_tcp_endpoint                   → TCP :10000 → Unity

  └─► Optional, pluggable (not started by the manager)
        ur_rtde_torque_bridge (RTDE receive-only) → /joint_torques (N·m)
              feeds joint_torque_bridge_ros2unity above

  └─► Separate workspace, observed read-only: teleoperation_general_ros2
        publishes /teleop/command, /teleop/validated/*, /teleop/status
              command_supervisor_ros2unity observes these → /unity/cmd_observed/*
```

The workspaces communicate only through the ROS graph. This workspace contains no
hardcoded path to the UR driver workspace, and never publishes to `/teleop/*` or
any controller command interface.

---

## Repository layout

```text
src/
  digital_twin_interfaces/         Custom ROS 2 message definitions
  digital_twin_on_unity_and_ros2/  Bridge nodes, manager node, launch files
    digital_twin_on_unity_and_ros2/
      state_bridges/               Robot-state → /unity/* bridges
      command_bridges/             Read-only teleop command observer
    manager/                       Supervisor node
    launch/
  ur_rtde_torque_bridge/           Optional, pluggable RTDE torque source (N·m)
  third_party/                     Vendored msg packages (see third_party/README.md)
    ur_dashboard_msgs/             RobotMode / SafetyMode / dashboard services
    ur_msgs/                       IOStates / ToolDataMsg / GetRobotSoftwareVersion
    teleop_msgs/                   TeleopCommand / TeleopStatus (observed)
  ROS-TCP-Endpoint/                Unity ROS-TCP-Endpoint (clone, git-ignored)
```

---

## Message interfaces

All custom messages live in `digital_twin_interfaces`. Unity generates them under
the `RosMessageTypes.DigitalTwinInterfaces` namespace; regenerate them in Unity
(ROS-TCP-Connector → *Generate ROS Messages*) whenever a `.msg` changes.

### `/unity/*` topics

| Topic | Type | Source | Rate |
| --- | --- | --- | --- |
| `/unity/joint_states` | `JointStateUnity` | `/joint_states` | on-change |
| `/unity/joint_torques` | `JointTorqueUnity` | `/joint_torques` (`ur_rtde_torque_bridge`) | follows input |
| `/unity/tcp_pose` | `TcpPoseUnity` | `/tcp_pose_broadcaster/pose` | 30 Hz |
| `/unity/robot_status` | `RobotStatusUnity` | robot/safety/program/speed topics | 10 Hz |
| `/unity/wrench` | `WrenchUnity` | `/force_torque_sensor_broadcaster/wrench` | 30 Hz |
| `/unity/io_states` | `IoStatesUnity` | `/io_and_status_controller/io_states` | follows input |
| `/unity/tool_data` | `ToolDataUnity` | `/io_and_status_controller/tool_data` | follows input |
| `/unity/controller_status` | `ControllerStatusUnity` | `/controller_manager/list_controllers` | 2 Hz |
| `/unity/robot_info` | `RobotInfoUnity` | UR read-only services | ~1 Hz |
| `/unity/cmd_observed/command` | `CmdObservedUnity` | `/teleop/command` (observed) | follows input |
| `/unity/cmd_observed/status` | `CmdStatusUnity` | `/teleop/status` (observed) | follows input |
| `/unity/cmd_observed/{joint_velocity,joint_position}` | `sensor_msgs/JointState` | `/teleop/validated/*` | follows input |
| `/unity/cmd_observed/cartesian_pose` | `geometry_msgs/PoseStamped` | `/teleop/validated/cartesian_pose` | follows input |
| `/unity/cmd_observed/twist` | `geometry_msgs/TwistStamped` | `/servo_node/delta_twist_cmds` | follows input |

### Field reference

**`JointStateUnity`** — `names[]`, `positions[]` (**deg**), `velocities[]`
(**deg/s**), `efforts[]`. Positions/velocities are converted from rad and reordered
to the fixed UR7e joint sequence. `efforts` is the driver's `actual_current`
(motor **current in A**, *not* torque); real torque is on `/unity/joint_torques`.

**`JointTorqueUnity`** — `stamp`, `names[]`, `torques_nm[]` (**N·m**), in fixed
UR7e order. Published only while `ur_rtde_torque_bridge` is running.

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

---

## Runtime nodes

`digital_twin_manager` supervises the processes below and respawns them if they
exit. Executable names are stable; the manager launches them via `ros2 run`.

| Process | Function |
| --- | --- |
| `joint_state_bridge_ros2unity` | `/joint_states` → `/unity/joint_states` |
| `joint_torque_bridge_ros2unity` | `/joint_torques` → `/unity/joint_torques` (idle until a torque source publishes) |
| `ur_state_bridge_ros2unity` | UR state → `/unity/tcp_pose`, `/unity/robot_status`, `/unity/wrench` |
| `io_states_bridge_ros2unity` | `/io_and_status_controller/io_states` → `/unity/io_states` |
| `tool_data_bridge_ros2unity` | `/io_and_status_controller/tool_data` → `/unity/tool_data` |
| `controller_status_bridge_ros2unity` | polls `/controller_manager/list_controllers` → `/unity/controller_status` |
| `robot_info_bridge_ros2unity` | polls UR read-only services → `/unity/robot_info` |
| `command_supervisor_ros2unity` | observes `teleoperation_general_ros2` → `/unity/cmd_observed/*` (read-only) |
| `ros_tcp_endpoint default_server_endpoint` | bridges `/unity/*` to Unity over TCP |

Source nodes are organised into `state_bridges/` (robot state) and
`command_bridges/` (command observation) Python subpackages.

### Optional joint-torque source

`ur_rtde_torque_bridge` is **pluggable** and **not** started by the manager. It
opens an independent RTDE receive-only connection (raw RTDE wire protocol, Python
standard library only — no external dependency) and publishes real joint torques
(N·m) on `/joint_torques`, which `joint_torque_bridge_ros2unity` forwards to Unity.

```bash
ros2 launch ur_rtde_torque_bridge torque_publisher.launch.py robot_ip:=<ROBOT_IP>
```

`rtde_field` defaults to `actual_current_as_torque` (measured torque, PolyScope
≥ 5.23). If the firmware lacks it, the node logs `NOT_FOUND`; pass
`rtde_field:=target_moment` (commanded torque, all versions). Any source can
replace it by publishing the same `/joint_torques` (`sensor_msgs/JointState`,
effort = N·m). See `src/ur_rtde_torque_bridge/README.md`.

---

## Dependencies

### ROS 2 Humble

ROS 2 Humble on Ubuntu 22.04. Source it before building:
`source /opt/ros/humble/setup.bash`.

### Universal Robots ROS 2 Driver

Provides the hardware interface and controllers whose topics this workspace
consumes. Build it in a **separate** workspace first.

- Repository: <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>
- Branch: `humble`

```bash
mkdir -p ~/ur_ros2_ws/src && cd ~/ur_ros2_ws
git clone -b humble \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git \
  src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing \
  --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
rosdep update && rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Packages consumed: `ur_robot_driver`, `ur_controllers`,
`Universal_Robots_ROS2_Description`. The message packages `ur_msgs` and
`ur_dashboard_msgs` are **vendored** here under `src/third_party/` so this
workspace builds offline; see `src/third_party/README.md`.

### URSim (simulation)

Official UR simulator as a Docker image; behaves like a real controller from the
driver's perspective.

- Docker Hub: <https://hub.docker.com/r/universalrobots/ursim_e-series>

```bash
docker run --rm -it -e ROBOT_MODEL=UR7E -p 5900:5900 -p 6080:6080 \
  universalrobots/ursim_e-series
```

URSim boots in ~30 s. Its IP on the Docker bridge is typically `10.255.255.254`;
PolyScope UI is at <http://localhost:6080>.

### Unity ROS-TCP-Endpoint

- Repository: <https://github.com/Unity-Technologies/ROS-TCP-Endpoint>
- Branch: `main-ros2`

Clone into `src/ROS-TCP-Endpoint` (git-ignored):

```bash
git clone -b main-ros2 \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint
```

---

## Build

```bash
git clone <repository-url>
cd digital_twin_on_unity_and_ros2_ws

# ROS-TCP-Endpoint is git-ignored; clone it into src/
git clone -b main-ros2 \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint

# Source the UR driver workspace BEFORE building, then build
source <path-to-ur-driver-workspace>/install/setup.bash
colcon build
source install/setup.bash
```

> The UR driver workspace must be sourced before `colcon build` so the controller
> and interface packages resolve.

---

## Running the system

### Step 1 — Start the robot

URSim:

```bash
docker run --rm -it -e ROBOT_MODEL=UR7E -p 5900:5900 -p 6080:6080 \
  universalrobots/ursim_e-series   # wait ~30 s
```

Real UR7e: power on and confirm network reachability.

### Step 2 — Launch the UR driver

```bash
source <path-to-ur-driver-workspace>/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:=10.255.255.254 launch_rviz:=false   # URSim IP
```

Replace `robot_ip` with the real robot IP when using hardware.

### Step 3 — Launch the digital twin manager

```bash
source <path-to-ur-driver-workspace>/install/setup.bash
source <path-to-this-workspace>/install/setup.bash

ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_tcp_port:=10000
```

Leave `ros_ip` at its default — see [Connecting Unity](#connecting-unity-networking).

### Step 4 — Verify

```bash
ros2 topic echo --once /unity/joint_states
ros2 topic echo --once /unity/robot_status
ros2 topic hz   /unity/tcp_pose
ros2 topic echo --once /unity/joint_torques   # only with ur_rtde_torque_bridge
```

---

## Connecting Unity (networking)

`ros_ip` is the address the ROS-TCP endpoint binds/listens on — it is **not** the
address Unity dials. Keep it **empty or `0.0.0.0`** (the default) so the endpoint
accepts connections on all interfaces. Do **not** set `127.0.0.1`: that binds
localhost only, and Unity on a different host (the usual WSL2 to Windows case)
cannot reach it.

In Unity (*Robotics -> ROS Settings -> ROS IP Address*, port `10000`), the only
case that needs attention is when **Unity runs on a different machine than ROS**
— which is the normal setup here (ROS in WSL, Unity on Windows). Then enter the
**reachable IP of the ROS machine**: get it with `hostname -I` in WSL (e.g.
`172.x.x.x`), and note it **changes on every WSL restart**. Alternatively, enable
WSL2 mirrored networking and use `127.0.0.1`.

If the connection still fails after the IP is correct, check the Windows firewall:
`Test-NetConnection <ros-ip> -Port 10000` should report `TcpTestSucceeded : True`.

---

## Unity integration

The Unity project uses
[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) to
subscribe to `/unity/*`. Reference C# subscribers are in `plan/shared/script/`:

| Script | Topic |
| --- | --- |
| `UR7eJointStateSubscriber.cs` | `/unity/joint_states` |
| `UR7eTcpMarkerSubscriber.cs` | `/unity/tcp_pose` |
| `UR7eRobotStatusSubscriber.cs` | `/unity/robot_status` |
| `UR7eWrenchSubscriber.cs` | `/unity/wrench` (force/torque + \|F\|) |
| `UR7eTcpForceArrow.cs` | `/unity/wrench` (3D force arrow at the TCP) |
| `UR7eJointTorquePanel.cs` | `/unity/joint_torques` (N·m + % of rated torque) |
| `UR7eIoStatesSubscriber.cs` | `/unity/io_states` |
| `UR7eToolDataSubscriber.cs` | `/unity/tool_data` |
| `UR7eControllerStatusSubscriber.cs` | `/unity/controller_status` |
| `UR7eRobotInfoSubscriber.cs` | `/unity/robot_info` |
| `UR7eCommandObservedPanel.cs` | `/unity/cmd_observed/command` + `/status` |

After changing any `.msg`, regenerate message types in Unity. The shared handoff
package (`plan/shared/`) holds the `.msg` snapshots, the scripts, and a units /
coordinate-system reference.

---

## Development

```bash
# Rebuild after code changes
source <path-to-ur-driver-workspace>/install/setup.bash
colcon build
source install/setup.bash
```

Verify a build succeeds before committing. Keep new bridge nodes read-only and
route any new robot state through a `/unity/*` topic.

---

## License

Copyright 2026 HAOYU LUO. Licensed under the Apache License 2.0; see
[LICENSE](LICENSE) and [NOTICE](NOTICE).
