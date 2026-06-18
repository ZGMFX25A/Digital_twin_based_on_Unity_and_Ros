# Digital Twin on Unity and ROS 2

A ROS 2 workspace that implements the robot-side backend of a Unity-based
digital twin for the Universal Robots UR7e collaborative robot.

The system subscribes to the UR ROS 2 driver topics, converts the robot state
into Unity-friendly custom messages, and exposes them to Unity through the
ROS-TCP-Endpoint. The current stage covers **read-only state synchronisation**:
joint positions, TCP pose, robot/safety mode, program state, speed scaling, and
force/torque wrench are all streamed to Unity in real time. No control commands
are sent from Unity to the robot in this stage.

Unity-side integration (C# subscriber scripts, scene setup, and UI) will be
documented separately as that work progresses.

---

## Architecture

```text
URSim (Docker) / Real UR7e
  └─► Universal Robots ROS 2 Driver
        ├─ joint_state_broadcaster          → /joint_states
        ├─ tcp_pose_broadcaster             → /tcp_pose_broadcaster/pose
        ├─ io_and_status_controller         → robot_mode / safety_mode / program_running
        ├─ speed_scaling_state_broadcaster  → /speed_scaling_state_broadcaster/speed_scaling
        └─ force_torque_sensor_broadcaster  → /force_torque_sensor_broadcaster/wrench

  └─► This workspace (digital_twin_on_unity_and_ros2_ws)
        ├─ joint_state_bridge_ros2unity       → /unity/joint_states   (on-change)
        ├─ joint_torque_bridge_ros2unity      → /unity/joint_torques  (follows input)
        ├─ ur_state_bridge_ros2unity          → /unity/tcp_pose        (30 Hz)
        │                                       /unity/robot_status    (10 Hz)
        │                                       /unity/wrench          (30 Hz)
        ├─ io_states_bridge_ros2unity         → /unity/io_states
        ├─ tool_data_bridge_ros2unity         → /unity/tool_data
        ├─ controller_status_bridge_ros2unity → /unity/controller_status
        ├─ robot_info_bridge_ros2unity        → /unity/robot_info      (~1 Hz)
        ├─ command_supervisor_ros2unity       → /unity/cmd_observed/*  (read-only)
        └─ ros_tcp_endpoint                   → TCP port 10000

  └─► Optional, pluggable (not started by the manager)
        ur_rtde_torque_bridge (RTDE receive-only) → /joint_torques (N·m)
              ↑ feeds joint_torque_bridge_ros2unity above

  └─► A separate workspace (not edited here): teleoperation_general_ros2
        publishes /teleop/command, /teleop/validated/*, /teleop/status
              ↑ command_supervisor_ros2unity OBSERVES these (read-only)

  └─► Unity (ROS-TCP-Connector)
        subscribes to /unity/* topics
```

The two workspaces communicate only through the ROS graph. This workspace does
not depend on any hardcoded path from the UR driver workspace.

---

## Repository Layout

```text
src/
  digital_twin_interfaces/        Custom ROS 2 message definitions
  digital_twin_on_unity_and_ros2/ Bridge nodes, manager node, and launch files
  ur_rtde_torque_bridge/          Optional, pluggable RTDE torque source (N·m)
  third_party/                    Vendored third-party deps (see third_party/README.md)
    ur_dashboard_msgs/              UR dashboard messages (vendored from UR driver)
    ur_msgs/                        UR messages: IOStates / ToolDataMsg (vendored)
    teleop_msgs/                    Teleop command/status messages (vendored, for observation)
    ROS-TCP-Endpoint/               Unity ROS-TCP-Endpoint (local clone, git-ignored)
```

---

## Published Unity Topics

| Topic | Message type | Upstream source | Rate |
| --- | --- | --- | --- |
| `/unity/joint_states` | `JointStateUnity` | `/joint_states` | on-change |
| `/unity/joint_torques` | `JointTorqueUnity` | `/joint_torques` (from `ur_rtde_torque_bridge`) | follows input |
| `/unity/tcp_pose` | `TcpPoseUnity` | `/tcp_pose_broadcaster/pose` | 30 Hz |
| `/unity/robot_status` | `RobotStatusUnity` | robot/safety/program/speed topics | 10 Hz |
| `/unity/wrench` | `WrenchUnity` | `/force_torque_sensor_broadcaster/wrench` | 30 Hz |
| `/unity/io_states` | `IoStatesUnity` | `/io_and_status_controller/io_states` | follows input |
| `/unity/tool_data` | `ToolDataUnity` | `/io_and_status_controller/tool_data` | follows input |
| `/unity/controller_status` | `ControllerStatusUnity` | `/controller_manager/list_controllers` | 2 Hz |
| `/unity/robot_info` | `RobotInfoUnity` | UR read-only dashboard/version services | ~1 Hz |
| `/unity/cmd_observed/*` | `CmdObservedUnity` / `CmdStatusUnity` (+ std types) | `teleoperation_general_ros2` topics (**observed, read-only**) | follows input |

**`JointStateUnity`** — joint positions and velocities converted from rad / rad·s⁻¹
to degrees / degrees·s⁻¹, reordered to a fixed UR7e joint sequence. The `efforts`
field carries the UR driver's `actual_current` (motor **current in A**, *not*
torque); real joint torque is published separately on `/unity/joint_torques`.

**`JointTorqueUnity`** — real joint torques (N·m) in the fixed UR7e joint order,
sourced from the optional `ur_rtde_torque_bridge` (RTDE `actual_current_as_torque`
or `target_moment`). Only published while that package is running.

**`RobotStatusUnity`** — aggregates `robot_mode` (int8 + label), `safety_mode`
(uint8 + label), `robot_program_running` (bool), `robot_program_running_received`
(bool, distinguishes "false" from "not yet received"), and `speed_scaling` (float64).

**`TcpPoseUnity`** — TCP position (m) and orientation (quaternion) in the `base`
frame, with timestamp and frame_id. Coordinate conversion to Unity's left-handed
frame is handled on the Unity side.

**`WrenchUnity`** — force (N) and torque (N·m) in the `tool0_controller` frame.

---

## Nodes Managed at Runtime

The `digital_twin_manager` node supervises the permanent child processes below
and automatically respawns them if they exit:

| Process | What it does |
| --- | --- |
| `joint_state_bridge_ros2unity` | `/joint_states` → `/unity/joint_states` |
| `joint_torque_bridge_ros2unity` | `/joint_torques` → `/unity/joint_torques` (idle until a torque source publishes) |
| `ur_state_bridge_ros2unity` | UR state topics → `/unity/tcp_pose`, `/unity/robot_status`, `/unity/wrench` |
| `io_states_bridge_ros2unity` | `/io_and_status_controller/io_states` → `/unity/io_states` |
| `tool_data_bridge_ros2unity` | `/io_and_status_controller/tool_data` → `/unity/tool_data` |
| `controller_status_bridge_ros2unity` | polls `/controller_manager/list_controllers` → `/unity/controller_status` |
| `robot_info_bridge_ros2unity` | polls UR read-only services → `/unity/robot_info` |
| `command_supervisor_ros2unity` | observes `teleoperation_general_ros2` command topics → `/unity/cmd_observed/*` (read-only) |
| `ros_tcp_endpoint default_server_endpoint` | bridges the `/unity/*` topics to Unity over TCP |

### Optional joint torque source

The `ur_rtde_torque_bridge` package is **pluggable** and **not** started by the
manager. It opens an independent RTDE receive-only connection (raw RTDE wire
protocol, Python standard library only — **no external dependency**) to publish
real joint torques (N·m) on `/joint_torques`, which `joint_torque_bridge_ros2unity`
then forwards to Unity. Start it on demand:

```bash
ros2 launch ur_rtde_torque_bridge torque_publisher.launch.py \
  robot_ip:=<ROBOT_IP>
```

The default `rtde_field` is `actual_current_as_torque` (measured torque,
PolyScope ≥ 5.23/10.11). If the firmware lacks it, the node logs `NOT_FOUND`;
pass `rtde_field:=target_moment` (commanded torque, all versions) instead.

Any other source can replace it by publishing the same `/joint_torques`
(`sensor_msgs/JointState`, effort = N·m) — see `src/ur_rtde_torque_bridge/README.md`.

---

## External Dependencies

### 1. ROS 2 Humble

This workspace targets ROS 2 Humble on Ubuntu 22.04.

### 2. Universal Robots ROS 2 Driver

The UR driver provides the hardware interface and the controllers whose topics
this workspace consumes.

**Repository:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver  
**Branch for Humble:** `humble`

Clone and build it in a separate workspace before building this one:

```bash
# Create and enter a workspace directory of your choice
mkdir -p ~/ur_ros2_ws/src && cd ~/ur_ros2_ws

# Import the driver and its dependencies using vcs
git clone -b humble \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git \
  src/Universal_Robots_ROS2_Driver

vcs import src --skip-existing \
  --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos

# Install ROS dependencies
rosdep update
rosdep install --ignore-src --from-paths src -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Packages from the driver used by this workspace:

| Package | Role |
| --- | --- |
| `ur_robot_driver` | Hardware interface and launch files |
| `ur_controllers` | `tcp_pose_broadcaster`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster` |
| `Universal_Robots_ROS2_Description` | URDF / xacro model used by the driver launch |
| `ur_dashboard_msgs` | `RobotMode.msg`, `SafetyMode.msg` — also vendored in `src/ur_dashboard_msgs/` |

### 3. URSim (for simulation)

URSim is the official Universal Robots simulator distributed as a Docker image.
It behaves identically to a real UR controller from the driver's perspective.

**Docker Hub:** https://hub.docker.com/r/universalrobots/ursim_e-series

Pull and start URSim for UR7e:

```bash
docker run --rm -it \
  -e ROBOT_MODEL=UR7E \
  -p 5900:5900 \
  -p 6080:6080 \
  universalrobots/ursim_e-series
```

URSim takes ~30 seconds to boot. Once ready, its IP on the Docker bridge
network is typically `10.255.255.254`. You can open a browser at
`http://localhost:6080` to view the PolyScope interface.

### 4. Unity ROS-TCP-Endpoint

**Repository:** https://github.com/Unity-Technologies/ROS-TCP-Endpoint  
**Branch for ROS 2:** `main-ros2`

Clone into `src/ROS-TCP-Endpoint` (already git-ignored):

```bash
git clone -b main-ros2 \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
  src/ROS-TCP-Endpoint
```

---

## Setup

```bash
# 1. Clone this repository
git clone <repository-url>
cd digital_twin_on_unity_and_ros2_ws

# 2. Clone the ROS-TCP-Endpoint into src/
git clone -b main-ros2 \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
  src/ROS-TCP-Endpoint

# 3. Source the UR driver workspace (built separately, see above), then build
source <path-to-ur-driver-workspace>/install/setup.bash
colcon build
source install/setup.bash
```

> The UR driver workspace must be sourced **before** `colcon build` so the
> compiler can find the controller and interface packages.

---

## Running the System

### Step 1 — Start URSim or power on the real robot

**URSim:**

```bash
# Terminal A
docker run --rm -it \
  -e ROBOT_MODEL=UR7E \
  -p 5900:5900 \
  -p 6080:6080 \
  universalrobots/ursim_e-series
# wait ~30 s for the simulator to finish booting
```

**Real UR7e:** power on the robot and confirm it is reachable on the network.

### Step 2 — Launch the UR ROS 2 driver

```bash
# Terminal B
source <path-to-ur-driver-workspace>/install/setup.bash

# URSim
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=10.255.255.254 \
  launch_rviz:=false

# Real UR7e (replace with the actual robot IP)
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=<robot-ip> \
  launch_rviz:=false
```

When the driver is running, these topics are available:

```text
/joint_states
/tcp_pose_broadcaster/pose
/io_and_status_controller/robot_mode
/io_and_status_controller/safety_mode
/io_and_status_controller/robot_program_running
/speed_scaling_state_broadcaster/speed_scaling
/force_torque_sensor_broadcaster/wrench
```

### Step 3 — Launch the digital twin manager

```bash
# Terminal C — source the UR driver workspace first, then this workspace
source <path-to-ur-driver-workspace>/install/setup.bash
source <path-to-this-workspace>/install/setup.bash

# Unity on the same machine as ROS
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=127.0.0.1 \
  ros_tcp_port:=10000

# Unity on a different machine (use the IP of the ROS machine)
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=<ros-machine-ip> \
  ros_tcp_port:=10000
```

### Step 4 — Verify topics

```bash
# In a new terminal, source both workspaces, then run:

# Upstream UR driver topics
ros2 topic echo --once /joint_states
ros2 topic echo --once /tcp_pose_broadcaster/pose
ros2 topic echo --once /io_and_status_controller/robot_mode

# Unity-facing topics
ros2 topic echo --once /unity/joint_states
ros2 topic echo --once /unity/tcp_pose
ros2 topic echo --once /unity/robot_status
ros2 topic echo --once /unity/wrench
# Only with the optional ur_rtde_torque_bridge running:
ros2 topic echo --once /unity/joint_torques

# Publish rates
ros2 topic hz /unity/joint_states
ros2 topic hz /unity/tcp_pose
ros2 topic hz /unity/wrench
```

---

## Unity Side

The Unity project uses the
[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
package to subscribe to the `/unity/*` topics. Reference C# subscriber scripts
are in `plan/shared/script/`:

| Script | Topic |
| --- | --- |
| `UR7eJointStateSubscriber.cs` | `/unity/joint_states` |
| `UR7eTcpMarkerSubscriber.cs` | `/unity/tcp_pose` |
| `UR7eRobotStatusSubscriber.cs` | `/unity/robot_status` |
| `UR7eWrenchSubscriber.cs` | `/unity/wrench` (force/torque numbers + \|F\|) |
| `UR7eJointTorquePanel.cs` | `/unity/joint_torques` (N·m + % of rated torque) |
| `UR7eTcpForceArrow.cs` | `/unity/wrench` (3D force arrow at the TCP) |

Generated message types are under the `RosMessageTypes.DigitalTwinInterfaces`
namespace. Full Unity scene setup, UI panel design, and integration guide will
be added as that work progresses.

---

## Rebuilding After Code Changes

```bash
cd <path-to-this-workspace>
source <path-to-ur-driver-workspace>/install/setup.bash
colcon build
source install/setup.bash
```

---

## Collaboration Workflow

```bash
colcon build          # verify it builds before committing
git checkout -b feature/<short-name>
git add <changed-files>
git commit -m "<clear message>"
git push origin feature/<short-name>
```

Open a pull request on GitHub and describe what changed, how it was tested, and
any Unity-side steps needed to verify it.

---

## License

Copyright 2026 HAOYU LUO.

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE)
and [NOTICE](NOTICE).
