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

## Project Overview

```text
URSim / Real UR7e
  └─► UR ROS 2 Driver  (~/ur_drive/ur_drive_ws)
        ├─ joint_state_broadcaster       → /joint_states
        ├─ tcp_pose_broadcaster          → /tcp_pose_broadcaster/pose
        ├─ io_and_status_controller      → robot_mode / safety_mode / program_running
        ├─ speed_scaling_state_broadcaster → /speed_scaling_state_broadcaster/speed_scaling
        └─ force_torque_sensor_broadcaster → /force_torque_sensor_broadcaster/wrench

  └─► Digital Twin Workspace  (this repository)
        ├─ joint_state_bridge_ros2unity  → /unity/joint_states   (on-change)
        ├─ ur_state_bridge_ros2unity     → /unity/tcp_pose        (30 Hz)
        │                                  /unity/robot_status    (10 Hz)
        │                                  /unity/wrench          (30 Hz)
        └─ ros_tcp_endpoint              → TCP port 10000

  └─► Unity  (ROS-TCP-Connector)
        subscribes to /unity/* topics
```

The digital twin workspace only consumes standard ROS topics from the UR driver.
The two workspaces are independent and communicate solely through the ROS graph.

---

## Repository Layout

```text
src/
  digital_twin_interfaces/        Custom ROS 2 message definitions
  digital_twin_on_unity_and_ros2/ Bridge nodes, manager node, and launch files
  ur_dashboard_msgs/              UR dashboard messages (vendored from UR driver)
  ROS-TCP-Endpoint/               Unity ROS-TCP-Endpoint (local clone, git-ignored)
```

---

## Published Unity Topics

| Topic | Message type | Upstream source | Publish rate |
| --- | --- | --- | --- |
| `/unity/joint_states` | `JointStateUnity` | `/joint_states` | on-change |
| `/unity/tcp_pose` | `TcpPoseUnity` | `/tcp_pose_broadcaster/pose` | 30 Hz |
| `/unity/robot_status` | `RobotStatusUnity` | robot/safety/program/speed topics | 10 Hz |
| `/unity/wrench` | `WrenchUnity` | `/force_torque_sensor_broadcaster/wrench` | 30 Hz |

**`JointStateUnity`** — joint positions and velocities converted from rad / rad·s⁻¹
to degrees / degrees·s⁻¹, reordered to a fixed UR7e joint sequence.

**`RobotStatusUnity`** — aggregates `robot_mode` (int8 + label), `safety_mode`
(uint8 + label), `robot_program_running` (bool), `robot_program_running_received`
(bool, distinguishes "false" from "not yet received"), and `speed_scaling` (float64).

**`TcpPoseUnity`** — TCP position (m) and orientation (quaternion) in the `base`
frame, with timestamp and frame_id. Coordinate conversion to Unity's left-handed
frame is handled on the Unity side.

**`WrenchUnity`** — force (N) and torque (N·m) in the `tool0_controller` frame.

---

## Nodes Managed at Runtime

The `digital_twin_manager` node supervises three permanent child processes and
automatically respawns them if they exit:

| Process | What it does |
| --- | --- |
| `joint_state_bridge_ros2unity` | `/joint_states` → `/unity/joint_states` |
| `ur_state_bridge_ros2unity` | UR state topics → `/unity/tcp_pose`, `/unity/robot_status`, `/unity/wrench` |
| `ros_tcp_endpoint default_server_endpoint` | bridges the `/unity/*` topics to Unity over TCP |

---

## External Dependencies

### 1. ROS 2 Humble

This workspace targets ROS 2 Humble on Ubuntu 22.04.

### 2. Universal Robots ROS 2 Driver workspace

The UR driver provides the hardware interface and the controllers that publish
the upstream topics this workspace consumes.

**Location (on this machine):** `~/ur_drive/ur_drive_ws`

Relevant packages inside that workspace:

| Package | Role |
| --- | --- |
| `ur_robot_driver` | Hardware interface, launch files (`ur_control.launch.py`) |
| `ur_controllers` | `tcp_pose_broadcaster`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster` |
| `Universal_Robots_ROS2_Description` | URDF / xacro model and meshes (used by the driver launch) |
| `ur_dashboard_msgs` | `RobotMode.msg`, `SafetyMode.msg` etc. — vendored copy is in `src/ur_dashboard_msgs/` |
| `ur_msgs` | `IOStates.msg`, `SetIO.srv`, etc. — not yet used; reserved for future IO sync stage |

### 3. URSim (for simulation)

URSim is the official Universal Robots simulator. It runs inside a Docker
container and behaves identically to a real UR controller from the driver's
perspective.

Start URSim for UR7e:

```bash
source ~/ur_drive/ur_drive_ws/install/setup.bash
ros2 run ur_client_library start_ursim.sh -m ur7e
```

URSim takes ~30 seconds to boot. Once ready, its default IP is
`10.255.255.254` (Docker bridge address).

### 4. Unity ROS-TCP-Endpoint

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

# 2. Clone the ROS-TCP-Endpoint
git clone -b main-ros2 \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
  src/ROS-TCP-Endpoint

# 3. Source the UR driver workspace, then build
source ~/ur_drive/ur_drive_ws/install/setup.bash
colcon build
source install/setup.bash
```

> The UR driver workspace must be sourced **before** `colcon build` so that
> `ur_dashboard_msgs` and the controller packages are found by the build system.

---

## Running the System

### Step 1 — Start URSim (simulation) or connect to a real robot

**URSim:**

```bash
# Terminal A — start the simulator
source ~/ur_drive/ur_drive_ws/install/setup.bash
ros2 run ur_client_library start_ursim.sh -m ur7e
# wait ~30 s for URSim to finish booting
```

**Real UR7e:** power on the robot and ensure it is reachable on the network.

### Step 2 — Launch the UR ROS 2 driver

```bash
# Terminal B
source ~/ur_drive/ur_drive_ws/install/setup.bash

# For URSim
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=10.255.255.254 \
  launch_rviz:=false

# For a real UR7e (replace IP with the actual robot IP)
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=<robot-ip> \
  launch_rviz:=false
```

When the driver is up, the following topics are available:

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
# Terminal C — source both workspaces (order matters)
source ~/ur_drive/ur_drive_ws/install/setup.bash
source ~/digital_twin_on_unity_and_ros2/digital_twin_on_unity_and_ros2_ws/install/setup.bash

# Unity on the same machine
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=127.0.0.1 \
  ros_tcp_port:=10000

# Unity on a different machine (use the IP of this ROS machine)
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=<this-machine-ip> \
  ros_tcp_port:=10000
```

### Step 4 — Verify topics

```bash
# Source both workspaces in a new terminal, then:

# Check upstream UR driver topics
ros2 topic echo --once /joint_states
ros2 topic echo --once /tcp_pose_broadcaster/pose
ros2 topic echo --once /io_and_status_controller/robot_mode

# Check Unity-facing topics
ros2 topic echo --once /unity/joint_states
ros2 topic echo --once /unity/tcp_pose
ros2 topic echo --once /unity/robot_status
ros2 topic echo --once /unity/wrench

# Check publish rates
ros2 topic hz /unity/joint_states
ros2 topic hz /unity/tcp_pose
ros2 topic hz /unity/wrench
```

---

## Unity Side

The Unity project uses the
[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
package to subscribe to the `/unity/*` topics published by this workspace.
Reference C# subscriber scripts are in `plan/script/`:

| Script | Topic |
| --- | --- |
| `UR7eJointStateSubscriber.cs` | `/unity/joint_states` |
| `UR7eTcpMarkerSubscriber.cs` | `/unity/tcp_pose` |
| `UR7eRobotStatusSubscriber.cs` | `/unity/robot_status` |
| `UR7eWrenchSubscriber.cs` | `/unity/wrench` |

Generated message types are under the `RosMessageTypes.DigitalTwinInterfaces`
namespace. Full Unity scene setup, UI panel design, and integration guide will
be added as that work progresses.

---

## Rebuilding After Code Changes

```bash
cd ~/digital_twin_on_unity_and_ros2/digital_twin_on_unity_and_ros2_ws
source ~/ur_drive/ur_drive_ws/install/setup.bash
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
