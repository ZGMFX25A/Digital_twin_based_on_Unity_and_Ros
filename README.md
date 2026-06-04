# Digital Twin on Unity and ROS 2

ROS 2 workspace for a Unity-based digital twin of a UR7e collaborative robot.
The ROS side subscribes to the UR ROS 2 driver topics, converts them into
Unity-friendly messages, and exposes the data to Unity through the
ROS-TCP-Endpoint.

## Repository Layout

```text
src/
  digital_twin_interfaces/        Custom ROS 2 messages
  digital_twin_on_unity_and_ros2/ Bridge nodes, manager node, and launch files
  ur_dashboard_msgs/              UR dashboard message definitions (vendored)
  ROS-TCP-Endpoint/               Local dependency, ignored by git
```

## Published Unity Topics

| Topic | Message | Source topic | Rate |
| --- | --- | --- | --- |
| `/unity/joint_states` | `JointStateUnity` | `/joint_states` | on-change |
| `/unity/tcp_pose` | `TcpPoseUnity` | `/tcp_pose_broadcaster/pose` | 30 Hz |
| `/unity/robot_status` | `RobotStatusUnity` | robot/safety/program/speed topics | 10 Hz |
| `/unity/wrench` | `WrenchUnity` | `/force_torque_sensor_broadcaster/wrench` | 30 Hz |

`JointStateUnity` positions and velocities are converted from radians to degrees.
`RobotStatusUnity` includes `robot_mode`, `safety_mode`, `robot_program_running`,
`robot_program_running_received`, `speed_scaling`, and human-readable label fields.

## Nodes Started by the Manager

The manager node supervises three permanent processes and respawns them on crash:

| Node | Role |
| --- | --- |
| `joint_state_bridge_ros2unity` | `/joint_states` → `/unity/joint_states` |
| `ur_state_bridge_ros2unity` | TCP pose / robot status / wrench → `/unity/*` |
| `ros_tcp_endpoint default_server_endpoint` | Unity ROS-TCP-Connector bridge |

## Requirements

- ROS 2 Humble
- Python 3
- `colcon`
- Universal Robots ROS 2 Driver workspace sourced (provides `ur_dashboard_msgs`
  and the UR hardware interface)
- Unity project configured with ROS-TCP-Connector

## Setup

Clone this repository and install the ROS-TCP-Endpoint dependency:

```bash
git clone <repository-url>
cd digital_twin_on_unity_and_ros2_ws
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint
```

Source the UR driver workspace, then build:

```bash
source /path/to/ur_drive_ws/install/setup.bash
colcon build
source install/setup.bash
```

## Run

Start the digital twin manager:

```bash
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=127.0.0.1 \
  ros_tcp_port:=10000
```

For a remote Unity machine, replace `ros_ip` with the ROS host IP that Unity
should connect to:

```bash
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py \
  ros_ip:=172.19.25.57 \
  ros_tcp_port:=10000
```

The UR ROS 2 driver and URSim (or a real UR7e) must be running and publishing
the standard UR controller topics before starting this workspace.

## Unity Side

The `plan/script/` directory contains reference C# subscriber scripts:

| Script | Subscribed topic |
| --- | --- |
| `UR7eJointStateSubscriber.cs` | `/unity/joint_states` |
| `UR7eTcpMarkerSubscriber.cs` | `/unity/tcp_pose` |
| `UR7eRobotStatusSubscriber.cs` | `/unity/robot_status` |
| `UR7eWrenchSubscriber.cs` | `/unity/wrench` |

Generated message types are under `RosMessageTypes.DigitalTwinInterfaces`.

## Collaboration Workflow

Before pushing changes:

```bash
colcon build
git status
```

Recommended branch flow:

```bash
git checkout -b feature/<short-name>
git add <changed-files>
git commit -m "<clear commit message>"
git push origin feature/<short-name>
```

Open a pull request on GitHub and describe:

- What changed
- How it was tested
- Any Unity-side setup needed to verify it

## License

Copyright 2026 HAOYU LUO.

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE)
and [NOTICE](NOTICE).
