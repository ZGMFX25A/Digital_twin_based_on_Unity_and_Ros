# Digital Twin on Unity and ROS 2

ROS 2 workspace for a Unity-based digital twin. The ROS side receives robot joint states, converts them into a Unity-friendly message, and exposes the data to Unity through Unity's ROS-TCP-Endpoint.

## Repository Layout

```text
src/
  digital_twin_interfaces/        Custom ROS 2 messages
  digital_twin_on_unity_and_ros2/ ROS 2 bridge, manager node, and launch files
  ROS-TCP-Endpoint/               Local dependency, ignored by git
```

## Requirements

- ROS 2 Humble
- Python 3
- `colcon`
- Unity project configured with ROS-TCP-Connector
- Unity ROS-TCP-Endpoint cloned into `src/ROS-TCP-Endpoint`

## Setup

Clone this repository, then install the ROS-TCP-Endpoint dependency:

```bash
git clone <repository-url>
cd digital_twin_on_unity_and_ros2_ws
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint
```

Build the workspace:

```bash
colcon build
source install/setup.bash
```

## Run

Start the digital twin manager:

```bash
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py ros_ip:=127.0.0.1 ros_tcp_port:=10000
```

For a remote Unity machine, replace `ros_ip` with the ROS host IP address that Unity should connect to:

```bash
ros2 launch digital_twin_on_unity_and_ros2 digital_twin_manager.launch.py ros_ip:=172.19.25.57 ros_tcp_port:=10000
```

The manager starts these permanent nodes automatically:

- `joint_state_bridge_ros2unity`
- `ros_tcp_endpoint default_server_endpoint`

The bridge subscribes to `/joint_states`, converts joint positions and velocities from radians to degrees, and publishes `digital_twin_interfaces/msg/JointStateUnity` on `/unity/joint_states`.

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

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).
