# ur_rtde_torque_bridge

Standalone, **pluggable** data-source node that publishes UR joint torques in
**N·m** on the intermediate topic `/joint_torques`.

It opens an independent **RTDE receive-only** connection to the robot/URSim, so it
coexists with the UR driver's own RTDE connection and never sends a command. The
driver's `/joint_states.effort` is motor **current (A)**, not torque; this node
fills the missing real-torque (N·m) channel.

## Topic contract

| Topic | Type | Content |
| --- | --- | --- |
| `/joint_torques` (output) | `sensor_msgs/JointState` | `name` = UR7e joint order, `effort` = torque in **N·m**; `position`/`velocity` left empty |

A standard `sensor_msgs/JointState` is used on purpose: any other data source can
publish the same topic with **zero dependency** on this project's custom
interfaces, and replace this package entirely (see *Pluggable* below).

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `robot_ip` | `127.0.0.1` | Robot/URSim RTDE host |
| `rtde_port` | `30004` | RTDE port (used for a reachability pre-check) |
| `rtde_field` | `actual_current_as_torque` | RTDE field to read (see below) |
| `publish_rate_hz` | `20.0` | Publish rate for `/joint_torques` |
| `output_topic` | `/joint_torques` | Output topic name |

`rtde_field` options:

- `actual_current_as_torque` — **measured** torque (N·m), the default. Requires
  PolyScope ≥ 5.23 / 10.11 (e-Series 5.x qualifies). Under URSim it equals
  `target_moment` (ideal simulation).
- `target_moment` — **commanded** torque (N·m), available on all versions.
  Fallback when the firmware lacks the measured field.
- `actual_current` — motor current (A), diagnostic only (not a torque).

The node reads the chosen field directly via the RTDE wire protocol, so any
VECTOR6D field the firmware exposes works. If the field is missing, the node logs
`NOT_FOUND` and tells you to use `target_moment`.

## Dependency

None beyond ROS 2 / `rclpy` / `sensor_msgs`. The RTDE client is implemented with
the Python standard library only (`socket`, `struct`) — no `pip install` needed.

## Run

```bash
ros2 launch ur_rtde_torque_bridge torque_publisher.launch.py \
    robot_ip:=<ROBOT_IP> rtde_field:=actual_current_as_torque
```

## Pluggable (by design)

1. **Optional** — the digital twin manager does **not** launch this package. Start
   it only when you want joint torques. Without it, everything else works; only the
   Unity torque panel has no data.
2. **Removable** — depends only on `rclpy` + `sensor_msgs`, with nothing in the
   workspace depending on it. Delete `src/ur_rtde_torque_bridge/` and the workspace
   still builds.
3. **Replaceable** — any other source can publish `/joint_torques`
   (`sensor_msgs/JointState`, effort = N·m) and drive the same Unity panel with no
   code change here. For a quick test:

   ```bash
   ros2 topic pub /joint_torques sensor_msgs/msg/JointState \
     '{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
              wrist_1_joint, wrist_2_joint, wrist_3_joint],
       effort: [1.0, 2.0, 3.0, 0.4, 0.5, 0.6]}'
   ```

   To keep RTDE but change the source semantics, implement `TorqueSource`
   (`torque_source.py`) instead.

## Downstream

`joint_torque_bridge_ros2unity` (in package `digital_twin_on_unity_and_ros2`)
subscribes to `/joint_torques` and republishes `JointTorqueUnity` on
`/unity/joint_torques` for Unity.
