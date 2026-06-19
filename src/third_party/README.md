# src/third_party — vendored message packages

These are **not our packages**. They are vendored (copied from upstream) so the
workspace builds offline and self-contained, with message definitions that match
exactly what the UR driver / teleop package use. `colcon build` discovers them
recursively, so nesting them here does not change the build.

> Do not edit these in place. To update, re-copy from upstream (see *Re-sync*
> below), then `colcon build`.

## Inventory

| Package | Upstream (GitHub) | Used by |
| --- | --- | --- |
| `ur_dashboard_msgs` | [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) (`humble`, subdir `ur_dashboard_msgs`) | `ur_state_bridge` / `robot_info_bridge` (RobotMode / SafetyMode + dashboard srv) |
| `ur_msgs` | [ros-industrial/ur_msgs](https://github.com/ros-industrial/ur_msgs) (`humble`) | M1 bridges / `robot_info_bridge` (IOStates / ToolDataMsg + GetRobotSoftwareVersion) |
| `teleop_msgs` | [ning2407/Teleoperation_general_ros2](https://github.com/ning2407/Teleoperation_general_ros2) (subdir `src/teleop_msgs`) | `command_supervisor` (TeleopCommand / TeleopStatus, observed read-only) |

All three are small msg-only packages and are **committed** into this repo for
offline / clone-and-build reproducibility. `teleop_msgs` is source-only (not
released to apt).

> `ROS-TCP-Endpoint` is **not** here. It is the live Unity bridge (its own clone
> of [Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)),
> kept at `src/ROS-TCP-Endpoint/` and git-ignored — not a vendored msg lib.

## Re-sync from upstream

Clone each upstream repo (URLs above) and copy the relevant package directory
over the vendored copy, excluding VCS metadata, then rebuild:

```bash
WS=~/digital_twin_on_unity_and_ros2/digital_twin_on_unity_and_ros2_ws

rsync -a --delete --exclude='.git' --exclude='.github' \
  <ur_driver_clone>/ur_dashboard_msgs/ "$WS/src/third_party/ur_dashboard_msgs/"
rsync -a --delete --exclude='.git' --exclude='.github' \
  <ur_msgs_clone>/ "$WS/src/third_party/ur_msgs/"
rsync -a --delete --exclude='.git' --exclude='.github' \
  <teleop_clone>/src/teleop_msgs/ "$WS/src/third_party/teleop_msgs/"
# then: colcon build
```
