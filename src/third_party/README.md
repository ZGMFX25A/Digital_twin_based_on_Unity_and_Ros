# src/third_party — vendored third-party packages

These are **not our packages**. They are vendored (copied from upstream) so the
workspace builds offline and self-contained, with message definitions that match
exactly what the UR driver / teleop package use. `colcon build` discovers them
recursively, so nesting them here does not change the build.

> Do not edit these in place. To update, re-copy from upstream (see each row's
> sync command), then `colcon build`.

## Inventory

| Package | Upstream source | Version | Git tracked | Why vendored |
| --- | --- | --- | --- | --- |
| `ur_dashboard_msgs` | `~/ur_drive/ur_drive_ws/src/Universal_Robots_ROS2_Driver/ur_dashboard_msgs` | matches driver ws | yes (committed) | RobotMode/SafetyMode + dashboard srv used by `ur_state_bridge` / `robot_info_bridge`; vendored to match the driver's exact definitions |
| `ur_msgs` | `~/ur_drive/ur_drive_ws/src/ur_msgs` | 2.5.0 | yes (commit it) | IOStates/ToolDataMsg + GetRobotSoftwareVersion used by M1 bridges / `robot_info_bridge` |
| `teleop_msgs` | `~/digital_twin_on_unity_and_ros2/teleoperation_general_ros2/Teleoperation_general_ros2/src/teleop_msgs` | tracks teleop repo | yes (commit it) | TeleopCommand/Status consumed (read-only) by `command_supervisor`; **not released to apt**, source-only |

> `ROS-TCP-Endpoint` is **not** here. It is the live Unity bridge (its own clone),
> kept at `src/ROS-TCP-Endpoint/` and git-ignored — not a vendored msg lib.

## Tracking policy

For offline / clone-and-build reproducibility, these three small msg packages
(`ur_dashboard_msgs`, `ur_msgs`, `teleop_msgs`) are committed into the repo.

## Sync commands (re-vendor from upstream)

```bash
WS=~/digital_twin_on_unity_and_ros2/digital_twin_on_unity_and_ros2_ws
UR=~/ur_drive/ur_drive_ws
TELE=~/digital_twin_on_unity_and_ros2/teleoperation_general_ros2/Teleoperation_general_ros2

rsync -a --delete --exclude='.git' --exclude='.github' \
  "$UR/src/Universal_Robots_ROS2_Driver/ur_dashboard_msgs/" "$WS/src/third_party/ur_dashboard_msgs/"
rsync -a --delete --exclude='.git' --exclude='.github' \
  "$UR/src/ur_msgs/" "$WS/src/third_party/ur_msgs/"
rsync -a --delete --exclude='.git' --exclude='.github' \
  "$TELE/src/teleop_msgs/" "$WS/src/third_party/teleop_msgs/"
# then: colcon build
```
