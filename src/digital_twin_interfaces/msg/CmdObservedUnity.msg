builtin_interfaces/Time stamp

# Observed teleoperation command intent (converted from teleop_msgs/TeleopCommand
# so Unity does not depend on teleop_msgs). OBSERVATION ONLY - not executed.
uint8  control_mode          # 0 IDLE 1 JOINT_POSITION 2 JOINT_VELOCITY
                             # 3 CARTESIAN_POSE 4 CARTESIAN_TWIST 5 GRIPPER
string control_mode_label
bool   enable
bool   emergency_stop
string command_frame

# Joint intent: values are position or velocity depending on control_mode.
string[]  joint_names
float64[] joint_values

# Cartesian twist intent (linear/angular [x, y, z]).
float64[] twist_linear
float64[] twist_angular
