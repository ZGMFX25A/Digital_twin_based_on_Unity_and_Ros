string[] names
float64[] positions
float64[] velocities

# UR `actual_current` (motor current in amperes, NOT torque in N.m).
# The UR driver maps RTDE actual_current onto /joint_states.effort with no
# conversion, so this is current. Real joint torque (N.m) is published
# separately on /unity/joint_torques (see JointTorqueUnity.msg).
float64[] efforts
