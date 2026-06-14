builtin_interfaces/Time stamp

# UR7e standard joint order:
# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
string[] names

# Real joint torques in N.m. Source depends on the data-source layer's
# rtde_field: `actual_current_as_torque` (measured) or `target_moment`
# (commanded; label as "couple cible" in the UI when used).
float64[] torques_nm
