builtin_interfaces/Time stamp

# UR tool flange data (ur_msgs/ToolDataMsg passed through).
float32 tool_voltage_48v       # volts
uint8   tool_output_voltage    # volts (0/12/24)
float32 tool_current           # amperes
float32 tool_temperature       # celsius

# 249 = BOOTLOADER, 253 = RUNNING, 255 = IDLE. Label mapped by the bridge.
uint8   tool_mode
string  tool_mode_label
