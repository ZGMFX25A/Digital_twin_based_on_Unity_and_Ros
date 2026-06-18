builtin_interfaces/Time stamp

# Digital IO. Parallel arrays: pin number and its boolean state.
uint8[] digital_in_pins
bool[]  digital_in_states
uint8[] digital_out_pins
bool[]  digital_out_states

# Analog IO. domain: 0 = CURRENT, 1 = VOLTAGE (ur_msgs/Analog convention).
uint8[]   analog_in_pins
float32[] analog_in_values
uint8[]   analog_in_domains
uint8[]   analog_out_pins
float32[] analog_out_values
uint8[]   analog_out_domains
