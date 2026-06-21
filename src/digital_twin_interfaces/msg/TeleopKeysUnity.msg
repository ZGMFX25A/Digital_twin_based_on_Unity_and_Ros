# Keyboard teleoperation input forwarded from Unity to keyboard_unity_servo.
# held_keys: jog keys currently held; the servo publishes a stable command at a
#   fixed rate from this set (so motion is continuous while a key is held).
# action_key: a single edge-triggered momentary action this frame, "" if none.
string held_keys     # subset of "w s a d q e i k j l u o", concatenated, e.g. "wq"
string action_key    # one of "", "space", "h", "c", "r", "z", "x"
