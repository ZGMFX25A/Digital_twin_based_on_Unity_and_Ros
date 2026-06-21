"""Pure keyboard-jog key mapping shared by keyboard_unity_servo (no ROS deps)."""

# Mirrors the key semantics of teleoperation_general's keyboard_servo so the two
# stay in lock-step. If the upstream key bindings change, update this module too.
#
# Jog keys (held, continuous while pressed):
#     w/s : +X / -X        i/k : +Rx / -Rx
#     a/d : +Y / -Y        j/l : +Ry / -Ry
#     q/e : +Z / -Z        u/o : +Rz / -Rz
#
# Action keys (momentary, edge-triggered, one per frame):
#     space : stop (IDLE)        r : record current pose as home
#     h     : go to home         z : record current pose as zero/neutral
#     c     : toggle controller  x : stop and clear held keys

JOG_KEYS = "wsadqeikjluo"

ACTION_STOP = "space"
ACTION_HOME = "h"
ACTION_TOGGLE_CONTROLLER = "c"
ACTION_RECORD_HOME = "r"
ACTION_RECORD_ZERO = "z"
ACTION_STOP_AND_CLEAR = "x"

ACTION_KEYS = frozenset(
    {
        ACTION_STOP,
        ACTION_HOME,
        ACTION_TOGGLE_CONTROLLER,
        ACTION_RECORD_HOME,
        ACTION_RECORD_ZERO,
        ACTION_STOP_AND_CLEAR,
    }
)


def filter_held_keys(held_keys):
    """Keep only recognised jog keys, preserving uniqueness."""
    return "".join(key for key in (held_keys or "") if key in JOG_KEYS)


def held_keys_to_twist(held_keys, linear_step, angular_step):
    """Map currently-held jog keys to ((lx,ly,lz),(ax,ay,az)) twist tuples."""
    # Opposing keys cancel (e.g. "ws" -> 0 on X), matching summed per-key steps.
    keys = set(held_keys or "")

    def axis(positive, negative, step):
        return (int(positive in keys) - int(negative in keys)) * step

    linear = (
        axis("w", "s", linear_step),
        axis("a", "d", linear_step),
        axis("q", "e", linear_step),
    )
    angular = (
        axis("i", "k", angular_step),
        axis("j", "l", angular_step),
        axis("u", "o", angular_step),
    )
    return linear, angular
