"""Tests for the Unity keyboard servo jog mapping and action dispatch."""

from digital_twin_on_unity_and_ros2.command_bridges.keyboard_jog import (
    ACTION_KEYS,
    filter_held_keys,
    held_keys_to_twist,
)


LINEAR_STEP = 8.0
ANGULAR_STEP = 5.0


def _twist(held_keys):
    return held_keys_to_twist(held_keys, LINEAR_STEP, ANGULAR_STEP)


def test_linear_jog_keys_map_to_signed_axes():
    assert _twist("w") == ((LINEAR_STEP, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert _twist("s") == ((-LINEAR_STEP, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert _twist("a") == ((0.0, LINEAR_STEP, 0.0), (0.0, 0.0, 0.0))
    assert _twist("d") == ((0.0, -LINEAR_STEP, 0.0), (0.0, 0.0, 0.0))
    assert _twist("q") == ((0.0, 0.0, LINEAR_STEP), (0.0, 0.0, 0.0))
    assert _twist("e") == ((0.0, 0.0, -LINEAR_STEP), (0.0, 0.0, 0.0))


def test_angular_jog_keys_map_to_signed_axes():
    assert _twist("i") == ((0.0, 0.0, 0.0), (ANGULAR_STEP, 0.0, 0.0))
    assert _twist("k") == ((0.0, 0.0, 0.0), (-ANGULAR_STEP, 0.0, 0.0))
    assert _twist("j") == ((0.0, 0.0, 0.0), (0.0, ANGULAR_STEP, 0.0))
    assert _twist("l") == ((0.0, 0.0, 0.0), (0.0, -ANGULAR_STEP, 0.0))
    assert _twist("u") == ((0.0, 0.0, 0.0), (0.0, 0.0, ANGULAR_STEP))
    assert _twist("o") == ((0.0, 0.0, 0.0), (0.0, 0.0, -ANGULAR_STEP))


def test_empty_held_keys_is_zero_twist():
    assert _twist("") == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_combined_keys_sum_per_axis():
    # +X and +Z held together.
    assert _twist("wq") == ((LINEAR_STEP, 0.0, LINEAR_STEP), (0.0, 0.0, 0.0))
    # +X linear and +Rx angular held together.
    assert _twist("wi") == (
        (LINEAR_STEP, 0.0, 0.0),
        (ANGULAR_STEP, 0.0, 0.0),
    )


def test_opposing_keys_cancel():
    # Opposing pairs on the same axis cancel; non-opposing keys do not.
    assert _twist("ws") == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert _twist("uo") == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert _twist("io") == ((0.0, 0.0, 0.0), (ANGULAR_STEP, 0.0, -ANGULAR_STEP))


def test_unknown_keys_are_ignored():
    assert filter_held_keys("w1!d ") == "wd"
    assert _twist("wXYZ") == ((LINEAR_STEP, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_action_keys_cover_documented_set():
    assert ACTION_KEYS == frozenset({"space", "h", "c", "r", "z", "x"})
