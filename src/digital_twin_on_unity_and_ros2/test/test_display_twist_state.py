"""Tests for the display-only twist hold state machine."""

from digital_twin_on_unity_and_ros2.command_bridges.display_twist_state import (
    DisplayAction,
    DisplayTwistState,
)


def test_first_sample_is_held_through_initial_repeat_delay():
    state = DisplayTwistState(initial_hold_s=0.65, active_timeout_s=0.25)

    state.observe_nonzero(1.0)

    assert state.tick(1.60) == DisplayAction.PUBLISH
    assert state.tick(1.651) == DisplayAction.STOP
    assert state.tick(1.70) == DisplayAction.NONE


def test_second_sample_switches_to_short_active_timeout():
    state = DisplayTwistState(initial_hold_s=0.65, active_timeout_s=0.25)

    state.observe_nonzero(1.0)
    state.observe_nonzero(1.5)

    assert state.tick(1.749) == DisplayAction.PUBLISH
    assert state.tick(1.751) == DisplayAction.STOP


def test_active_sample_refreshes_release_timeout():
    state = DisplayTwistState(initial_hold_s=0.65, active_timeout_s=0.25)

    state.observe_nonzero(1.0)
    state.observe_nonzero(1.5)
    state.observe_nonzero(1.7)

    assert state.tick(1.949) == DisplayAction.PUBLISH
    assert state.tick(1.951) == DisplayAction.STOP


def test_late_second_sample_starts_a_new_priming_window():
    state = DisplayTwistState(initial_hold_s=0.65, active_timeout_s=0.25)

    state.observe_nonzero(1.0)
    state.observe_nonzero(1.7)

    assert state.phase == state._PRIMING
    assert state.tick(2.349) == DisplayAction.PUBLISH
    assert state.tick(2.351) == DisplayAction.STOP


def test_force_stop_is_immediate_and_only_reported_once():
    state = DisplayTwistState(initial_hold_s=0.65, active_timeout_s=0.25)
    state.observe_nonzero(1.0)

    assert state.force_stop() is True
    assert state.force_stop() is False
    assert state.tick(1.1) == DisplayAction.NONE
