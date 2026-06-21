"""State machine for stabilizing display-only Cartesian twist commands."""

from enum import Enum


class DisplayAction(Enum):
    """Action requested on one display publisher timer tick."""

    NONE = "none"
    PUBLISH = "publish"
    STOP = "stop"


class DisplayTwistState:
    """Hold intermittent nonzero input without affecting the control stream."""

    _IDLE = "idle"
    _PRIMING = "priming"
    _ACTIVE = "active"

    def __init__(self, initial_hold_s: float, active_timeout_s: float) -> None:
        self.initial_hold_s = max(float(initial_hold_s), 0.0)
        self.active_timeout_s = max(float(active_timeout_s), 0.0)
        self.phase = self._IDLE
        self.first_nonzero_s = 0.0
        self.last_nonzero_s = 0.0

    @property
    def displaying(self) -> bool:
        """Return whether a nonzero command is currently being displayed."""
        return self.phase != self._IDLE

    def observe_nonzero(self, now_s: float) -> None:
        """Record a nonzero sample and advance repeat detection."""
        now_s = float(now_s)
        if self._has_expired(now_s):
            self._reset()

        if self.phase == self._IDLE:
            self.phase = self._PRIMING
            self.first_nonzero_s = now_s
        elif self.phase == self._PRIMING:
            self.phase = self._ACTIVE

        self.last_nonzero_s = now_s

    def force_stop(self) -> bool:
        """Stop immediately and report whether a visible command was cleared."""
        was_displaying = self.displaying
        self._reset()
        return was_displaying

    def tick(self, now_s: float) -> DisplayAction:
        """Return the action for the current timer tick."""
        if self.phase == self._IDLE:
            return DisplayAction.NONE
        if self._has_expired(float(now_s)):
            self._reset()
            return DisplayAction.STOP
        return DisplayAction.PUBLISH

    def _has_expired(self, now_s: float) -> bool:
        if self.phase == self._PRIMING:
            return now_s - self.first_nonzero_s > self.initial_hold_s
        if self.phase == self._ACTIVE:
            return now_s - self.last_nonzero_s > self.active_timeout_s
        return False

    def _reset(self) -> None:
        self.phase = self._IDLE
        self.first_nonzero_s = 0.0
        self.last_nonzero_s = 0.0
