"""Decides when the Pi should shut down to protect the Roomba battery.

Kept free of ROS imports so it can be unit-tested anywhere with plain pytest.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

# Values of create_msgs/ChargingState.state
CHARGE_NONE = 0
CHARGE_RECONDITION = 1
CHARGE_FULL = 2
CHARGE_TRICKLE = 3
CHARGE_WAITING = 4
CHARGE_FAULT = 5

# States that mean the robot sits on the dock (or a charger is plugged in).
ON_CHARGER_STATES = frozenset({CHARGE_RECONDITION, CHARGE_FULL, CHARGE_TRICKLE, CHARGE_WAITING})


class Action(Enum):
    OK = 'ok'
    WARN = 'warn'
    SHUTDOWN = 'shutdown'


@dataclass
class PowerPolicy:
    shutdown_charge_ratio: float = 0.12
    warn_charge_ratio: float = 0.25
    grace_seconds: float = 120.0
    stale_seconds: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.shutdown_charge_ratio <= self.warn_charge_ratio <= 1.0:
            raise ValueError('need 0 <= shutdown_charge_ratio <= warn_charge_ratio <= 1')
        if self.grace_seconds < 0 or self.stale_seconds < 0:
            raise ValueError('grace_seconds and stale_seconds must be >= 0')
        self._low_since: Optional[float] = None
        self._last_reading: Optional[float] = None
        self._charge_ratio: Optional[float] = None
        self._charging_state: Optional[int] = None

    def on_charge_ratio(self, now: float, ratio: float) -> None:
        self._charge_ratio = ratio
        self._last_reading = now

    def on_charging_state(self, now: float, state: int) -> None:
        self._charging_state = state
        self._last_reading = now

    @property
    def on_charger(self) -> bool:
        return self._charging_state in ON_CHARGER_STATES

    def evaluate(self, now: float) -> Action:
        """Return what to do right now. Call this periodically (e.g. once a second)."""
        if self.stale_seconds > 0 and self._last_reading is not None:
            if now - self._last_reading > self.stale_seconds:
                return Action.SHUTDOWN

        if self._charge_ratio is None or self.on_charger:
            self._low_since = None
            return Action.OK

        if self._charge_ratio < self.shutdown_charge_ratio:
            if self._low_since is None:
                self._low_since = now
            if now - self._low_since >= self.grace_seconds:
                return Action.SHUTDOWN
            return Action.WARN

        self._low_since = None
        if self._charge_ratio < self.warn_charge_ratio:
            return Action.WARN
        return Action.OK
