import pytest

from roomba_bringup.power_policy import (
    CHARGE_FAULT,
    CHARGE_FULL,
    CHARGE_NONE,
    CHARGE_WAITING,
    Action,
    PowerPolicy,
)


def make(**kwargs):
    defaults = dict(shutdown_charge_ratio=0.12, warn_charge_ratio=0.25, grace_seconds=120.0)
    defaults.update(kwargs)
    return PowerPolicy(**defaults)


def test_ok_before_any_reading():
    assert make().evaluate(0.0) == Action.OK


def test_ok_when_charge_is_healthy():
    p = make()
    p.on_charging_state(0.0, CHARGE_NONE)
    p.on_charge_ratio(0.0, 0.8)
    assert p.evaluate(1.0) == Action.OK


def test_warn_between_thresholds():
    p = make()
    p.on_charging_state(0.0, CHARGE_NONE)
    p.on_charge_ratio(0.0, 0.2)
    assert p.evaluate(1.0) == Action.WARN


def test_shutdown_only_after_grace_period():
    p = make()
    p.on_charging_state(0.0, CHARGE_NONE)
    p.on_charge_ratio(0.0, 0.05)
    assert p.evaluate(10.0) == Action.WARN
    assert p.evaluate(129.0) == Action.WARN
    assert p.evaluate(130.0) == Action.SHUTDOWN


def test_brief_dip_resets_grace_timer():
    p = make()
    p.on_charging_state(0.0, CHARGE_NONE)
    p.on_charge_ratio(0.0, 0.05)
    assert p.evaluate(100.0) == Action.WARN
    p.on_charge_ratio(101.0, 0.30)  # recovers, e.g. vacuum motor stopped
    assert p.evaluate(101.0) == Action.OK
    p.on_charge_ratio(102.0, 0.05)
    assert p.evaluate(200.0) == Action.WARN  # timer restarted at 200, not 100
    assert p.evaluate(320.0) == Action.SHUTDOWN


@pytest.mark.parametrize('state', [CHARGE_FULL, CHARGE_WAITING])
def test_never_shuts_down_on_the_dock(state):
    p = make()
    p.on_charging_state(0.0, state)
    p.on_charge_ratio(0.0, 0.01)
    assert p.evaluate(1000.0) == Action.OK


def test_charge_fault_is_not_treated_as_docked():
    p = make()
    p.on_charging_state(0.0, CHARGE_FAULT)
    p.on_charge_ratio(0.0, 0.05)
    p.evaluate(0.0)
    assert p.evaluate(200.0) == Action.SHUTDOWN


def test_leaving_dock_starts_grace_timer_fresh():
    p = make()
    p.on_charging_state(0.0, CHARGE_FULL)
    p.on_charge_ratio(0.0, 0.05)
    assert p.evaluate(500.0) == Action.OK
    p.on_charging_state(501.0, CHARGE_NONE)
    assert p.evaluate(501.0) == Action.WARN
    assert p.evaluate(620.0) == Action.WARN
    assert p.evaluate(621.0) == Action.SHUTDOWN


def test_stale_readings_trigger_shutdown_when_enabled():
    p = make(stale_seconds=30.0)
    p.on_charging_state(0.0, CHARGE_NONE)
    p.on_charge_ratio(0.0, 0.9)
    assert p.evaluate(29.0) == Action.OK
    assert p.evaluate(31.0) == Action.SHUTDOWN


def test_stale_check_disabled_by_default():
    p = make()
    p.on_charge_ratio(0.0, 0.9)
    assert p.evaluate(10_000.0) == Action.OK


def test_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        make(shutdown_charge_ratio=0.3, warn_charge_ratio=0.2)
