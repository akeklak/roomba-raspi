import pytest

from roomba_sim.realtime_pacer import MAX_TARGET, MIN_TARGET, WINDUP, next_target


def settle(stall: float, periods: int = 30) -> list:
    """Runs the pacer against a sim that loses `stall` wall seconds to rendering per sim second,
    on top of the 1/target it spends stepping; returns the speeds it achieved each period."""
    target, speeds = 1.0, []
    for _ in range(periods):
        measured = 1.0 / (1.0 / target + stall)
        speeds.append(measured)
        target = next_target(target, measured)
    return speeds


def test_makes_up_for_time_lost_to_rendering():
    speeds = settle(stall=0.2)
    assert speeds[0] == pytest.approx(1 / 1.2)
    assert speeds[-1] == pytest.approx(1.0, abs=0.01)


def test_stays_at_real_time_when_nothing_is_lost():
    assert next_target(1.0, 1.0) == pytest.approx(1.0)
    assert settle(stall=0.0)[-1] == pytest.approx(1.0)


def test_backs_off_when_running_fast_but_never_below_real_time():
    # The camera was turned off while the target was raised: the sim runs fast for a moment.
    assert next_target(1.5, 1.5) < 1.5
    assert next_target(1.1, 1.3) == MIN_TARGET


def test_does_not_wind_up_when_the_machine_cannot_keep_up():
    target = 1.0
    for _ in range(50):
        target = next_target(target, 0.6)  # stuck at 0.6x whatever it asks for
    assert target <= WINDUP * 0.6 + 1e-9
    assert target <= MAX_TARGET


def test_ignores_a_sim_that_did_not_move():
    assert next_target(1.3, 0.0) == 1.3
