import sys
import time

from roomba_nav.process import ChildProcess

SLEEPER = [sys.executable, '-c', 'import time\ntry:\n    time.sleep(30)\nexcept KeyboardInterrupt:\n    pass']


def test_start_and_stop_with_ctrl_c():
    child = ChildProcess(log=lambda _: None)
    assert not child.running() and child.exit_code() is None
    child.start(SLEEPER)
    time.sleep(0.3)
    assert child.running()
    start = time.monotonic()
    child.stop()
    assert time.monotonic() - start < 5  # it handled Ctrl-C rather than waiting to be killed
    assert not child.running()


def test_starting_again_stops_the_first():
    child = ChildProcess(log=lambda _: None)
    child.start(SLEEPER)
    first = child.proc
    child.start(SLEEPER)
    assert first.poll() is not None
    child.stop()


def test_reports_a_child_that_died():
    child = ChildProcess(log=lambda _: None)
    child.start([sys.executable, '-c', 'raise SystemExit(3)'])
    child.proc.wait(5)
    assert not child.running()
    assert child.exit_code() == 3


def test_kills_a_child_that_ignores_ctrl_c():
    child = ChildProcess(log=lambda _: None)
    child.start([sys.executable, '-c', 'import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                 'print("ready", flush=True)\ntime.sleep(30)'])
    time.sleep(0.5)
    proc = child.proc
    child.stop(timeout=0.5)
    assert proc.poll() is not None
