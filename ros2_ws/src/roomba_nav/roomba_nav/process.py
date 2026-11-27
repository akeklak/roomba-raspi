"""Runs one child process at a time (a `ros2 launch` of mapping or localization) and stops it cleanly.

No ROS imports, so this can be unit tested anywhere.
"""

import signal
import subprocess
from typing import Callable, List, Optional


class ChildProcess:

    def __init__(self, log: Callable[[str], None] = print) -> None:
        self.log = log
        self.proc: Optional[subprocess.Popen] = None
        self.argv: List[str] = []

    def start(self, argv: List[str]) -> None:
        """Stops whatever is running, then starts `argv`. Its output goes to ours."""
        self.stop()
        self.argv = list(argv)
        self.log('starting: ' + ' '.join(self.argv))
        self.proc = subprocess.Popen(self.argv)

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def exit_code(self) -> Optional[int]:
        """None while running or when nothing was started."""
        return None if self.proc is None else self.proc.poll()

    def stop(self, timeout: float = 20.0) -> None:
        """Ctrl-C, like you would in a terminal: `ros2 launch` then shuts its nodes down in order.
        Kills it if it hasn't finished after `timeout` seconds."""
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        self.log('stopping: ' + ' '.join(self.argv))
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.log('did not stop in time; killing it')
            proc.kill()
            proc.wait(5)
