"""Keeps the simulation's clock in step with the wall clock.

Gazebo paces itself one physics step at a time: it sleeps after each step so that a step takes
max_step_size / real_time_factor of wall time. While it renders the camera and lidar (in software
on a Mac, inside Docker) the physics waits, and that time is never made up. In the kitchen the
camera costs about a quarter of every second, so the sim ran at about 0.7x real time: the clock,
the robot and everything that follows it moved slower than they should.

This watches how fast sim time actually advances and raises Gazebo's target real-time factor just
enough to make up for the time spent rendering, so the sim runs at 1x overall. With nothing to
make up (the camera off, a simple world) the target settles back to 1. It also sets the physics
step, which is how much sim time each step covers. A step costs about the same however long it
is, so longer steps leave time to catch up: with the web page and camera open, the kitchen
topped out at 0.9x with 2 ms steps and at 1.6x with 4 ms, and the robot drives and turns the
same (measured against Gazebo's true pose).

Runs next to Gazebo in the sim only (sim.launch.py); it talks to Gazebo directly, not over ROS.
"""

import re
import sys
import threading
import time
from typing import Optional, Tuple

# The target never drops below real time, nor runs away if the machine simply can't keep up.
MIN_TARGET = 1.0
MAX_TARGET = 3.0
STEP_SIZE = 0.004            # physics step, seconds of sim time
PERIOD = 0.5                 # how often to measure and adjust, wall seconds
GAIN = 0.7                   # how much of the measured shortfall to correct each period
WINDUP = 2.5                 # the target stays below this times the speed it achieved
DEADBAND = 0.01              # don't bother Gazebo with changes smaller than this (1%)


def next_target(target: float, measured: float, gain: float = GAIN,
                lo: float = MIN_TARGET, hi: float = MAX_TARGET) -> float:
    """The real-time factor to ask Gazebo for next, given the one asked for now (`target`) and
    the one it achieved over the last period (`measured`, sim seconds per wall second).

    Running at `measured` while asking for `target` means each period loses a share to work
    outside the physics steps, so asking for more makes up for it; repeating this settles on 1x.
    Only part of each correction is taken (`gain`) so one noisy measurement can't make it swing,
    and the target stays within WINDUP of what was achieved, so that a machine that simply
    can't keep up doesn't wind it up to a burst of fast-forward once it can. Running fast (the
    camera was just turned off) is corrected in one go, so that lasts a single period."""
    if measured <= 0.0:
        return target
    wanted = target / measured
    step = target * (wanted / target) ** (1.0 if measured > 1.0 else gain)
    cap = max(lo, min(hi, WINDUP * measured))
    return min(cap, max(lo, step))


def _seconds(duration) -> float:
    return duration.sec + duration.nsec * 1e-9


class Pacer:

    def __init__(self) -> None:
        from gz.transport13 import Node
        self.node = Node()
        self.lock = threading.Lock()
        self.latest: Optional[Tuple[float, float, bool]] = None  # (sim, real, paused)
        self.target = 1.0

    def on_stats(self, msg) -> None:
        with self.lock:
            self.latest = (_seconds(msg.sim_time), _seconds(msg.real_time), msg.paused)

    def world(self) -> str:
        """The world's name, once Gazebo is serving it."""
        while True:
            for service in self.node.service_list():
                m = re.fullmatch(r'/world/([^/]+)/set_physics', service)
                if m:
                    return m.group(1)
            time.sleep(1.0)

    def set_physics(self, world: str, rtf: float) -> bool:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.physics_pb2 import Physics
        req = Physics(max_step_size=STEP_SIZE, real_time_factor=rtf)
        ok, rep = self.node.request(f'/world/{world}/set_physics', req, Physics, Boolean, 2000)
        return bool(ok and rep.data)

    def run(self) -> None:
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        world = self.world()
        # The world's own stats come ten times a second (/stats comes every step).
        self.node.subscribe(WorldStatistics, f'/world/{world}/stats', self.on_stats)
        while not self.set_physics(world, self.target):
            time.sleep(1.0)
        log(f'keeping "{world}" at real time ({STEP_SIZE * 1000:g} ms physics steps)')
        prev = None
        last_logged = self.target
        while True:
            time.sleep(PERIOD)
            with self.lock:
                now = self.latest
            if now is None or now[2]:  # no stats yet, or paused
                prev = None
                continue
            if prev is not None and now[1] - prev[1] > 0.5 * PERIOD:
                measured = (now[0] - prev[0]) / (now[1] - prev[1])
                target = next_target(self.target, measured)
                if abs(target - self.target) > DEADBAND * self.target and self.set_physics(world, target):
                    self.target = target
                    if abs(target - last_logged) > 0.25:
                        log(f'rendering is slowing the sim (it ran at {measured:.2f}x), so Gazebo '
                            f'now aims for {target:.2f}x to keep up')
                        last_logged = target
            prev = now


def log(text: str) -> None:
    print(f'[realtime_pacer] {text}', flush=True)


def main() -> None:
    del sys.argv[1:]  # launch may pass --ros-args; there is nothing to configure
    try:
        Pacer().run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
