"""End-to-end check of mapping and navigation in the running sim, done the way the web UI does it:
through the web bridge at ws://localhost:8765.

    ros2 run roomba_nav nav_test        (with sim.launch.py running; CI runs it)

1. Mapping: the map appears, and the robot drives itself into the bedroom through a doorway.
2. Saves the map and switches to navigating on it (AMCL), then drives to the kitchen.
3. Stopping: a goal is canceled from the page, and another by driving by hand.
4. Adds to the saved map (mapping on top of it), drives to the living room, and switches back.

Where the robot really ends up is checked against Gazebo's true position. Exits 0 on success.
"""

import asyncio
import json
import math
import struct
import sys
import time
import zlib

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Goals in the map frame. The map starts where the robot spawns: world (-3.4, 0), facing east.
SPAWN = (-3.4, 0.0)
BEDROOM = (5.9, -0.5, 0.0)       # world (2.5, -0.5): through the doorway at x = 1
KITCHEN = (1.4, -1.6, -1.57)     # world (-2.0, -1.6): through the doorway at y = -0.5
LIVING_ROOM = (2.6, 1.9, 1.57)   # world (-0.8, 1.9)
TOLERANCE = 0.35                 # meters between the goal and where Gazebo says the robot is


class Truth(Node):

    def __init__(self) -> None:
        super().__init__('nav_test_truth')
        self.pose = None
        self.speed = None
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 10)

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)


class Page:
    """A pretend web page."""

    def __init__(self, ws, truth: Truth) -> None:
        self.ws = ws
        self.truth = truth
        self.latest = {}
        self.notices = []
        self.plans = 0
        self.nav_updates = 0
        self.map_header = None
        self.map_cells = b''

    async def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.truth, timeout_sec=0)
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            if isinstance(raw, bytes):
                if raw[0] == 2:
                    (n,) = struct.unpack('<I', raw[1:5])
                    header = json.loads(raw[5:5 + n])
                    if header['kind'] == 'map':
                        self.map_header = header
                        self.map_cells = zlib.decompress(raw[5 + n:])
                continue
            msg = json.loads(raw)
            op = msg.get('op')
            self.latest[op] = msg
            if op == 'notice':
                self.notices.append(msg)
                print(f'  notice ({msg["level"]}): {msg["text"]}')
            elif op == 'plan' and msg['points']:
                self.plans += 1
            elif op == 'nav':
                self.nav_updates += 1

    async def wait_for(self, check, timeout: float, what: str) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check():
                return
            await self.pump(0.2)
        raise AssertionError(f'timed out after {timeout:.0f} s waiting for {what}')

    async def send(self, **msg) -> None:
        await self.ws.send(json.dumps(msg))

    def mapping(self) -> dict:
        return self.latest.get('mapping', {})

    def nav(self) -> dict:
        return self.latest.get('nav', {})

    async def map_command(self, command: str, mode: str, timeout: float = 60.0) -> None:
        before = len(self.notices)
        await self.send(op='map_command', command=command)
        await self.wait_for(lambda: len(self.notices) > before, 30, f'an answer to {command}')
        notice = self.notices[-1]
        assert notice['level'] == 'info', f'{command} failed: {notice["text"]}'
        await self.wait_for(lambda: self.mapping().get('mode') == mode and not self.mapping().get('busy'),
                            timeout, f'mode {mode}')

    async def go(self, goal: tuple, name: str, timeout: float = 180.0) -> None:
        """Sends a goal (retrying while Nav2 is still starting) and waits until it arrives."""
        x, y, yaw = goal
        start = time.monotonic()
        plans_before = self.plans
        while True:
            updates = self.nav_updates
            await self.send(op='goal', x=x, y=y, yaw=yaw)
            await self.wait_for(lambda: self.nav_updates > updates and self.nav().get('goal')
                                and self.nav()['goal']['x'] == x, 10, 'the goal to be taken')
            await self.wait_for(lambda: self.nav()['state'] not in ('starting', 'active'), timeout, f'{name}')
            nav = self.nav()
            if nav['state'] == 'failed' and 'not running yet' in nav['message'] and time.monotonic() - start < 90:
                await self.pump(2.0)
                continue
            break
        took = time.monotonic() - start
        assert nav['state'] == 'succeeded', f'{name}: {nav["state"]}: {nav["message"]}'
        assert self.plans > plans_before, f'{name}: the page never got the planned path'
        print(f'  reached the {name} in {took:.0f} s')
        self.check_truth(goal, name)

    def check_truth(self, goal: tuple, name: str) -> None:
        if self.truth.pose is None:
            print('  (no ground truth: not in the sim?)')
            return
        world_goal = (goal[0] + SPAWN[0], goal[1] + SPAWN[1])
        error = math.dist(self.truth.pose, world_goal)
        print(f'  Gazebo says the robot is {error:.2f} m from the {name} goal')
        assert error < TOLERANCE, f'{name}: robot is really {error:.2f} m from the goal'

    async def stops_when(self, how: str, action) -> None:
        await self.send(op='goal', x=LIVING_ROOM[0], y=LIVING_ROOM[1], yaw=LIVING_ROOM[2])
        await self.wait_for(lambda: self.nav().get('state') == 'active' and (self.nav().get('distance') or 0) > 0,
                            30, 'navigation to start')
        await self.pump(3.0)
        await action()
        await self.wait_for(lambda: self.nav()['state'] == 'canceled', 15, f'the goal to be canceled ({how})')
        await self.pump(1.5)
        speed = self.truth.speed or 0.0
        print(f'  {how}: canceled ("{self.nav()["message"]}"), speed now {speed:.2f} m/s')
        assert speed < 0.05, f'{how}: robot still moving at {speed:.2f} m/s'


async def run(url: str) -> None:
    import websockets

    truth = Truth()
    async with websockets.connect(url, max_size=None) as ws:
        page = Page(ws, truth)
        await page.wait_for(lambda: page.mapping().get('mode') in ('mapping', 'navigation'), 90,
                            'map_manager to start')
        if page.mapping()['mode'] != 'mapping' or page.mapping().get('extending'):
            print('a saved map exists; starting a new map for the test')
            await page.map_command('new_map', 'mapping')

        print('1. mapping')
        await page.wait_for(lambda: page.map_header is not None, 60, 'the map')
        await page.wait_for(lambda: page.mapping().get('nav_ready'), 120, 'navigation to start')
        h = page.map_header
        print(f'  map {h["width"]} x {h["height"]} cells at {h["resolution"]:.2f} m, frame {h["frame"]}')
        assert len(page.map_cells) == h['width'] * h['height'], 'map cells do not match its size'
        await page.go(BEDROOM, 'bedroom')
        occupied = sum(1 for c in page.map_cells if c == 100)
        print(f'  map now {page.map_header["width"]} x {page.map_header["height"]}, {occupied} wall cells')
        assert occupied > 200, 'the map has hardly any walls in it'

        print('2. save the map and navigate on it')
        await page.map_command('save_map', 'mapping')
        await page.map_command('use_saved_map', 'navigation')
        assert page.mapping()['saved'], 'no saved map reported'
        await page.go(KITCHEN, 'kitchen')

        print('3. stopping')
        await page.stops_when('Stop button', lambda: page.send(op='cancel'))
        await page.stops_when('driving by hand', lambda: page.send(op='drive', linear=0.0, angular=0.5))

        print('4. add to the saved map, then use it again')
        await page.map_command('extend_map', 'mapping')
        assert page.mapping().get('extending'), 'not mapping on top of the saved map'
        await page.go(LIVING_ROOM, 'living room')
        await page.map_command('use_saved_map', 'navigation')
    truth.destroy_node()


def main() -> None:
    rclpy.init()
    try:
        asyncio.run(run('ws://localhost:8765'))
    except AssertionError as e:
        print(f'FAIL: {e}')
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('PASS: mapping and navigation test')


if __name__ == '__main__':
    main()
