"""Checks that a running simulation behaves like the robot: sensors publish and it drives.
It also checks the web bridge: a page connecting to ws://localhost:8765 gets the world, the
lidar and the transforms, and can drive the robot.

    ros2 run roomba_sim smoke_test        (with sim.launch.py already running)

Exits 0 on success, 1 on failure. Used by CI.
"""

import asyncio
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import MarkerArray

EXPECTED_BEAMS = 455


class SmokeTest(Node):

    def __init__(self) -> None:
        super().__init__('sim_smoke_test')
        self.scan = None
        self.image = None
        self.odom = None
        self.world = None
        self.truth = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Image, '/camera/image_raw', lambda m: setattr(self, 'image', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'truth', m), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(MarkerArray, '/sim/world', lambda m: setattr(self, 'world', m), latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, attr: str, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while getattr(self, attr) is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return getattr(self, attr) is not None


def run(timeout: float = 90.0) -> list:
    node = SmokeTest()
    failures = []
    try:
        for attr, topic in (('odom', '/odom'), ('scan', '/scan'), ('image', '/camera/image_raw'),
                            ('world', '/sim/world'), ('truth', '/ground_truth/odom')):
            if not node.wait_for(attr, timeout):
                failures.append(f'no message on {topic} within {timeout:.0f} s')
        if failures:
            return failures

        beams = len(node.scan.ranges)
        hits = sum(1 for r in node.scan.ranges if math.isfinite(r))
        print(f'scan: {beams} beams, {hits} hits, frame {node.scan.header.frame_id}')
        if beams != EXPECTED_BEAMS:
            failures.append(f'scan has {beams} beams, expected {EXPECTED_BEAMS}')
        if hits < beams // 2:
            failures.append(f'only {hits} of {beams} lidar beams hit something; walls missing?')
        print(f'camera: {node.image.width}x{node.image.height}, frame {node.image.header.frame_id}')
        shapes = len(node.world.markers)
        print(f'world: {shapes} shapes for the web UI')
        if shapes < 10:
            failures.append(f'/sim/world has only {shapes} shapes; is the world file parsed?')

        start_x = node.odom.pose.pose.position.x
        cmd = Twist()
        cmd.linear.x = 0.3
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            node.cmd_pub.publish(cmd)
            node.spin_for(0.1)
        node.cmd_pub.publish(Twist())
        node.spin_for(0.5)
        moved = node.odom.pose.pose.position.x - start_x
        print(f'drove {moved:.2f} m forward in 3 s at 0.3 m/s')
        if moved < 0.4:
            failures.append(f'robot moved only {moved:.2f} m on /cmd_vel (expected about 0.9 m)')

        failures += check_web_bridge(node)
    finally:
        node.destroy_node()
    return failures


async def _web_bridge_session(node: SmokeTest, url: str) -> tuple:
    import websockets  # only needed for this check

    seen = set()
    start_x = node.odom.pose.pose.position.x
    async with websockets.connect(url, max_size=None) as ws:
        end = time.monotonic() + 15
        while time.monotonic() < end and not {'hello', 'world', 'scan', 'tf'} <= seen:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            if isinstance(raw, str):
                seen.add(json.loads(raw).get('op'))
        # Drive forward through the bridge for 2 s, as the page does (10 commands a second).
        for _ in range(20):
            await ws.send(json.dumps({'op': 'drive', 'linear': 0.3, 'angular': 0.0}))
            node.spin_for(0.1)
        # Then go quiet: the bridge's watchdog should stop the robot by itself.
    node.spin_for(1.5)
    moved = node.odom.pose.pose.position.x - start_x
    speed = node.odom.twist.twist.linear.x
    return seen, moved, speed


def check_web_bridge(node: SmokeTest, url: str = 'ws://localhost:8765') -> list:
    try:
        seen, moved, speed = asyncio.run(_web_bridge_session(node, url))
    except Exception as e:  # noqa: BLE001 - report any failure to connect as a test failure
        return [f'web bridge at {url}: {e!r}']
    print(f'web bridge: got {sorted(seen)}, drove {moved:.2f} m, speed after stopping {speed:.2f} m/s')
    failures = []
    missing = {'hello', 'world', 'scan', 'tf'} - seen
    if missing:
        failures.append(f'web bridge never sent {sorted(missing)}')
    if moved < 0.25:
        failures.append(f'driving through the web bridge moved only {moved:.2f} m (expected about 0.6 m)')
    if abs(speed) > 0.05:
        failures.append(f'robot still moving at {speed:.2f} m/s after drive commands stopped')
    return failures


def main() -> None:
    rclpy.init()
    try:
        failures = run()
    finally:
        rclpy.shutdown()
    if failures:
        for f in failures:
            print(f'FAIL: {f}')
        sys.exit(1)
    print('PASS: simulation smoke test')


if __name__ == '__main__':
    main()
