"""Checks that a running simulation behaves like the robot: sensors publish and it drives.

    ros2 run roomba_sim smoke_test        (with sim.launch.py already running)

Exits 0 on success, 1 on failure. Used by CI.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

EXPECTED_BEAMS = 455


class SmokeTest(Node):

    def __init__(self) -> None:
        super().__init__('sim_smoke_test')
        self.scan = None
        self.image = None
        self.odom = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Image, '/camera/image_raw', lambda m: setattr(self, 'image', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
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
        for attr, topic in (('odom', '/odom'), ('scan', '/scan'), ('image', '/camera/image_raw')):
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
    finally:
        node.destroy_node()
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
