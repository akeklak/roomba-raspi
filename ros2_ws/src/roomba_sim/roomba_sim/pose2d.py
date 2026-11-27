"""Planar (x, y, yaw) pose math, with no ROS dependency so it can be unit tested anywhere."""

import math
from typing import Tuple

Pose2D = Tuple[float, float, float]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a then b: the pose b, given relative to a, expressed in a's parent frame."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, normalize(ath + bth))


def inverse(p: Pose2D) -> Pose2D:
    x, y, th = p
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, normalize(-th))


def normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_odom(world_base: Pose2D, odom_base: Pose2D) -> Pose2D:
    """The world -> odom offset that puts the wheel-odometry pose onto the true pose."""
    return compose(world_base, inverse(odom_base))


def quaternion_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation of `yaw` about Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
