import math

import pytest

from roomba_sim.pose2d import compose, inverse, quaternion_yaw, world_to_odom, yaw_from_quaternion


def close(a, b):
    return all(x == pytest.approx(y, abs=1e-9) for x, y in zip(a, b))


def test_inverse_undoes_compose():
    p = (1.0, -2.0, 0.7)
    assert close(compose(p, inverse(p)), (0.0, 0.0, 0.0))


def test_world_to_odom_maps_wheel_pose_onto_true_pose():
    true_pose = (-3.78, 0.36, -1.76)
    wheel_pose = (-0.68, 1.85, -2.43)  # drifted after pushing into a wall
    offset = world_to_odom(true_pose, wheel_pose)
    assert close(compose(offset, wheel_pose), true_pose)


def test_no_drift_gives_the_spawn_pose():
    spawn = (-3.4, 0.0, 0.0)
    wheel = (1.0, 0.5, 0.3)
    assert close(world_to_odom(compose(spawn, wheel), wheel), spawn)


def test_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert yaw_from_quaternion(*quaternion_yaw(yaw)) == pytest.approx(yaw)
    assert yaw_from_quaternion(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)) == pytest.approx(math.pi / 2)
