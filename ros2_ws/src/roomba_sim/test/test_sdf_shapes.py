import math
import os

from roomba_sim.sdf_shapes import parse_world, quaternion

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'apartment.sdf')


def test_apartment_has_walls_furniture_and_legs():
    shapes = parse_world(WORLD)
    names = {s['name'] for s in shapes}
    assert 'wall_north_visual' in names
    assert 'couch_visual' in names
    assert 'dock_visual' in names
    legs = [s for s in shapes if s['kind'] == 'cylinder']
    assert len(legs) == 8


def test_box_pose_size_and_colour():
    shapes = {s['name']: s for s in parse_world(WORLD)}
    couch = shapes['couch_visual']
    assert couch['kind'] == 'box'
    assert couch['pose'][:3] == (-2.4, 2.45, 0.2)
    assert couch['size'] == [2.0, 0.9, 0.4]
    assert couch['rgba'] == [0.45, 0.3, 0.25, 1.0]


def test_cylinder_size_is_diameter_and_length():
    shapes = {s['name']: s for s in parse_world(WORLD)}
    leg = shapes['coffee_table_leg_0_visual']
    assert leg['size'] == [0.05, 0.05, 0.415]


def test_floor_plane_is_skipped():
    # The floor model's only visual (named "visual") is a plane, which is not a box or cylinder.
    assert 'visual' not in {s['name'] for s in parse_world(WORLD)}


def test_quaternion_yaw():
    x, y, z, w = quaternion(0.0, 0.0, math.pi / 2)
    assert abs(x) < 1e-9 and abs(y) < 1e-9
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9
