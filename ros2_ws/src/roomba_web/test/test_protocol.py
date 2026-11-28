import json
import math
import struct
from types import SimpleNamespace as NS

from roomba_web import protocol


def header(frame='base', sec=12, nanosec=500_000_000):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def test_transforms_strip_leading_slash_and_round():
    tr = NS(header=header('/odom'), child_frame_id='base_footprint',
            transform=NS(translation=NS(x=1.234567891, y=0, z=0), rotation=NS(x=0, y=0, z=0, w=1)))
    out = protocol.transforms(NS(transforms=[tr]), static=False)
    assert out == [{'parent': 'odom', 'child': 'base_footprint', 't': [1.23457, 0.0, 0.0],
                    'q': [0.0, 0.0, 0.0, 1.0], 'stamp': 12.5, 'static': False}]
    json.dumps(out)


def test_scan_sends_misses_as_null():
    msg = NS(header=header('ldlidar_link'), angle_min=-3.14, angle_increment=0.01, range_min=0.03,
             range_max=12.0, ranges=[1.23456, math.inf, float('nan')])
    out = protocol.scan(msg)
    assert out['ranges'] == [1.235, None, None]
    assert out['frame'] == 'ldlidar_link'
    json.dumps(out, allow_nan=False)


def test_world_keeps_boxes_and_cylinders():
    def marker(type_, action=0):
        return NS(type=type_, action=action, ns='couch_visual', id=type_, header=header('world'),
                  pose=NS(position=NS(x=1, y=2, z=0.2), orientation=NS(x=0, y=0, z=0, w=1)),
                  scale=NS(x=2, y=0.9, z=0.4), color=NS(r=0.4, g=0.3, b=0.2, a=1))
    out = protocol.world(NS(markers=[marker(1), marker(3), marker(2), marker(1, action=2)]))
    assert [s['kind'] for s in out['shapes']] == ['box', 'cylinder']
    assert out['shapes'][0]['size'] == [2, 0.9, 0.4]


def test_camera_frame_layout_and_jpeg():
    img = NS(header=header('camera_optical_link'), width=2, height=1, encoding='bgr8', step=6,
             data=bytes([0, 0, 255, 255, 0, 0]))
    data = protocol.jpeg(img)
    assert data[:2] == b'\xff\xd8'
    frame = protocol.camera_frame(img, data, {'fx': 1})
    assert frame[0] == protocol.CAMERA_FRAME
    (n,) = struct.unpack('<I', frame[1:5])
    head = json.loads(frame[5:5 + n])
    assert head['width'] == 2 and head['model'] == {'fx': 1}
    assert frame[5 + n:] == data
    assert protocol.jpeg(NS(encoding='16uc1')) is None


def test_clamp_drive():
    assert protocol.clamp_drive({'linear': 2, 'angular': -9}, 0.5, 2.0) == (0.5, -2.0)
    assert protocol.clamp_drive({'linear': 'fast'}, 0.5, 2.0) == (0.0, 0.0)
    assert protocol.clamp_drive({'linear': float('nan')}, 0.5, 2.0) == (0.0, 0.0)
    assert protocol.clamp_drive({}, 0.5, 2.0) == (0.0, 0.0)


def test_grid_frame_packs_header_and_compressed_cells():
    import zlib
    q = NS(x=0.0, y=0.0, z=0.0, w=1.0)
    msg = NS(header=header('/map'), info=NS(width=3, height=2, resolution=0.05,
                                            origin=NS(position=NS(x=-1.0, y=2.0, z=0.0), orientation=q)),
             data=[0, 100, -1, 0, 50, -1])
    out = protocol.grid_frame(msg, 'map')
    assert out[0] == protocol.GRID_FRAME
    (n,) = struct.unpack('<I', out[1:5])
    head = json.loads(out[5:5 + n])
    assert head['kind'] == 'map' and head['frame'] == 'map'
    assert (head['width'], head['height'], head['resolution']) == (3, 2, 0.05)
    assert head['origin']['x'] == -1.0 and head['origin']['q'] == [0.0, 0.0, 0.0, 1.0]
    assert list(zlib.decompress(out[5 + n:])) == [0, 100, 255, 0, 50, 255]


def test_path_is_thinned_but_keeps_the_end():
    poses = [NS(pose=NS(position=NS(x=i * 0.01, y=0.0, z=0.0))) for i in range(101)]
    out = protocol.path(NS(header=header('map'), poses=poses))
    assert out['op'] == 'plan' and out['frame'] == 'map'
    assert len(out['points']) < 30
    assert out['points'][0] == [0.0, 0.0] and out['points'][-1] == [1.0, 0.0]
    assert protocol.path(NS(header=header('map'), poses=[]))['points'] == []


def test_parse_pose_accepts_numbers_only():
    assert protocol.parse_pose({'x': 1, 'y': '2.5', 'yaw': 0.5}) == (1.0, 2.5, 0.5)
    assert protocol.parse_pose({'x': 1, 'y': 2}) == (1.0, 2.0, 0.0)
    assert protocol.parse_pose({'x': 1}) is None
    assert protocol.parse_pose({'x': 'a', 'y': 2}) is None
    assert protocol.parse_pose({'x': float('nan'), 'y': 2}) is None
    assert protocol.parse_pose({'x': None, 'y': 2}) is None


def test_yaw_quaternion():
    x, y, z, w = protocol.yaw_quaternion(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9


def test_nav_errors_in_plain_words():
    assert protocol.nav_error(208) == 'there is no way through to that spot'
    assert protocol.nav_error(999, 'odd failure') == 'odd failure'
    assert protocol.nav_error(999) == 'Nav2 error 999'
    assert 'stuck' in protocol.nav_error(0)


def static_tf(parent, child, t, yaw=0.0):
    return {'parent': parent, 'child': child, 't': t, 'q': list(protocol.yaw_quaternion(yaw)), 'static': True}


def test_static_pose_chains_fixed_transforms():
    static = {
        'base_link': static_tf('base_footprint', 'base_link', [0, 0, 0.03]),
        'lidar': static_tf('base_link', 'lidar', [-0.02, 0.01, 0.1], yaw=math.pi / 2),
    }
    xx, xy, yx, yy, x, y = protocol.static_pose(static, '/lidar', 'base_footprint')
    assert (x, y) == (-0.02, 0.01)
    # The lidar's x axis points along the robot's y axis.
    assert abs(xx) < 1e-9 and abs(yx - 1) < 1e-9 and abs(xy + 1) < 1e-9 and abs(yy) < 1e-9
    assert protocol.static_pose(static, 'camera', 'base_footprint') is None
    assert protocol.static_pose(static, 'base_footprint', 'base_footprint') == (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def test_nearby_points_keeps_close_hits_on_the_robot():
    scan = NS(angle_min=0.0, angle_increment=math.pi / 2, range_min=0.05, range_max=8.0,
              ranges=[0.3, float('inf'), 2.0, 0.01])
    pts = protocol.nearby_points(scan, (1.0, 0.0, 0.0, 1.0, -0.02, 0.0), reach=0.5)
    assert len(pts) == 1 and abs(pts[0][0] - 0.28) < 1e-9 and abs(pts[0][1]) < 1e-9


def test_free_travel_stops_at_a_wall_but_lets_you_back_away():
    wall = [(0.30, y / 100) for y in range(-30, 31, 2)]  # a wall 30 cm in front of the center
    ahead = protocol.free_travel(wall, 0.3, 0.0, 0.19, 0.15, 0.3)
    assert abs(ahead - 0.11) < 0.011
    assert protocol.free_travel(wall, -0.3, 0.0, 0.19, 0.15, 0.3) == 0.3
    assert protocol.free_travel(wall, 0.0, 1.0, 0.19, 0.15, 0.3) == 0.3
    # Already touching it: can't go on, can still back away.
    touching = [(0.18, y / 100) for y in range(-30, 31, 2)]
    assert protocol.free_travel(touching, 0.3, 0.0, 0.19, 0.15, 0.3) == 0.0
    assert protocol.free_travel(touching, -0.3, 0.0, 0.19, 0.15, 0.3) == 0.3


def test_free_travel_follows_the_turn():
    # A post beside the robot on its left: straight on misses it, curving left runs into it.
    post = [(0.15, 0.25)]
    assert protocol.free_travel(post, 0.3, 0.0, 0.19, 0.15, 0.3) == 0.3
    assert protocol.free_travel(post, 0.3, 1.5, 0.19, 0.15, 0.3) < 0.2
    assert protocol.free_travel(post, 0.3, -1.5, 0.19, 0.15, 0.3) == 0.3
    # Points inside the robot's own body are ignored.
    assert protocol.free_travel([(0.05, 0.0)], 0.3, 0.0, 0.19, 0.15, 0.3) == 0.3


def test_guard_speed_slows_then_stops():
    assert protocol.guard_speed(0.5, 0.3) == 0.5
    assert abs(protocol.guard_speed(0.5, 0.11) - 0.25) < 1e-9
    assert abs(protocol.guard_speed(-0.5, 0.11) + 0.25) < 1e-9
    assert protocol.guard_speed(0.3, 0.02) == 0.0
    assert protocol.guard_speed(0.3, 0.0) == 0.0


def room_points(x, y):
    """What a lidar at (x, y) facing +x sees of a 4 x 2 m room centered on the origin."""
    pts = []
    for k in range(-40, 41):
        pts += [(2.0 - x, k / 40 - y), (-2.0 - x, k / 40 - y)]           # end walls
        pts += [(2 * k / 40 - x, 1.0 - y), (2 * k / 40 - x, -1.0 - y)]   # side walls
    return pts


def test_stalled_when_the_wheels_turn_but_the_view_stays():
    view = room_points(0, 0)
    # Odometry and lidar agree it drove 15 cm forward: fine.
    drove = [(0.0, 0.0, 0.0, view), (0.15, 0.0, 0.0, room_points(0.15, 0))]
    assert not protocol.stalled(drove, 0.3)
    # Odometry says 15 cm forward but the lidar sees the same room from the same spot: stuck.
    stuck = [(0.0, 0.0, 0.0, view), (0.15, 0.0, 0.0, view)]
    assert protocol.stalled(stuck, 0.3)
    # Backwards too, along the robot's heading (here facing +y in the odom frame).
    assert protocol.stalled([(0.0, 0.0, math.pi / 2, view), (0.0, -0.15, math.pi / 2, view)], -0.3)
    assert not protocol.stalled([(0.0, 0.0, math.pi / 2, view), (0.0, -0.15, math.pi / 2, room_points(-0.15, 0))], -0.3)
    # Not far enough yet, turning, or the other way: no verdict.
    assert not protocol.stalled([(0.0, 0.0, 0.0, view), (0.05, 0.0, 0.0, view)], 0.3)
    assert not protocol.stalled([(0.0, 0.0, 0.0, view), (0.15, 0.0, 0.3, view)], 0.3)
    assert not protocol.stalled(stuck, -0.3)
    assert not protocol.stalled(stuck[:1], 0.3)


def test_not_stalled_driving_down_a_hallway():
    # Only walls alongside: both answers fit, so it doesn't call it stuck.
    hall = [(k / 20, 0.5) for k in range(-20, 61)] + [(k / 20, -0.5) for k in range(-20, 61)]
    assert not protocol.stalled([(0.0, 0.0, 0.0, hall), (0.15, 0.0, 0.0, hall)], 0.3)


def test_parse_speed_caps_at_the_limit_and_rejects_nonsense():
    assert protocol.parse_speed({'speed': 0.35}, 0.5) == 0.35
    assert protocol.parse_speed({'speed': '0.45'}, 0.5) == 0.45
    assert protocol.parse_speed({'speed': 2.0}, 0.5) == 0.5
    for bad in ({}, {'speed': None}, {'speed': 'fast'}, {'speed': 0}, {'speed': -0.2},
                {'speed': float('nan')}, {'speed': float('inf')}):
        assert protocol.parse_speed(bad, 0.5) is None
