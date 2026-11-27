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
