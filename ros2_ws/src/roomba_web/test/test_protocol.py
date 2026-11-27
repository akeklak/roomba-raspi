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
