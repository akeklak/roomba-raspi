"""The messages the bridge sends to the web UI, built from ROS messages.

No ROS imports here, so this can be unit tested anywhere. Every function takes a ROS message
(or anything with the same fields) and returns plain JSON-ready data.

Text frames are JSON objects with an "op" field:
    hello     {sim}                                   sent once on connect
    status    {time}                                  twice a second
    tf        {transforms: [{parent, child, t, q, stamp, static}]}
    scan      {frame, stamp, angle_min, angle_increment, range_min, range_max, ranges}
    odom      {speed, turn}
    world     {shapes: [{key, frame, kind, position, quaternion, size, color}]}
    battery   {charge}
    charging  {state}
Binary frames are camera pictures: one byte 1, a little-endian uint32 header length, a JSON
header {frame, stamp, width, height, model}, then the JPEG.

The UI sends:
    drive     {linear, angular}   repeated while driving; the robot stops if they stop coming
    camera    {enabled}           whether this page wants camera pictures
"""

import io
import json
import math
import struct
from typing import Optional

CAMERA_FRAME = 1

# visualization_msgs/Marker types and actions.
_CUBE, _CYLINDER, _ADD = 1, 3, 0


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _r(v: float, digits: int = 4) -> float:
    return round(float(v), digits)


def frame(name: str) -> str:
    return name[1:] if name.startswith('/') else name


def transforms(msg, static: bool) -> list:
    out = []
    for tr in msg.transforms:
        t = tr.transform.translation
        q = tr.transform.rotation
        out.append({
            'parent': frame(tr.header.frame_id),
            'child': frame(tr.child_frame_id),
            't': [_r(t.x, 5), _r(t.y, 5), _r(t.z, 5)],
            'q': [_r(q.x, 6), _r(q.y, 6), _r(q.z, 6), _r(q.w, 6)],
            'stamp': stamp(tr.header),
            'static': static,
        })
    return out


def scan(msg) -> dict:
    return {
        'op': 'scan',
        'frame': frame(msg.header.frame_id),
        'stamp': stamp(msg.header),
        'angle_min': msg.angle_min,
        'angle_increment': msg.angle_increment,
        'range_min': msg.range_min,
        'range_max': msg.range_max,
        # JSON has no infinity, so "no hit" is sent as null.
        'ranges': [_r(r, 3) if math.isfinite(r) else None for r in msg.ranges],
    }


def odom(msg) -> dict:
    return {
        'op': 'odom',
        'speed': _r(msg.twist.twist.linear.x, 3),
        'turn': _r(msg.twist.twist.angular.z, 3),
    }


def world(msg) -> dict:
    shapes = []
    for m in msg.markers:
        if m.action != _ADD or m.type not in (_CUBE, _CYLINDER):
            continue
        p, q = m.pose.position, m.pose.orientation
        shapes.append({
            'key': f'{m.ns}/{m.id}',
            'frame': frame(m.header.frame_id),
            'kind': 'box' if m.type == _CUBE else 'cylinder',
            'position': [p.x, p.y, p.z],
            'quaternion': [q.x, q.y, q.z, q.w],
            'size': [m.scale.x, m.scale.y, m.scale.z],
            'color': [m.color.r, m.color.g, m.color.b, m.color.a],
        })
    return {'op': 'world', 'shapes': shapes}


def camera_model(info) -> Optional[dict]:
    k = list(info.k)
    if not k or not k[0]:
        return None
    return {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5], 'width': info.width, 'height': info.height}


# sensor_msgs/Image encoding -> (Pillow mode, Pillow raw mode)
_PIL_MODES = {
    'rgb8': ('RGB', 'RGB'),
    'bgr8': ('RGB', 'BGR'),
    'rgba8': ('RGBA', 'RGBA'),
    'bgra8': ('RGBA', 'BGRA'),
    'mono8': ('L', 'L'),
    '8uc1': ('L', 'L'),
}


def jpeg(img, quality: int = 70) -> Optional[bytes]:
    """Compresses a sensor_msgs/Image to JPEG. None for encodings we don't handle."""
    from PIL import Image  # imported here so the rest of this module works without Pillow

    modes = _PIL_MODES.get(img.encoding.lower())
    if not modes:
        return None
    mode, raw = modes
    picture = Image.frombuffer(mode, (img.width, img.height), bytes(img.data), 'raw', raw, img.step, 1)
    if picture.mode == 'RGBA':
        picture = picture.convert('RGB')
    buf = io.BytesIO()
    picture.save(buf, 'JPEG', quality=quality)
    return buf.getvalue()


def camera_frame(img, jpeg_bytes: bytes, model: Optional[dict]) -> bytes:
    header = json.dumps({
        'frame': frame(img.header.frame_id),
        'stamp': stamp(img.header),
        'width': img.width,
        'height': img.height,
        'model': model,
    }).encode()
    return bytes([CAMERA_FRAME]) + struct.pack('<I', len(header)) + header + jpeg_bytes


def clamp_drive(cmd: dict, max_linear: float, max_angular: float) -> tuple:
    """(linear, angular) from a UI drive message, limited to safe speeds. Bad input means stop."""
    try:
        linear = float(cmd.get('linear', 0.0))
        angular = float(cmd.get('angular', 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0.0, 0.0
    return (max(-max_linear, min(max_linear, linear)), max(-max_angular, min(max_angular, angular)))
