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
    mapping   map_manager's state: {mode, busy, message, error, extending, map_name, saved}
    nav       {state, goal: {x, y, yaw, frame} | null, distance, eta, recoveries, message}
              state is idle | starting | active | succeeded | canceled | failed
    plan      {frame, points: [[x, y], ...]}      the path Nav2 is following (empty when done)
    notice    {level: info | error, text}         the answer to a map command
Binary frames start with one byte saying what they are, then a little-endian uint32 header
length and a JSON header:
    1  camera picture: header {frame, stamp, width, height, model}, then the JPEG
    2  grid (the map, or Nav2's costmap): header {kind: map | costmap, frame, stamp, width,
       height, resolution, origin: {x, y, z, q: [x, y, z, w]}}, then the cells compressed with
       zlib: one byte per cell, row by row from the origin corner, 0..100 = free..occupied,
       255 = unknown

The UI sends:
    drive        {linear, angular}   repeated while driving; the robot stops if they stop coming.
                                     Driving also cancels a navigation goal (you took over).
    camera       {enabled}           whether this page wants camera pictures
    costmap      {enabled}           whether this page wants Nav2's costmap
    goal         {x, y, yaw}         drive to this pose on the map
    cancel       {}                  stop navigating
    initial_pose {x, y, yaw}         "the robot is here": corrects localization on a saved map
    map_command  {command}           save_map | use_saved_map | new_map | extend_map
"""

import array
import io
import json
import math
import struct
import zlib
from typing import Optional

CAMERA_FRAME = 1
GRID_FRAME = 2

MAP_COMMANDS = ('save_map', 'use_saved_map', 'new_map', 'extend_map')

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


def binary_frame(kind: int, header: dict, payload: bytes) -> bytes:
    head = json.dumps(header).encode()
    return bytes([kind]) + struct.pack('<I', len(head)) + head + payload


def cell_bytes(data) -> bytes:
    """OccupancyGrid cells (int8, -1 = unknown) as raw bytes, so -1 becomes 255."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if isinstance(data, array.array) and data.typecode == 'b':
        return data.tobytes()
    return bytes(int(v) & 0xFF for v in data)


def grid_frame(msg, kind: str) -> bytes:
    """A nav_msgs/OccupancyGrid (the map or a costmap) as a binary frame (see the top)."""
    info = msg.info
    o = info.origin
    header = {
        'kind': kind,
        'frame': frame(msg.header.frame_id),
        'stamp': stamp(msg.header),
        'width': info.width,
        'height': info.height,
        'resolution': info.resolution,
        'origin': {
            'x': o.position.x, 'y': o.position.y, 'z': o.position.z,
            'q': [o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w],
        },
    }
    return binary_frame(GRID_FRAME, header, zlib.compress(cell_bytes(msg.data), 1))


def path(msg, min_step: float = 0.05) -> dict:
    """A nav_msgs/Path, thinned to a point every `min_step` meters (always keeping the last)."""
    points = []
    poses = list(msg.poses)
    for i, ps in enumerate(poses):
        p = ps.pose.position
        if points and i < len(poses) - 1 and math.hypot(p.x - points[-1][0], p.y - points[-1][1]) < min_step:
            continue
        points.append([_r(p.x, 3), _r(p.y, 3)])
    return {'op': 'plan', 'frame': frame(msg.header.frame_id), 'points': points}


def seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


def parse_pose(msg: dict) -> Optional[tuple]:
    """(x, y, yaw) from a goal or initial_pose message, or None if it isn't three finite numbers."""
    try:
        x, y, yaw = float(msg['x']), float(msg['y']), float(msg.get('yaw', 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    return x, y, yaw


# Nav2's error codes (nav2_msgs FollowPath 1xx, ComputePathToPose 2xx) in plain words.
NAV_ERRORS = {
    102: 'it lost track of where it is on the map',
    104: 'it stopped making progress',
    105: 'it stopped making progress',
    106: 'it found no safe way to move',
    107: 'driving took too long',
    202: 'it does not know where it is on the map yet',
    203: 'the robot is off the map',
    204: 'that spot is off the map',
    205: 'the robot is too close to something to plan from there',
    206: 'that spot is too close to a wall or furniture',
    207: 'planning took too long',
    208: 'there is no way through to that spot',
}


def nav_error(code: int, detail: str = '') -> str:
    """Why a goal failed, for the page."""
    if code in NAV_ERRORS:
        return NAV_ERRORS[code]
    if detail:
        return detail
    return f'Nav2 error {code}' if code else 'no route found, or it got stuck'


def yaw_quaternion(yaw: float) -> tuple:
    """(x, y, z, w) for a rotation of `yaw` about Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
