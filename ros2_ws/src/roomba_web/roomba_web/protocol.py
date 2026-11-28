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
                                     Driving also cancels a navigation goal (you took over). The
                                     robot slows and stops short of anything the lidar sees in
                                     its way (turning is always allowed).
    camera       {enabled}           whether this page wants camera pictures
    costmap      {enabled}           whether this page wants Nav2's costmap
    top_speed    {speed}             the speed slider, in m/s: Go to (Nav2) drives at most this fast
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


def parse_speed(cmd: dict, max_speed: float) -> Optional[float]:
    """The top speed a page asks for (m/s, at most `max_speed`), or None if it isn't one."""
    try:
        speed = float(cmd.get('speed'))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(speed) or speed <= 0.0:
        return None
    return round(min(speed, max_speed), 3)


# Driving by hand stops short of what the lidar sees. Pushing into a wall makes the wheels spin in
# place, so odometry says the robot moved when it didn't, and SLAM loses track of where it is
# for good (a smeared map and doubled walls). Nav2 keeps its own distance, so this is only for
# the drive keys.

def _quat_matrix(q) -> list:
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def static_pose(static: dict, frame_name: str, base: str) -> Optional[tuple]:
    """Where `frame_name` sits on the robot, as (xx, xy, yx, yy, x, y): the floor-plane part of
    its rotation and its position in `base`. Built from static transforms (child -> {parent, t,
    q}, as `transforms` makes them). None if `frame_name` doesn't hang below `base` that way."""
    rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    pos = [0.0, 0.0, 0.0]
    f = frame(frame_name)
    for _ in range(64):
        if f == base:
            return rot[0][0], rot[0][1], rot[1][0], rot[1][1], pos[0], pos[1]
        tr = static.get(f)
        if tr is None:
            return None
        r = _quat_matrix(tr['q'])
        rot = [[sum(r[i][k] * rot[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        pos = [sum(r[i][k] * pos[k] for k in range(3)) + tr['t'][i] for i in range(3)]
        f = tr['parent']
    return None


def nearby_points(msg, pose: tuple, reach: float, every: int = 1) -> list:
    """The lidar hits within `reach` of the robot's center, as (x, y) on the robot (`pose` from
    `static_pose`), from every `every`th beam."""
    xx, xy, yx, yy, ox, oy = pose
    out = []
    for i in range(0, len(msg.ranges), every):
        r = msg.ranges[i]
        if msg.range_min <= r <= msg.range_max:  # also false for NaN
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x, y = xx * lx + xy * ly + ox, yx * lx + yy * ly + oy
            if x * x + y * y <= reach * reach:
                out.append((x, y))
    return out


def free_travel(points: list, linear: float, angular: float, radius: float, body: float,
                horizon: float, step: float = 0.01) -> float:
    """How far the robot, a circle of `radius` around its center, can drive along the arc that
    (linear, angular) makes before it comes up against one of `points`, up to `horizon` meters.
    Points that are getting farther away don't count (so it can always back away from a wall),
    nor do points inside `body` (the robot itself, or noise)."""
    if linear == 0.0:
        return horizon  # turning in place: a round robot sweeps nothing new
    sign = 1.0 if linear > 0 else -1.0
    turn = angular / abs(linear)  # radians per meter driven
    near = [(px, py, math.hypot(px, py)) for px, py in points]
    near = [p for p in near if p[2] >= body]
    d = step
    while d <= horizon + 1e-9:
        if abs(turn * d) < 1e-6:
            cx, cy = sign * d, 0.0
        else:
            cx, cy = sign * math.sin(turn * d) / turn, sign * (1.0 - math.cos(turn * d)) / turn
        for px, py, d0 in near:
            dist = math.hypot(px - cx, py - cy)
            if dist < radius and dist < d0:
                return d - step
        d += step
    return horizon


def guard_speed(linear: float, free: float, stop_at: float = 0.01, slow_time: float = 0.4,
                min_speed: float = 0.03) -> float:
    """`linear`, slowed so the robot stops `stop_at` meters before the first thing in its way:
    at most the speed that covers the free distance in `slow_time` seconds, and 0 below
    `min_speed`."""
    allowed = (free - stop_at) / slow_time
    if allowed < min_speed:
        return 0.0
    return math.copysign(min(abs(linear), allowed), linear)


# Some things are too low for the lidar to see (the dock, in the sim). Running into one shows as
# the wheels saying the robot moved while the lidar sees everything stay where it was.

STALL_TRAVEL = 0.12  # how far odometry must say it drove before judging, in meters
_STALL_CELL = 0.03


def stalled(history: list, linear: float) -> bool:
    """Whether the robot is pushing against something. `history` holds (odom x, y, yaw, lidar
    points on the robot from `nearby_points`), oldest first. Looks back to where odometry says it
    drove `STALL_TRAVEL` in the direction of `linear` (without turning much), and asks which fits
    the lidar better: that it drove there, or that it never moved."""
    if len(history) < 2:
        return False
    x1, y1, yaw1, now = history[-1]
    for x0, y0, yaw0, then in reversed(history[:-1]):
        turn = math.remainder(yaw1 - yaw0, 2 * math.pi)
        if abs(turn) > 0.1:
            return False  # turning: no verdict
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)
        if dx * math.copysign(1.0, linear) >= STALL_TRAVEL:
            break
    else:
        return False  # hasn't driven far enough yet
    if len(now) < 20:
        return False  # too little in view to judge by
    seen = {(math.floor(px / _STALL_CELL), math.floor(py / _STALL_CELL)) for px, py in then}

    def matches(points) -> int:
        n = 0
        for px, py in points:
            i, j = math.floor(px / _STALL_CELL), math.floor(py / _STALL_CELL)
            if any((i + a, j + b) in seen for a in (-1, 0, 1) for b in (-1, 0, 1)):
                n += 1
        return n

    c, s = math.cos(turn), math.sin(turn)
    still = matches(now)
    moved = matches([(c * px - s * py + dx, s * px + c * py + dy) for px, py in now])
    return still >= 0.5 * len(now) and still - moved >= 0.2 * len(now)


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
    204: "that spot isn't on the map yet. Drive closer so the robot can see it first",
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
