"""Saved maps on disk. No ROS imports, so this can be unit tested anywhere.

A saved map called "home" is a handful of files in the maps folder:

    home.pgm, home.yaml        the picture of the map and its scale and position, in the format
                               Nav2's map_server loads (also opens in any image viewer)
    home.posegraph, home.data  slam_toolbox's own copy, needed to add to the map later
    home.json                  when it was saved, and where the robot was when mapping started

Saving writes everything under a temporary name first and then renames it into place, so a crash
halfway never leaves a broken map. The previous save is kept as home.previous.* in case the new
one turns out worse.
"""

import array
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

# Cell values in nav_msgs/OccupancyGrid.
UNKNOWN, FREE, OCCUPIED = -1, 0, 100

# Pixel values map_server's "trinary" mode reads back as free, occupied and unknown.
PIXEL_FREE, PIXEL_OCCUPIED, PIXEL_UNKNOWN = 254, 0, 205
OCCUPIED_THRESH = 0.65
# A pixel of 205 is 0.196 occupied; map_server treats anything below free_thresh as free, so this
# must stay at or below 0.196 or unknown space would load as free.
FREE_THRESH = 0.196

EXTENSIONS = ('.pgm', '.posegraph', '.data', '.json', '.yaml')  # .yaml last: it marks a complete save


@dataclass
class Grid:
    """An occupancy grid: row-major cells starting at the bottom-left corner (origin)."""
    width: int
    height: int
    resolution: float
    origin: Sequence[float]  # x, y, yaw of the bottom-left corner of cell (0, 0)
    data: Sequence[int]      # -1 unknown, 0 free ... 100 occupied


@dataclass
class MapInfo:
    """What home.json records about a saved map."""
    name: str
    saved_at: float
    width_m: float
    height_m: float
    # Robot pose (x, y, yaw in the map) when this map was started. If the robot starts there next
    # time (usually on its dock), localization needs no help.
    start_pose: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    can_extend: bool = False


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pixel(value: int) -> int:
    if value < 0:
        return PIXEL_UNKNOWN
    if value >= OCCUPIED_THRESH * 100:
        return PIXEL_OCCUPIED
    if value <= FREE_THRESH * 100:
        return PIXEL_FREE
    return PIXEL_UNKNOWN


def int8_bytes(data) -> bytes:
    """Cells as raw signed bytes (-1 becomes 255), however they are stored."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if isinstance(data, array.array) and data.typecode == 'b':
        return data.tobytes()
    return bytes(int(v) & 0xFF for v in data)


# Raw cell byte (0..255, two's complement) -> PGM pixel.
_PIXEL_LUT = bytes(pixel(b - 256 if b > 127 else b) for b in range(256))


def pgm_bytes(grid: Grid) -> bytes:
    """Binary PGM. Image rows run top to bottom, so the map's last row comes first."""
    if len(grid.data) != grid.width * grid.height:
        raise ValueError(f'grid has {len(grid.data)} cells, expected {grid.width} x {grid.height}')
    raw = int8_bytes(grid.data).translate(_PIXEL_LUT)
    rows = [raw[r * grid.width:(r + 1) * grid.width] for r in range(grid.height - 1, -1, -1)]
    head = f'P5\n# roomba_nav map\n{grid.width} {grid.height}\n255\n'.encode()
    return head + b''.join(rows)


def yaml_text(grid: Grid, image: str) -> str:
    x, y, yaw = (float(v) for v in grid.origin)
    return (
        f'image: {image}\n'
        'mode: trinary\n'
        f'resolution: {grid.resolution:.6g}\n'
        f'origin: [{x:.6f}, {y:.6f}, {yaw:.6f}]\n'
        'negate: 0\n'
        f'occupied_thresh: {OCCUPIED_THRESH}\n'
        f'free_thresh: {FREE_THRESH}\n'
    )


def read_pgm(data: bytes) -> Grid:
    """Reads a binary PGM written by pgm_bytes (or map_saver) back into cells. For tests and tools."""
    tokens: List[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError('not a binary PGM')
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[pos + 1:pos + 1 + width * height]
    cells: List[int] = []
    for r in range(height - 1, -1, -1):
        for p in pixels[r * width:(r + 1) * width]:
            occ = (255 - p) / 255.0
            cells.append(OCCUPIED if occ > OCCUPIED_THRESH else FREE if occ < FREE_THRESH else UNKNOWN)
    return Grid(width, height, 0.0, (0.0, 0.0, 0.0), cells)


class MapStore:
    """The saved map called `name` in `directory`."""

    def __init__(self, directory: str, name: str = 'home') -> None:
        self.directory = os.path.expanduser(directory)
        self.name = name

    @property
    def prefix(self) -> str:
        """Path without extension. slam_toolbox takes this to load the pose graph."""
        return os.path.join(self.directory, self.name)

    @property
    def yaml_path(self) -> str:
        return self.prefix + '.yaml'

    @property
    def temp_prefix(self) -> str:
        return os.path.join(self.directory, f'.{self.name}.saving')

    @property
    def backup_prefix(self) -> str:
        return os.path.join(self.directory, f'{self.name}.previous')

    def exists(self) -> bool:
        return os.path.isfile(self.yaml_path) and os.path.isfile(self.prefix + '.pgm')

    def has_pose_graph(self) -> bool:
        return os.path.isfile(self.prefix + '.posegraph') and os.path.isfile(self.prefix + '.data')

    def info(self) -> Optional[MapInfo]:
        if not self.exists():
            return None
        try:
            with open(self.prefix + '.json') as f:
                meta = json.load(f)
        except (OSError, ValueError):
            meta = {}
        start = meta.get('start_pose')
        if not (isinstance(start, list) and len(start) == 3):
            start = [0.0, 0.0, 0.0]
        return MapInfo(
            name=self.name,
            saved_at=float(meta.get('saved_at') or os.path.getmtime(self.yaml_path)),
            width_m=float(meta.get('width_m') or 0.0),
            height_m=float(meta.get('height_m') or 0.0),
            start_pose=[float(v) for v in start],
            can_extend=self.has_pose_graph(),
        )

    def begin_save(self) -> str:
        """Clears leftovers of an interrupted save. Returns the prefix to write the new files to."""
        os.makedirs(self.directory, exist_ok=True)
        for ext in EXTENSIONS:
            _remove(self.temp_prefix + ext)
        return self.temp_prefix

    def write_grid(self, grid: Grid, start_pose: Sequence[float]) -> None:
        """Writes the map picture, its YAML and home.json under the temporary prefix."""
        prefix = self.temp_prefix
        with open(prefix + '.pgm', 'wb') as f:
            f.write(pgm_bytes(grid))
        with open(prefix + '.yaml', 'w') as f:
            f.write(yaml_text(grid, image=self.name + '.pgm'))
        meta = {
            'name': self.name,
            'saved_at': time.time(),
            'width_m': round(grid.width * grid.resolution, 3),
            'height_m': round(grid.height * grid.resolution, 3),
            'resolution': grid.resolution,
            'start_pose': [round(float(v), 4) for v in start_pose],
        }
        with open(prefix + '.json', 'w') as f:
            json.dump(meta, f, indent=2)

    def commit(self) -> None:
        """Moves the previous save to <name>.previous.* and the new files into place."""
        if not os.path.isfile(self.temp_prefix + '.yaml'):
            raise FileNotFoundError('nothing to commit: write_grid was not called')
        for ext in EXTENSIONS:
            _remove(self.backup_prefix + ext)
        for ext in reversed(EXTENSIONS):  # .yaml first, so a half-moved old map is never "complete"
            if os.path.exists(self.prefix + ext):
                os.replace(self.prefix + ext, self.backup_prefix + ext)
        for ext in EXTENSIONS:
            if os.path.exists(self.temp_prefix + ext):
                os.replace(self.temp_prefix + ext, self.prefix + ext)


def _remove(path: str) -> None:
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
