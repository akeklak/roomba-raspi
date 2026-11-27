"""Reads static boxes and cylinders out of a Gazebo SDF world file. No ROS imports."""

import math
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

Pose = Tuple[float, float, float, float, float, float]


def _floats(text: Optional[str], count: int, default: float = 0.0) -> List[float]:
    values = [float(v) for v in (text or '').split()]
    return (values + [default] * count)[:count]


def _pose(elem: Optional[ET.Element]) -> Pose:
    return tuple(_floats(elem.text if elem is not None else None, 6))  # type: ignore[return-value]


def _compose(parent: Pose, child: Pose) -> Pose:
    """parent * child for 2.5D poses (only yaw is composed; roll/pitch are passed through)."""
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, croll, cpitch, pyaw + cyaw)


def quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_world(path: str) -> List[dict]:
    """Return a list of shapes: dict(name, kind, pose, size, rgba)."""
    root = ET.parse(path).getroot()
    shapes = []
    for model in root.iter('model'):
        model_pose = _pose(model.find('pose'))
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose(link.find('pose')))
            for visual in link.findall('visual'):
                geometry = visual.find('geometry')
                if geometry is None:
                    continue
                pose = _compose(link_pose, _pose(visual.find('pose')))
                diffuse = visual.find('material/diffuse')
                rgba = _floats(diffuse.text if diffuse is not None else None, 4, 1.0)
                box = geometry.find('box/size')
                cylinder = geometry.find('cylinder')
                if box is not None:
                    shapes.append(dict(name=visual.get('name', ''), kind='box', pose=pose,
                                       size=_floats(box.text, 3), rgba=rgba))
                elif cylinder is not None:
                    r = float(cylinder.findtext('radius', '0'))
                    h = float(cylinder.findtext('length', '0'))
                    shapes.append(dict(name=visual.get('name', ''), kind='cylinder', pose=pose,
                                       size=[2 * r, 2 * r, h], rgba=rgba))
    return shapes
