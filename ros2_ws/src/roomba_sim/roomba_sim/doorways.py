"""Shut doors for worlds that end at their doorways. No ROS imports.

The parents' kitchen models only the kitchen and the hall: its floor and walls stop at five
doorways, each marked by a threshold box named floor_jamb_<opening>, and the space beyond is
unmodeled. Left open, a doorway looks like a way out to the lidar, so on a fresh map Nav2 plans
out through one and back in through walls it hasn't seen yet. sim.launch.py puts a shut door in
each doorway instead: the lidar sees it, SLAM maps it as wall, and the robot stays in the rooms
that exist. Worlds without thresholds (the apartment) get none.
"""

import xml.etree.ElementTree as ET
from typing import List, Optional

from roomba_sim.sdf_shapes import _compose, _pose

JAMB_PREFIX = 'floor_jamb_'
DOOR_HEIGHT = 2.0
DOOR_THICKNESS = 0.04
DOOR_COLOR = '0.86 0.84 0.80 1'


def doorways(path: str) -> List[dict]:
    """The doorway thresholds in a world file: dict(name, x, y, yaw, width), in the world frame.
    `width` is the threshold's size along its own x axis, the opening's clear width."""
    found = []
    for model in ET.parse(path).getroot().iter('model'):
        model_pose = _pose(model.find('pose'))
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose(link.find('pose')))
            for collision in link.findall('collision'):
                name = collision.get('name', '')
                size = collision.findtext('geometry/box/size')
                if not name.startswith(JAMB_PREFIX) or size is None:
                    continue
                x, y, _, _, _, yaw = _compose(link_pose, _pose(collision.find('pose')))
                found.append(dict(name=name[len(JAMB_PREFIX):], x=x, y=y, yaw=yaw,
                                  width=float(size.split()[0])))
    return found


def doors_sdf(path: str) -> Optional[str]:
    """A static model with a shut door in each doorway of the world file, or None if it has none."""
    links = []
    for d in doorways(path):
        geometry = f'<geometry><box><size>{d["width"]:.4f} {DOOR_THICKNESS} {DOOR_HEIGHT}</size></box></geometry>'
        links.append(
            f'<link name="door_{d["name"]}">'
            f'<pose>{d["x"]:.4f} {d["y"]:.4f} {DOOR_HEIGHT / 2} 0 0 {d["yaw"]:.6f}</pose>'
            f'<collision name="collision">{geometry}</collision>'
            f'<visual name="visual">{geometry}'
            f'<material><ambient>{DOOR_COLOR}</ambient><diffuse>{DOOR_COLOR}</diffuse></material>'
            f'</visual></link>')
    if not links:
        return None
    return ('<sdf version="1.9"><model name="closed_doors"><static>true</static>'
            + ''.join(links) + '</model></sdf>')
