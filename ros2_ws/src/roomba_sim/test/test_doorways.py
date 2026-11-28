import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from roomba_sim.doorways import DOOR_HEIGHT, doors_sdf, doorways

WORLDS = Path(__file__).resolve().parents[1] / 'worlds'

WORLD = """<sdf version="1.9"><world name="w">
  <model name="Architecture"><static>true</static><pose>1 2 0 0 0 0</pose>
    <link name="link">
      <collision name="floor_hall_0"><geometry><box><size>2 2 0.08</size></box></geometry></collision>
      <collision name="floor_jamb_D1_3"><pose>0.5 1.0 -0.04 0 0 0</pose>
        <geometry><box><size>0.74 0.139 0.08</size></box></geometry></collision>
      <collision name="floor_jamb_O1_4"><pose>-1.0 0.2 -0.04 0 0 1.5708</pose>
        <geometry><box><size>1.5 0.139 0.08</size></box></geometry></collision>
    </link>
  </model>
</world></sdf>"""


def test_finds_each_threshold_in_the_world_frame(tmp_path):
    path = tmp_path / 'world.sdf'
    path.write_text(WORLD)
    found = {d['name']: d for d in doorways(str(path))}
    assert sorted(found) == ['D1_3', 'O1_4']
    assert (found['D1_3']['x'], found['D1_3']['y'], found['D1_3']['width']) == pytest.approx((1.5, 3.0, 0.74))
    assert (found['O1_4']['x'], found['O1_4']['y'], found['O1_4']['yaw']) == pytest.approx((0.0, 2.2, 1.5708))


def test_puts_a_shut_door_with_collision_and_visual_in_each(tmp_path):
    path = tmp_path / 'world.sdf'
    path.write_text(WORLD)
    model = ET.fromstring(doors_sdf(str(path))).find('model')
    assert model.findtext('static') == 'true'
    doors = model.findall('link')
    assert len(doors) == 2
    for door in doors:
        assert door.find('collision/geometry/box') is not None
        assert door.find('visual/geometry/box') is not None  # so the lidar sees it
        x, y, z, _, _, yaw = map(float, door.findtext('pose').split())
        assert z == pytest.approx(DOOR_HEIGHT / 2)
    width, thickness, height = map(float, doors[0].findtext('collision/geometry/box/size').split())
    assert (width, height) == pytest.approx((0.74, DOOR_HEIGHT)) and thickness < 0.139


def test_worlds_without_thresholds_get_no_doors():
    assert doors_sdf(str(WORLDS / 'apartment.sdf')) is None


@pytest.mark.skipif(not (WORLDS / 'parents_kitchen.sdf').exists(), reason='kitchen world not present')
def test_closes_all_five_kitchen_doorways():
    found = doorways(str(WORLDS / 'parents_kitchen.sdf'))
    assert sorted(d['name'].split('_')[0] for d in found) == ['D1', 'D2', 'O1', 'O2', 'O3']
    assert all(0.7 < d['width'] < 1.6 for d in found)
    assert all(math.isfinite(d['yaw']) for d in found)
