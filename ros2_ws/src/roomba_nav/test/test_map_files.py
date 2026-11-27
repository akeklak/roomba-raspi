import json
import os

import pytest

from roomba_nav.map_files import (FREE, OCCUPIED, UNKNOWN, Grid, MapStore, pgm_bytes, pixel, read_pgm,
                                  yaml_text, yaw_from_quaternion)


def grid(data=(FREE, OCCUPIED, UNKNOWN, FREE, 30, 90), width=3, height=2):
    return Grid(width=width, height=height, resolution=0.05, origin=(-1.5, 2.0, 0.0), data=list(data))


def test_pixels_follow_map_server_trinary_mode():
    assert pixel(UNKNOWN) == 205
    assert pixel(FREE) == 254
    assert pixel(OCCUPIED) == 0
    assert pixel(90) == 0     # above occupied_thresh
    assert pixel(10) == 254   # below free_thresh
    assert pixel(40) == 205   # in between: unknown


def test_pgm_round_trip_with_rows_top_first():
    g = grid()
    data = pgm_bytes(g)
    assert data.startswith(b'P5\n')
    # The image's first row is the map's last row.
    body = data[-6:]
    assert list(body[:3]) == [254, 205, 0]  # cells 3..5: free, 30 (unknown), 90 (occupied)
    back = read_pgm(data)
    assert (back.width, back.height) == (3, 2)
    assert back.data == [FREE, OCCUPIED, UNKNOWN, FREE, UNKNOWN, OCCUPIED]


def test_pgm_takes_raw_signed_bytes_like_rclpy_arrays():
    import array
    g = grid(data=array.array('b', [0, 100, -1, 0, 0, 0]))
    assert read_pgm(pgm_bytes(g)).data[:3] == [FREE, OCCUPIED, UNKNOWN]


def test_pgm_rejects_wrong_size():
    with pytest.raises(ValueError):
        pgm_bytes(grid(data=[0, 0, 0]))


def test_yaml_keeps_unknown_unknown():
    text = yaml_text(grid(), 'home.pgm')
    assert 'image: home.pgm' in text
    assert 'mode: trinary' in text
    assert 'origin: [-1.500000, 2.000000, 0.000000]' in text
    free_thresh = float(text.split('free_thresh: ')[1].split()[0])
    assert (255 - 205) / 255 >= free_thresh  # an unknown pixel must not load as free


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0, 0, 0, 1) == 0
    assert abs(yaw_from_quaternion(0, 0, 0.7071068, 0.7071068) - 1.5707963) < 1e-6


def save(store, g, start=(0.0, 0.0, 0.0), graph=False):
    prefix = store.begin_save()
    if graph:  # slam_toolbox writes these itself
        for ext in ('.posegraph', '.data'):
            with open(prefix + ext, 'w') as f:
                f.write('graph')
    store.write_grid(g, start)
    store.commit()


def test_store_saves_and_keeps_the_previous_map(tmp_path):
    store = MapStore(str(tmp_path / 'maps'), 'home')
    assert not store.exists() and store.info() is None
    save(store, grid(), start=(1.0, 2.0, 0.5), graph=True)
    info = store.info()
    assert store.exists() and info.can_extend
    assert info.start_pose == [1.0, 2.0, 0.5]
    assert info.width_m == pytest.approx(0.15) and info.height_m == pytest.approx(0.1)

    save(store, grid(width=6, height=1))
    files = sorted(os.listdir(tmp_path / 'maps'))
    assert 'home.previous.pgm' in files and 'home.previous.posegraph' in files
    assert not any(f.startswith('.home.saving') for f in files)
    info = store.info()
    assert info.width_m == pytest.approx(0.3)
    # The new save had no pose graph, and the old one went to the backup with the old map.
    assert not info.can_extend
    with open(tmp_path / 'maps' / 'home.previous.json') as f:
        assert json.load(f)['start_pose'] == [1.0, 2.0, 0.5]


def test_store_clears_a_half_finished_save(tmp_path):
    store = MapStore(str(tmp_path), 'home')
    prefix = store.begin_save()
    with open(prefix + '.pgm', 'w') as f:
        f.write('junk')
    store.begin_save()
    assert not os.path.exists(prefix + '.pgm')
    with pytest.raises(FileNotFoundError):
        store.commit()


def test_info_survives_a_missing_json(tmp_path):
    store = MapStore(str(tmp_path), 'home')
    save(store, grid())
    os.remove(store.prefix + '.json')
    info = store.info()
    assert info.start_pose == [0.0, 0.0, 0.0] and info.saved_at > 0
