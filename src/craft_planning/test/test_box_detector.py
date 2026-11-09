"""box_detector on a synthetic arena-frame map: walls removed, partial boxes completed to 205 mm squares."""
import math

import numpy as np
import pytest
import yaml
from PIL import Image

from craft_planning.box_detector import BoxParams, detect_boxes, load_map, match_layout, write_completed_map

RES = 0.02
ORIGIN = (-0.2, -0.2)
H = 0.205 / 2


def draw(pixels, x0, y0, x1, y1, value=0):
    n = int(math.hypot(x1 - x0, y1 - y0) / (RES / 4)) + 1
    ix = np.floor((np.linspace(x0, x1, n) - ORIGIN[0]) / RES).astype(int)
    iy = np.floor((np.linspace(y0, y1, n) - ORIGIN[1]) / RES).astype(int)
    pixels[pixels.shape[0] - 1 - iy, ix] = value


def faces(pixels, x, y, which):
    seg = {'E': (x + H, y - H, x + H, y + H), 'W': (x - H, y - H, x - H, y + H),
           'N': (x - H, y + H, x + H, y + H), 'S': (x - H, y - H, x + H, y - H)}
    for f in which:
        draw(pixels, *seg[f])


@pytest.fixture
def arena(tmp_path):
    w, h = int(2.4 / RES), int(2.4 / RES)
    pixels = np.full((h, w), 254, np.uint8)
    for seg in [(0, 0, 2, 0), (2, 0, 2, 2), (2, 2, 0, 2), (0, 2, 0, 0), (0.6, 0.0, 0.6, 0.9)]:
        draw(pixels, *seg)                         # outer walls and one inner wall
    faces(pixels, 1.2, 1.5, 'ENWS')                # fully seen
    faces(pixels, 1.4, 0.5, 'WS')                  # two faces
    faces(pixels, 0.6 + H, 1.3, 'ENS')             # against nothing, west face unseen
    Image.fromarray(pixels).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(
        image='map.pgm', mode='trinary', resolution=RES, origin=[*ORIGIN, 0.0], negate=0,
        occupied_thresh=0.65, free_thresh=0.196)))
    layout = {'A': (1.2, 1.5), 'B': (1.4, 0.5), 'C': (0.6 + H, 1.3)}
    return tmp_path / 'map.yaml', layout


def test_partial_boxes_are_found_as_full_squares(arena):
    path, layout = arena
    grid = load_map(path)
    boxes, _ = detect_boxes(grid, BoxParams(arena_margin=10.0), layout)
    matched, extra = match_layout(boxes, layout)
    assert not extra
    for name, (x, y) in layout.items():
        b = matched[name]
        assert b is not None, name
        assert math.hypot(b.x - x, b.y - y) <= RES, (name, b)
        assert abs(b.yaw) < 1e-9


def test_completed_map_has_all_four_faces(arena, tmp_path):
    path, layout = arena
    boxes, _ = detect_boxes(load_map(path), BoxParams(arena_margin=10.0), layout)
    write_completed_map(path, tmp_path / 'done', boxes)
    again, _ = detect_boxes(load_map(tmp_path / 'done.yaml'), BoxParams(arena_margin=10.0), layout)
    assert sorted(b.faces for b in again) == ['ENWS'] * 3
    with pytest.raises(FileExistsError):
        write_completed_map(path, tmp_path / 'done', boxes)
