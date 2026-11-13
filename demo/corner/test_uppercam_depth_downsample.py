"""Box depth on the validated 640x360 grid while colour runs at 1280x720 (uppercam_bridge_utils).

Synthetic D435-like rig: depth camera 424x240, colour 1280x720 15 mm to its right, floor at 1.0 m and a
20 cm square box top at 0.8 m. Both downsampling paths are compared with the exact depth seen from the
colour camera on the 640x360 grid.
  python3 -m pytest -q demo/corner/test_uppercam_depth_downsample.py
"""
import numpy as np
import pytest

from uppercam_bridge_utils import box_grid, depth_align_nearest, depth_reproject, scale_intrinsics

KD = np.array([[211.0, 0, 211.5], [0, 211.0, 119.5], [0, 0, 1]])     # 424x240 depth
KC = np.array([[912.0, 0, 639.5], [0, 912.0, 359.5], [0, 0, 1]])     # 1280x720 colour
R, T = np.eye(3), np.array([-0.015, 0.0, 0.0])                       # depth -> colour (colour 15 mm right)
BOX = (640, 360)


def cast(K, size, origin):
    """Depth (camera z) of floor z=1.0 with a box top z=0.8, |x|,|y| <= 0.1, seen from `origin` (depth frame)."""
    w, h = size
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    dx, dy = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
    z = np.full((h, w), 1.0)
    top = 0.8 - origin[2]
    x, y = origin[0] + dx * top, origin[1] + dy * top
    z[(np.abs(x) <= 0.1) & (np.abs(y) <= 0.1)] = top
    return z


@pytest.fixture(scope='module')
def scene():
    raw = cast(KD, (424, 240), np.zeros(3))
    k_box = scale_intrinsics(KC, *box_grid((1280, 720), BOX))
    truth = cast(k_box, BOX, -T)
    return raw, k_box, truth


def check(depth, truth):
    err = np.abs(depth - truth)
    holes = depth == 0
    edge = np.zeros_like(truth, bool)  # 3 px around the box outline: mixed pixels allowed there
    jump = np.abs(np.diff(truth, axis=0)) > 0.05
    edge[:-1] |= jump; edge[1:] |= jump
    jump = np.abs(np.diff(truth, axis=1)) > 0.05
    edge[:, :-1] |= jump; edge[:, 1:] |= jump
    for _ in range(3):
        edge[1:] |= edge[:-1]; edge[:-1] |= edge[1:]; edge[:, 1:] |= edge[:, :-1]; edge[:, :-1] |= edge[:, 1:]
    inner = ~edge
    inner[:6] = inner[-6:] = False; inner[:, :6] = inner[:, -6:] = False  # image border: depth FOV differs
    return float(np.median(err[inner & ~holes])), float(holes[inner].mean()), float((err[inner & ~holes] > 0.01).mean())


def test_box_grid_and_intrinsics():
    assert box_grid((1280, 720), (640, 360)) == (0.5, 0.5)
    with pytest.raises(ValueError):
        box_grid((1280, 720), (640, 480))   # other aspect
    with pytest.raises(ValueError):
        box_grid((640, 360), (1280, 720))   # larger than colour
    k = scale_intrinsics(KC, 0.5, 0.5)
    assert k[0, 0] == pytest.approx(456.0) and k[0, 2] == pytest.approx(319.5) and k[1, 2] == pytest.approx(179.5)


def test_reproject_matches_truth(scene):
    raw, k_box, truth = scene
    depth = depth_reproject(raw, KD, R, T, k_box, BOX)
    median, holes, bad = check(depth, truth)
    assert median < 1e-3 and holes < 0.01 and bad < 0.01, (median, holes, bad)


def test_align_then_nearest_matches_truth(scene):
    raw, k_box, truth = scene
    aligned_hi = depth_reproject(raw, KD, R, T, KC, (1280, 720))  # stand-in for rs.align to 1280x720
    depth, k = depth_align_nearest(aligned_hi, KC, BOX)
    assert np.allclose(k, k_box)
    median, holes, bad = check(depth, truth)
    assert median < 1e-3 and holes < 0.01 and bad < 0.01, (median, holes, bad)


def test_both_paths_agree_on_the_box_outline(scene):
    raw, k_box, truth = scene
    a = depth_reproject(raw, KD, R, T, k_box, BOX)
    b, _ = depth_align_nearest(depth_reproject(raw, KD, R, T, KC, (1280, 720)), KC, BOX)
    top = lambda d: (np.abs(d - 0.8) < 0.01)  # box top mask
    rows = lambda m: np.nonzero(m.any(axis=1))[0]
    cols = lambda m: np.nonzero(m.any(axis=0))[0]
    for f in (rows, cols):
        ra, rb, rt = f(top(a)), f(top(b)), f(top(truth))
        assert abs(ra[0] - rt[0]) <= 1 and abs(ra[-1] - rt[-1]) <= 1
        assert abs(rb[0] - rt[0]) <= 1 and abs(rb[-1] - rt[-1]) <= 1


def test_no_depth_gives_zeros():
    out = depth_reproject(np.zeros((240, 424)), KD, R, T, scale_intrinsics(KC, .5, .5), BOX)
    assert out.shape == (360, 640) and not out.any()
