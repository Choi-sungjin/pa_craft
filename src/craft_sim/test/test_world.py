"""시뮬레이터 기하가 로봇 코드의 정의와 같은지 확인하는 단위 시험 (ROS 없이 실행)."""
import math
import os
import random
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
REPO_SRC = Path(os.environ.get('CRAFT_SIM_SRC', PKG.parent))
CORNER = REPO_SRC.parent / 'demo' / 'corner'

from craft_sim.world import Box, DiffDrive, GridMap, box_geometry, lidar_box_fit  # noqa: E402


def test_diff_drive_arc_and_straight():
    r = DiffDrive(0.033, 0.333, (0.0, 0.0, 0.0))
    for _ in range(50):
        r.step(5.0, 5.0, 0.02)          # 1 s 직진
    assert r.x == pytest.approx(0.165, abs=1e-9) and abs(r.y) < 1e-12
    r = DiffDrive(0.033, 0.333)
    w = 0.5 * 0.333 / 0.033              # 제자리 0.5 rad/s
    for _ in range(100):
        r.step(-w / 2, w / 2, 0.01)
    assert r.th == pytest.approx(0.5, abs=1e-9) and math.hypot(r.x, r.y) < 1e-9


def test_map_loads():
    m = GridMap(PKG / 'maps' / 'arena5_legend_opt_arena1cm.yaml')
    assert m.res == pytest.approx(0.01) and m.occupied.any() and m.free.any()
    assert m.wall_distance(np.array([0.527]), np.array([5.889]))[0] > 0.3   # 시나리오 출발점은 빈 곳


@pytest.mark.skipif(not (CORNER / 'uppercam_bridge_utils.py').exists(), reason='demo/corner 없음')
def test_camera_geometry_matches_bridge():
    sys.path.insert(0, str(CORNER))
    from uppercam_bridge_utils import camera_box_in_map
    rng = random.Random(1)
    for _ in range(500):
        b = Box('L', rng.uniform(1, 4), rng.uniform(1, 5), math.radians(rng.uniform(-180, 180)))
        n = b.yaw + rng.randrange(4) * math.pi / 2
        d = rng.uniform(0.3, 0.9)
        off = rng.uniform(-0.05, 0.05)
        rx, ry = b.x + math.cos(n) * d - math.sin(n) * off, b.y + math.sin(n) * d + math.cos(n) * off
        th = n + math.pi + math.radians(rng.uniform(-20, 20))
        g = box_geometry((rx, ry, th), b, 0.165)
        cx, cy, _ = camera_box_in_map(g['gap_m'], g['lateral_m'], g['yaw_rad'], (rx, ry, th), 0.165, 0.205)
        assert math.hypot(cx - b.x, cy - b.y) < 1e-9


@pytest.mark.skipif(not (CORNER / 'uppercam_lidar_geometry.py').exists(), reason='demo/corner 없음')
def test_lidar_fit_matches_fit_box_face():
    """같은 1° 광선 점을 실제 fit_box_face 에 넣은 결과와 시뮬레이터 참값 관측이 같은지 (유효 여부 포함)."""
    sys.path.insert(0, str(CORNER))
    from uppercam_lidar_geometry import fit_box_face
    rng = random.Random(3)
    compared = 0
    for _ in range(300):
        ex = (3.577, 1.076, math.radians(90))
        tb = Box('L1', ex[0] + rng.uniform(-0.04, 0.04), ex[1] + rng.uniform(-0.04, 0.04),
                 ex[2] + math.radians(rng.uniform(-8, 8)))
        d = rng.uniform(0.02, 0.85)
        rx, ry = 3.4745 - 0.165 - d, 1.076 + rng.uniform(-0.03, 0.03)
        th = math.radians(rng.uniform(-6, 6))
        mine, _ = lidar_box_fit((rx, ry, th), tb, ex, 0.205, 0.165)
        L = np.array([rx + 0.155 * math.cos(th), ry + 0.155 * math.sin(th)])
        cands = [np.array([math.cos(tb.yaw + k * math.pi / 2), math.sin(tb.yaw + k * math.pi / 2)]) for k in range(4)]
        nt = max(cands, key=lambda c: c @ (L - np.array([tb.x, tb.y])))
        tt = np.array([-nt[1], nt[0]])
        mt = np.array([tb.x, tb.y]) + nt * tb.side / 2
        pts = []
        for a in np.radians(np.arange(-90, 91, 1.0)):
            u = np.array([math.cos(th + a), math.sin(th + a)])
            den = u @ nt
            if abs(den) < 1e-9:
                continue
            t = (mt - L) @ nt / den
            if t > 0 and 0.10 <= t <= 1.0 and abs((L + t * u - mt) @ tt) <= tb.side / 2:
                pts.append(L + t * u)
        real = fit_box_face(np.array(pts), ex, (rx, ry, th), side=0.205, front_face_x=0.165) if pts else None
        assert (real is None) == (mine is None)
        if real is None:
            continue
        compared += 1
        assert math.dist(real['box_center_map'], mine['box_center_map']) < 1e-9
        assert real['gap_m'] == pytest.approx(mine['gap_m'], abs=1e-9)
        assert real['lateral_m'] == pytest.approx(mine['lateral_m'], abs=1e-9)
        assert abs(math.remainder(real['yaw_relative'] - mine['yaw_rad'], 2 * math.pi)) < 1e-9
    assert compared > 250
