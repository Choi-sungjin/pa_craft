#!/usr/bin/env python3
"""Offline check of the global localizer and the corner goal on a real map, with synthetic merged scans.

Each LiDAR keeps +-90 deg around its outward axis (front at +0.155 m, rear at -0.1295 m), like
/lidar/merged with the 180 deg FOV; 1 cm range noise.
  test_map_tools.py [~/maps/arena3_clean.yaml] [--trials 10]
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_tools import MapGrid, ScanMatcher, corner_goal  # noqa: E402

LIDARS = ((0.155, 0.0), (-0.1295, math.pi))  # x in base_link, outward direction


def synthetic_scan(grid, pose, rng, n=400, max_range=8.0):
    x0, y0, yaw = pose
    pts = []
    for lx, out in LIDARS:
        px, py = x0 + lx * math.cos(yaw), y0 + lx * math.sin(yaw)
        for a in np.linspace(-math.pi / 2, math.pi / 2, n):
            th = yaw + out + a
            for r in np.arange(0.05, max_range, 0.01):
                c, rr = grid.to_cell(px + r * math.cos(th), py + r * math.sin(th))
                c, rr = int(c), int(rr)
                if not (0 <= rr < grid.h and 0 <= c < grid.w) or grid.occupied[rr, c]:
                    break
            else:
                continue
            r += rng.normal(0, 0.01)
            wx, wy = px + r * math.cos(th), py + r * math.sin(th)  # back to base_link
            dx, dy = wx - x0, wy - y0
            pts.append((dx * math.cos(-yaw) - dy * math.sin(-yaw), dx * math.sin(-yaw) + dy * math.cos(-yaw)))
    return np.array(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map', nargs='?', default=str(Path.home() / 'maps/arena3_clean.yaml'))
    ap.add_argument('--trials', type=int, default=8)
    args = ap.parse_args()
    grid = MapGrid(args.map)
    matcher = ScanMatcher(grid)
    rng = np.random.default_rng(1)
    starts = grid.free_cells(0.25, 0.05)
    ok = 0
    for k in range(args.trials):
        truth = (*starts[rng.integers(len(starts))], rng.uniform(-math.pi, math.pi))
        pts = synthetic_scan(grid, truth, rng)
        pts = pts[rng.choice(len(pts), min(240, len(pts)), replace=False)]
        t0 = time.time()
        res = matcher.localize(pts)
        dt = time.time() - t0
        x, y, yaw = res['pose']
        e, ea = math.hypot(x - truth[0], y - truth[1]), math.degrees(abs(math.remainder(yaw - truth[2], 2 * math.pi)))
        good = e < 0.05 and ea < 2.0
        ok += good and not res['ambiguous']
        print(f'trial {k}: truth ({truth[0]:+.2f}, {truth[1]:+.2f}, {math.degrees(truth[2]):+6.1f}) -> '
              f'({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+6.1f})  err {e*100:4.1f} cm {ea:4.1f} deg  '
              f'score {res["score"]:.2f} second {res["second"]:.2f}{" AMBIGUOUS" if res["ambiguous"] else ""}  '
              f'{dt:.1f} s  {"OK" if good else "WRONG"}')
        gx, gy = corner_goal(grid, truth[:2])
        if k == 0:
            print(f'corner goal (bottom right) from trial 0 start: ({gx:+.3f}, {gy:+.3f}), '
                  f'{grid.dist[tuple(int(v) for v in grid.to_cell(gx, gy))[::-1]]:.2f} m from walls')
    print(f'{ok}/{args.trials} localized (< 5 cm, < 2 deg, not ambiguous)')
    return 0 if ok == args.trials else 1


if __name__ == '__main__':
    sys.exit(main())
