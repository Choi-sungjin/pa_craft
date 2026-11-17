#!/usr/bin/python3
"""Shelf (puck stand) from D435 depth alone, camera optical frame: top plane + footprint rectangle.

  1. deproject depth (stride) -> points; RANSAC up to 3 planes
  2. top = a plane with a parallel plane SHELF_HEIGHT below it (the floor); if only one plane is seen
     (close up, floor out of view) that plane is taken as the top
  3. top inliers -> 2D plane coords -> 5 mm occupancy grid -> largest blob -> minAreaRect
  4. accept when the rect matches shelf_size; front edge = rect edge nearest the camera
The LiDAR (z 20..53 mm) sees only the shelf front face, so `front edge` is what gets matched to it.

Usage: /usr/bin/python3 shelf_detect.py SESSION... -> <session>/results/shelf.csv
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

SHELF_SIZE = (0.205, 0.208)   # plan.py ASSUME shelf_size [m]
SHELF_HEIGHT = 0.07           # plan.py ASSUME shelf_height [m]
HEIGHT_TOL, PARALLEL_DEG, SIZE_TOL = 0.015, 5.0, 0.03
PLANE_THRESHOLD, GRID = 0.006, 0.005


def deproject(depth, K, unit, stride=2, max_depth=2.0):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(float) * unit
    ok = (z > 0.1) & (z < max_depth)
    z, u, v = z[ok], u[ok], v[ok]
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)


def ransac_plane(p, rng, iterations=200):
    best, best_n, best_d = None, None, None
    for _ in range(iterations):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((p - a) @ n) < PLANE_THRESHOLD
        if best is None or inl.sum() > best.sum():
            best, best_n, best_d = inl, n, -a @ n
    q = p[best]  # least-squares refit, normal toward the camera
    mean = q.mean(0)
    n = np.linalg.eigh((q - mean).T @ (q - mean))[1][:, 0]
    if n @ mean > 0:
        n = -n
    return n, -n @ mean, best


def planes(p, rng, count=3, min_points=300):
    out = []
    for _ in range(count):
        if len(p) < min_points:
            break
        n, d, inl = ransac_plane(p, rng)
        if inl.sum() < min_points:
            break
        out.append((n, d, p[inl]))
        p = p[~inl]
    return out


def detect_shelf(depth, K, unit, rng=None):
    rng = rng or np.random.default_rng(0)
    found = planes(deproject(depth, K, unit), rng)
    if not found:
        return None
    top, floor_seen = None, False
    for i, (n, d, q) in enumerate(found):
        for j, (n2, d2, _) in enumerate(found):
            # n faces the camera: a plane above the floor is nearer, so its d is SHELF_HEIGHT smaller
            if i != j and np.degrees(np.arccos(min(1, abs(n @ n2)))) < PARALLEL_DEG and \
                    abs((d2 - d) - SHELF_HEIGHT) < HEIGHT_TOL:
                top, floor_seen = (n, d, q), True
    if top is None:
        top = found[0]
    n, d, q = top
    e1 = np.cross(n, [0, 1, 0]) if abs(n[1]) < 0.9 else np.cross(n, [1, 0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    origin = q.mean(0)
    uv = np.stack([(q - origin) @ e1, (q - origin) @ e2], 1)
    lo = uv.min(0)
    cells = np.floor((uv - lo) / GRID).astype(int)
    grid = np.zeros(cells.max(0)[::-1] + 1, np.uint8)
    grid[cells[:, 1], cells[:, 0]] = 255
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats_, _ = cv2.connectedComponentsWithStats(grid)
    if count < 2:
        return None
    blob = 1 + np.argmax(stats_[1:, cv2.CC_STAT_AREA])
    ys, xs = np.nonzero(labels == blob)
    (rx, ry), (rw, rh), _ = cv2.minAreaRect(np.stack([xs, ys], 1).astype(np.float32))
    corners2 = cv2.boxPoints(((rx, ry), (rw, rh), _)) * GRID + lo + GRID / 2
    corners = np.array([origin + a * e1 + b * e2 for a, b in corners2])
    size = sorted([rw * GRID, rh * GRID])
    ok = all(abs(s - t) < SIZE_TOL for s, t in zip(size, sorted(SHELF_SIZE)))
    mids = [(corners[k] + corners[(k + 1) % 4]) / 2 for k in range(4)]
    k = int(np.argmin([np.linalg.norm(m) for m in mids]))
    return dict(valid=ok, floor_seen=floor_seen, center=corners.mean(0), normal=n, size=size,
                front_a=corners[k], front_b=corners[(k + 1) % 4], points=len(q))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', type=Path)
    args = ap.parse_args()
    cols = ['frame', 'valid', 'floor_seen', 'cx', 'cy', 'cz', 'nx', 'ny', 'nz', 'w', 'l',
            'fax', 'fay', 'faz', 'fbx', 'fby', 'fbz', 'points']
    for session in args.sessions:
        session = session.expanduser()
        meta = json.loads((session / 'meta.json').read_text())
        K = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1]])
        (session / 'results').mkdir(exist_ok=True)
        n = valid = 0
        with open(session / 'results' / 'shelf.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(cols)
            for row in csv.DictReader(open(session / 'frames.csv')):
                depth = cv2.imread(str(session / 'frames' / f"{row['frame']}_depth.png"), cv2.IMREAD_ANYDEPTH)
                s = detect_shelf(depth, K, meta['depth_unit_m'])
                n += 1
                if s is None:
                    w.writerow([row['frame'], 0] + [''] * (len(cols) - 2))
                    continue
                valid += s['valid']
                w.writerow([row['frame'], int(s['valid']), int(s['floor_seen']), *np.round(s['center'], 4),
                            *np.round(s['normal'], 4), *np.round(s['size'], 4), *np.round(s['front_a'], 4),
                            *np.round(s['front_b'], 4), s['points']])
        print(f'{session.name}: {valid}/{n} frames with a shelf-sized top -> results/shelf.csv')


if __name__ == '__main__':
    main()
