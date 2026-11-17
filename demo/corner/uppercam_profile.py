#!/usr/bin/env python3
"""Per-stage timing of the square_far20 geometry on recorded upper cam frames (no camera, no ROS).

  /usr/bin/python3 demo/corner/uppercam_profile.py <frames dir> [--n 60]
  /usr/bin/python3 demo/corner/uppercam_profile.py <frames dir> --compare   # box_top_cropped == box_top, all frames

Frames as written by boxgap code/read_bag.py: t<ms>_depth.u16 (640x360 aligned to colour, 1 mm) + t<ms>_meta.txt.
Prints the median / p90 ms of each stage and of the whole geometry() call.
"""
import argparse, glob, os, sys, time
from collections import defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import uppercam_box_range as br
import uppercam_fast5_box_range as fast
import uppercam_square_far20_estimator as sq

TIMES = defaultdict(list)


def timed(module, name, label):
    original = getattr(module, name)
    def wrapper(*args, **kwargs):
        begin = time.perf_counter()
        try:
            return original(*args, **kwargs)
        finally:
            TIMES[label].append((time.perf_counter() - begin) * 1000)
    setattr(module, name, wrapper)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames')
    ap.add_argument('--n', type=int, default=60)
    ap.add_argument('--urdf', default=str(Path(__file__).parents[2] / 'src/craft_description/urdf/v307_full_robot.urdf'))
    ap.add_argument('--compare', action='store_true', help='check box_top_cropped against box_top on every frame')
    a = ap.parse_args()
    if a.compare:
        return compare(a)
    timed(fast, 'box_top_cropped', 'box_top_cropped (RANSAC+blob)')
    timed(br, 'rectangle', 'rectangle')
    timed(br, 'far_corner_visibility', 'far_corner_visibility')
    timed(br, 'complete_square_far', 'complete_square_far')
    timed(sq, 'fit_square', 'fit_square (least squares)')
    est = sq.SquareFar20Estimator(a.urdf)
    files = sorted(glob.glob(os.path.join(a.frames, '*_depth.u16')))
    files = files[::max(1, len(files) // a.n)][:a.n]
    ok = 0
    for f in files:
        meta = open(f.replace('_depth.u16', '_meta.txt')).read().split()
        w, h, fx, fy, cx, cy, scale = (float(v) for v in meta[:7])
        d = np.fromfile(f, '<u2').reshape(int(h), int(w)) * 1e-3
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        begin = time.perf_counter()
        try:
            est.geometry(d, K); ok += 1
        except Exception:
            pass
        TIMES['geometry total'].append((time.perf_counter() - begin) * 1000)
    print(f'{len(files)} frames, {ok} with geometry')
    for label, v in TIMES.items():
        v = sorted(v)
        print(f'{label:28s} n={len(v):3d}  median {v[len(v) // 2]:6.1f} ms  p90 {v[9 * len(v) // 10]:6.1f} ms')


def compare(a):
    up = sq.SquareFar20Estimator(a.urdf).up
    prm = dict(br.RANSAC, edge_drop=.020)
    files = sorted(glob.glob(os.path.join(a.frames, '*_depth.u16')))
    same = different = 0
    for f in files:
        meta = open(f.replace('_depth.u16', '_meta.txt')).read().split()
        w, h, fx, fy, cx, cy = (float(v) for v in meta[:6])
        d = np.fromfile(f, '<u2').reshape(int(h), int(w)) * 1e-3
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        out = []
        for fn in (fast.box_top, fast.box_top_cropped):
            try:
                out.append(fn(d, K, up, .6229, np.random.default_rng(0), .5, None, prm))
            except Exception as e:
                out.append(str(e))
        x, y = out
        equal = x == y if isinstance(x, str) or isinstance(y, str) else all(np.array_equal(p, q) for p, q in zip(x, y))
        same += equal; different += not equal
        if not equal:
            print('DIFFERENT', f)
    print(f'{len(files)} frames: {same} identical, {different} different')
    return 1 if different else 0


if __name__ == '__main__':
    sys.exit(main())
