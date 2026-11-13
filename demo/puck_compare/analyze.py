#!/usr/bin/python3
"""compare.py results -> one table per session: tag vs depth vs centerpose (results/<method>.csv).

The puck is still during a session, so without ground truth:
  hit      frames with a valid axis / frames
  jit50/95 axis scatter around the session mean [deg] (stability)
  c_std    center scatter [mm], norm of per-axis std
  tilt     angle from the axis to the shelf normal (the depth method's median per session) [deg]:
           standing should be ~0, lying ~90 -> shown as error |tilt - expected|
  stand    standing flag agrees with meta label (tag/depth)
  ms50/95  per-frame latency (tag/depth: box + axis; centerpose: YOLO + whole inference)
cross: same-frame angle between two methods' axes and center distance, for every pair.
Axes are compared unsigned (a cylinder axis is 180 deg symmetric).

Usage: /usr/bin/python3 analyze.py ~/bags/puck_compare/*_*  [--csv summary.csv]
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load(path):
    """-> {frame: best valid row}, frames seen; first valid row per frame (single puck per session)."""
    if not path.exists():
        return None, 0
    best, frames = {}, set()
    for r in csv.DictReader(open(path)):
        frames.add(r['frame'])
        if r['axis_valid'] == '1' and r['frame'] not in best:
            best[r['frame']] = r
    return best, len(frames)


def vec(r, k):
    return np.array([float(r[k + 'x']), float(r[k + 'y']), float(r[k + 'z'])])


def unsigned_deg(a, b):
    return np.degrees(np.arccos(np.clip(np.abs(np.sum(a * b, -1)), 0, 1)))


def mean_axis(axes):
    """Principal direction of +-axes (sign free)."""
    return np.linalg.eigh(axes.T @ axes)[1][:, -1]


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float('nan')


def stats(rows, n_frames, normal, label):
    s = dict(hit=len(rows) / n_frames if n_frames else float('nan'))
    ms = [float(r['detect_ms']) + float(r.get('axis_ms') or 0) for r in rows.values()]
    s['ms50'], s['ms95'] = pct(ms, 50), pct(ms, 95)
    if not rows:
        return s
    axes = np.array([vec(r, 'a') for r in rows.values()])
    centers = np.array([vec(r, 'c') for r in rows.values()])
    jitter = unsigned_deg(axes, mean_axis(axes))
    s['jit50'], s['jit95'] = pct(jitter, 50), pct(jitter, 95)
    s['c_std'] = float(np.linalg.norm(centers.std(0)) * 1e3)
    if normal is not None and label in ('standing', 'lying'):
        tilt = unsigned_deg(axes, normal)
        s['tilt'] = pct(np.abs(tilt - (0 if label == 'standing' else 90)), 50)
    if 'standing' in next(iter(rows.values())) and label in ('standing', 'lying'):
        s['stand'] = np.mean([(r['standing'] == '1') == (label == 'standing') for r in rows.values()])
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', type=Path)
    ap.add_argument('--csv', type=Path, help='also write all rows here')
    args = ap.parse_args()
    keys = ['hit', 'jit50', 'jit95', 'c_std', 'tilt', 'stand', 'ms50', 'ms95']
    out = []
    for session in args.sessions:
        session = session.expanduser()
        meta = json.loads((session / 'meta.json').read_text())
        results = {}
        for name in ('tag', 'depth', 'centerpose'):
            path = session / 'results' / f'{name}.csv'
            if name == 'depth' and not path.exists():
                path = session / 'results' / 'ours.csv'  # older compare.py runs
            results[name] = load(path)
        depth = results['depth'][0]
        normal = mean_axis(np.array([vec(r, 'n') for r in depth.values()])) if depth else None
        print(f"\n== {session.name}  label={meta['label']} yaw={meta.get('yaw_deg')}  {meta.get('note', '')}")
        print(f"{'method':<11}" + ''.join(f'{k:>8}' for k in keys))
        for name, (rows, n) in results.items():
            if rows is None:
                print(f'{name:<11}  (no results)')
                continue
            s = stats(rows, n, normal, meta['label'])
            print(f'{name:<11}' + ''.join(f"{s.get(k, float('nan')):>8.2f}" for k in keys))
            out.append(dict(session=session.name, label=meta['label'], method=name, frames=n, **s))
        names = list(results)
        for i, x in enumerate(names):
            for y in names[i + 1:]:
                ra, rb = results[x][0], results[y][0]
                common = sorted(set(ra or {}) & set(rb or {}))
                if not common:
                    continue
                a = np.array([vec(ra[f], 'a') for f in common])
                b = np.array([vec(rb[f], 'a') for f in common])
                d = np.linalg.norm(np.array([vec(ra[f], 'c') - vec(rb[f], 'c') for f in common]), axis=1) * 1e3
                ang = unsigned_deg(a, b)
                print(f'{x}-{y} ({len(common)} frames): axis diff p50 {pct(ang, 50):.1f} p95 {pct(ang, 95):.1f} deg,'
                      f' center diff p50 {pct(d, 50):.1f} p95 {pct(d, 95):.1f} mm')
                out.append(dict(session=session.name, label=meta['label'], method=f'{x}-{y}', frames=len(common),
                                axis_diff50=pct(ang, 50), axis_diff95=pct(ang, 95),
                                center_diff50=pct(d, 50), center_diff95=pct(d, 95)))
    if args.csv:
        fields = sorted({k for r in out for k in r}, key=lambda k: (k not in ('session', 'label', 'method'), k))
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fields)
            w.writeheader()
            w.writerows(out)
        print(f'\n-> {args.csv}')


if __name__ == '__main__':
    main()
