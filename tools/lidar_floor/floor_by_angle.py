#!/usr/bin/python3
"""Where the floor shows up: off-wall points per angle band from each LiDAR's outward axis (arena bag).

  /usr/bin/python3 floor_by_angle.py <bag> --map ~/maps/arena5_legend_arena.yaml [--source raw|merged] [--skip-s 16]

Raw scans (default) show the floor the ROI would face; --source merged shows what is left after the current ROI.
A point > 10 cm from every map wall is off-wall (floor, or a person / box not in the map). Per side (left/right in
base_link) and per LiDAR: points, off-wall share, off-wall range median / 1st percentile in each band. The floor-start
angle is where the share jumps (derive_roi.py: >= 3x the 0-45 deg level).
"""
import argparse
import math

import numpy as np

from floor_common import BC, FRONT, REAR, XF, XR, WallDist, comp, read_bag, scan_xy, to_base, T2

BANDS = [(0, 45)] + [(lo, lo + 5) for lo in range(45, 90, 5)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('--map', required=True)
    ap.add_argument('--source', choices=('raw', 'merged'), default='raw')
    ap.add_argument('--skip-s', type=float, default=16.0, help='ignore the first seconds (before localization)')
    a = ap.parse_args()
    wall = WallDist(a.map)
    topics = {'/tf'} | ({'/lidar/merged'} if a.source == 'merged' else {FRONT, REAR})
    tf, t0, rows, nscan = {}, None, [], 0
    for topic, m, t in read_bag(a.bag, topics):
        t0 = t if t0 is None else t0
        if topic == '/tf':
            for tr in m.transforms:
                tf[(tr.header.frame_id, tr.child_frame_id)] = T2(tr)
            continue
        if t - t0 < a.skip_s or ('map', 'odom') not in tf or ('odom', 'base_link') not in tf:
            continue
        nscan += 1
        base = comp(tf[('map', 'odom')], tf[('odom', 'base_link')])
        rg, th = scan_xy(m, rmax=3.0)
        if topic == '/lidar/merged':
            bx, by = rg * np.cos(th), rg * np.sin(th)
            parts = [(0, bx >= XF - 0.005), (1, bx <= XR + 0.005)]
        else:
            bx, by = to_base(topic, rg, th)
            parts = [(0 if topic == FRONT else 1, np.ones(len(bx), bool))]
        for lid, sel in parts:
            x, y = bx[sel], by[sel]
            lx = (x - XF) if lid == 0 else (XR - x)
            out = lx >= -0.005
            x, y, lx = x[out], y[out], lx[out]
            keep = np.hypot(x - BC, y) >= 0.30
            mx = base[0] + math.cos(base[2]) * x - math.sin(base[2]) * y
            my = base[1] + math.sin(base[2]) * x + math.cos(base[2]) * y
            rows.append(np.c_[np.full(keep.sum(), lid), np.sign(y[keep]), np.degrees(np.arctan2(np.abs(y[keep]), lx[keep])),
                              np.hypot(lx, y)[keep], wall(mx[keep], my[keep])])
    if not rows:
        raise SystemExit('no scans after --skip-s with map->odom TF')
    lid, side, ang, rng, w = np.vstack(rows).T
    off = w > 0.10
    print(f'{nscan} scans ({a.source}), {len(ang)} outward-half points < 3 m, off-wall {off.sum()}')
    for li, ln in ((0, 'front'), (1, 'rear')):
        for sd, sn in ((1, 'left'), (-1, 'right')):
            print(f'\n{ln} {sn}:  band | points | off-wall share | off-wall range median / 1st pct')
            for lo, hi in BANDS:
                s = (lid == li) & (side == sd) & (ang >= lo) & (ang < hi); o = s & off
                print(f'    {lo:2d}-{hi:2d} | {s.sum():6d} | {100 * o.sum() / max(1, s.sum()):5.1f} % | '
                      + (f'{np.median(rng[o]):.2f} / {np.percentile(rng[o], 1):.2f} m' if o.sum() else '-'))


if __name__ == '__main__':
    main()
