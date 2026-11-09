#!/usr/bin/python3
"""Arena bags -> per-LiDAR, per-side floor ROI table (perception.yaml lidar.floor_roi) + cross-validation.

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/lidar_floor/derive_roi.py --map ~/maps/arena3_clean.yaml \
      --bags pocket_2248,pocket_2309 fov180_2359            # groups (comma = one group); each is tested on the others

A point counts as floor when it is more than --floor-m (10 cm) from every map wall, and as a wall point when it is
within 4 cm of one (needs good localization: rows start once map->odom has jumped > 0.3 m = SLAM localized, or after --start-s). Angles
are from each LiDAR's outward axis, measured at the LiDAR; only the outward half (0-90 deg) is judged, the body
(0.30 m around the LiDAR midpoint) and points beyond 3 m are dropped.

Rule (2026-11-01, the values in perception.yaml): the floor-start angle is the first 5 deg bin from 45 deg whose
floor fraction is >= 3x the 0-45 deg level; clear_deg = that angle - --clear-back (5 deg); each 5 deg bin from
clear_deg to 90 keeps points within (1st percentile floor range in the bin) - --margin (0.15 m); a bin with < 20
floor points keeps everything (8.0 = range_max; the 2026-11-01 script wrote 0 = drop all, never hit in its table). Prints the table from all bags as a perception.yaml block.

--source raw (default) uses /lidar/front/scan + /lidar/rear/scan: use this for bags recorded with the ROI on, since
/lidar/merged is already cut by it. --source merged reads /lidar/merged (the 2026-11-01 run did; ROI off then).
Extracted rows are cached in --cache (default ./out/rows_<bag>_<source>.npy); delete them after changing the map.
"""
import argparse
import math
import os

import numpy as np

from floor_common import BC, FRONT, MOUNT, REAR, XF, XR, WallDist, comp, read_bag, scan_xy, to_base, T2

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {(0, 1): 'front_left', (0, -1): 'front_right', (1, 1): 'rear_left', (1, -1): 'rear_right'}


def rows_from(lidar, bx, by, base, wall):
    """lidar 0 front / 1 rear -> [lidar, side (+1 left), angle from outward, range from the LiDAR, wall distance]."""
    lx = (bx - XF) if lidar == 0 else (XR - bx)
    keep = lx >= -0.005                                   # outward half
    bx, by, lx = bx[keep], by[keep], lx[keep]
    ang = np.degrees(np.arctan2(np.abs(by), lx))
    rng = np.hypot(lx, by)
    body = np.hypot(bx - BC, by) < 0.30
    mx = base[0] + math.cos(base[2]) * bx - math.sin(base[2]) * by
    my = base[1] + math.sin(base[2]) * bx + math.cos(base[2]) * by
    k = ~body & (rng < 3.0)
    return np.c_[np.full(k.sum(), lidar), np.sign(by[k]), ang[k], rng[k], wall(mx[k], my[k])]


def extract(bag, source, wall, cache, start_s=None):
    if os.path.exists(cache):
        return np.load(cache)
    topics = {'/tf'} | ({'/lidar/merged'} if source == 'merged' else {FRONT, REAR})
    tf, localized, last, n, rows, t0 = {}, False, None, 0, [], None
    for topic, m, t in read_bag(bag, topics):
        t0 = t if t0 is None else t0
        if start_s is not None:
            localized = t - t0 >= start_s
        if topic == '/tf':
            for tr in m.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                tf[key] = T2(tr)
                if key == ('map', 'odom'):
                    if start_s is None and last and math.hypot(tf[key][0] - last[0], tf[key][1] - last[1]) > 0.3:
                        localized = True
                    last = tf[key]
            continue
        if not localized or ('odom', 'base_link') not in tf:
            continue
        n += 1
        if n % 2:
            continue
        base = comp(tf[('map', 'odom')], tf[('odom', 'base_link')])
        if topic == '/lidar/merged':                      # base_link frame, both LiDARs
            rg, th = scan_xy(m)
            bx, by = rg * np.cos(th), rg * np.sin(th)
            f, r = bx >= XF - 0.005, bx <= XR + 0.005
            rows.append(rows_from(0, bx[f], by[f], base, wall))
            rows.append(rows_from(1, bx[r], by[r], base, wall))
        else:
            rg, th = scan_xy(m)
            bx, by = to_base(topic, rg, th)
            rows.append(rows_from(0 if topic == FRONT else 1, bx, by, base, wall))
    if not rows:
        raise SystemExit(f'{bag}: no rows. Either {sorted(topics - {"/tf"})} is not in the bag (pocket_* have only '
                         f'/lidar/merged: use --source merged) or map->odom never jumped > 0.3 m (never localized)')
    R = np.vstack(rows)
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    np.save(cache, R)
    print(f'  {bag}: {len(R)} points after localization')
    return R


def derive(R, floor_m, clear_back, margin, step=5.0):
    lid, side, ang, rng, wall = R.T
    floor = wall > floor_m
    base = floor[ang < 45].mean()
    table = {}
    for key in NAMES:
        li, sd = key
        sel = (lid == li) & (side == sd)
        start = 90.0
        for lo in range(45, 90, 5):
            s = sel & (ang >= lo) & (ang < lo + 5)
            if (s & floor).sum() / max(1, s.sum()) >= 3 * base:
                start = lo
                break
        clear = max(0.0, start - clear_back) if start < 90 else 90.0
        ranges = []
        lo = clear
        while lo < 90 - 1e-9:
            f = sel & floor & (ang >= lo) & (ang < lo + step)
            ranges.append(round(max(0.0, np.percentile(rng[f], 1) - margin), 2) if f.sum() >= 20 else 8.0)
            lo += step
        table[key] = (clear, ranges)
    return table


def apply(R, table, step=5.0):
    """Same cut as scan_merger.cpp: beyond clear_deg up to 90 deg keep only rng <= max_range_m[bin]."""
    lid, side, ang, rng, _ = R.T
    keep = np.ones(len(R), bool)
    for (li, sd), (clear, ranges) in table.items():
        s = (lid == li) & (side == sd) & (ang > clear) & (ang <= 90)
        b = np.clip(((ang - clear) / step).astype(int), 0, len(ranges))
        lim = np.array(ranges + [-1.0])[b]
        keep &= ~(s & (rng > lim))
    return keep


def report(name, R, table, floor_m):
    _, _, ang, _, wall = R.T
    floor, walls, k, edge = wall > floor_m, wall < 0.04, apply(R, table), ang >= 55
    print(f'  {name}: floor pts per 1000: {1000 * floor.mean():.1f} -> {1000 * (floor & k).sum() / max(1, k.sum()):.1f}'
          f' | 55-90 deg: {1000 * floor[edge].mean():.1f} -> {1000 * (floor & k & edge).sum() / max(1, (k & edge).sum()):.1f}'
          f' | map-wall pts kept: {100 * (walls & k).sum() / max(1, walls.sum()):.1f} %,'
          f' 55-90 deg {100 * (walls & k & edge).sum() / max(1, (walls & edge).sum()):.1f} %')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', required=True, help='bag groups; comma joins bags into one group')
    ap.add_argument('--map', required=True, help='map yaml of the arena the bags were recorded in')
    ap.add_argument('--source', choices=('raw', 'merged'), default='raw')
    ap.add_argument('--floor-m', type=float, default=0.10)
    ap.add_argument('--clear-back', type=float, default=5.0)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--start-s', type=float, help='use points from this many s into the bag instead of waiting for a '
                    'map->odom jump > 0.3 m (when the robot started already localized)')
    ap.add_argument('--cache', default=os.path.join(HERE, 'out'))
    a = ap.parse_args()
    wall = WallDist(a.map)
    groups = []
    for grp in a.bags:
        names = [b for b in grp.split(',') if b]
        R = np.vstack([extract(b, a.source, wall, os.path.join(a.cache, f'rows_{os.path.basename(b.rstrip("/"))}_{a.source}.npy'), a.start_s)
                       for b in names])
        groups.append((grp, R))
    if len(groups) > 1:
        for i, (gn, R) in enumerate(groups):
            tab = derive(R, a.floor_m, a.clear_back, a.margin)
            print(f'table from {gn}: ' + ', '.join(f'{NAMES[k]} {v[0]:.0f}' for k, v in tab.items()))
            report(f'on {gn} (seen)', R, tab, a.floor_m)
            for j, (on, O) in enumerate(groups):
                if j != i:
                    report(f'on {on} (unseen)', O, tab, a.floor_m)
    R = np.vstack([g[1] for g in groups])
    tab = derive(R, a.floor_m, a.clear_back, a.margin)
    print('\ntable from all bags:')
    report('on all', R, tab, a.floor_m)
    print('\n      floor_roi:\n        enabled: true\n        step_deg: 5.0')
    for k, (clear, ranges) in tab.items():
        print(f'        {NAMES[k]}_clear_deg: {clear:.1f}\n        {NAMES[k]}_max_range_m: [{", ".join(f"{v:.2f}" for v in ranges)}]')


if __name__ == '__main__':
    main()
