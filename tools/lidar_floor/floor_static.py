#!/usr/bin/python3
"""Standing-still bag in a wide empty area -> every point is floor -> floor ROI table (perception.yaml lidar.floor_roi).

  /usr/bin/python3 floor_static.py <bag> [<bag> ...] [--max-floor-m 2.0] [--margin 0.15] [--clear-back 5]

Setup (2026-11-01 floor_cal_0034 was made this way): the robot stands still on the floor it will drive on, nothing
(walls, boxes, people, cables) within --max-floor-m of the LiDARs, raw scans recorded. Several bags (other spots /
headings) are pooled; the closest floor hit wins. Points within body_radius 0.30 m of the LiDAR midpoint and beyond
--max-floor-m are ignored (far hits = walls of the room, not floor).

Same geometry and cut as scan_merger.cpp: angle from each LiDAR's outward axis measured at the LiDAR, side = sign of
base_link y (+ left), only the outward half (<= 90 deg; the 180 deg FOV drops the rest). Per side:
  floor bin    = a 5 deg bin with >= --min-pts floor points
  clear_deg    = first floor bin - --clear-back (90 = no floor seen: nothing to cut)
  max_range_m  = per 5 deg bin from clear_deg to 90: 1st percentile floor range - --margin (closest hit - margin when
                 the bin has < --min-pts points); 8.0 (keep all) if no floor
Also prints how many of the bag's floor points the current perception.yaml ROI lets through.
"""
import argparse
import os

import numpy as np
import yaml

from floor_common import BC, FRONT, REAR, REPO, XF, XR, read_bag, scan_xy, to_base

NAMES = {(0, 1): 'front_left', (0, -1): 'front_right', (1, 1): 'rear_left', (1, -1): 'rear_right'}
STEP = 5.0


def extract(bags, skip_s, max_floor, cut_end_s=0.0):
    rows, nscan = [], 0
    for bag in bags:
        t0 = t_end = None
        if cut_end_s > 0:
            for _, _, t in read_bag(bag, {FRONT, REAR}):
                t_end = t
        for topic, m, t in read_bag(bag, {FRONT, REAR}):
            t0 = t if t0 is None else t0
            if t - t0 < skip_s or (t_end is not None and t > t_end - cut_end_s):
                continue
            nscan += 1
            rg, th = scan_xy(m)
            bx, by = to_base(topic, rg, th)
            lid = 0 if topic == FRONT else 1
            lx = (bx - XF) if lid == 0 else (XR - bx)
            ang = np.degrees(np.arctan2(np.abs(by), lx))          # 0 outward .. 180 inward
            r = np.hypot(lx, by)
            k = (np.hypot(bx - BC, by) >= 0.30) & (r < max_floor) & (ang <= 90)
            rows.append(np.c_[np.full(k.sum(), lid), np.sign(by[k]), ang[k], r[k]])
    if not rows or not sum(len(x) for x in rows):
        raise SystemExit('no floor points: are /lidar/front/scan and /lidar/rear/scan in the bag?')
    return np.vstack(rows), nscan


def derive(R, min_pts, clear_back, margin):
    lid, side, ang, rng = R.T
    table = {}
    for key in NAMES:
        sel = (lid == key[0]) & (side == key[1])
        first = next((lo for lo in range(0, 90, 5) if (sel & (ang >= lo) & (ang < lo + 5)).sum() >= min_pts), None)
        if first is None:
            table[key] = (90.0, [8.0])                   # no-op; [] has no type as a ROS parameter
            continue
        clear = max(0.0, first - clear_back)
        ranges, lo = [], clear
        while lo < 90 - 1e-9:
            f = sel & (ang >= lo) & (ang < lo + STEP)
            if f.sum() >= min_pts:
                ranges.append(round(max(0.0, np.percentile(rng[f], 1) - margin), 2))
            elif f.sum():                                     # a few hits: the closest one, not a percentile
                ranges.append(round(max(0.0, rng[f].min() - margin), 2))
            else:
                ranges.append(8.0)
            lo += STEP
        table[key] = (clear, ranges)
    return table


def passed(R, table, step=STEP):
    """Points scan_merger keeps with this table (all points here are floor, so this is what leaks through)."""
    lid, side, ang, rng = R.T
    keep = np.ones(len(R), bool)
    for (li, sd), (clear, ranges) in table.items():
        s = (lid == li) & (side == sd) & (ang > clear) & (ang <= 90)
        b = np.clip(((ang - clear) / step).astype(int), 0, len(ranges))
        keep &= ~(s & (rng > np.array(list(ranges) + [-1.0])[b]))
    return keep


def current_table():
    cfg = yaml.safe_load(open(REPO / 'src/craft_perception/config/perception.yaml'))

    def find(d):
        if isinstance(d, dict):
            if 'floor_roi' in d:
                return d['floor_roi']
            for v in d.values():
                r = find(v)
                if r is not None:
                    return r
        return None
    roi = find(cfg)
    if not roi or not roi.get('enabled', False):
        return None
    return {k: (float(roi[f'{n}_clear_deg']), list(roi[f'{n}_max_range_m'])) for k, n in NAMES.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--max-floor-m', type=float, default=2.0)
    ap.add_argument('--min-pts', type=int, default=20)
    ap.add_argument('--clear-back', type=float, default=5.0)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--skip-s', type=float, default=0.0, help='drop the first seconds of each bag (person walking away)')
    ap.add_argument('--cut-end-s', type=float, default=0.0, help='drop the last seconds of each bag (person walking back)')
    a = ap.parse_args()
    R, nscan = extract(a.bags, a.skip_s, a.max_floor_m, a.cut_end_s)
    lid, side, ang, rng = R.T
    print(f'{nscan} scans, {len(R)} floor points in the outward half (0.30 m .. {a.max_floor_m} m)')
    for key, name in NAMES.items():
        sel = (lid == key[0]) & (side == key[1])
        print(f'\n{name}: {sel.sum()} points ({sel.sum() / max(1, nscan / 2):.1f} per scan)')
        print('    bin    | points | floor range 1st pct / median [m]')
        for lo in range(0, 90, 5):
            f = sel & (ang >= lo) & (ang < lo + 5)
            if f.sum():
                print(f'    {lo:2d}-{lo + 5:2d}  | {f.sum():6d} | {np.percentile(rng[f], 1):.2f} / {np.median(rng[f]):.2f}')
    cur = current_table()
    if cur:
        k = passed(R, cur)
        print(f'\ncurrent perception.yaml ROI lets through {k.sum()} of {len(R)} floor points ({k.sum() / max(1, nscan / 2):.2f} per scan)')
        for key, name in NAMES.items():
            sel = (lid == key[0]) & (side == key[1])
            if (sel & k).sum():
                print(f'    {name}: {(sel & k).sum()} at {np.round(np.percentile(ang[sel & k], [0, 50, 100]), 1).tolist()} deg, '
                      f'{np.round(np.percentile(rng[sel & k], [0, 50, 100]), 2).tolist()} m (min / median / max)')
    tab = derive(R, a.min_pts, a.clear_back, a.margin)
    k = passed(R, tab)
    print(f'\nnew table lets through {k.sum()} of {len(R)} floor points')
    print('\n      floor_roi:\n        enabled: true\n        step_deg: 5.0')
    for key, (clear, ranges) in tab.items():
        print(f'        {NAMES[key]}_clear_deg: {clear:.1f}\n        {NAMES[key]}_max_range_m: [{", ".join(f"{v:.2f}" for v in ranges)}]')


if __name__ == '__main__':
    main()
