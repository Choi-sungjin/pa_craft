#!/usr/bin/python3
"""Upper-cam distance output over time from a bag (diagnostics JSON, so bags without craft_interfaces work too).

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/realsense_check/uppercam_gap_plot.py ~/bags/uppercam_1109_2246 [--final-gap-mm 0]

Top: gap_mm (calibrated) and raw_gap_mm per capture, at capture time (filled) and arrival time (hollow), coloured by
the bridge state; the trigger line (camera_final_gap + tolerance = 13 mm); reference = odom distance still to travel
to where the robot finally stopped, plus --final-gap-mm (0 = it ended touching the box). Bottom: odom speed.
Vertical lines: box phases (camera_dock, final_stop, ...).
"""
import argparse
import bisect
import json
import math
import os
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lidar_floor'))
from floor_common import read_bag  # noqa: E402

COLORS = {'valid': '#2ca02c', 'geometry_unverified': '#ff7f0e', 'warmup': '#9467bd'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('--out'); ap.add_argument('--final-gap-mm', type=float, default=0.0)
    a = ap.parse_args()
    name = os.path.basename(a.bag.rstrip('/'))
    out = Path(a.out or Path(__file__).resolve().parent / 'out' / name); out.mkdir(parents=True, exist_ok=True)
    diag, odom, phase = [], [], []
    for topic, m, t in read_bag(a.bag, {'/perception/uppercam/diagnostics', '/odometry/local', '/planning/box_phase'}):
        if topic == '/perception/uppercam/diagnostics':
            try:
                diag.append((t, json.loads(m.data)))
            except ValueError:
                pass
        elif topic == '/odometry/local':
            q = m.pose.pose.orientation
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x))
        elif topic == '/planning/box_phase' and (not phase or phase[-1][1] != m.data):
            phase.append((t, m.data))
    if not diag:
        raise SystemExit('no /perception/uppercam/diagnostics')
    O = np.array(odom); t0 = diag[0][0] - 2
    fin = O[-1]
    h = fin[3]

    def remaining(t):  # odom distance still to go to the final rest, along the final heading
        i = min(bisect.bisect_left(O[:, 0], t), len(O) - 1)
        return ((fin[1] - O[i, 1]) * math.cos(h) + (fin[2] - O[i, 2]) * math.sin(h)) * 1000 + a.final_gap_mm

    rows = []
    for t, d in diag:
        cap = d.get('capture_stamp_s') or t
        rows.append((cap - t0, t - t0, d.get('gap_mm'), d.get('raw_gap_mm'), 'valid' if d.get('valid') else (d.get('state') or 'invalid'),
                     d.get('processing_ms'), d.get('capture_age_ms')))
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={'height_ratios': [3, 1]})
    for st in sorted({r[4] for r in rows}):
        R = [r for r in rows if r[4] == st and r[2] is not None]
        if R:
            R = np.array([(r[0], r[1], r[2]) for r in R], float)
            ax.plot(R[:, 0], R[:, 2], 'o', ms=4, color=COLORS.get(st, '#d62728'), label=f'gap_mm ({st}) at capture')
            ax.plot(R[:, 1], R[:, 2], 'o', ms=4, mfc='none', color=COLORS.get(st, '#d62728'), alpha=.5, label=f'gap_mm ({st}) at arrival')
    Rr = np.array([(r[0], r[3]) for r in rows if r[3] is not None], float)
    if len(Rr):
        ax.plot(Rr[:, 0], Rr[:, 1], '.', ms=3, color='#7f7f7f', label='raw_gap_mm at capture')
    tt = np.arange(rows[0][1] - 1 + t0, O[-1, 0], 0.05)
    ax.plot(tt - t0, [remaining(x) for x in tt], '-', color='#1f77b4', lw=1.5,
            label=f'reference: odom distance to the final rest + {a.final_gap_mm:.0f} mm')
    ax.axhline(13, color='k', ls=':', lw=1, label='trigger 13 mm (10 + 3)')
    ax.axhline(0, color='k', lw=.8)
    for pt, pn in phase:
        if pt - t0 > -1:
            for axx in (ax, bx):
                axx.axvline(pt - t0, color='#bbbbbb', lw=.8)
            ax.text(pt - t0, 1.0, pn, rotation=90, fontsize=7, va='bottom', transform=ax.get_xaxis_transform())
    ax.set_ylim(-20, max(260, np.nanmax([r[2] or 0 for r in rows]) + 20)); ax.set_ylabel('gap [mm]'); ax.grid(alpha=.3)
    ax.legend(fontsize=7, loc='upper right'); ax.set_title(f'{name}: upper-cam distance output')
    sel = (O[:, 0] - t0 > -1)
    bx.plot(O[sel, 0] - t0, O[sel, 4] * 1000, color='#1f77b4'); bx.set_ylabel('odom v [mm/s]'); bx.grid(alpha=.3)
    bx.set_xlabel('time [s] (capture time for camera points)')
    bx.set_xlim(rows[0][0] - 1, O[-1, 0] - t0)
    fig.tight_layout(); fig.savefig(out / f'{name}_gap.png', dpi=100)
    print('t_capture  arrival-capture  gap_mm  raw_mm  reference_mm  state')
    for r in rows:
        if r[2] is not None or r[3] is not None:
            ref = remaining(r[0] + t0)
            print(f'{r[0]:8.2f}  {1000 * (r[1] - r[0]):6.0f} ms  {r[2] if r[2] is None else round(r[2], 1)!s:>7}  '
                  f'{r[3] if r[3] is None else round(r[3], 1)!s:>7}  {ref:8.1f}  {r[4]}')
    print('figure:', out / f'{name}_gap.png')


if __name__ == '__main__':
    main()
