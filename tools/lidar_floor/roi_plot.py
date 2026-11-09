#!/usr/bin/python3
"""Floor ROI picture (base_link, top view): floor points of standing-still bags, the current perception.yaml ROI and
the table floor_static.py derives from the same bags.

  /usr/bin/python3 roi_plot.py <bag> [<bag> ...] [--out out/roi.png] [--show-m 2.5] (+ floor_static.py options)

Shaded = what scan_merger keeps (outward half of each LiDAR, up to --show-m for drawing). Dots = floor hits.
"""
import argparse
import math
import os

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

from floor_common import BC, XF, XR  # noqa: E402
from floor_static import NAMES, STEP, current_table, derive, extract, passed  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def xy(lid, side, ang, rng):
    a = np.radians(ang)
    x0 = np.where(lid == 0, XF, XR)
    sx = np.where(lid == 0, 1.0, -1.0)
    return x0 + sx * rng * np.cos(a), side * rng * np.sin(a)


def kept_polys(table, show):
    """One polygon per side: the kept area (clear wedge to `show`, then each bin up to its max range)."""
    polys = []
    for (li, sd), (clear, ranges) in table.items():
        segs = [(0.0, clear, show)]
        lo = clear
        for r in ranges:
            if lo >= 90 - 1e-9:
                break
            segs.append((lo, min(lo + STEP, 90.0), min(r, show)))
            lo += STEP
        x0, sx = (XF, 1.0) if li == 0 else (XR, -1.0)
        pts = [(x0, 0.0)]
        for a0, a1, r in segs:
            for a in np.linspace(a0, a1, max(2, int(a1 - a0) + 1)):
                t = math.radians(a)
                pts.append((x0 + sx * r * math.cos(t), sd * r * math.sin(t)))
        pts.append((x0, 0.0))
        polys.append(np.array(pts))
    return polys


def panel(ax, title, table, P, keep, show, color):
    for poly in kept_polys(table, show):
        ax.add_patch(Polygon(poly, closed=True, fc=color, ec=color, alpha=0.18, lw=1))
    ax.scatter(P[0][~keep], P[1][~keep], s=3, c='#888888', label=f'floor removed ({(~keep).sum()})')
    if keep.any():
        ax.scatter(P[0][keep], P[1][keep], s=6, c='#d62728', label=f'floor LEFT IN ({keep.sum()})')
    ax.add_patch(Rectangle((-0.141, -0.1772), 0.3665, 0.3544, fc='none', ec='k', lw=1.2))
    ax.add_patch(Circle((BC, 0), 0.30, fc='none', ec='k', ls=':', lw=0.8))
    for x, n in ((XF, 'F'), (XR, 'R')):
        ax.plot(x, 0, 'k^' if n == 'F' else 'kv', ms=7)
    ax.annotate('', xy=(0.55, 0), xytext=(0.30, 0), arrowprops=dict(arrowstyle='->', lw=1.5))
    ax.text(0.57, 0.02, '+x (front)', fontsize=8)
    ax.text(0.0, show * 0.93, 'left (+y)', ha='center', fontsize=8)
    for (li, sd), (clear, ranges) in table.items():
        x0, sx = (XF, 1.0) if li == 0 else (XR, -1.0)
        t = math.radians(clear)
        ax.text(x0 + sx * 0.75 * math.cos(t) * 1.6, sd * 0.75 * math.sin(t) * 1.6, f'{NAMES[(li, sd)]}\nclear {clear:.0f}°',
                fontsize=7, ha='center', va='center', color=color)
    ax.set_xlim(-show - 0.2, show + 0.2); ax.set_ylim(-show - 0.2, show + 0.2); ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title(title, fontsize=10); ax.legend(fontsize=8, loc='lower left'); ax.set_xlabel('base_link x [m]'); ax.set_ylabel('y [m]')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default=os.path.join(HERE, 'out', 'roi.png'))
    ap.add_argument('--show-m', type=float, default=2.5)
    ap.add_argument('--max-floor-m', type=float, default=2.0)
    ap.add_argument('--min-pts', type=int, default=20)
    ap.add_argument('--clear-back', type=float, default=5.0)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--skip-s', type=float, default=0.0)
    ap.add_argument('--cut-end-s', type=float, default=0.0)
    a = ap.parse_args()
    R, nscan = extract(a.bags, a.skip_s, a.max_floor_m, a.cut_end_s)
    P = xy(*R.T)
    new = derive(R, a.min_pts, a.clear_back, a.margin)
    cur = current_table()
    fig, axes = plt.subplots(1, 2 if cur else 1, figsize=(16 if cur else 8, 8), squeeze=False)
    k = 0
    if cur:
        panel(axes[0][0], 'current perception.yaml floor_roi', cur, P, passed(R, cur), a.show_m, '#1f77b4'); k = 1
    panel(axes[0][k], f'from these bags (margin {a.margin} m, clear-back {a.clear_back}°)', new, P, passed(R, new), a.show_m, '#2ca02c')
    fig.suptitle(f'{", ".join(os.path.basename(b.rstrip("/")) for b in a.bags)}: {nscan} scans, {len(R)} floor points '
                 f'(outward half, 0.30-{a.max_floor_m} m). Shaded = kept by scan_merger', fontsize=10)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out, dpi=90)
    print('figure:', os.path.abspath(a.out))


if __name__ == '__main__':
    main()
