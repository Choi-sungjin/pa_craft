#!/usr/bin/env python3
"""Before / after of erase_boxes: the matched scan points of the whole pose graph, with the box footprints.

  /usr/bin/python3 plot_erase.py before.csv after.csv boxes.json out.png [--zoom L1]
"""
import argparse, json, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm


def outline(b, half):
    c, s = math.cos(b['yaw']), math.sin(b['yaw'])
    pts = [(half, half), (-half, half), (-half, -half), (half, -half), (half, half)]
    return [b['x'] + c * u - s * v for u, v in pts], [b['y'] + s * u + c * v for u, v in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('before'); ap.add_argument('after'); ap.add_argument('boxes'); ap.add_argument('out')
    ap.add_argument('--zoom', default='L1', help='box to show close up in the second row')
    ap.add_argument('--margin', type=float, default=.03)
    a = ap.parse_args()
    boxes = json.load(open(a.boxes))
    data = [np.loadtxt(f, delimiter=',', skiprows=1, usecols=(1, 2)) for f in (a.before, a.after)]
    lo = np.minimum(data[0].min(0), data[1].min(0)); hi = np.maximum(data[0].max(0), data[1].max(0))
    res = .01
    bins = [np.arange(lo[0], hi[0] + res, res), np.arange(lo[1], hi[1] + res, res)]
    fig, axes = plt.subplots(2, 2, figsize=(13, 14), gridspec_kw={'height_ratios': [2.2, 1]})
    z = next(b for b in boxes if b['name'] == a.zoom)
    for col, (pts, title) in enumerate(zip(data, ('before', 'after: boxes erased'))):
        for row, ax in enumerate(axes[:, col]):
            h, xe, ye = np.histogram2d(pts[:, 0], pts[:, 1], bins=bins)
            ax.imshow(h.T, origin='lower', extent=(xe[0], xe[-1], ye[0], ye[-1]), cmap='magma',
                      norm=LogNorm(vmin=1, vmax=max(h.max(), 2)), interpolation='nearest')
            for b in boxes:
                x, y = outline(b, .1025); ax.plot(x, y, color='#3fb0ff', lw=.9)
                x, y = outline(b, .1025 + a.margin); ax.plot(x, y, color='#3fb0ff', lw=.6, ls=':')
                if row == 0: ax.text(b['x'], b['y'] + .17, b['name'], color='#3fb0ff', ha='center', fontsize=9)
            if row == 1:
                ax.set_xlim(z['x'] - .45, z['x'] + .45); ax.set_ylim(z['y'] - .45, z['y'] + .45)
                ax.set_title(f'{title}: {a.zoom} close up')
            else:
                ax.set_title(f'{title} ({len(pts):,} matched points)')
            ax.set_aspect('equal'); ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]')
    fig.suptitle('slam_toolbox pose graph: scan points used for localization (log density, 1 cm bins)\n'
                 'blue: 205 mm box, dotted: erased footprint (+%.0f mm)' % (a.margin * 1000))
    fig.tight_layout(); fig.savefig(a.out, dpi=110)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
