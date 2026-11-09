#!/usr/bin/python3
"""Side-by-side figure: maze-dice drawing (L2 as a 205 mm box) next to an arena-frame map, same mm axes.

  /usr/bin/python3 compare_figure.py ~/maps/arena5_legend_arena_1cm.yaml out.png [--title TEXT]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

MAPMAKER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MAPMAKER / 'register'))
from register_maze import ARENA, X0, X1, Y0, Y1  # noqa: E402

MAZE_L2 = MAPMAKER / 'reference/maze-dice-L2box.png'   # L2 pedestal replaced by a 205 mm box like L1/L3
CROP = (190, 1499, 155, 2010)                           # plot area of the drawing [px]


def figure(yaml_file, png, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for f in fm.findSystemFonts():
        if 'NotoSansCJK' in f:
            fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
    a = np.array(Image.open(MAZE_L2).convert('RGB'))
    sx, sy = (X1 - X0) / ARENA[0], (Y0 - Y1) / ARENA[1]
    cx0, cx1, cy0, cy1 = CROP
    d = yaml.safe_load(Path(yaml_file).read_text())
    p = np.flipud(np.array(Image.open(Path(yaml_file).parent / d['image'])))
    res, (ox, oy) = float(d['resolution']), map(float, d['origin'][:2])
    fig, ax = plt.subplots(1, 2, figsize=(20, 13.5), dpi=110)
    ax[0].imshow(a[cy0:cy1, cx0:cx1], extent=[(cx0 - X0) / sx, (cx1 - X0) / sx, (Y0 - cy1) / sy, (Y0 - cy0) / sy])
    ax[0].set_title('maze-dice (L2 → 205 mm 박스)', fontsize=16)
    ax[1].imshow(p, cmap='gray', vmin=0, vmax=255, origin='lower', interpolation='nearest',
                 extent=[ox * 1000, (ox + p.shape[1] * res) * 1000, oy * 1000, (oy + p.shape[0] * res) * 1000])
    ax[1].plot([0, ARENA[0], ARENA[0], 0, 0], [0, 0, ARENA[1], ARENA[1], 0], '--', color='tab:orange', lw=1, alpha=.7)
    ax[1].set_title(title, fontsize=16)
    for A in ax:
        A.set_xlim(-250, ARENA[0] + 250); A.set_ylim(-250, ARENA[1] + 250); A.set_aspect('equal')
        A.set_xlabel('X (mm)'); A.set_ylabel('Y (mm)'); A.grid(alpha=.25)
        A.set_xticks(range(0, 4465, 1000)); A.set_yticks(range(0, 6365, 1000))
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(png)
    return png


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map'); ap.add_argument('png')
    ap.add_argument('--title', default=None)
    args = ap.parse_args()
    print('wrote', figure(args.map, args.png, args.title or Path(args.map).stem))


if __name__ == '__main__':
    main()
