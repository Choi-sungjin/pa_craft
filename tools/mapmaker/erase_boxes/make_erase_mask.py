#!/usr/bin/env python3
"""Where erase_boxes erases: every arena box footprint grown by a margin, walls left out, as a ROS map (black = erase).

  /usr/bin/python3 make_erase_mask.py <planning map.yaml> <out dir> [--margin 0.08]

Boxes and walls both come from the planning map (box_detector.py: its walls are straight hit runs >= 0.32 m, so a
box face is never one). The pose graph's box points smear up to ~7 cm off the faces (2026-11-10, arena5 L1), hence
the margin; a wall inside a grown footprint (S2, S6 stand by one) is kept. Writes <out dir>/boxes.json and
erase_mask.yaml/.pgm in the planning map's geometry (same frame as the pose graph).
"""
import argparse, json, math, sys
from pathlib import Path
import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/craft_planning/src'))
import box_detector as bd  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map'); ap.add_argument('out')
    ap.add_argument('--margin', type=float, default=.08)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    grid = bd.load_map(a.map)
    prm = bd.BoxParams()
    boxes, _ = bd.detect_boxes(grid, prm)
    matched, _ = bd.match_layout(boxes)
    found = [dict(name=n, x=round(b.x, 4), y=round(b.y, 4), yaw=round(b.yaw, 4))
             for n, b in (matched.items() if isinstance(matched, dict) else matched) if b is not None]
    json.dump(found, open(out / 'boxes.json', 'w'), indent=1)
    walls = bd.wall_mask(grid.hit, grid.res, prm)
    h, w = grid.hit.shape
    iy, ix = np.mgrid[0:h, 0:w]
    x, y = grid.centre(ix, iy)
    erase = np.zeros_like(grid.hit)
    half = prm.side / 2 + a.margin
    for b in found:
        c, s = math.cos(b['yaw']), math.sin(b['yaw'])
        dx, dy = x - b['x'], y - b['y']
        erase |= (np.abs(c * dx + s * dy) <= half) & (np.abs(-s * dx + c * dy) <= half)
    kept = int((erase & walls).sum())
    erase &= ~walls
    image = np.full((h, w), 254, np.uint8); image[erase] = 0
    Image.fromarray(image[::-1]).save(out / 'erase_mask.pgm')
    meta = yaml.safe_load(Path(a.map).read_text()); meta['image'] = 'erase_mask.pgm'
    yaml.safe_dump(meta, open(out / 'erase_mask.yaml', 'w'))
    print(f'{len(found)} boxes ({", ".join(b["name"] for b in found)}); {int(erase.sum())} cells to erase, '
          f'{kept} wall cells kept; wrote {out}/boxes.json, erase_mask.yaml/.pgm')


if __name__ == '__main__':
    main()
