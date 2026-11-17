#!/usr/bin/python3
"""record.py sessions -> Ultralytics YOLO dataset with pre-labels to review, not ground truth.

  class 0 target_blue  (same id as models/puck_blue) - pre-labelled only with --puck-model
  class 1 shelf        - pre-labelled from shelf_detect (depth top rect + 7 cm sides projected to a box)
Frames are thinned (--every) and near-duplicates dropped (still puck -> many identical frames).
Review/fix the boxes in any YOLO-format tool (CVAT, Label Studio, labelImg) before training.

Usage: ~/craft/venv-craft/bin/python3 export_yolo.py ~/bags/puck_compare/* --out ~/datasets/puck_shelf
       [--puck-model src/craft_perception/models/puck_blue] [--every 5] [--val 0.2]
"""
import argparse
import csv
import json
import shutil
import tempfile
from pathlib import Path

import cv2
import numpy as np

from shelf_detect import SHELF_HEIGHT, detect_shelf

NAMES = ['target_blue', 'shelf']


def shelf_box(s, K, w, h):
    """Top rect corners + the same corners SHELF_HEIGHT down -> clipped pixel box, or None."""
    a, b = s['front_a'], s['front_b']
    c = s['center']
    top = np.array([a, b, 2 * c - a, 2 * c - b])
    pts = np.vstack([top, top - s['normal'] * SHELF_HEIGHT])
    uv = (K @ pts.T).T
    uv = uv[:, :2] / uv[:, 2:]
    x0, y0 = np.clip(uv.min(0), 0, [w, h])
    x1, y1 = np.clip(uv.max(0), 0, [w, h])
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return (x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w, (y1 - y0) / h


def puck_model(model_dir):
    """Ultralytics loads an NCNN export only from a dir named *_ncnn_model."""
    from ultralytics import YOLO
    tmp = Path(tempfile.mkdtemp()) / 'puck_blue_ncnn_model'
    shutil.copytree(model_dir, tmp)
    return YOLO(str(tmp), task='detect')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', type=Path)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--dup', type=float, default=2.0, help='mean abs diff (64x36 gray) below = duplicate')
    ap.add_argument('--val', type=float, default=0.2)
    ap.add_argument('--puck-model', type=Path)
    ap.add_argument('--conf', type=float, default=0.3)
    args = ap.parse_args()
    out = args.out.expanduser()
    for split in ('train', 'val'):
        (out / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / split).mkdir(parents=True, exist_ok=True)
    model = puck_model(args.puck_model) if args.puck_model else None
    rng = np.random.default_rng(0)
    kept = shelves = pucks = 0
    for session in args.sessions:
        session = session.expanduser()
        meta = json.loads((session / 'meta.json').read_text())
        K = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1]])
        # split per session: neighbouring frames of one session never land in both train and val
        split = 'val' if rng.random() < args.val else 'train'
        last = None
        for i, row in enumerate(csv.DictReader(open(session / 'frames.csv'))):
            if i % args.every:
                continue
            base = session / 'frames' / row['frame']
            bgr = cv2.imread(f'{base}_color.png')
            thumb = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (64, 36)).astype(float)
            if last is not None and np.abs(thumb - last).mean() < args.dup:
                continue
            last = thumb
            h, w = bgr.shape[:2]
            labels = []
            if model:
                for x, y, bw, bh in model(bgr, conf=args.conf, verbose=False)[0].boxes.xywhn.tolist():
                    labels.append((0, x, y, bw, bh))
                    pucks += 1
            s = detect_shelf(cv2.imread(f'{base}_depth.png', cv2.IMREAD_ANYDEPTH), K, meta['depth_unit_m'])
            box = shelf_box(s, K, w, h) if s and s['valid'] else None
            if box:
                labels.append((1, *box))
                shelves += 1
            name = f"{session.name}_{row['frame']}"
            cv2.imwrite(str(out / 'images' / split / f'{name}.jpg'), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            (out / 'labels' / split / f'{name}.txt').write_text(
                ''.join(f'{c} {x:.6f} {y:.6f} {bw:.6f} {bh:.6f}\n' for c, x, y, bw, bh in labels))
            kept += 1
    (out / 'data.yaml').write_text(f'path: {out}\ntrain: images/train\nval: images/val\n'
                                   f'names:\n' + ''.join(f'  {i}: {n}\n' for i, n in enumerate(NAMES)))
    print(f'{kept} images, {shelves} shelf / {pucks} puck pre-labels -> {out} (review before training)')


if __name__ == '__main__':
    main()
