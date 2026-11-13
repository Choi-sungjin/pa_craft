#!/usr/bin/python3
"""Run the three puck-axis methods on the same record.py sessions (D435 colour + aligned depth):
  tag        PuckTagAxis  (puck_axis_offline --box tag): AprilTag faces only, no depth
  depth      PuckAxis     (puck_axis_offline --box yolo|apriltag): puck box + depth shelf plane (donut)
  centerpose CenterPose   (centerpose_adapter.py): YOLO box crop -> 9 cuboid points -> PnP
Writes <session>/results/{tag,depth,centerpose}.csv; analyze.py turns them into tables.
Methods run one after another so the latency numbers do not fight over the CPU/GPU.

Usage: source ~/craft/install/setup.bash
       PY=~/yolov8-ubuntu24-practice/.venv-yolo/bin/python   # torch + ultralytics for centerpose
       $PY compare.py ~/bags/puck_compare/*_*                       # all three
       /usr/bin/python3 compare.py SESSIONS --methods tag,depth    # no torch needed
"""
import argparse
import csv
import json
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

CenterPoseAdapter = None  # imported only when centerpose runs (needs torch)

HERE = Path(__file__).resolve().parent
COLUMNS = ['frame', 'stamp_ns', 'id', 'score', 'axis_valid', 'cx', 'cy', 'cz', 'ax', 'ay', 'az', 'detect_ms']


def run_offline(session, args, method):
    """tag -> PuckTagAxis, depth -> PuckAxis on the --depth-box boxes; both by the production C++."""
    exe = shutil.which('puck_axis_offline') or str(
        Path('~/craft/install/craft_perception/lib/craft_perception/puck_axis_offline').expanduser())
    box = 'tag' if method == 'tag' else args.depth_box
    cmd = [exe, str(session), str(session / 'results' / f'{method}.csv'), '--box', box,
           '--tag-scale', str(args.tag_scale), '--family', args.family]
    if args.tags:
        cmd += ['--tags', args.tags]
    if box == 'yolo':
        cmd += ['--model-dir', str(args.model_dir), '--classes', str(args.classes)]
    if box == 'tag' and args.cylinders:
        cmd += ['--cylinders', args.cylinders]
    subprocess.run(cmd, check=True)


def run_centerpose(session, adapter):
    meta = json.loads((session / 'meta.json').read_text())
    K = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1]])
    lines = []
    for f in csv.DictReader(open(session / 'frames.csv')):
        bgr = cv2.imread(str(session / 'frames' / f"{f['frame']}_color.png"))
        rows, ms = adapter.predict(bgr, K)  # NotImplementedError before any file is written
        lines += [{**r, 'frame': f['frame'], 'stamp_ns': f['stamp_ns']}
                  for r in rows or [dict(id=-1, axis_valid=0, detect_ms=ms)]]
    with open(session / 'results' / 'centerpose.csv', 'w', newline='') as out:
        w = csv.DictWriter(out, COLUMNS)
        w.writeheader()
        w.writerows(lines)
    n, valid = len({r['frame'] for r in lines}), len({r['frame'] for r in lines if r['axis_valid']})
    print(f'{n} frames, {valid} with a CenterPose puck -> {session / "results" / "centerpose.csv"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', type=Path)
    ap.add_argument('--methods', default='tag,depth,centerpose', help='comma list of tag, depth, centerpose')
    ap.add_argument('--depth-box', choices=['yolo', 'apriltag'], default='yolo', help='boxes the depth method uses')
    ap.add_argument('--classes', type=int, default=4, help='YOLO class count of --model-dir')
    ap.add_argument('--cylinders', default='', help='tag ids of cylinder pucks (tag method: end faces only)')
    ap.add_argument('--tags', default='', help='puck tag ids, e.g. 23,24 (empty: every tag)')
    ap.add_argument('--tag-scale', type=float, default=2.0, help='perception.yaml d435.puck_axis.tag_box_scale')
    ap.add_argument('--family', default='tag36h11')
    ap.add_argument('--model-dir', type=Path, default=HERE.parents[1] / 'src/craft_perception/models/pucks_v8n_v2')
    ap.add_argument('--centerpose-repo', type=Path, help='perception-v2 package dir (default: ~/Downloads/...)')
    ap.add_argument('--centerpose-weights', type=Path)
    ap.add_argument('--yolo-weights', type=Path, help='puck YOLO .pt for the CenterPose crop')
    args = ap.parse_args()
    methods = [m for m in args.methods.split(',') if m]
    if set(methods) - {'tag', 'depth', 'centerpose'}:
        ap.error('--methods: tag, depth, centerpose')
    adapter = None
    if 'centerpose' in methods:
        from centerpose_adapter import CenterPoseAdapter as Adapter
        adapter = Adapter(args.centerpose_repo, args.centerpose_weights, args.yolo_weights)
    for session in args.sessions:
        session = session.expanduser()
        (session / 'results').mkdir(exist_ok=True)
        print(f'== {session.name}', flush=True)
        for method in ('tag', 'depth'):
            if method in methods:
                run_offline(session, args, method)
        if adapter:
            try:
                run_centerpose(session, adapter)
            except NotImplementedError as e:
                print(f'centerpose skipped: {e}', file=sys.stderr)
                adapter = None


if __name__ == '__main__':
    main()
