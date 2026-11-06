#!/usr/bin/python3
"""Run PuckAxis (puck_axis_offline, production C++) and CenterPose on the same record.py sessions.

Writes <session>/results/{ours.csv, centerpose.csv}; analyze.py turns them into tables.
Methods run one after another so the latency numbers do not fight over the CPU/GPU.

Usage: source ~/craft/install/setup.bash
       /usr/bin/python3 compare.py ~/bags/puck_compare/stand_a_*            # both methods
       /usr/bin/python3 compare.py SESSIONS --only ours --box yolo          # puck_blue box instead of tags
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

from centerpose_adapter import CenterPoseAdapter

HERE = Path(__file__).resolve().parent
COLUMNS = ['frame', 'stamp_ns', 'id', 'score', 'axis_valid', 'cx', 'cy', 'cz', 'ax', 'ay', 'az', 'detect_ms']


def run_ours(session, args):
    exe = shutil.which('puck_axis_offline') or str(
        Path('~/craft/install/craft_perception/lib/craft_perception/puck_axis_offline').expanduser())
    cmd = [exe, str(session), str(session / 'results' / 'ours.csv'), '--box', args.box,
           '--tag-scale', str(args.tag_scale), '--family', args.family]
    if args.tags:
        cmd += ['--tags', args.tags]
    if args.box == 'yolo':
        cmd += ['--model-dir', str(args.model_dir)]
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
    ap.add_argument('--only', choices=['ours', 'centerpose'])
    ap.add_argument('--box', choices=['apriltag', 'yolo'], default='apriltag')
    ap.add_argument('--tags', default='', help='puck tag ids, e.g. 23,24 (empty: every tag)')
    ap.add_argument('--tag-scale', type=float, default=2.0, help='perception.yaml d435.puck_axis.tag_box_scale')
    ap.add_argument('--family', default='tag36h11')
    ap.add_argument('--model-dir', type=Path, default=HERE.parents[1] / 'src/craft_perception/models/puck_blue')
    ap.add_argument('--centerpose-repo', type=Path)
    ap.add_argument('--centerpose-weights', type=Path)
    args = ap.parse_args()
    adapter = None
    if args.only != 'ours':
        adapter = CenterPoseAdapter(args.centerpose_repo, args.centerpose_weights)
    for session in args.sessions:
        session = session.expanduser()
        (session / 'results').mkdir(exist_ok=True)
        print(f'== {session.name}', flush=True)
        if args.only != 'centerpose':
            run_ours(session, args)
        if adapter:
            try:
                run_centerpose(session, adapter)
            except NotImplementedError as e:
                print(f'centerpose skipped: {e}', file=sys.stderr)
                adapter = None


if __name__ == '__main__':
    main()
