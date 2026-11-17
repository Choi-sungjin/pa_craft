#!/usr/bin/env python3
"""Upper-cam (어퍼캠) shots of one puck at several angles, Enter per angle (2026-11-15, lying-puck data).
Arm at the nominal pose (it must not hide the box). Per angle: upper_puck (C++) saves colour + depth and prints what
it detects; everything goes to ~/bags/upperang_MMDD_HHMM/ with meta.json (angle, detections).

  python3 upper_angle_shots.py --tag lying --cell 2 [--yaws 0,30,60,90,120,150]
Keys: Enter = shoot, s = skip, r = back one angle, q = quit.
"""
import argparse
import json
import os
import re
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
UPPER = os.path.join(HERE, 'build', 'upper_puck')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaws', default='0,30,60,90,120,150')
    ap.add_argument('--tag', default='')
    ap.add_argument('--cell', type=int, default=0)
    ap.add_argument('--out', default=os.path.expanduser(time.strftime('~/bags/upperang_%m%d_%H%M')))
    a = ap.parse_args()
    yaws = [float(y) for y in a.yaws.split(',')]
    os.makedirs(a.out, exist_ok=True)
    meta = dict(recorded=time.strftime('%Y-%m-%d %H:%M:%S'), tag=a.tag, cell=a.cell, shots=[])
    k = 0
    while k < len(yaws):
        y = yaws[k]
        ans = input(f'\n[{k + 1}/{len(yaws)}] 퍽을 {y:g} deg로 놓고 Enter (s 건너뛰기, r 이전 다시, q 끝): ').strip().lower()
        if ans == 'q':
            break
        if ans == 's':
            k += 1; continue
        if ans == 'r' and k > 0:
            k -= 1; continue
        name = f'yaw_{int(round(y * 10)):04d}'
        r = subprocess.run([UPPER, '--save', os.path.join(a.out, name)], capture_output=True, text=True, timeout=60)
        dets = []
        for l in r.stdout.splitlines():
            print('  ' + l)
            m = re.match(r'PUCK (\S+) ([\d.]+) ahead ([-\d.]+) lateral ([-+\d.]+) top ([-\d.]+)', l)
            if m:
                dets.append(dict(cls=m.group(1), score=float(m.group(2)), ahead=float(m.group(3)),
                                 lateral=float(m.group(4)), top=float(m.group(5))))
        meta['shots'] = [s for s in meta['shots'] if s['yaw_deg'] != y] + [
            dict(yaw_deg=y, color=name + '_color.png', depth=name + '_depth_mm.png', detections=dets)]
        json.dump(meta, open(os.path.join(a.out, 'meta.json'), 'w'), indent=1)
        k += 1
    print(f'\n{len(meta["shots"])} angles -> {a.out}')


if __name__ == '__main__':
    main()
