#!/usr/bin/env python3
"""Play grip_yaw_bag.py recordings: colour | depth (colour map) side by side, one yaw after the other, looping.

  python3 grip_yaw_play.py [~/bags/gripyaw_MMDD_HHMM]     (default: the newest gripyaw_* dir)

Keys: space = pause, n / p = next / previous yaw, s = save the shown frame (png) into the bag dir, q / Esc = quit.
Depth is raw (not aligned to colour), shown 0.1 .. 0.4 m.
"""
import glob
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_bag import cdr_compressed_image  # noqa: E402

COL = '/device_0/sensor_1/Color_0/image/data/compressed'
DEP = '/device_0/sensor_0/Depth_0/image/data/compressedDepth'


def load(path):
    db = sqlite3.connect(path)
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    q = lambda n: db.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[n],)).fetchall()
    col, dep = q(COL), q(DEP)
    dts = np.array([t for t, _ in dep])
    frames = []
    for tc, bc in col:
        img = cv2.imdecode(np.frombuffer(cdr_compressed_image(bc)[1], np.uint8), cv2.IMREAD_COLOR)
        j = int(np.argmin(np.abs(dts - tc)))
        d16 = cv2.imdecode(np.frombuffer(cdr_compressed_image(dep[j][1])[1][12:], np.uint8), cv2.IMREAD_UNCHANGED)
        frames.append((img, d16))
    return frames


def view(img, d16, title):
    d = np.clip((d16.astype(np.float32) - 100) / 300, 0, 1)  # 100..400 mm (depth unit 1 mm)
    dc = cv2.applyColorMap((255 - d * 255).astype(np.uint8), cv2.COLORMAP_JET)
    dc[d16 == 0] = 0
    dc = cv2.resize(dc, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    v = np.hstack([img, dc])
    v = cv2.resize(v, (v.shape[1] // 2, v.shape[0] // 2))
    cv2.putText(v, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return v


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.expanduser('~/bags/gripyaw_*')))
    bags = sorted(glob.glob(os.path.join(d, 'yaw_*.db3')))
    if not bags:
        sys.exit(f'no yaw_*.db3 in {d}')
    print(f'{d}: {len(bags)} yaws  (space pause, n/p next/prev, s save, q quit)')
    k, cache = 0, {}
    while True:
        b = bags[k]
        if b not in cache:
            cache[b] = load(b)
        frames = cache[b]
        yaw = int(os.path.basename(b)[4:8]) / 10
        i, pause = 0, False
        while True:
            img, d16 = frames[i]
            v = view(img, d16, f'yaw {yaw:g} deg  frame {i + 1}/{len(frames)}' + ('  PAUSE' if pause else ''))
            cv2.imshow('grip yaw bag', v)
            key = cv2.waitKey(33) & 0xFF
            if key in (ord('q'), 27):
                return
            if key == ord(' '):
                pause = not pause
            if key == ord('s'):
                p = os.path.join(d, f'shot_yaw{int(yaw * 10):04d}_f{i:03d}.png'); cv2.imwrite(p, img); print('saved', p)
            if key == ord('n'):
                k = (k + 1) % len(bags); break
            if key == ord('p'):
                k = (k - 1) % len(bags); break
            if not pause:
                i += 1
                if i == len(frames):  # end of this yaw: on to the next
                    k = (k + 1) % len(bags); break


if __name__ == '__main__':
    main()
