#!/usr/bin/env python3
"""Gripper-cam (wrist D435 261622073363) puck check: where the puck is relative to the open jaws, from one aligned
colour + depth frame, before the arm goes down to it.

  grip_locate.py [prefix ...]          saved rs_snap captures
  grip_locate.py --capture             grab one frame first (close the live gripper window, it holds the camera)
  grip_locate.py --set-ref <prefix>    store that frame's puck as the "lined up" reference (grip_ref.json)
  grip_locate.py --capture --set-ref   the same from a fresh frame: arm at the approach pose (bigbro stops there when
                                       the check fails), puck put exactly between the jaws by hand

One reference per shape (cylinder / prism: their boxes sit differently in the image); a shape without its own is not
compared (execute.py stores the frame of its first held grasp). The reference only holds at the height it was taken at (the camera does not look along the tool axis, so a
height change moves the puck in the image): REF_DZ_MM further / nearer than the reference is reported as "REF HEIGHT"
and execute.py stops on it. Re-take the reference whenever cam_pick.py GRIP_MM or plan.py approach_above changes
(2026-11-14: grip 45 -> 20 mm raised the approach 25-31 mm and every puck read 16 mm off).

Output per puck: pixel centre, depth, the centre in the camera optical frame (x right, y down, z ahead) [mm] and,
with a reference, the offset from it [mm]. Turning that offset into an arm correction needs the camera axes in the
arm frame: one LM jog of known size seen in two frames fixes the lateral axis (not done yet).
"""
import json
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import box_range as br  # noqa: E402
import puck_locate as pl  # noqa: E402

GRIP_SERIAL = '261622073363'
REF = os.path.join(HERE, 'grip_ref.json')
REF_DZ_MM = 12.0


def shape(cls):
    return cls.split('_')[-1]


def load_refs():
    """{shape: reference}; the 2026-11-13 file held one bare reference."""
    if not os.path.exists(REF):
        return {}
    r = json.load(open(REF))
    return {shape(r['cls']): r} if 'cls' in r else r


def locate(prefix):
    d, K, ser = br.load(prefix)
    bgr = cv2.imread(prefix + '_color.ppm')
    out = []
    for (x, y, w, h), c, p in pl.detect(bgr):
        patch = d[max(int(y + 0.3 * h), 0):int(y + 0.7 * h), max(int(x + 0.3 * w), 0):int(x + 0.7 * w)]
        if not np.any(patch > 0.05):
            continue
        z = float(np.median(patch[patch > 0.05]))
        u, v = x + w / 2, y + h / 2
        X = np.linalg.inv(K) @ np.array([u, v, 1.0]) * z
        out.append({'cls': pl.CLASSES[c], 'score': p, 'px': [u, v], 'box': [x, y, w, h], 'depth': z, 'cam_mm': (X * 1000).tolist()})
    return sorted(out, key=lambda o: -o['score']), bgr


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--capture' in sys.argv:
        p = os.path.join(HERE, 'out', time.strftime('grip_%H%M%S'))
        subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), p, '640', '360', GRIP_SERIAL], check=True)
        args.append(p)
    refs = load_refs()
    for prefix in args:
        found, bgr = locate(prefix)
        if not found:
            print(f'{prefix}: no puck in the gripper cam'); continue
        for o in found:
            cx, cy, cz = o['cam_mm']
            line = (f"{prefix}: {o['cls']} {o['score']:.2f} px ({o['px'][0]:.0f},{o['px'][1]:.0f}) depth {o['depth'] * 1000:.0f} mm"
                    f" -> camera x {cx:+.0f} y {cy:+.0f} z {cz:.0f} mm")
            ref = refs.get(shape(o['cls']))  # no reference of this shape: no comparison (execute.py learns it from a grasp)
            if ref and '--set-ref' not in sys.argv:
                d = np.array(o['cam_mm']) - np.array(ref['cam_mm'])
                if abs(d[2]) > REF_DZ_MM:
                    line += f"  | REF HEIGHT: {d[2]:+.0f} mm from the {shape(ref['cls'])} reference's distance, x / y not comparable"
                else:
                    line += f"  | vs lined-up ref ({shape(ref['cls'])}): dx {d[0]:+.0f} dy {d[1]:+.0f} dz {d[2]:+.0f} mm"
            print(line)
            x, y, w, h = map(int, o['box'])
            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 1)
        if ref:
            rx, ry = map(int, ref['px'])
            cv2.drawMarker(bgr, (rx, ry), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.imwrite(prefix + '_grip.png', bgr)
        if '--set-ref' in sys.argv:
            refs[shape(found[0]['cls'])] = found[0]
            json.dump(refs, open(REF, 'w'), indent=1)
            print(f"reference stored for {shape(found[0]['cls'])}: {REF} (now: {', '.join(refs)})")


if __name__ == '__main__':
    main()
