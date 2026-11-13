#!/usr/bin/env python3
"""Upper-cam puck -> plan.py IK -> sis1 steps (plan_cam.yaml). Software path only; it moves nothing.

  cam_pick.py --capture            grab one upper-cam frame (tools/realsense_check/puck_locate.py) and plan
  cam_pick.py <rs_snap prefix>     plan from a saved capture
  --count N                        plan the N nearest pucks from the one frame: plan_cam.yaml, plan_cam2.yaml, ...
                                   (bigbro2: one shot at the nominal pose, the arm then never has to clear the camera)
Then:  puckload --plan plan_cam.yaml [--dry-run]

Camera centre (V307 base_link: x ahead, y left) -> plan.py inputs:
  puck_ahead    = x - (front_laser x + LiDAR face offset)     [LiDAR front face -> puck centre, straight ahead]
  puck_x_offset = LATERAL_SIGN * (y - front_laser y)         [along the LM axis, + = away from the LM motor]
LATERAL_SIGN +1 was checked on the robot on 2026-11-13 (a puck 41 mm right of the centre, grasped; the gripper cam
showed it between the jaws). --pick nearest|best chooses the puck; a second puck closer than MIN_GAP stops it.
"""
import os
import subprocess

import numpy as np
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RS = HERE.parent.parent / 'tools' / 'realsense_check'
sys.path.insert(0, str(RS))
import box_range as br  # noqa: E402
import puck_locate as pl  # noqa: E402

LATERAL_SIGN = +1.0   # verified 2026-11-13: two grasps, the gripper cam showed the puck between the jaws
MIN_GAP = 0.060       # [m] centre to centre: closer pucks are not picked (2026-11-14, user: 60, open the gripper less; was
                      # 0.070, 0.080). From the model, not measured: jaw inner face from the puck centre + ~10 mm jaw
                      # thickness (assumed) + 15 mm the other puck's half width + 10 mm the gripper-cam check lets through.
                      # Full open (94 deg, inner face 34 mm) needs 69 -> FULL_OPEN_GAP; nearer than that the jaws open
                      # only to NARROW_OPEN_DEG (inner face 25 mm: 10 mm free each side of the 30 mm puck) -> 60.
FULL_OPEN_GAP = 0.070
# Gripper motor angle for a 50 mm jaw gap: 94 deg = 68.4 mm (URDF full open), 168.4 deg = 30 mm (reading with a puck
# held, 2026-11-06), linear between (0.516 mm/deg). Not checked with a ruler.
NARROW_OPEN_DEG = 130.0
PUCK_H = 0.060
MAX_AHEAD = 0.150     # [m] LiDAR face -> puck centre: further out the wrist motor 4 cut out (2026-11-13, 182 mm)
GRIP_MM = 20          # jaws cover 20 mm from the puck top (2026-11-14, user: still too deep, the jaws hit the box and
                      # moved it; was 30, 45, before that plan.py 52). Model: fingertips 40 mm over the box top
OUT = 'plan_cam.yaml'


def puck_centres(prefix):
    import cv2
    import numpy as np
    d, K, _ = br.load(prefix)
    bgr = cv2.imread(prefix + '_color.ppm')
    R, t, z_floor, laser = pl.camera_pose()
    out = []
    for (x, y, w, h), c, p in pl.detect(bgr):
        patch = d[max(int(y + 0.3 * h), 0):int(y + 0.7 * h), max(int(x + 0.3 * w), 0):int(x + 0.7 * w)]
        if not np.any(patch > 0.1):
            continue
        z = float(np.median(patch[patch > 0.1]))
        surf = (np.linalg.inv(K) @ np.array([x + w / 2, y + h / 2, 1.0]) * z) @ R.T + t
        hz = surf - t; hz[2] = 0; hz /= np.linalg.norm(hz) + 1e-9
        # puck top: 95th percentile height of the box's depth points (the upper cam looks down at 43.6 deg)
        x0, y0, x1, y1 = max(int(x), 0), max(int(y), 0), int(x + w), int(y + h)
        vv, uu = np.mgrid[y0:y1, x0:x1]; zz = d[y0:y1, x0:x1]; ok = zz > 0.1
        B = ((np.stack([uu[ok], vv[ok], np.ones(ok.sum())], 1) @ np.linalg.inv(K).T) * zz[ok][:, None]) @ R.T + t
        cen = surf + hz * pl.PUCK_RADIUS
        # Only the points within 22 mm of the puck axis: the box also holds whatever stands in front of or over the
        # puck in the image (2026-11-14: the gripper over a puck near the centre line gave a 337 mm top).
        own = np.hypot(B[:, 0] - cen[0], B[:, 1] - cen[1]) < 0.022
        if own.sum() < 30:
            continue
        top = float(np.percentile(B[own, 2], 95)) - z_floor
        out.append((p, pl.CLASSES[c], cen, top))
    return sorted(out, key=lambda o: -o[0]), laser


def main():
    prefixes = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('--') and sys.argv[i - 1] not in ('--count', '--pick')]
    if '--capture' in sys.argv:
        import time
        p = str(RS / 'out' / time.strftime('campick_%H%M%S'))
        subprocess.run([str(RS / 'build' / 'rs_snap'), p, '1280', '720', pl.UPPER_SERIAL], check=True)
        prefixes.append(p)
    if len(prefixes) != 1:
        print(__doc__); return 2
    pucks, laser = puck_centres(prefixes[0])
    if not pucks:
        print('no puck detected'); return 1
    # Which puck: --pick nearest (to the LiDAR face, default) | best (detector score). The rest stay put.
    mode = sys.argv[sys.argv.index('--pick') + 1] if '--pick' in sys.argv else 'nearest'
    if mode == 'nearest':
        pucks = sorted(pucks, key=lambda o: o[2][0])
    for i, (sc, nm, c, tp) in enumerate(pucks):
        print(f'  puck {i + 1}: {nm} {sc:.2f} at x {c[0] * 1000:.0f} y {c[1] * 1000:+.0f} mm, top {tp * 1000:.0f} mm' + ('  <- picked' if i == 0 else ''))
    # --count N: up to N pucks, nearest first, only the ones that are sure (2026-11-14, user: pick what can be picked,
    # the robot is re-parked for the rest). A puck is skipped, with the reason, when it stands beyond MAX_AHEAD, when
    # another puck still on the box is within MIN_GAP of it (pucks planned before it are gone by then), or when
    # plan.py finds no pose inside the limits (LM rail: about -60 .. +37 mm sideways).
    count = int(sys.argv[sys.argv.index('--count') + 1]) if '--count' in sys.argv else 1
    face_x = laser[0] + br.LIDAR_FACE_TO_CENTRE
    outs, picked, gone = [], [], set()
    for i, (score, name, cen, top) in enumerate(pucks):
        if len(outs) == count:
            break
        tag = f'puck {i + 1} ({name})'
        ahead = float(cen[0] - face_x)
        x_off = float(LATERAL_SIGN * (cen[1] - laser[1]))
        if ahead > MAX_AHEAD:
            print(f'SKIP {tag}: {ahead * 1000:.0f} mm ahead of the LiDAR face (> {MAX_AHEAD * 1000:.0f}: wrist motor 4 cut out at 182): park closer'); continue
        near = [float(np.hypot(*(c[:2] - cen[:2]))) for j, (_, _, c, _) in enumerate(pucks) if j != i and j not in gone]
        if near and min(near) < MIN_GAP:
            print(f'SKIP {tag}: another puck is {min(near) * 1000:.0f} mm from it (< {MIN_GAP * 1000:.0f} mm, the open jaws may hit it)'); continue
        if top - PUCK_H < 0.03 or top - PUCK_H > 0.12:
            print(f'SKIP {tag}: its top reads {top * 1000:.0f} mm (shelf {1000 * (top - PUCK_H):.0f} mm, expected 30-120): something over it (the arm?) or a bad depth'); continue
        open_deg = NARROW_OPEN_DEG if near and min(near) < FULL_OPEN_GAP else None
        if open_deg:
            print(f'  another puck {min(near) * 1000:.0f} mm away: the jaws open only to {open_deg:g} deg (50 mm) for this one')
        out = OUT if not outs else OUT.replace('.yaml', f'{len(outs) + 1}.yaml')
        print(f'{tag} {score:.2f}: base_link x {cen[0] * 1000:.0f} y {cen[1] * 1000:+.0f} mm'
              f' -> puck_ahead {ahead * 1000:.0f} mm, puck_x_offset {x_off * 1000:+.0f} mm (LATERAL_SIGN {LATERAL_SIGN:+.0f})')
        shelf = top - PUCK_H
        print(f'  shelf under it {shelf * 1000:.0f} mm (puck top {top * 1000:.0f} - {PUCK_H * 1000:.0f}), grip {GRIP_MM} mm from the top')
        r = subprocess.run([sys.executable, str(HERE / 'plan.py'), '--puck-ahead', f'{ahead:.4f}', '--puck-x', f'{x_off:.4f}',
                            '--shelf-height', f'{shelf:.4f}', '--grip-mm', str(GRIP_MM), '--out', out], capture_output=True, text=True)
        if r.returncode != 0:
            if "'LM'" in r.stdout:  # the LM rail carries the arm sideways: about -60 .. +37 mm of puck_x_offset
                side, lim = ('left', 37) if x_off > 0 else ('right', 60)
                print(f'SKIP {tag}: {abs(x_off) * 1000:.0f} mm to the {side} of the LiDAR centre, the LM rail reaches {lim} mm: '
                      f'{abs(x_off) * 1000 - lim + 5:.0f} mm closer to the centre needed')
            else:
                print(f'SKIP {tag}: no arm pose inside the joint limits')
            continue
        print(f'  {out} written; all waypoints within limits')
        outs.append(out); gone.add(i)
        picked.append({'plan': out, 'open_deg': open_deg, 'class': name, 'score': round(float(score), 2),
                       'x_mm': round(float(cen[0]) * 1000), 'y_mm': round(float(cen[1]) * 1000)})
    # What each plan's puck is (colour + shape from the detector): execute.py writes it into carousel.yaml per slot.
    import json
    json.dump(picked, open(HERE / 'plan_cam_pucks.json', 'w'), indent=1)
    if not outs:
        print('STOP: no puck here can be picked for sure: re-park'); return 3
    print(f'{len(outs)} of {len(pucks)} puck(s) planned')
    print(f'next: bigbro --plan {",".join(outs)} [--slot N[,N]]')
    return 0

if __name__ == '__main__':
    sys.exit(main())
