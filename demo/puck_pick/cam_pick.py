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
import math
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
SHELF_FIXED = 0.074  # [m] box top over the floor: hand-measured 72-74 (11-13), gripper cam 74-77 (11-14)
MIN_SCORE = 0.5
# The arm stops short of the upper cam's puck position (2026-11-13: ~15 mm; 2026-11-14 with the jaws' centre known
# (grip_zero.json): closing the jaws pushed a puck 19 mm, the next approach read 24.5 mm short): the planned puck goes
# this much further ahead. The gripper-cam correction takes out what is left. --ahead-offset-mm N overrides it.
AHEAD_OFFSET = 0.022
# Sideways the same way (2026-11-14, user, photo of a prism caught on its edge): the gripper has to be 8 mm further away
# from the LM motor than the upper cam puts it (+puck_x). --side-offset-mm N overrides it.
SIDE_OFFSET = 0.008
PRISM_GRIP_MM = 38    # the prisms that went in at 12:35-12:51 really had ~38 (planned 30 on a shelf read ~8 mm low)
MAX_AHEAD = 0.200     # [m] LiDAR face -> puck centre (user 2026-11-14: was 150 from the motor 2/4 cut-outs at 158-182; plan.py IK still decides)
GRIP_MM = 34          # jaws cover 34 mm from the puck top: what the cylinder that went in at 12:27 really had (planned 20
                      # on a shelf read 14 mm low; 2026-11-14 the shelf became fixed, so the depth is set to the real one)
                      # moved it; was 30, 45, before that plan.py 52). Model: fingertips 40 mm over the box top
OUT = 'plan_cam.yaml'


_TAGS = None
YAW = {}   # id(puck centre array) -> yaw of a prism's faces [rad] (tag_yaw), None: cylinder or no tag seen


def tag_yaw(bgr, K, R, t, z_top, cen):
    """Yaw [rad] of the puck's faces about the vertical, folded into -45..+45 deg (a square looks the same every 90), from
    the tag36h11 on its top: the tag corners are put on the plane z_top by their camera rays and the first edge's
    direction in base_link (x ahead, y left) is taken. None: no tag within 20 mm of the puck centre cen.
    Checked on saved frames 2026-11-14: tag centre 1-3 mm from the detector's puck centre, two shots 0.2 deg apart."""
    global _TAGS
    import cv2
    if _TAGS is None:
        p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)
        # The plain frame first (2026-11-14: its readings of one prism were 0.2 deg apart; enlarged twice the same
        # frames gave 21.5 and nothing), the enlarged one after it for tags the plain frame misses.
        g1 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g1, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        _TAGS = list(det.detectMarkers(g1)[0]) + [c / 2 for c in det.detectMarkers(g)[0]]
    best = None
    for c in _TAGS:
        rays = (np.linalg.inv(K) @ np.c_[c.reshape(4, 2), np.ones(4)].T).T @ R.T
        P = t + rays * ((z_top - t[2]) / rays[:, 2])[:, None]
        sides = [float(np.linalg.norm(P[k] - P[(k + 1) % 4])) for k in range(4)]
        dist = float(np.hypot(*(P.mean(0)[:2] - cen[:2])))
        # The top tag only: on the top plane it is a square (a side-face tag put on that plane is stretched) and sits
        # at the puck centre.
        if dist < 0.020 and max(sides) / min(sides) < 1.25 and (best is None or dist < best[0] - 0.003):
            e = P[1] - P[0]
            best = (dist, float((np.arctan2(e[1], e[0]) + np.pi / 4) % (np.pi / 2) - np.pi / 4))
    return None if best is None else best[1]


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
        yaw = tag_yaw(bgr, K, R, t, z_floor + top, cen) if 'prism' in pl.CLASSES[c] else None
        YAW[id(cen)] = yaw
        out.append((p, pl.CLASSES[c], cen, top))
    return sorted(out, key=lambda o: -o[0]), laser


def main():
    prefixes = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('--') and sys.argv[i - 1] not in ('--count', '--pick', '--ahead-offset-mm', '--side-offset-mm', '--max-ahead-mm')]
    if '--capture' in sys.argv:
        import time
        p = str(RS / 'out' / time.strftime('campick_%H%M%S'))
        subprocess.run([str(RS / 'build' / 'rs_snap'), p, '1280', '720', pl.UPPER_SERIAL], check=True)
        prefixes.append(p)
    if len(prefixes) != 1:
        print(__doc__); return 2
    pucks, laser = puck_centres(prefixes[0])
    pucks = [p for p in pucks if p[0] >= MIN_SCORE]  # 2026-11-14: a 0.28 'red_prism' off the table was taken as the nearest puck
    global MAX_AHEAD
    if '--max-ahead-mm' in sys.argv:  # one run with another reach limit
        MAX_AHEAD = float(sys.argv[sys.argv.index('--max-ahead-mm') + 1]) / 1000
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
    off_a = float(sys.argv[sys.argv.index('--ahead-offset-mm') + 1]) / 1000 if '--ahead-offset-mm' in sys.argv else AHEAD_OFFSET
    off_s = float(sys.argv[sys.argv.index('--side-offset-mm') + 1]) / 1000 if '--side-offset-mm' in sys.argv else SIDE_OFFSET
    if off_a or off_s:
        print(f'  constant offset applied: ahead {off_a * 1000:+.1f} mm, sideways {off_s * 1000:+.1f} mm')
    for i, (score, name, cen, top) in enumerate(pucks):
        if len(outs) == count:
            break
        tag = f'puck {i + 1} ({name})'
        # Constant camera-vs-arm error (pa22 patch 2026-11-14; 11-13: the grasp sat ~15 mm short of the camera x):
        # --ahead-offset-mm N / --side-offset-mm N are ADDED to the planned puck position; default 0.
        ahead = float(cen[0] - face_x) + off_a
        x_off = float(LATERAL_SIGN * (cen[1] - laser[1])) + off_s
        # The reach the arm is sent to, offset included: that is what loads the motors (2026-11-14 11:52: at 163 mm,
        # camera 141 + 22, the supply dipped to 10.6 V and shoulder motor 2 stopped answering, the board cut the torque).
        if ahead > MAX_AHEAD:
            print(f'SKIP {tag}: the arm would reach {ahead * 1000:.0f} mm ahead of the LiDAR face (> {MAX_AHEAD * 1000:.0f}; camera {(ahead - off_a) * 1000:.0f} + {off_a * 1000:.0f}): motors 2 and 4 cut out further out, park closer'); continue
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
        # The box is fixed (2026-11-14, user): its top is a constant. The upper cam's reading of it wandered 59-69 mm
        # for the same box and the gripper cam found the top 5-16 mm higher than read. --measure-shelf uses the reading.
        shelf = top - PUCK_H if '--measure-shelf' in sys.argv else SHELF_FIXED
        print(f'  (upper cam reads the shelf at {(top - PUCK_H) * 1000:.0f} mm)')
        print(f'  shelf under it {shelf * 1000:.0f} mm (puck top {top * 1000:.0f} - {PUCK_H * 1000:.0f}), grip {PRISM_GRIP_MM if "prism" in name else GRIP_MM} mm from the top')
        # A prism is gripped deeper (2026-11-14, user: gripped at 20 mm, it fell out on the way up; a cylinder sits in
        # the jaws' groove and holds at 20). The approach goes up by as much, so the gripper cam looks from the same
        # height for both (its jaw-centre reference holds at that height only).
        grip = PRISM_GRIP_MM if 'prism' in name else GRIP_MM
        # Approach height over the grasp such that the gripper cam stands as far from the top as when its jaw-centre
        # reference was taken (cylinder 12:10: 40 mm over a 34 mm grip, i.e. 6 over the top; prism: 22 over the top).
        # Both shapes: jaw tips 22 mm over the top while the gripper cam looks and corrects (user 2026-11-14: at the old
        # cylinder 40 mm = 6 mm over the top the jaws hit a cylinder's top during the correction and tipped it).
        # The cylinder's grip_zero must be re-taken at this height (execute.py --gripzero).
        above = (grip + 22) / 1000  # prism: 13:20 (50 over 30) and 13:26 (53 over 38, 7 mm low) runs
        # A prism standing turned: the jaws turn to its faces (2026-11-14, user: a turned prism was not picked).
        # URDF tool_roll = -yaw (FK: roll +17.2 deg turns the jaw closing direction -17.2 deg about the vertical).
        yaw = YAW.get(id(cen))
        roll = -yaw if yaw is not None else 0.0
        if 'prism' in name:
            print(f'  prism faces turned {math.degrees(yaw):+.1f} deg (top tag) -> tool_roll {math.degrees(roll):+.1f} deg' if yaw is not None
                  else '  !! prism: no top tag seen, its turn is unknown -> jaws straight (tool_roll 0)')
        r = subprocess.run([sys.executable, str(HERE / 'plan.py'), '--puck-ahead', f'{ahead:.4f}', '--puck-x', f'{x_off:.4f}',
                            '--shelf-height', f'{shelf:.4f}', '--grip-mm', str(grip), '--approach-above', f'{above:.3f}',
                            '--tool-roll', f'{roll:.4f}',
                            '--out', out], capture_output=True, text=True)
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
        picked.append({'plan': out, 'open_deg': open_deg, 'yaw_deg': None if yaw is None else round(math.degrees(yaw), 1), 'class': name, 'score': round(float(score), 2),
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
