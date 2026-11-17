#!/usr/bin/env python3
"""Throwaway demo, dice step 2: where on the box top can a puck taken out of the clock be put down (URDF FK/IK).

The box (BOX) stands BOX_GAP ahead of the LiDAR front face, centred on the LiDAR. A puck centre is placeable when
  - the puck sits on the box top with BOX_MARGIN to spare,
  - the put-down line, from approach_above (plan.py) over the box down to the puck bottom touching the box top,
    has an IK solution inside the URDF limits (plan.limit_margin_deg) and the motor limits (control.yaml) every
    centimetre, the LM with LM_BACKLASH added.
The puck is held GRASP_DEPTH deep (fingertips that far below its top), as taken out of the clock: it stands out
of the slot 35 mm and the jaws close 28 mm down (2026-11-06). The TCP is the URDF tool_tcp (middle of the 54 mm
usable grip, 27 mm behind the fingertip) with plan.hw_z_correction. The tool stays vertical where it can,
otherwise leans its tip away from the robot by the smallest whole degree that fits (map: # vertical, + leaned).
Not checked yet: the swing from the carry pose (after taking the puck out, step 1) to the approach, the arm
against the robot body. --dice also fits the 3x3 dice grids (step 3).
Usage: python3 reach.py [--box-x MM] [--box-ahead MM] [--dice]   (box centre: LiDAR centre offset, face -> centre)
"""
import math
import sys

import numpy as np
import yaml

import execute as ex
import plan

A = plan.ASSUME
BOX = (0.205, 0.205, 0.07)  # x (left-right), y (front-back), height [m]
BOX_GAP = 0.01              # LiDAR front face -> box near edge [m]
BOX_MARGIN = 0.005          # puck edge inside the box edge [m]
PUCK_R = A['puck_size'][0] / 2
GRASP_DEPTH = 0.028         # fingertips below the puck top [m]
A['grasp_coverage'] = GRASP_DEPTH / A['puck_height']
A['shelf_height'] = BOX[2]
# Jaws around a held puck, from the jaw meshes (tool_roll 0 closes them front-back): the part beside the puck
# (28 mm from the fingertip) is <= 17.5 mm half wide across and up to 30 mm thick past the puck surface along the
# closing direction. Opening RELEASE_OPEN per jaw to let go. MEASURE on the robot before trusting d_min.
FINGER_HALF_WIDTH = 0.0175
FINGER_THICK = 0.030
RELEASE_OPEN = 0.004
CLEAR = 0.003               # jaw to another puck [m]
CELL = 0.005
# Picture rows top -> bottom = far -> near the robot, columns left -> right = robot left -> right (looking from
# behind the robot). (row, col) in -1..1.
DICE = {1: [(0, 0)],
        2: [(-1, -1), (1, 1)],
        3: [(-1, -1), (0, 0), (1, 1)],
        4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
        5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)],
        6: [(-1, -1), (0, -1), (1, -1), (-1, 1), (0, 1), (1, 1)]}
ROLLS = (0.0, -math.pi / 2)  # tool_roll at the put-down: jaws close front-back / left-right


def box_center(argv):
    """(x offset from the LiDAR centre, LiDAR face -> box centre) [m], --box-x/--box-ahead in mm override."""
    x, ahead = 0.0, BOX_GAP + BOX[1] / 2
    if '--box-x' in argv:
        x = float(argv[argv.index('--box-x') + 1]) / 1000
    if '--box-ahead' in argv:
        ahead = float(argv[argv.index('--box-ahead') + 1]) / 1000
    return x, ahead


def spot(x, ahead, box=(0.0, BOX_GAP + BOX[1] / 2), ch=None, ctrl=None):
    """Put-down check of a puck centre x [m] along +X from the LiDAR centre, ahead [m] of the LiDAR face.
    Returns (ok, why, largest tool lean on the put-down line [deg])."""
    ch = ch or plan.Chain(plan.URDF)
    ctrl = ctrl or yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    room = np.array(BOX[:2]) / 2 - PUCK_R - BOX_MARGIN
    if abs(x - box[0]) > room[0] or abs(ahead - box[1]) > room[1]:
        return False, 'off the box', None
    grasp = plan.grasp_at(x, ahead)
    lo, hi = ch.limits('slide_joint')
    tilt = 0.0
    for h in np.r_[np.arange(A['approach_above'], 0.0, -0.01), 0.0]:
        q = plan.ik_tilted(ch, grasp + [0, 0, h])
        if q and not lo <= q[0]['slide_joint'] <= hi:
            return False, 'LM range', None
        if not q or not plan.inside(ch, q[0]):
            return False, f'no IK {h * 100:.0f} cm up', None
        q = q[0]
        if plan.motor_bad(q, ctrl):
            return False, f'motor limit {plan.motor_bad(q, ctrl)}', None
        if not lo <= q['slide_joint'] + ex.LM_BACKLASH <= hi:
            return False, 'LM end', None
        z = ch.fk(q)[:3, 3]
        if np.linalg.norm(z - (grasp + [0, 0, h])) > 1e-4:
            return False, 'FK miss', None
        tool = ch.fk(q)[:3, 1]  # tool axis (tool_tcp y) points base -> tip
        tilt = max(tilt, round(math.degrees(math.acos(max(-1.0, min(1.0, -tool[2])))), 1))
    return True, '', tilt


def jaws_clear(pips, roll):
    """Every puck's jaws (tool_roll roll, opened RELEASE_OPEN) stay CLEAR off every other puck of pips [(x, ahead)]."""
    along = PUCK_R + FINGER_THICK + RELEASE_OPEN  # closing direction
    across = FINGER_HALF_WIDTH
    hx, hy = (across, along) if roll == 0.0 else (along, across)
    for i, (x0, y0) in enumerate(pips):
        for j, (x1, y1) in enumerate(pips):
            if i != j and math.hypot(max(abs(x1 - x0) - hx, 0), max(abs(y1 - y0) - hy, 0)) < PUCK_R + CLEAR:
                return False
    return True


def dice_spots(face, cx, cy, d):
    """Puck centres (x, ahead) of dice face around grid centre (cx, cy) with spacing d."""
    return [(cx - c * d, cy - r * d) for r, c in DICE[face]]


def face_roll(face, d):
    """The first tool_roll in ROLLS whose jaws clear the other pucks of the face, None if none."""
    return next((r for r in ROLLS if jaws_clear(dice_spots(face, 0, 0, d), r)), None)


def main():
    ch = plan.Chain(plan.URDF)
    ctrl = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    box = box_center(sys.argv)
    xs = np.round(np.arange(-0.11, 0.1101, CELL), 3)
    ys = np.round(np.arange(0.0, 0.2301, CELL), 3)
    ok, on, tilts, why = {}, {}, {}, {}
    for y in ys:
        for x in xs:
            good, reason, tilt = spot(x, y, box, ch, ctrl)
            ok[x, y] = good
            on[x, y] = reason != 'off the box'
            if good:
                tilts[x, y] = tilt
            why[reason] = why.get(reason, 0) + 1
    print(f'box {BOX[0] * 1000:.0f} x {BOX[1] * 1000:.0f} x {BOX[2] * 1000:.0f} mm, centre {box[0] * 1000:+.0f} mm '
          f'(LiDAR centre, + = robot left / away from the LM motor), {box[1] * 1000:.0f} mm ahead of the LiDAR face')
    print(f'puck held {GRASP_DEPTH * 1000:.0f} mm deep; put-down line {A["approach_above"] * 100:.0f} cm above -> '
          'puck bottom on the box top')
    print('placeable puck centres (# vertical tool, + tool leaned, . on the box but not reachable, blank off the box);'
          ' rows far -> near, cols left -> right')
    print('ahead  ' + ''.join('|' if round(x * 1000) % 50 == 0 else ' ' for x in xs[::-1]))
    for y in ys[::-1]:
        print(f'{y * 1000:4.0f}   ' + ''.join(('#' if tilts[x, y] == 0 else '+') if (x, y) in tilts
                                              else '.' if on[x, y] else ' ' for x in xs[::-1]))
    print('x mm  ' + ''.join(f'{x * 1000:+.0f}'.ljust(10) for x in xs[::-1][::10]))
    good = list(tilts)
    if not good:
        print('nothing placeable'); return 1
    gx, gy = [k[0] for k in good], [k[1] for k in good]
    print(f'placeable: x {min(gx) * 1000:+.0f}..{max(gx) * 1000:+.0f} mm, ahead {min(gy) * 1000:.0f}..'
          f'{max(gy) * 1000:.0f} mm ({len(good)} cells); rejected by: '
          + ', '.join(f'{k} {v}' for k, v in why.items() if k))
    vert = [k for k, v in tilts.items() if v == 0]
    if vert:
        print(f'vertical tool: x {min(k[0] for k in vert) * 1000:+.0f}..{max(k[0] for k in vert) * 1000:+.0f} mm, '
              f'ahead {min(k[1] for k in vert) * 1000:.0f}..{max(k[1] for k in vert) * 1000:.0f} mm ({len(vert)} cells)')
    for y in sorted({k[1] for k in good}):
        row = sorted(k[0] for k in good if k[1] == y)
        lean = [tilts[x, y] for x in row]
        print(f'  ahead {y * 1000:3.0f}: x {row[0] * 1000:+4.0f}..{row[-1] * 1000:+4.0f} mm, lean {min(lean):.0f}..{max(lean):.0f} deg')
    if '--dice' not in sys.argv:
        return 0
    ok = {k: k in tilts for k in ok}
    print('\ndice grid (3x3 spacing d, centre on the 5 mm cells; jaws per face: roll 0 front-back, -90 left-right)')
    for d in np.arange(0.030, 0.0601, 0.005):
        rolls = {f: face_roll(f, d) for f in DICE}
        fits = [(cx, cy) for cx in xs for cy in ys
                if all(ok.get((round(px, 3), round(py, 3)), False) for r in (-1, 0, 1) for c in (-1, 0, 1)
                       for px, py in [(cx - c * d, cy - r * d)])]
        no = [f for f, r in rolls.items() if r is None]
        line = f'  d {d * 1000:.0f} mm: '
        if not fits:
            print(line + 'no grid fits'); continue
        cx = float(np.median([f[0] for f in fits])); cy = float(np.median([f[1] for f in fits if f[0] == cx] or [fits[0][1]]))
        print(line + f'{len(fits)} centres, middle x {cx * 1000:+.0f} ahead {cy * 1000:.0f} mm; '
              + ('jaws hit on faces ' + str(no) if no else 'all faces clear') + '  rolls '
              + ' '.join(f'{f}:{"-" if r is None else round(math.degrees(r))}' for f, r in rolls.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
