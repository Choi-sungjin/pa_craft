#!/usr/bin/env python3
"""Throwaway demo, dice step 3: take the stored pucks out of the clock one by one and stand them on the box as a
dice face (execute.py --dice N). This file builds the steps and checks them in URDF FK; execute.py runs them.

Layout (2026-11-06): box 205 x 205 x 70 mm, BOX_GAP ahead of the LiDAR face, centred on the LM centre (BOX_X);
pips on the 3x3 grid PIP mm apart around the box centre (box quarters are 51.25: the LM does not reach them),
reach.DICE picture order (top row = far). Put down in column order from the robot's left, far to near in each
column, so the held puck never passes over a placed one; jaws close left-right (PUT_ROLL) so they miss the
neighbours in the column.

Per puck: clock to the slot, gripper part open, swing back over the slot (UNLOAD_ABOVE over the grasp), straight
down, close, straight up LIFT (tool angle kept), swing to CARRY_ABOVE over the spot (puck bottom over the box top;
placed pucks are 60 mm), LM to the spot line, down the vertical line to RELEASE_ABOVE, open, back up.
check() samples every held move (joint-linear) and reports the held puck and jaws against the box and the placed
pucks.
Usage: python3 dice.py [N] [--slots 1,2,...]   (steps + check, no robot)
"""
import math
import sys

import numpy as np
import yaml

import execute as ex
import plan
import reach
import teach

M = ex.TaskCommand.MANIPULATOR
A = plan.ASSUME
BOX_X = -0.0125        # box centre: LM centre, 12.5 mm toward the LM motor from the LiDAR centre [m]
BOX_AHEAD = reach.BOX_GAP + reach.BOX[1] / 2
PIP = 0.0475           # pip spacing [m]; quarters of the box (0.05125) are 1-3 mm past the LM ends
PUT_ROLL = -math.pi / 2  # tool_roll over the box: jaws close left-right
CARRY_ABOVE = 0.075    # puck bottom over the box top while moving over the box [m]
RELEASE_ABOVE = 0.005  # puck bottom over the box top when the jaws open [m]
DOWN_STEP = 0.01       # vertical line in pieces this tall [m]
LIFT = 0.03            # straight up out of the slot (2.68 cm clears the slot when the wrist turns) [m]
CLEAR = 0.005          # held puck / jaws to a placed puck or the box top in check() [m]
# Clock grasp, taught by hand after the 2026-11-06 reassembly: raw motor degrees LM, shoulder, elbow, wrist_pitch,
# tool_roll, gripper with the jaws closed on a stored puck 28 mm down from its top (it stands 35 mm out of the slot)
# at the loading guide. None: not taught yet; the dry run uses the old loading pose 3 cm down and the real run stops.
CLOCK_GRASP_RAW_DEG = None
DEFAULT_SLOTS = [1, 2, 3, 4, 5, 6]


def spots(face):
    """Puck centres (x from the LiDAR centre, ahead of the LiDAR face) [m] in put-down order."""
    return sorted(reach.dice_spots(face, BOX_X, BOX_AHEAD, PIP), key=lambda p: (-round(p[0], 4), -p[1]))


def clock_grasp():
    """(slide, [shoulder, elbow, wrist_pitch, tool_roll], taught) at the clock grasp."""
    if CLOCK_GRASP_RAW_DEG is not None:
        t = teach.raw_to_target(CLOCK_GRASP_RAW_DEG)
        return t[0], t[1:5], True
    return ex.nominal_pose()[0], ex.below(ex.load_pose(), ex.UNLOAD_DEPTH), False


def line(ch, x, ahead, heights):
    """Poses [slide, shoulder, elbow, wrist_pitch, PUT_ROLL] on the vertical line over spot (x, ahead), puck bottom
    heights over the box top; the tool leans the least that fits (plan.ik_tilted)."""
    out = []
    for h in heights:
        q = plan.ik_tilted(ch, plan.grasp_at(x, ahead) + [0, 0, h])
        if not q or not plan.inside(ch, q[0]):
            raise RuntimeError(f'no IK {h * 1000:.0f} mm over spot {x * 1000:+.1f}, {ahead * 1000:.1f} mm')
        out.append([q[0]['slide_joint'] + ex.LM_BACKLASH] + [q[0][n] for n in ex.ARM[1:4]] + [PUT_ROLL])
    return out


def steps(face, slots, current_slide=None, start=None):
    """execute.py steps for dice face `face`, puck k from clock slot slots[k]."""
    A['grasp_coverage'] = reach.GRASP_DEPTH / A['puck_height']
    A['shelf_height'] = reach.BOX[2]
    ch = plan.Chain(plan.URDF)
    slide, grasp, _ = clock_grasp()
    high = ex.above(grasp, ex.UNLOAD_ABOVE)
    lift = ex.below(grasp, -LIFT)
    release = math.radians(ex.RELEASE_GRIPPER_DEG)
    downs = np.r_[np.arange(CARRY_ABOVE - DOWN_STEP, RELEASE_ABOVE, -DOWN_STEP), RELEASE_ABOVE]
    s = []
    if current_slide is not None:
        s.append(('LM to the clock line, arm stays', M, [slide] + ex.via_pose()[1:5]))
    for k, ((x, ahead), slot) in enumerate(zip(spots(face), slots), 1):
        carry = line(ch, x, ahead, [CARRY_ABOVE])[0]
        down = line(ch, x, ahead, downs)
        tag = f'puck {k}/{len(slots)}'
        if k > 1:
            s.append((f'{tag}: LM to the clock line, arm stays', M, [slide] + prev_carry[1:]))
        s.append((f'{tag}: clock to slot {slot} (loading guide)', ex.TaskCommand.CLOCK, slot))
        s.append((f'{tag}: open gripper to {ex.RELEASE_GRIPPER_DEG:g} deg', ex.TaskCommand.GRIPPER, release))
        s.append((f'{tag}: swing back over the slot, {ex.UNLOAD_ABOVE * 100:.0f} cm above the grasp, LM stays', M,
                  [slide] + high))
        s.append((f'straight down to the grasp ({tag}), LM stays', M, [slide] + grasp))
        s.append((f'{tag}: close gripper (grasp the puck in the clock)', ex.TaskCommand.GRIPPER, True))
        s.append((f'straight up {LIFT * 100:.0f} cm out of the slot ({tag}), tool angle kept, LM stays', M,
                  [slide] + lift))
        s.append((f'{tag}: swing to the front, {CARRY_ABOVE * 1000:.0f} mm over the box, jaws left-right, LM stays',
                  M, [slide] + carry[1:]))
        s.append((f'{tag}: LM to the spot line ({(x - BOX_X) * 1000:+.1f} mm from the box centre)', M, carry))
        s.append((f'put down ({tag}) at {ahead * 1000:.1f} mm ahead, to {RELEASE_ABOVE * 1000:.0f} mm over the box',
                  M, down))
        s.append((f'{tag}: open gripper to {ex.RELEASE_GRIPPER_DEG:g} deg (release)', ex.TaskCommand.GRIPPER, release))
        s.append((f'rise above the puck ({tag})', M, down[-2::-1] + [carry]))
        prev_carry = carry
    if start is not None:
        s.append(('LM back to the start position', M, [start[0]] + prev_carry[1:]))
        s.append(('arm back to the start pose', M, start))
    return s


def check(face, slots, staged):
    """Held puck (from its top GRASP_DEPTH above the fingertips to its bottom) and jaws along every move between
    the grasp and the release, joint-linear: worst clearance to the box top and to every placed puck."""
    ch = plan.Chain(plan.URDF)
    top_z = A['floor_z'] + reach.BOX[2]
    lift_z = -A['hw_z_correction']  # the real TCP sits this much above the model
    pr = reach.PUCK_R
    placed, held, worst, rows = [], False, {}, []
    prev = None
    centres = spots(face)
    for name, kind, arg in staged:
        if kind == ex.TaskCommand.GRIPPER:
            if arg is True:
                held = True
            elif held:
                held = False
                placed.append(centres[len(placed)])
            continue
        if kind != M:
            continue
        for q in arg:
            if prev is not None and held:
                for f in np.linspace(0, 1, 21):
                    p = [a + (b - a) * f for a, b in zip(prev, q)]
                    T = ch.fk(dict(zip(ex.ARM, p[:5])))
                    d, tcp = T[:3, 1], T[:3, 3] + [0, 0, lift_z]
                    tip = tcp + A['finger_beyond_tcp'] * d
                    side = T[:3, 2]  # jaw closing axis (jaw joints move along tool_roll_link z)
                    pts = [tip + (u - reach.GRASP_DEPTH) * d for u in np.linspace(0, A['puck_height'], 7)]
                    pts = [(c, pr) for c in pts]
                    for sgn in (-1, 1):  # jaw tips beside the puck, half a jaw out
                        pts.append((tip + sgn * (pr + reach.RELEASE_OPEN + reach.FINGER_THICK / 2) * side, 0.0))
                    for c, r in pts:
                        x_off = c[0] - A['lidar_center_x']
                        ahead = A['lidar_front_y'] - c[1]
                        on_box = abs(x_off - BOX_X) < reach.BOX[0] / 2 + r and abs(ahead - BOX_AHEAD) < reach.BOX[1] / 2 + r
                        if on_box:
                            worst['box top'] = min(worst.get('box top', 1), c[2] - top_z)
                        for px, py in placed:
                            if c[2] < top_z + A['puck_height'] + CLEAR:
                                gap = math.hypot(x_off - px, ahead - py) - pr - r
                                key = f'placed puck {centres.index((px, py)) + 1}'
                                if gap < worst.get(key, 1):
                                    worst[key] = gap
                                    rows.append((gap, key, name))
            prev = q
    return worst, sorted(rows)[:5]


def main():
    face = next((int(a) for a in sys.argv[1:] if a.isdigit()), 6)
    slots = [int(v) for v in sys.argv[sys.argv.index('--slots') + 1].split(',')] if '--slots' in sys.argv \
        else DEFAULT_SLOTS[:len(reach.DICE[face])]
    if len(slots) != len(reach.DICE[face]):
        print(f'face {face} needs {len(reach.DICE[face])} slots'); return 1
    _, _, taught = clock_grasp()
    if not taught:
        print('!! CLOCK_GRASP_RAW_DEG not taught: the clock part uses the old loading pose (dry run only)')
    start = ex.nominal_pose()
    s = ex.stage(ex.with_via(steps(face, slots, ex.via_pose()[0], start), start), start)
    for name, kind, arg in s:
        print(f'  {name:45s}', ex.describe(kind, arg))
    worst, rows = check(face, slots, s)
    print('\nclearance while holding a puck (worst over the joint-linear moves):')
    for k, v in worst.items():
        print(f'  {k:16s} {v * 1000:+6.1f} mm' + ('   <-- HIT' if v < CLEAR else ''))
    for gap, key, name in rows:
        if gap < CLEAR:
            print(f'  {gap * 1000:+.1f} mm to {key} in "{name}"')
    return 0


if __name__ == '__main__':
    sys.exit(main())
