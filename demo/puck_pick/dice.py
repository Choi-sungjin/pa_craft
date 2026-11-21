#!/usr/bin/env python3
"""Throwaway demo, dice step 3: take the stored pucks out of the clock one by one and stand them on the box as a
dice face (puck_execute --dice N). This file builds the steps and checks them in URDF FK; puck_execute runs them.

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

from functools import lru_cache
from craft_interfaces.msg import TaskCommand
from executor_preview import preview
import plan
import reach
import teach

M = TaskCommand.MANIPULATOR
ARM = plan.Chain.NAMES
UNLOAD_DEPTH = 0.03
UNLOAD_ABOVE = 0.06
RELEASE_GRIPPER_DEG = 147.9
NO_STAGING = ('arm over the slot', 'straight down to the grasp', 'straight up ', 'descend',
              'lift the puck', 'put down', 'rise above the puck', 'via pose:', 'via pose (off',
              'to the nominal pose')
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



@lru_cache(maxsize=1)
def fixed_poses():
    return preview('--goto', 'nominal')


def via_pose():
    return fixed_poses()['via']


def close_but_shoulder(a, b, joint_deg=15.0, slide_m=0.005):
    """True when LM (slide) and the elbow, wrist_pitch and tool_roll of the poses a and b are within joint_deg / slide_m:
    only the shoulder differs, so the arm can go in one move (2026-11-13, user: the stop-and-go steps looked jerky)."""
    return abs(a[0] - b[0]) < slide_m and all(abs(a[i] - b[i]) < math.radians(joint_deg) for i in (2, 3, 4))


def wrist_first(prev, target, what):
    """Step: prev with only the wrist_pitch moved to target's (the elbow follows in the next step)."""
    q = list(prev)
    q[3] = target[3]
    return (f'wrist_pitch to {what} first, the rest stays', TaskCommand.MANIPULATOR, q)


def out_to_via(prev):
    """Steps prev -> via pose, the reverse of the way back: the shoulder alone, then the elbow with LM, tool_roll and
    gripper, then the wrist_pitch alone."""
    via = via_pose()
    m = TaskCommand.MANIPULATOR
    if close_but_shoulder(prev, via):  # one smooth move instead of three stop-and-go ones
        return [('via pose: one move (only the shoulder moves)', m, via)]
    shoulder = list(prev[:1]) + [via[1]] + list(prev[2:])
    elbow = via[:3] + [prev[3]] + via[4:]
    return [('via pose: shoulder first, the rest stays', m, shoulder),
            ('via pose: elbow, LM, tool_roll, gripper next (wrist_pitch stays)', m, elbow),
            ('via pose (off the nominal pose): wrist_pitch last', m, via)]


def with_via(steps, start, out=True):
    """steps with, before the last move to start: the wrist_pitch to the via pose, the via pose, the wrist_pitch to
    start; and (out) the via pose as the first move. A step's target is a pose or, in steps_from_plan, a list of
    poses."""
    via = via_pose()
    nested = lambda arg: isinstance(arg[0], list)
    target = lambda arg: arg[-1] if nested(arg) else arg
    last = max(i for i, (_, kind, arg) in enumerate(steps)
               if kind == TaskCommand.MANIPULATOR and target(arg) == start)
    wrap = (lambda q: [q]) if nested(steps[last][2]) else (lambda q: q)
    prev = next(target(arg) for _, kind, arg in reversed(steps[:last]) if kind == TaskCommand.MANIPULATOR)
    if all(abs(a - b) < 1e-6 for a, b in zip(prev[:5], via[:5])):  # already comes back through the via pose
        return [(n, k, wrap(q)) for n, k, q in out_to_via(start)] + steps if out else steps
    name, kind, wrist = wrist_first(prev, via, 'the via pose')
    back = ('via pose before the nominal pose', TaskCommand.MANIPULATOR, wrap(via))
    name2, kind2, wrist2 = wrist_first(via, start, 'the nominal pose')
    steps = steps[:last] + [(name, kind, wrap(wrist)), back, (name2, kind2, wrap(wrist2))] + steps[last:]
    if out:
        steps = [(n, k, wrap(q)) for n, k, q in out_to_via(start)] + steps
    return steps


def below(arm, dz):
    """arm (shoulder, elbow, wrist_pitch, tool_roll) moved so the TCP goes dz straight down and the
    tool keeps its angle: Newton on the three parallel pitch joints, URDF FK."""
    ch = plan.Chain(plan.URDF)
    names = ARM[1:4]
    def pose(q):
        T = ch.fk(dict(zip(names, q), tool_roll_joint=arm[3]))
        return np.array([T[1, 3], T[2, 3], math.atan2(T[2, 2], T[1, 2])])
    q = np.array(arm[:3], dtype=float)
    goal = pose(q) - np.array([0.0, dz, 0.0])
    for _ in range(50):
        e = goal - pose(q)
        e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(pose(q + h) - pose(q)) / 1e-6 for h in np.eye(3) * 1e-6])
        q += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm below the loading pose')
    for n, v in zip(names, q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits '
                               f'{math.degrees(lo):.1f}..{math.degrees(hi):.1f} at {dz * 100:.1f} cm below the loading pose')
    return [float(v) for v in q] + [arm[3]]


def above(arm, dz):
    """arm with wrist_pitch and tool_roll kept, shoulder/elbow solved so the TCP sits dz straight above.
    The tool leans by however much shoulder+elbow change (the wrist limit forbids keeping it)."""
    ch = plan.Chain(plan.URDF)
    tcp = lambda q: ch.fk(dict(zip(ARM[1:], q)))[1:3, 3]
    q = np.array(arm, dtype=float)
    goal = tcp(q) + np.array([0.0, dz])
    for _ in range(50):
        e = goal - tcp(q)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(tcp(q + h) - tcp(q)) / 1e-6 for h in (np.r_[1e-6, 0, 0, 0], np.r_[0, 1e-6, 0, 0])])
        q[:2] += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm above the unload grasp')
    for n, v in zip(ARM[1:3], q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits at {dz * 100:.1f} cm above')
    return [float(v) for v in q]


def stage(steps, prev):
    """Elbow first, shoulder last: each MANIPULATOR target (a pose or a path of poses) becomes the pose with the
    shoulder still at its previous angle, then the pose. prev: the arm pose before steps (None: first not staged)."""
    out = []
    for name, kind, arg in steps:
        if kind != TaskCommand.MANIPULATOR:
            out.append((name, kind, arg)); continue
        path = arg if isinstance(arg[0], list) else [arg]
        poses = []
        for q in path:
            mid = [q[0], prev[1]] + q[2:] if prev is not None and not name.startswith(NO_STAGING) else q
            if mid != q and mid != prev:
                poses.append(mid)
            poses.append(q); prev = q
        out.append((name, kind, poses))
    return out

def spots(face):
    """Puck centres (x from the LiDAR centre, ahead of the LiDAR face) [m] in put-down order."""
    return sorted(reach.dice_spots(face, BOX_X, BOX_AHEAD, PIP), key=lambda p: (-round(p[0], 4), -p[1]))


def clock_grasp():
    """(slide, [shoulder, elbow, wrist_pitch, tool_roll], taught) at the clock grasp."""
    if CLOCK_GRASP_RAW_DEG is not None:
        t = teach.raw_to_target(CLOCK_GRASP_RAW_DEG)
        return t[0], t[1:5], True
    return fixed_poses()['nominal'][0], below(fixed_poses()['load'], UNLOAD_DEPTH), False


def line(ch, x, ahead, heights):
    """Poses [slide, shoulder, elbow, wrist_pitch, PUT_ROLL] on the vertical line over spot (x, ahead), puck bottom
    heights over the box top; the tool leans the least that fits (plan.ik_tilted)."""
    out = []
    for h in heights:
        q = plan.ik_tilted(ch, plan.grasp_at(x, ahead) + [0, 0, h])
        if not q or not plan.inside(ch, q[0]):
            raise RuntimeError(f'no IK {h * 1000:.0f} mm over spot {x * 1000:+.1f}, {ahead * 1000:.1f} mm')
        out.append([q[0]['slide_joint'] + reach.LM_BACKLASH] + [q[0][n] for n in ARM[1:4]] + [PUT_ROLL])
    return out


def steps(face, slots, current_slide=None, start=None):
    """puck_execute steps for dice face `face`, puck k from clock slot slots[k]."""
    A['grasp_coverage'] = reach.GRASP_DEPTH / A['puck_height']
    A['shelf_height'] = reach.BOX[2]
    ch = plan.Chain(plan.URDF)
    slide, grasp, _ = clock_grasp()
    high = above(grasp, UNLOAD_ABOVE)
    lift = below(grasp, -LIFT)
    release = math.radians(RELEASE_GRIPPER_DEG)
    downs = np.r_[np.arange(CARRY_ABOVE - DOWN_STEP, RELEASE_ABOVE, -DOWN_STEP), RELEASE_ABOVE]
    s = []
    if current_slide is not None:
        s.append(('LM to the clock line, arm stays', M, [slide] + via_pose()[1:5]))
    for k, ((x, ahead), slot) in enumerate(zip(spots(face), slots), 1):
        carry = line(ch, x, ahead, [CARRY_ABOVE])[0]
        down = line(ch, x, ahead, downs)
        tag = f'puck {k}/{len(slots)}'
        if k > 1:
            s.append((f'{tag}: LM to the clock line, arm stays', M, [slide] + prev_carry[1:]))
        s.append((f'{tag}: clock to slot {slot} (loading guide)', TaskCommand.CLOCK, slot))
        s.append((f'{tag}: open gripper to {RELEASE_GRIPPER_DEG:g} deg', TaskCommand.GRIPPER, release))
        s.append((f'{tag}: swing back over the slot, {UNLOAD_ABOVE * 100:.0f} cm above the grasp, LM stays', M,
                  [slide] + high))
        s.append((f'straight down to the grasp ({tag}), LM stays', M, [slide] + grasp))
        s.append((f'{tag}: close gripper (grasp the puck in the clock)', TaskCommand.GRIPPER, True))
        s.append((f'straight up {LIFT * 100:.0f} cm out of the slot ({tag}), tool angle kept, LM stays', M,
                  [slide] + lift))
        s.append((f'{tag}: swing to the front, {CARRY_ABOVE * 1000:.0f} mm over the box, jaws left-right, LM stays',
                  M, [slide] + carry[1:]))
        s.append((f'{tag}: LM to the spot line ({(x - BOX_X) * 1000:+.1f} mm from the box centre)', M, carry))
        s.append((f'put down ({tag}) at {ahead * 1000:.1f} mm ahead, to {RELEASE_ABOVE * 1000:.0f} mm over the box',
                  M, down))
        s.append((f'{tag}: open gripper to {RELEASE_GRIPPER_DEG:g} deg (release)', TaskCommand.GRIPPER, release))
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
        if kind == TaskCommand.GRIPPER:
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
                    T = ch.fk(dict(zip(ARM, p[:5])))
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
    start = fixed_poses()['nominal']
    s = stage(with_via(steps(face, slots, via_pose()[0], start), start), start)
    for name, kind, arg in s:
        print(f'  {name:45s}', arg[-1] if kind == M else arg)
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
