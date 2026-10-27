#!/usr/bin/env python3
"""Throwaway demo: IK for picking a floor puck in front of the front LiDAR (approach -> grasp; no lift).

Delete with: rm -rf ~/craft/demo
No package code is used or changed. All assumptions are the ASSUME block below; edit and rerun.
Writes plan.yaml next to this file (joint waypoints for show.py).
--lying: puck on its side (long axis front-back), its near end gripped like the standing puck with the tool
exactly level (pointing -Y) and the jaws closing left-right; writes plan_lying.yaml (execute.py keeps plan.yaml).
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent
URDF = WS / 'src/craft_planning/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf'
CONTROL_YAML = WS / 'src/craft_control/config/control.yaml'

# ---------------- ASSUME (not measured; edit when measured) ----------------
ASSUME = dict(
    # Front LiDAR as modelled in the URDF base mesh (block x 108..148, y 34..71, z 20..53 mm).
    lidar_center_x=0.128,  # lateral centre of the LiDAR = robot lateral centre
    puck_x_offset=0.04,    # [m] puck this far from the LiDAR centre along the LM axis (+X: away from the LM motor)
    lidar_center_y=0.0525,
    lidar_front_y=0.034,   # front face; the robot front is -Y
    lidar_z=(0.020, 0.053),
    floor_z=0.000,      # wheel bottom in the base mesh
    hw_z_correction=0.0,    # [m] was 0.025: the 2.5 cm drop on 10/24 was the URDF wrist->tool-roll horn
                            # length (55 -> measured 78 mm, 2026-10-26); the 10/24 grasp on the new URDF is 25.4 mm lower
    shelf_size=(0.205, 0.208),  # shelf top x (left-right), y (front-back) [m]; centred under the puck
    shelf_height=0.07,  # the puck stands on the shelf top [m]
    puck_ahead=0.13,    # LiDAR front face -> puck CENTRE, straight ahead (-Y) [m]
    puck_size=(0.03, 0.03, 0.06),  # box x, y, height [m]; cylinder r 0.03 h 0.06 uses the same grasp height
    puck_height=0.06,   # [m]
    finger_beyond_tcp=0.015,   # jaw mesh tip past tool_tcp along the tool axis [m] (TCP = middle of the 30 mm usable grip)
    finger_clearance=0.003,    # fingertip height above the shelf top at grasp [m]
    approach_above=0.04,  # TCP height above the grasp point for approach [m] (needs 29 of the 30 deg lean)
    # On the 7 cm shelf the wrist_pitch lower limit binds as the TCP rises above the grasp.
    # The tool stays vertical where it can and otherwise leans its tip forward
    # (away from the robot) by the smallest angle that fits the limits.
    grasp_coverage=0.30,   # fraction of the puck height the jaws cover from its top (keep <= 0.5 for loading);
                           # 0.3 x 60 mm = 18 mm = 60% of the 30 mm usable grip (was 0.50: 100%)
    max_tilt_deg=30.0,     # tool tip lean toward -Y (robot front) allowed above the grasp
    limit_margin_deg=3.0,  # every joint stays this far inside its URDF limits (hardware sync is approximate)
    tool_roll=0.0,         # URDF tool_roll_joint; aligned (jaws close front-back)
    jaw_open=-0.021, jaw_grip=-0.0018,  # URDF jaw_upper_joint values (ik_rviz_node defaults)
    lying=False,           # puck on its side, long axis front-back (--lying)
    tool_tilt_deg=None,    # fixed tool lean instead of the smallest that fits (90 = level, pointing -Y)
    shelf_ahead=None,      # LiDAR front face -> shelf CENTRE [m]; None: centred under the puck
)
# --lying (2026-10-27): only the tool pointing -Y works. The planned level grasp at 41 cm overloaded the shoulder;
# a grasp taught by hand reaches 31.7 cm with less shoulder torque. In this model it leans the tool tip up 7.4 deg
# with the TCP 88 mm high (offsets approximate). It sits on the elbow/wrist limit corner (no room to come down onto
# it), so the plan keeps its tool angle and height and backs the puck off until the 4 cm vertical approach fits
# (32.8 cm). The rolled jaws stand +-12 mm (tips) / +-29 mm (base, 28 mm behind the TCP) about the
# puck axis, so the shelf must end at the puck's near end.
LYING = dict(
    lying=True,
    taught_grasp_motor_deg=(177.4, 321.8, 345.1, 0.0),  # shoulder, elbow, wrist_pitch, tool_roll (by hand, 2026-10-27)
    tool_roll=-math.pi / 2,   # jaws left-right (motor 0); +90 deg is outside the URDF limit (1.4835)
    limit_margin_deg=0.5,
    puck_x_offset=-0.01,      # middle of the LM range (LiDAR centre -58..+38 mm)
    puck_size=(0.03, 0.06, 0.03),  # x, y (long axis), height [m]
    puck_height=0.03,
)  # tool_tilt_deg, puck_ahead and shelf_ahead (shelf near edge 5 mm before the puck's near end) follow the grasp
# ---------------------------------------------------------------------------


def rpy_xyz(origin):
    T = np.eye(4)
    if origin is None:
        return T
    T[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    T[:3, :3] = Rz @ Ry @ Rx
    return T


def axis_motion(axis, kind, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    T = np.eye(4)
    if kind == 'prismatic':
        T[:3, 3] = a * q
    elif kind in ('revolute', 'continuous'):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


class Chain:
    NAMES = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']

    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        self.j = {j.get('name'): j for j in root.findall('joint')}
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], 'tool_tcp'
        while link in by_child:
            chain.append(by_child[link]); link = by_child[link].find('parent').get('link')
        self.chain = chain[::-1]

    def limits(self, name):
        lim = self.j[name].find('limit')
        return float(lim.get('lower')), float(lim.get('upper'))

    def fk(self, q):
        T = np.eye(4)
        for j in self.chain:
            T = T @ rpy_xyz(j.find('origin'))
            if j.get('type') != 'fixed':
                ax = [float(v) for v in j.find('axis').get('xyz').split()]
                T = T @ axis_motion(ax, j.get('type'), q.get(j.get('name'), 0.0))
        return T

    def origin(self, name):
        return np.array([float(v) for v in self.j[name].find('origin').get('xyz').split()])


def ik_tilted(ch, tcp):
    """Smallest forward tool lean (whole degrees, up to max_tilt_deg) whose solution keeps limit_margin_deg
    inside every revolute URDF limit (the slide keeps the plain limits)."""
    fixed = ASSUME['tool_tilt_deg']
    for tilt in ([fixed] if fixed is not None else range(int(ASSUME['max_tilt_deg']) + 1)):
        for q in ik_top_down(ch, tcp, tilt):
            if inside(ch, q):
                return [q]
    return ik_top_down(ch, tcp, fixed or 0)  # out of limits; main() reports it


def inside(ch, q):
    """q keeps limit_margin_deg inside every revolute URDF limit (the slide keeps the plain limits)."""
    mg = math.radians(ASSUME['limit_margin_deg'])
    return all(ch.limits(n)[0] + (0 if n == 'slide_joint' else mg) <= q[n]
               <= ch.limits(n)[1] - (0 if n == 'slide_joint' else mg) for n in Chain.NAMES)


def ik_top_down(ch, tcp, tilt_deg=0.0):
    """slide sets X; shoulder/elbow/wrist solve the YZ plane with the tool tip tilt_deg forward of down."""
    q0 = {n: 0.0 for n in Chain.NAMES}
    base = ch.fk(q0)
    slide_axis = np.array([float(v) for v in ch.j['slide_joint'].find('axis').get('xyz').split()])
    slide = (tcp[0] - base[0, 3]) / slide_axis[0]
    S = ch.origin('slide_joint') + ch.origin('shoulder_joint')         # shoulder axis point (y, z)
    L1 = ch.origin('elbow_joint')[1:]; L2 = ch.origin('wrist_pitch_joint')[1:]
    L3 = ch.origin('tool_roll_joint')[1:] + ch.origin('tool_tcp_joint')[1:]
    a1, a2, l3 = np.linalg.norm(L1), np.linalg.norm(L2), np.linalg.norm(L3)
    al1, al2, al3 = (math.atan2(v[1], v[0]) for v in (L1, L2, L3))
    phi = -math.pi / 2 - math.radians(tilt_deg)                        # tool direction in the YZ plane
    W = np.array(tcp[1:]) - l3 * np.array([math.cos(phi), math.sin(phi)])
    d = W - S[1:]; r = np.linalg.norm(d)
    c2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2)
    if abs(c2) > 1:
        return []
    sols = []
    for s in (+1, -1):
        b = s * math.acos(c2)                                           # theta2 - theta1
        t1 = math.atan2(d[1], d[0]) - math.atan2(a2 * math.sin(b), a1 + a2 * math.cos(b))
        t2 = t1 + b
        q1 = t1 - al1; q2 = t2 - al2 - q1; q3 = phi - al3 - q1 - q2
        def fit(name, a):  # pick the 2*pi-equivalent angle that lies inside the URDF limits, if any
            lo, hi = ch.limits(name)
            cands = [a + k * 2 * math.pi for k in (-2, -1, 0, 1, 2)]
            inside = [c for c in cands if lo <= c <= hi]
            return inside[0] if inside else min(cands, key=lambda c: max(lo - c, c - hi))
        sols.append({'slide_joint': slide, 'shoulder_joint': fit('shoulder_joint', q1),
                     'elbow_joint': fit('elbow_joint', q2), 'wrist_pitch_joint': fit('wrist_pitch_joint', q3),
                     'tool_roll_joint': ASSUME['tool_roll']})
    return sols


def trajectory(ch, grasp, puck_off, hz=20):
    """Frames at `hz`: hold approach, straight-down descent, close, hold (no lift: the next step loads it behind)."""
    a = ASSUME
    prev = None

    def solve(tcp):
        nonlocal prev
        sols = ik_tilted(ch, tcp)
        q = min(sols, key=lambda s: sum(abs(s[n] - prev[n]) for n in Chain.NAMES)) if prev else sols[0]
        prev = q
        return q

    def frame(q, jaw, puck):
        return {'joints': {**{n: float(q[n]) for n in Chain.NAMES}, 'jaw_upper_joint': float(jaw)},
                'puck': [float(v) for v in puck + puck_off]}

    out = []
    top = grasp + [0, 0, a['approach_above']]
    q = solve(top)
    out += [frame(q, a['jaw_open'], grasp)] * hz                             # 1 s hold above the puck
    for s in np.linspace(0, 1, 2 * hz):                                       # 2 s straight descent
        out.append(frame(solve(top + (grasp - top) * s), a['jaw_open'], grasp))
    for s in np.linspace(0, 1, hz):                                           # 1 s close
        out.append(frame(prev, a['jaw_open'] + (a['jaw_grip'] - a['jaw_open']) * s, grasp))
    out += [out[-1]] * int(1.5 * hz)                                          # 1.5 s hold
    return out


def ros_to_urdf(p):
    a = ASSUME
    x, y, z = p
    if a['front'] == '-Y':      # ROS x (front) = -Y, ROS y (left) = +X
        return np.array([a['center_x'] + y, a['center_y'] - x, a['floor_z'] + z])
    return np.array([a['center_x'] - y, a['center_y'] + x, a['floor_z'] + z])  # front = +Y


def main():
    a = ASSUME
    if '--lying' in sys.argv:
        a.update(LYING)
    ch = Chain(URDF)
    ctrl = yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']
    off, dirs = ctrl['arm_joint_offsets'], ctrl['arm_joint_directions']
    lo_m, hi_m = ctrl['arm_min_angle'], ctrl['arm_max_angle']
    lm_m = (ctrl['lm_min_angle'], ctrl['lm_max_angle'])
    # Fingertips end grasp_coverage of the puck height below its top; TCP sits finger_beyond_tcp above them.
    grasp_z = max(a['puck_height'] * (1 - a['grasp_coverage']) + a['finger_beyond_tcp'],
                  a['finger_beyond_tcp'] + a['finger_clearance'])
    px, py = a['lidar_center_x'] + a['puck_x_offset'], a['lidar_front_y'] - a['puck_ahead']
    top = a['floor_z'] + a['shelf_height']
    grasp = np.array([px, py, top + grasp_z + a['hw_z_correction']])
    if a['lying']:  # the taught pose; the puck's near end sits grasp_coverage of its length inside the jaws
        q = {n: d * (math.radians(m) - o) for n, m, o, d in
             zip(Chain.NAMES[1:], a['taught_grasp_motor_deg'], off, dirs)}
        T = ch.fk(q)
        tool = T[:3, 1]
        a['tool_tilt_deg'] = float((-90 - math.degrees(math.atan2(tool[2], tool[1]))) % 360)
        for k in range(41):  # 2.5 mm steps away from the robot until the whole approach line fits
            grasp = np.array([px, T[1, 3] - 0.0025 * k, T[2, 3]])
            if all(any(inside(ch, s) for s in ik_top_down(ch, grasp + [0, 0, h], a['tool_tilt_deg']))
                   for h in np.linspace(0, a['approach_above'], 17)):
                break
        else:
            print('no grasp within 10 cm beyond the taught one fits the approach'); return 1
        length = a['puck_size'][1]
        py = grasp[1] - a['finger_beyond_tcp'] + length * a['grasp_coverage'] - length / 2
        a['puck_ahead'] = float(a['lidar_front_y'] - py)
        a['shelf_ahead'] = a['puck_ahead'] - length / 2 - 0.005 + a['shelf_size'][1] / 2
        print(f'taught grasp: tool {a["tool_tilt_deg"] - 90:+.1f} deg from level (+ = tip up), backed off '
              f'{k * 2.5:.1f} mm; puck centre {a["puck_ahead"] * 100:.1f} cm ahead of the LiDAR face')
    puck_center = np.array([px, py, top + a['puck_height'] / 2])  # lying: drawn level
    print(f'puck centre in URDF base_link: {np.round(puck_center, 4)}  (LiDAR front face y={a["lidar_front_y"]})')
    print(f'grasp point in URDF base_link:                               {np.round(grasp, 4)}')
    steps = [('approach', grasp + [0, 0, a['approach_above']], a['jaw_open']),
             ('grasp', grasp, a['jaw_open']),
             ('close', grasp, a['jaw_grip'])]
    plan, ok = [], True
    for name, tcp, jaw in steps:
        sols = ik_tilted(ch, tcp)
        best = None
        for q in sols:
            bad = [n for n in Chain.NAMES if not ch.limits(n)[0] <= q[n] <= ch.limits(n)[1]]
            if best is None or len(bad) < len(best[1]):
                best = (q, bad)
        if best is None:
            print(f'[{name}] UNREACHABLE (out of arm reach) tcp={np.round(tcp, 4)}'); ok = False; continue
        q, bad = best
        err = np.linalg.norm(ch.fk(q)[:3, 3] - tcp)
        tool_dir = ch.fk(q)[:3, 1]
        motor = [off[i] + dirs[i] * q[n] for i, n in enumerate(Chain.NAMES[1:])]
        lm_motor = ctrl['lm_motor_offset'] + ctrl['lm_motor_direction'] * q['slide_joint'] / ctrl['lm_meters_per_rad']
        mbad = [n for n, m, l, h in zip(Chain.NAMES[1:], motor, lo_m, hi_m) if not l <= m <= h]
        if not lm_m[0] <= lm_motor <= lm_m[1]:
            mbad.append('LM')
        print(f'[{name}] tcp={np.round(tcp, 4)}  fk_err={err * 1000:.2f} mm  tool_dir={np.round(tool_dir, 3)}')
        print('    URDF  ' + '  '.join(f'{n.split("_")[0]} {q[n]:+.4f}' for n in Chain.NAMES) + f'  jaw {jaw:+.4f}')
        print('    motor ' + f'LM {math.degrees(lm_motor):.1f}  ' +
              '  '.join(f'{n.split("_")[0]} {math.degrees(m):.1f}' for n, m in zip(Chain.NAMES[1:], motor)) + ' deg')
        if bad or mbad:
            print(f'    !! outside limits: URDF {bad} motor {mbad}'); ok = False
        plan.append({'name': name, 'joints': {**{n: float(q[n]) for n in Chain.NAMES},
                                              'jaw_upper_joint': float(jaw)}})
    frames = trajectory(ch, grasp, puck_center - grasp) if ok else []
    out = 'plan_lying.yaml' if a['lying'] else 'plan.yaml'
    yaml.safe_dump({'assume': {k: (list(v) if isinstance(v, tuple) else v) for k, v in a.items()},
                    'puck_grasp_urdf': [float(v) for v in grasp], 'waypoints': plan, 'frames': frames},
                   open(HERE / out, 'w'), sort_keys=False)
    print(f'{out} written;', 'all waypoints within limits' if ok else 'SOME WAYPOINTS FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
