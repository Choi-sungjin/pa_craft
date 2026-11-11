#!/usr/bin/env python3
"""V186 URDF (CAD 2026-11-01): IK for picking a standing shelf puck in front of the front LiDAR.

Same scenario as demo/puck_pick (approach 4 cm above -> straight descent -> close), on the V186 full-robot URDF.
No package code is used or changed. All assumptions are the ASSUME block below; edit and rerun.
Writes plan.yaml next to this file (joint waypoints and 20 Hz frames for show.py).
V186 frame (REP-103): x forward, y left, z up; base_link = wheel axle centre, floor 34.3 mm below.
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
URDF = HERE / 'model/urdf/v186_full_robot.urdf'

# ---------------- ASSUME (not measured; edit when measured) ----------------
ASSUME = dict(
    lidar_face_x=0.2098,    # front LiDAR front face in base_link (V186 base mesh; sensor centre x 0.19067)
    lidar_center=(0.19067, 0.0, 0.0567),
    floor_z=-0.034346,      # base_footprint (tread bottom)
    puck_ahead=0.13,        # LiDAR front face -> puck CENTRE, straight ahead (+x) [m]
    puck_y=0.0,             # puck lateral position (LiDAR centre line) [m]
    puck_diameter=0.030, puck_height=0.060,
    shelf_size=(0.208, 0.205),  # shelf top x (front-back), y (left-right) [m]; centred under the puck
    shelf_height=0.07,      # the puck stands on the shelf top [m]
    finger_beyond_tcp=0.015,    # jaw tips past the TCP along the approach axis (V186 tcp: 15 mm behind the tips)
    finger_clearance=0.003,     # fingertip height above the shelf top at grasp [m]
    grasp_coverage=0.30,    # fraction of the puck height the jaws cover from its top
    approach_above=0.04,    # TCP height above the grasp point for approach [m]
    max_tilt_deg=30.0,      # approach axis lean either way (+ = tip away from the robot), smallest that fits
    collision_margin=0.003, # link meshes keep this far from the shelf and (except the jaws) the puck [m]
    limit_margin_deg=3.0,   # every revolute joint stays this far inside its URDF limits
    tool_roll=0.0,
    jaw_open=0.012, jaw_grip=0.0028,  # jaw_upper/jaw_lower [m], + = open; gap 24 mm at 0 -> 29.6 mm at grip
)
# ---------------------------------------------------------------------------

ARM = ['lm_slide', 'shoulder', 'elbow', 'wrist_pitch', 'tool_roll']


def origin_T(origin):
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


def motion_T(axis, kind, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    T = np.eye(4)
    if kind == 'prismatic':
        T[:3, 3] = a * q
    elif kind in ('revolute', 'continuous'):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


class Chain:
    def __init__(self, urdf, tip='tcp'):
        root = ET.parse(urdf).getroot()
        self.j = {j.get('name'): j for j in root.findall('joint')}
        self.by_child = by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip
        while link in by_child:
            chain.append(by_child[link]); link = by_child[link].find('parent').get('link')
        self.chain = chain[::-1]

    def limits(self, name):
        lim = self.j[name].find('limit')
        return float(lim.get('lower')), float(lim.get('upper'))

    def fk_link(self, q, link):
        """Pose of any link on the robot (jaws included), walking up from it."""
        chain, l = [], link
        while l in self.by_child:
            chain.append(self.by_child[l]); l = self.by_child[l].find('parent').get('link')
        return self.fk(q, chain[::-1])

    def fk(self, q, chain=None):
        T = np.eye(4)
        for j in (self.chain if chain is None else chain):
            T = T @ origin_T(j.find('origin'))
            if j.get('type') != 'fixed':
                T = T @ motion_T([float(v) for v in j.find('axis').get('xyz').split()], j.get('type'),
                                 q.get(j.get('name'), 0.0))
        return T


def inside(ch, q, margin=None):
    mg = math.radians(ASSUME['limit_margin_deg'] if margin is None else margin)
    return all(ch.limits(n)[0] + (0 if n == 'lm_slide' else mg) <= q[n]
               <= ch.limits(n)[1] - (0 if n == 'lm_slide' else mg) for n in ARM)


def approach_dir(tilt_deg):
    t = math.radians(tilt_deg)
    return np.array([math.sin(t), 0.0, -math.cos(t)])  # down, tip leaning away from the robot


def ik(ch, tcp, tilt_deg, seeds):
    """All arm pitch axes are along y, so the slide alone sets the TCP y; shoulder/elbow/wrist solve x, z and
    the approach axis (tcp x) numerically. Returns every distinct in-limit solution."""
    a = approach_dir(tilt_deg)
    q0 = {n: 0.0 for n in ARM}; q0['tool_roll'] = ASSUME['tool_roll']
    slide = tcp[1] - ch.fk(q0)[1, 3]  # lm_slide axis is +y
    lo = [ch.limits(n)[0] for n in ARM[1:4]]; hi = [ch.limits(n)[1] for n in ARM[1:4]]

    def q_of(x):
        return {**q0, 'lm_slide': slide, 'shoulder': x[0], 'elbow': x[1], 'wrist_pitch': x[2]}

    def res(x):
        T = ch.fk(q_of(x))
        return np.concatenate([(T[:3, 3] - tcp) * 100.0, (T[:3, 0] - a) * 2.0])

    sols = []
    for s in seeds:
        r = least_squares(res, np.clip(s, lo, hi), bounds=(lo, hi))
        q = q_of(r.x); T = ch.fk(q)
        if np.linalg.norm(T[:3, 3] - tcp) < 5e-4 and T[:3, 0] @ a > math.cos(math.radians(0.5)) and inside(ch, q):
            if all(max(abs(q[n] - p[n]) for n in ARM) > 1e-3 for p in sols):
                sols.append(q)
    return sols


def load_stl(path):
    b = path.read_bytes()
    n = int.from_bytes(b[80:84], 'little')
    tri = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return np.unique(tri['v'].reshape(-1, 3), axis=0)[::4].astype(float)  # every 4th vertex is enough here


MESH = {}  # link -> vertices (link frame)


def link_points(ch):
    if not MESH:
        root = ET.parse(URDF).getroot()
        for link in root.findall('link'):
            pts = [load_stl((URDF.parent / v.find('geometry/mesh').get('filename')).resolve())
                   for v in link.findall('visual') if v.find('geometry/mesh') is not None]
            if pts:
                MESH[link.get('name')] = np.vstack(pts)
    return MESH


def hits(ch, q):
    """Any arm link vertex inside the shelf block or the puck cylinder (jaws may touch the puck)."""
    a = ASSUME; mg = a['collision_margin']
    shelf_top = a['floor_z'] + a['shelf_height']
    px = a['lidar_face_x'] + a['puck_ahead']; sx, sy = a['shelf_size']
    for link, P in link_points(ch).items():
        if link not in LINK_CHAIN:
            continue
        T = ch.fk_link(q, link)
        W = P @ T[:3, :3].T + T[:3, 3]
        in_shelf = ((abs(W[:, 0] - px) < sx / 2 + mg) & (abs(W[:, 1] - a['puck_y']) < sy / 2 + mg)
                    & (W[:, 2] < shelf_top + mg))
        if in_shelf.any():
            return True
        if not link.startswith('jaw'):
            r = np.hypot(W[:, 0] - px, W[:, 1] - a['puck_y'])
            if ((r < a['puck_diameter'] / 2 + mg) & (W[:, 2] < shelf_top + a['puck_height'] + mg)).any():
                return True
    return False


LINK_CHAIN = ['slide_link', 'shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link',
              'jaw_upper_link', 'jaw_lower_link']

SEEDS = [np.array(s) for s in ([0.3, 2.8, 1.5], [0.3, 3.0, 1.2], [0.0, 2.0, 2.0], [0.6, 1.5, 1.0],
                               [-0.2, 2.5, 0.5], [0.5, 2.9, 1.8], [0.2, 1.0, 1.9])]


def ik_tilted(ch, tcp, prev=None):
    """Smallest lean (whole degrees, either way) with an in-limit, collision-free solution; nearest the previous pose, else least elbow
    gravity moment (shortest horizontal lever from the shoulder to the wrist)."""
    seeds = ([np.array([prev[n] for n in ARM[1:4]])] if prev else []) + SEEDS
    n = int(ASSUME['max_tilt_deg'])
    for tilt in [0] + [s * t for t in range(1, n + 1) for s in (1, -1)]:
        sols = [q for q in ik(ch, tcp, tilt, seeds) if not hits(ch, q)]
        if sols:
            if prev:
                q = min(sols, key=lambda s: sum(abs(s[n] - prev[n]) for n in ARM))
            else:
                q = min(sols, key=lambda s: abs(ch.fk({**s, 'tool_roll': 0})[0, 3] - 0.0))
            return q, tilt
    return None, None


def main():
    a = ASSUME
    ch = Chain(URDF)
    shelf_top = a['floor_z'] + a['shelf_height']
    px = a['lidar_face_x'] + a['puck_ahead']
    puck_center = np.array([px, a['puck_y'], shelf_top + a['puck_height'] / 2])
    grasp_h = max(a['puck_height'] * (1 - a['grasp_coverage']) + a['finger_beyond_tcp'],
                  a['finger_beyond_tcp'] + a['finger_clearance'])
    grasp = np.array([px, a['puck_y'], shelf_top + grasp_h])
    print(f'puck centre in V186 base_link: {np.round(puck_center, 4)}  (LiDAR front face x={a["lidar_face_x"]})')
    print(f'grasp TCP:                     {np.round(grasp, 4)}')

    ok, prev, plan = True, None, []
    for name, tcp, jaw in (('approach', grasp + [0, 0, a['approach_above']], a['jaw_open']),
                           ('grasp', grasp, a['jaw_open']), ('close', grasp, a['jaw_grip'])):
        q, tilt = ik_tilted(ch, tcp, prev)
        if q is None:
            print(f'[{name}] NO SOLUTION within limits (margin {a["limit_margin_deg"]} deg, lean <= '
                  f'{a["max_tilt_deg"]} deg) tcp={np.round(tcp, 4)}')
            ok = False; continue
        prev = q
        margins = {n: math.degrees(min(q[n] - ch.limits(n)[0], ch.limits(n)[1] - q[n])) for n in ARM[1:4]}
        print(f'[{name}] tcp={np.round(tcp, 4)}  lean {tilt} deg')
        print('    q    ' + '  '.join(f'{n} {q[n]:+.4f}' for n in ARM) + f'  jaw {jaw:+.4f}')
        print('    deg  ' + '  '.join(f'{n} {math.degrees(q[n]):+.1f}' for n in ARM[1:4]) +
              f'  slide {q["lm_slide"] * 1000:+.1f} mm')
        print('    limit margin ' + '  '.join(f'{n} {m:.1f}' for n, m in margins.items()) + ' deg')
        plan.append({'name': name, 'lean_deg': tilt,
                     'joints': {**{n: float(q[n]) for n in ARM}, 'jaw_upper': float(jaw), 'jaw_lower': float(jaw)}})

    frames = []
    if ok:
        def frame(q, jaw):
            return {'joints': {**{n: float(q[n]) for n in ARM}, 'jaw_upper': float(jaw), 'jaw_lower': float(jaw)},
                    'puck': [float(v) for v in puck_center]}
        hz, top = 20, grasp + [0, 0, a['approach_above']]
        q, _ = ik_tilted(ch, top)
        frames += [frame(q, a['jaw_open'])] * hz                             # 1 s hold above the puck
        for s in np.linspace(0, 1, 2 * hz):                                   # 2 s straight descent
            q, _ = ik_tilted(ch, top + (grasp - top) * s, q)
            if q is None:
                print(f'descent: no solution at {s:.2f}'); ok = False; break
            frames.append(frame(q, a['jaw_open']))
        if ok:
            for s in np.linspace(0, 1, hz):                                   # 1 s close
                frames.append(frame(q, a['jaw_open'] + (a['jaw_grip'] - a['jaw_open']) * s))
            frames += [frames[-1]] * int(1.5 * hz)                           # 1.5 s hold
        else:
            frames = []
    yaml.safe_dump({'assume': {k: (list(v) if isinstance(v, tuple) else v) for k, v in a.items()},
                    'puck_center': [float(v) for v in puck_center], 'grasp_tcp': [float(v) for v in grasp],
                    'waypoints': plan, 'frames': frames}, open(HERE / 'plan.yaml', 'w'), sort_keys=False)
    print('plan.yaml written;', 'all waypoints within limits' if ok else 'SOME WAYPOINTS FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
