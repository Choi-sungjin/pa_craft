#!/usr/bin/env python3
"""shoulder_path model <-> URDF arm (2026-11-19).

URDF: src/craft_description/urdf/v307_robotis_arm.urdf (hardware branch, L1 = L2 = 110.3 mm). The arm moves in the x-z
plane of slide_link (shoulder axis +y, elbow and wrist_pitch axes -y). At zero L1 points up, L2 forward.

Joint mapping (model angles CCW from the forward horizontal, x forward, z up):
    phi1 = 90 deg - q_shoulder            ->  psi = 90 deg, theta1 = -q_shoulder
    phi2 = phi1 - 90 deg + q_elbow        ->  theta2 = q_elbow - 90 deg
    phi3 = phi2 + q_wrist_pitch + delta   ->  theta3 = q_wrist_pitch + delta
delta: the L3 line goes from the wrist axis to the TCP, which sits slightly above the wrist link x axis.

Masses (URDF links already hold their motors, so m1 = 0 and Lm = 0):
    M_L1 = shoulder_link (with the elbow motor), M_L2 = elbow_link (with the wrist motor), M_L3 = wrist_pitch_link,
    m2 = tool_roll_link + both jaws (at rm2 along L3). L3 ends at the grip point, grip_inset (30 mm) inside the finger
    tips (the URDF tcp frame is 15 mm inside). Centres of mass off the link lines are dropped; urdf_check()
    compares the model M with the full URDF gravity moment over random poses and tool_roll angles.

python3 urdf_arm.py     parameters from the URDF + check against the URDF gravity moment
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import shoulder_path as sp  # noqa: E402

URDF = Path(__file__).resolve().parents[2] / 'src/craft_description/urdf/v307_robotis_arm.urdf'
ARM = ['shoulder', 'elbow', 'wrist_pitch', 'tool_roll']
DOWNSTREAM = ['shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link', 'jaw_upper_link', 'jaw_lower_link']
G = sp.G


def _v(s, n=3):
    return np.array([float(x) for x in (s or ' '.join(['0'] * n)).split()])


def _rpy(r, p, y):
    cr, sr, cp, s_p, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * s_p * sr - sy * cr, cy * s_p * cr + sy * sr],
                     [sy * cp, sy * s_p * sr + cy * cr, sy * s_p * cr - cy * sr],
                     [-s_p, cp * sr, cp * cr]])


def _axis_rot(a, q):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


RAW_ZERO = {}   # joint -> raw motor angle [rad] at the geometric zero (filled by load())


def _mat_rpy(R):
    p = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        return np.array([math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])])
    return np.array([0.0, p, math.atan2(-R[0, 1], R[1, 1])])


def raw_from_geom(q):
    """{joint: geometric angle} -> {joint: URDF (= motor raw) angle} [rad]."""
    return {n: v + RAW_ZERO.get(n, 0.0) for n, v in q.items()}


def load(path=URDF):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall('joint'):
        o, a, lim = j.find('origin'), j.find('axis'), j.find('limit')
        joints[j.get('name')] = dict(
            type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
            xyz=_v(o.get('xyz') if o is not None else None), rpy=_v(o.get('rpy') if o is not None else None),
            axis=_v(a.get('xyz')) if a is not None else np.array([1.0, 0, 0]),
            lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None and lim.get('lower') else None)
    # URDF angle = motor raw (2026-11-19): the generator put each arm joint's zero (raw at the geometric zero pose:
    # L1 up, L2 forward) into its origin rpy and wrote it to <urdf>.raw_zero.json. Fold it back out so everything
    # here works in the geometric angle (geometric = raw - zero); raw_from_geom() maps back for joint_states.
    zf = Path(path).with_suffix('.raw_zero.json')
    RAW_ZERO.clear()
    if zf.exists():
        import json
        RAW_ZERO.update(json.loads(zf.read_text()))
        for n, z in RAW_ZERO.items():
            j = joints[n]
            j['rpy'] = _mat_rpy(_rpy(*j['rpy']) @ _axis_rot(j['axis'], z))
            if j['lim']:
                j['lim'] = (j['lim'][0] - z, j['lim'][1] - z)
    links = {}
    for L in root.findall('link'):
        i = L.find('inertial')
        if i is not None:
            links[L.get('name')] = dict(m=float(i.find('mass').get('value')), com=_v(i.find('origin').get('xyz')))
    return joints, links


def fk(joints, q, base='slide_link'):
    """Pose (R, p) of every link below `base` in the base frame. q: {joint name: value} (missing = 0)."""
    out = {base: (np.eye(3), np.zeros(3))}
    todo = True
    while todo:
        todo = False
        for n, j in joints.items():
            if j['parent'] in out and j['child'] not in out:
                R0, p0 = out[j['parent']]
                R = R0 @ _rpy(*j['rpy'])
                p = p0 + R0 @ j['xyz']
                v = q.get(n, 0.0)
                if j['type'] in ('revolute', 'continuous'):
                    R = R @ _axis_rot(j['axis'], v)
                elif j['type'] == 'prismatic':
                    p = p + R @ (j['axis'] * v)
                out[j['child']] = (R, p)
                todo = True
    return out


def urdf_M(joints, links, q):
    """Full URDF gravity moment about the shoulder axis, in the model's sign (CCW from +x toward +z = about -y)."""
    poses = fk(joints, q)
    axis_p = poses['shoulder_link'][1]
    M = 0.0
    for n in DOWNSTREAM:
        R, p = poses[n]
        r = p + R @ links[n]['com'] - axis_p
        M += -G * links[n]['m'] * r[0]   # (r x F) . (-y) with F = (0, 0, -m g)
    return M


def plane(v):
    return np.array([v[0], v[2]])


URDF_TCP_INSET = 0.015   # the URDF tcp sits 15 mm inside the finger tips (craft_description README)


def params(path=URDF, grip_inset=0.030):
    """shoulder_path P (and the joint mapping) from the URDF. Returns (P, info).
    grip_inset [m]: the grip point (the model's L3 end, puck centre) this far inside the finger tips, along the
    tool_roll axis (the fingers' direction). User 2026-11-19: 30 mm."""
    joints, links = load(path)
    J = {n: joints[n] for n in ARM}
    elbow = plane(J['elbow']['xyz'])        # in shoulder_link (zero pose: L1 straight up)
    wrist = plane(J['wrist_pitch']['xyz'])  # in elbow_link (L2 forward)
    roll = plane(J['tool_roll']['xyz'])     # in wrist_pitch_link
    ax = J['tool_roll']['axis'] / np.linalg.norm(J['tool_roll']['axis'])
    tcp = roll + plane(joints['tcp_joint']['xyz']) - (grip_inset - URDF_TCP_INSET) * plane(ax)
    delta = math.atan2(tcp[1], tcp[0])      # L3 line above the wrist link x axis
    u3 = np.array([math.cos(delta), math.sin(delta)])

    def along(c, u):
        return float(c @ u), float(abs(c[0] * u[1] - c[1] * u[0]))  # (distance along the line, off the line)

    off = {}
    r1, off['shoulder_link'] = along(plane(links['shoulder_link']['com']), elbow / np.linalg.norm(elbow))
    r2, off['elbow_link'] = along(plane(links['elbow_link']['com']), wrist / np.linalg.norm(wrist))
    r3, off['wrist_pitch_link'] = along(plane(links['wrist_pitch_link']['com']), u3)
    # gripper = tool_roll_link + jaws, in the wrist frame at tool_roll 0, jaws 0
    poses = fk(joints, {}, base='wrist_pitch_link')
    mg, cg = 0.0, np.zeros(3)
    for n in ('tool_roll_link', 'jaw_upper_link', 'jaw_lower_link'):
        R, p = poses[n]
        mg += links[n]['m']
        cg += links[n]['m'] * (p + R @ links[n]['com'])
    cg /= mg
    rm2, off['gripper'] = along(plane(cg), u3)
    lim = {n: J[n]['lim'] for n in ('shoulder', 'elbow', 'wrist_pitch')}
    to_th = [lambda qs: -qs, lambda qe: qe - math.pi / 2, lambda qw: qw + delta]
    th_lo = [to_th[i](v) for i, v in enumerate([lim['shoulder'][1], lim['elbow'][0], lim['wrist_pitch'][0]])]
    th_hi = [to_th[i](v) for i, v in enumerate([lim['shoulder'][0], lim['elbow'][1], lim['wrist_pitch'][1]])]
    P = dict(M_L1=links['shoulder_link']['m'], m1=0.0, M_L2=links['elbow_link']['m'],
             M_L3=links['wrist_pitch_link']['m'], m2=mg,
             L1=float(np.linalg.norm(elbow)), Lm=0.0, L2=float(np.linalg.norm(wrist)), L3=float(np.linalg.norm(tcp)),
             r1=r1, r2=r2, r3=r3, rm2=rm2, psi=math.pi / 2,
             q_min=np.array(th_lo), q_max=np.array(th_hi))
    info = dict(delta=delta, off=off, joints=joints, links=links, grip_shift=grip_inset - URDF_TCP_INSET,
                measured=False)
    return P, info


def measured(P, info, L1=None, L2=None, grip_tip=None, grip_inset=0.030):
    """P with the user's measured lengths [m] in place of the URDF ones (2026-11-19: L1 147, L2 122, wrist axis ->
    finger tips 203 mm). L3 = grip_tip - grip_inset. Masses stay the URDF ones; r1, r2 scale with their link (same
    centre-of-mass ratio). The URDF robot in RViz keeps its own lengths, so its TCP no longer sits on the model tip."""
    P = dict(P)
    if L1:
        P['r1'], P['L1'] = P['r1'] * L1 / P['L1'], L1
    if L2:
        P['r2'], P['L2'] = P['r2'] * L2 / P['L2'], L2
    if grip_tip:
        P['L3'] = grip_tip - grip_inset
    info = dict(info, measured=bool(L1 or L2 or grip_tip))
    return P, info


def to_model(q_sh, q_el, q_wr, delta):
    return np.array([-q_sh, q_el - math.pi / 2, q_wr + delta])


def to_urdf(th, delta):
    """Model (theta1, theta2, theta3) -> URDF (shoulder, elbow, wrist_pitch)."""
    return np.array([-th[0], th[1] + math.pi / 2, th[2] - delta])


def urdf_check(P, info, n=300, seed=0):
    """Model M vs the full URDF gravity moment, random joints inside the URDF limits and any tool_roll."""
    rng = np.random.default_rng(seed)
    J = info['joints']
    err = []
    for _ in range(n):
        q = {k: rng.uniform(*J[k]['lim']) for k in ARM}
        th = to_model(q['shoulder'], q['elbow'], q['wrist_pitch'], info['delta'])
        mu, mm = urdf_M(J, info['links'], q), sp.M_of(th, P)
        err.append(mm - mu)
        # the TCP: model tip vs URDF tcp (x, z) relative to the shoulder axis
        poses = fk(J, q)
        R, p = poses['tool_roll_link']
        ax = J['tool_roll']['axis'] / np.linalg.norm(J['tool_roll']['axis'])
        tcp = poses['tcp'][1] - info['grip_shift'] * (R @ ax) - poses['shoulder_link'][1]
        tip = sp.positions(th, P)[1]['tip']
        assert np.allclose(plane(tcp), tip, atol=1e-6), (plane(tcp), tip)
        assert np.allclose(to_urdf(th, info['delta']), [q['shoulder'], q['elbow'], q['wrist_pitch']])
    err = np.abs(err)
    return float(err.max()), float(err.mean())


def main():
    P, info = params()
    print(f'URDF: {URDF}')
    for k in ('M_L1', 'M_L2', 'M_L3', 'm2'):
        print(f'  {k:5s} {P[k] * 1000:7.1f} g')
    for k in ('L1', 'L2', 'L3', 'r1', 'r2', 'r3', 'rm2'):
        print(f'  {k:5s} {P[k] * 1000:7.1f} mm')
    print(f"  delta {math.degrees(info['delta']):.2f} deg (grip point above the wrist link x axis); grip point "
          f"{(info['grip_shift'] + URDF_TCP_INSET) * 1000:.0f} mm inside the finger tips")
    print('  theta limits [deg]', np.round(np.degrees(P['q_min']), 1), '..', np.round(np.degrees(P['q_max']), 1))
    print('  centre of mass off the link line [mm]:', {k: round(v * 1000, 1) for k, v in info['off'].items()})
    mx, mean = urdf_check(P, info)
    print(f'model M vs full URDF gravity moment (300 poses, any tool_roll): max |err| {mx * 1000:.1f} mN m, '
          f'mean {mean * 1000:.1f} mN m; TCP and joint mapping exact')
    q0 = {'shoulder': 0.0, 'elbow': 0.0, 'wrist_pitch': 0.0}
    print(f"zero pose (L1 up, L2 + tool forward): URDF M {urdf_M(info['joints'], info['links'], q0):+.3f} N m")
    return P, info


if __name__ == '__main__':
    main()
