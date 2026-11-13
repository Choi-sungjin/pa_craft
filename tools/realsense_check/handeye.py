#!/usr/bin/env python3
"""Wrist (gripper) D435 hand-eye calibration from AprilTag centres seen by both cameras.

  /usr/bin/python3 handeye.py capture [name]                 one sample: upper + gripper frames, the arm pose
  /usr/bin/python3 handeye.py reprocess                      redo the tag points of every saved sample
  ~/craft/venv-craft/bin/python3 handeye.py solve            fit the wrist camera mount, residual per sample

Per sample: every puck tag (tag36h11, OpenCV aruco) seen by BOTH cameras gives one point: its centre in base_link
from the upper cam (V307 pose, lens y +32.2 mm, yaw 1.5 deg, height 0.6229 m: puck_locate.camera_pose) and in the
wrist camera optical frame. The arm pose (raw motor degrees) comes from the newest teach.yaml entry (teacharm holds
the board), or from checkarm when nothing is teaching. Joint values go V75 (control) -> V307 URDF through a fitted
map (cached in v75_to_v307.json; LM origin anchored on the 2026-11-13 bigbro grasp); the solve then fits the
wrist_pitch_link -> camera transform (6 dof) so that FK(q) * X * p_cam matches p_base over all points.
Close the live camera windows first (they hold the cameras).
"""
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'demo', 'puck_pick'))
# capture runs on /usr/bin/python3 (box_range needs the system scipy), solve on venv-craft (torch): imports are lazy.
UPPER_SERIAL = '261322075477'

SAMPLES = os.path.join(HERE, 'out', 'handeye')
MAP = os.path.join(HERE, 'v75_to_v307.json')
RESULT = os.path.join(HERE, 'wrist_cam_extrinsic.json')
URDF7 = os.path.join(WS, 'src', 'craft_description', 'urdf', 'v307_full_robot.urdf')
N7 = ['lm_slide', 'shoulder', 'elbow', 'wrist_pitch', 'tool_roll']
GRIP_SERIAL = '261622073363'
_DET = None


def tags(prefix, upscale):
    """{id: (u, v)} tag centres in the colour image (subpixel corners, image upscaled for small tags)."""
    global _DET
    if _DET is None:
        p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.adaptiveThreshWinSizeMax = 53; p.minMarkerPerimeterRate = 0.01  # small, slanted tags (wrist cam missed them)
        _DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)
    g = cv2.cvtColor(cv2.imread(prefix + '_color.ppm'), cv2.COLOR_BGR2GRAY)
    if upscale > 1:
        g = cv2.resize(g, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
    c, ids, _ = _DET.detectMarkers(g)
    return {} if ids is None else {int(i): cc[0].mean(0) / upscale for cc, i in zip(c, ids.flatten())}


def cylinder_boxes(prefix):
    """YOLO boxes of cylinder pucks: a tag inside one is on a cylinder end face (cylinders carry tags only there), so a
    standing cylinder shows ONE tag from above and both cameras see the same point. A puck's id repeats on all its faces,
    so a prism tag seen by the two cameras may be two different faces."""
    import puck_locate as pl
    return [(x, y, w, h) for (x, y, w, h), c, p in pl.detect(cv2.imread(prefix + '_color.ppm')) if 'cylinder' in pl.CLASSES[c]]


def tag_points(prefix, upscale, cyl_only=False):
    """{id: xyz in the camera optical frame [m]} from the tag centre and the median depth around it."""
    import box_range as br
    d, K, _ = br.load(prefix)
    boxes = cylinder_boxes(prefix) if cyl_only else None
    out = {}
    for i, (u, v) in tags(prefix, upscale).items():
        if boxes is not None and not any(x <= u <= x + w and y <= v <= y + h for x, y, w, h in boxes):
            continue
        r = 3
        patch = d[max(int(v) - r, 0):int(v) + r + 1, max(int(u) - r, 0):int(u) + r + 1]
        if np.any(patch > 0.05):
            out[i] = np.linalg.inv(K) @ np.array([u, v, 1.0]) * float(np.median(patch[patch > 0.05]))
    return out


def arm_q75_ros(timeout=4.0):
    """V75 joint values [slide, shoulder, elbow, wrist_pitch, tool_roll] from /control/arm_joints (control running)."""
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except ImportError:
        return None
    rclpy.init(); n = rclpy.create_node('handeye_q'); got = []
    n.create_subscription(JointState, '/control/arm_joints', got.append, 10)
    t = time.time()
    while not got and time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()
    if not got:
        return None
    d = dict(zip(got[-1].name, got[-1].position))
    return [d[k] for k in ('slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint')]


def arm_raw():
    """Raw motor degrees LM..tool_roll: from the board (checkarm) when nothing holds it, else the newest teach.yaml
    entry (teacharm running: press Enter there first so that entry is this pose)."""
    import re
    import yaml
    r = subprocess.run(['bash', '-ic', 'checkarm'], capture_output=True, text=True, timeout=60)
    m = re.search(r'LM ([-\d.]+) deg\s+shoulder ([-\d.]+) deg\s+elbow ([-\d.]+) deg\s+wrist_pitch ([-\d.]+) deg\s+tool_roll ([-\d.]+) deg',
                  re.sub(r'\x1b\[[0-9;]*m', '', r.stdout + r.stderr))
    if m and float(m.group(1)) != 0.0:
        raw = [float(v) for v in m.groups()]
        print(f'arm pose from the board: raw {raw}')
        return raw, time.strftime('%Y-%m-%d %H:%M:%S')
    e = yaml.safe_load(open(os.path.join(WS, 'demo', 'puck_pick', 'teach.yaml')))[-1]
    age = time.time() - time.mktime(time.strptime(e['time'], '%Y-%m-%d %H:%M:%S'))
    print(f"arm pose from teach.yaml: {e['name']} {e['time']} raw {e['raw_deg'][:5]}" + (f'  !! {age / 60:.0f} min old' if age > 300 else ''))
    if age > 300:
        raise SystemExit('no current arm pose: the board is busy and teach.yaml has nothing from the last 5 min (Enter in teacharm)')
    return [float(v) for v in e['raw_deg'][:5]], e['time']


# ---- kinematics (torch so the map and the solve share it) ----
try:
    import torch
except ImportError:  # capture on the system python
    torch = None

_root = ET.parse(URDF7).getroot()
_by_child = {j.find('child').get('link'): j for j in _root.findall('joint')}


def _chain(link):
    c = []
    while link in _by_child:
        c.append(_by_child[link]); link = _by_child[link].find('parent').get('link')
    return c[::-1]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def _origin(j):
    o = j.find('origin'); T = torch.eye(4, dtype=torch.float64)
    if o is not None:
        T[:3, :3] = torch.tensor(_rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()]))
        T[:3, 3] = torch.tensor([float(v) for v in o.get('xyz', '0 0 0').split()])
    return T


def _motion(ax, typ, v):
    T = torch.eye(4, dtype=torch.float64); a = torch.tensor(ax, dtype=torch.float64); a = a / a.norm()
    if typ == 'prismatic':
        T[:3, 3] = a * v; return T
    Kx = torch.stack([torch.stack([torch.zeros((), dtype=torch.float64), -a[2], a[1]]), torch.stack([a[2], torch.zeros((), dtype=torch.float64), -a[0]]),
                      torch.stack([-a[1], a[0], torch.zeros((), dtype=torch.float64)])])
    T[:3, :3] = torch.eye(3, dtype=torch.float64) + torch.sin(v) * Kx + (1 - torch.cos(v)) * Kx @ Kx
    return T


_CH = {}


def fk7(link, q):
    if link not in _CH:
        _CH[link] = _chain(link)
    T = torch.eye(4, dtype=torch.float64)
    for j in _CH[link]:
        T = T @ _origin(j)
        if j.get('type') != 'fixed':
            v = q[N7.index(j.get('name'))] if j.get('name') in N7 else torch.zeros((), dtype=torch.float64)
            T = T @ _motion([float(x) for x in j.find('axis').get('xyz').split()], j.get('type'), v)
    return T


def v75_map():
    """V75 joint values -> V307 URDF joint values: slide sign -1, others offset only (fit + LM anchor, cached)."""
    if os.path.exists(MAP):
        return json.load(open(MAP))['p']
    import plan
    import yaml
    ch = plan.Chain(plan.URDF); N5 = ch.NAMES
    rng = np.random.default_rng(3)
    lo = np.array([ch.limits(n)[0] for n in N5]); hi = np.array([ch.limits(n)[1] for n in N5])
    Q = lo + (hi - lo) * rng.random((150, 5))
    T75 = torch.tensor(np.array([ch.fk(dict(zip(N5, q)))[:3, 3] for q in Q])); Qt = torch.tensor(Q)
    best = None
    for _ in range(5):
        p = torch.tensor(np.concatenate([[rng.uniform(-0.1, 0.1)], rng.uniform(-math.pi, math.pi, 3), rng.uniform(-0.3, 0.3, 3)]), requires_grad=True)
        opt = torch.optim.LBFGS([p], max_iter=200, line_search_fn='strong_wolfe')

        def loss():
            opt.zero_grad(); L = 0
            for i in range(len(Q)):
                q = torch.stack([-Qt[i, 0] + p[0], Qt[i, 1] + p[1], Qt[i, 2] + p[2], Qt[i, 3] + p[3], Qt[i, 4]])
                L = L + ((fk7('tcp', q)[:3, 3] - torch.stack([-T75[i, 1] + p[4], T75[i, 0] + p[5], T75[i, 2] + p[6]])) ** 2).sum()
            L = L / len(Q); L.backward(); return L
        for _ in range(2):
            opt.step(loss)
        rms = math.sqrt(float(loss()) / 3) * 1000
        if best is None or rms < best[0]:
            best = (rms, p.detach().clone())
    rms, p = best
    # LM origin: the 2026-11-13 bigbro grasp held the puck the upper cam saw at y -41 mm
    out = os.path.join(SAMPLES, 'anchor.yaml'); os.makedirs(SAMPLES, exist_ok=True)
    subprocess.run(['/usr/bin/python3', os.path.join(WS, 'demo', 'puck_pick', 'plan.py'), '--puck-ahead', '0.109', '--puck-x', '-0.041', '--out', out], capture_output=True)
    g = {w['name']: w['joints'] for w in yaml.safe_load(open(out))['waypoints']}['grasp']
    q = torch.tensor([g[n] for n in N5])
    p[0] += -0.041 - float(fk7('tcp', torch.stack([-q[0] + p[0], q[1] + p[1], q[2] + p[2], q[3] + p[3], q[4]]))[1, 3])
    json.dump({'p': p.tolist(), 'tcp_rms_mm': rms, 'note': 'q307 = [-slide + p0, sh + p1, el + p2, wr + p3, roll]'}, open(MAP, 'w'), indent=1)
    print(f'V75 -> V307 map fitted: TCP RMS {rms:.1f} mm, cached in {MAP}')
    return p.tolist()


def q307_from_raw(raw, q75=None):
    if q75 is None:
        import teach
        q75 = teach.raw_to_target(list(raw) + [0.0])[:5]
    p = v75_map()
    return torch.tensor([-q75[0] + p[0], q75[1] + p[1], q75[2] + p[2], q75[3] + p[3], q75[4]], dtype=torch.float64)


def capture(name):
    import puck_locate as pl
    os.makedirs(SAMPLES, exist_ok=True)
    pre = os.path.join(SAMPLES, f'{name}_{time.strftime("%H%M%S")}')
    subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), pre + '_up', '1280', '720', UPPER_SERIAL], check=True)
    subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), pre + '_grip', '640', '360', GRIP_SERIAL], check=True)
    q75 = arm_q75_ros()
    if q75 is not None:
        print('arm pose from /control/arm_joints (V75):', [round(v, 4) for v in q75])
        raw, when = None, time.strftime('%Y-%m-%d %H:%M:%S')
    else:
        raw, when = arm_raw()
    process(pre, raw, when, q75)


def process(pre, raw, when, q75=None):
    """Tag points of one sample (saved frames) -> <pre>.json. Re-run by 'reprocess' after a detector change."""
    import puck_locate as pl
    R, t, zf, _ = pl.camera_pose()
    up = {i: (R @ p + t) for i, p in tag_points(pre + '_up', 3, cyl_only=True).items()}
    gr = tag_points(pre + '_grip', 2, cyl_only=True)
    both = sorted(set(up) & set(gr))
    print(f'upper cam tags {sorted(up)}, gripper cam tags {sorted(gr)} -> shared {both}')
    json.dump({'raw': raw, 'q75': q75, 'teach_time': when, 'base': {i: up[i].tolist() for i in both}, 'cam': {i: gr[i].tolist() for i in both},
               'prefix': pre}, open(pre + '.json', 'w'), indent=1)
    if not both:
        print('no tag seen by both cameras: move the arm or turn the puck so one face shows to both')


def _X(v):
    """6-vector (rotation vector, translation) -> 4x4."""
    th = v[:3].norm() + 1e-12; a = v[:3] / th
    T = _motion([0, 0, 1], 'revolute', torch.zeros((), dtype=torch.float64))
    Kx = torch.stack([torch.stack([torch.zeros((), dtype=torch.float64), -a[2], a[1]]), torch.stack([a[2], torch.zeros((), dtype=torch.float64), -a[0]]),
                      torch.stack([-a[1], a[0], torch.zeros((), dtype=torch.float64)])])
    T = torch.eye(4, dtype=torch.float64).clone()
    T[:3, :3] = torch.eye(3, dtype=torch.float64) + torch.sin(th) * Kx + (1 - torch.cos(th)) * Kx @ Kx
    T[:3, 3] = v[3:]
    return T


def solve():
    S = [json.load(open(os.path.join(SAMPLES, f))) for f in sorted(os.listdir(SAMPLES)) if f.endswith('.json') and f != 'anchor.json']
    S = [s for s in S if s.get('base')]
    print(f'{len(S)} samples, {sum(len(s["base"]) for s in S)} points')
    if sum(len(s['base']) for s in S) < 4:
        print('need at least 4 shared tag points from 3 or more arm poses'); return
    W = [fk7('wrist_pitch_link', q307_from_raw(s['raw'], s.get('q75'))).detach() for s in S]
    # start: the URDF wrist camera mount
    T0 = (torch.linalg.inv(fk7('wrist_pitch_link', torch.zeros(5, dtype=torch.float64))) @ fk7('camera_wrist_color_optical_frame', torch.zeros(5, dtype=torch.float64))).detach()
    R0 = T0[:3, :3].numpy(); rv, _ = cv2.Rodrigues(R0)
    v = torch.tensor(np.concatenate([rv.flatten(), T0[:3, 3].numpy()]), requires_grad=True)
    opt = torch.optim.LBFGS([v], max_iter=500, line_search_fn='strong_wolfe')

    def resid(v):
        X = _X(v); r = []
        for s, Wi in zip(S, W):
            for i in s['base']:
                pc = torch.tensor(s['cam'][i] + [1.0], dtype=torch.float64)
                r.append((Wi @ X @ pc)[:3] - torch.tensor(s['base'][i], dtype=torch.float64))
        return torch.stack(r)

    def loss():
        opt.zero_grad(); L = (resid(v) ** 2).sum(1).mean(); L.backward(); return L
    before = float((resid(v) ** 2).sum(1).mean().sqrt()) * 1000
    for _ in range(5):
        opt.step(loss)
    r = resid(v).detach().numpy() * 1000
    X = _X(v.detach()).numpy()
    print(f'point error RMS: URDF mount {before:.1f} mm -> fitted {math.sqrt((r ** 2).sum(1).mean()):.1f} mm')
    for s in S:
        print(f"  {os.path.basename(s['prefix'])}: tags {list(s['base'])}")
    print('per point error [mm]:', ' '.join(f'{np.linalg.norm(e):.1f}' for e in r))
    d = X[:3, 3] - T0[:3, 3].numpy()
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(X[:3, :3].T @ T0[:3, :3].numpy()) - 1) / 2))))
    print(f'fitted mount vs URDF: moved {np.round(d * 1000, 1)} mm, turned {ang:.1f} deg')
    json.dump({'wrist_pitch_link_T_camera': X.tolist(), 'rms_mm': float(math.sqrt((r ** 2).sum(1).mean())), 'samples': len(S)}, open(RESULT, 'w'), indent=1)
    print(f'saved {RESULT}')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'capture':
        capture(sys.argv[2] if len(sys.argv) > 2 else 'he')
    elif cmd == 'reprocess':
        for f in sorted(os.listdir(SAMPLES)):
            if f.endswith('.json') and f != 'anchor.json':
                j = json.load(open(os.path.join(SAMPLES, f)))
                process(j['prefix'], j['raw'], j['teach_time'], j.get('q75'))
    elif cmd == 'solve':
        solve()
    else:
        print(__doc__)
