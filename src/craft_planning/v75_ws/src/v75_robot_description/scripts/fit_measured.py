#!/usr/bin/env python3
"""Fit the CAD meshes to hand-measured lengths (2026-10-24, arm remeasured 2026-10-26). Reads the original meshes from git
(ORIG_REV), so rerunning after editing MEASURED gives the same result. Writes meshes/ in place.

Jaws: the finger shank is stretched so the measured horn-to-fingertip length holds.
Arm links: the motor ends stay rigid and only the middle of each link (BAND of its axis-to-axis
length) is stretched along the link, keeping the joint direction (so joint offsets stay valid).
LM: the LM assembly (plate, motor, belt, idler, cable chain) is mapped in X onto the measured
plate span; the rail is then refit to its measured length, centred on the plate as in CAD.
The URDF joint origins are edited by hand to the values this prints.
Run with system python: /usr/bin/python3 scripts/fit_measured.py
"""
import io
import struct
import subprocess
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
ORIG_REV = 'f909f20'  # last commit with the untouched CAD meshes
BAND = (0.35, 0.65)

MEASURED = dict(  # [m]
    waffle_left_x=-0.0046,           # CAD waffle spans x -4.6..262.6 mm (267.1 mm, measured 26.7 cm)
    plate_start_from_waffle=-0.007,  # LM plate overhangs the waffle's LM-motor (-X) edge by 0.7 cm
    plate_length=0.257,
    rail_length=0.168,
    shoulder_elbow=0.187,            # 2026-10-26 remeasure (was 0.190)
    elbow_wrist=0.095,               # 2026-10-26 remeasure (was 0.092)
    wrist_tool_horn=0.078,           # 2026-10-26 remeasure (was 0.055)
    horn_fingertip=0.133,            # tool-roll horn face -> open jaw fingertip: horn->gripper 48 + gripper 85
    fingertip_tcp=0.015,             # TCP mid-way along the 30 mm usable grip length from the fingertip (was 36 mm)
)

# CAD values the measurements replace.
SHOULDER_ELBOW = np.array([0.0, 0.0764664888983, 0.171746546037])
ELBOW_WRIST = np.array([0.0, 0.0924, 0.0220])
WRIST_TOOL = np.array([0.0, 0.056, 0.0])
CAD_PLATE = (-0.0016, 0.2580)
CAD_RAIL = (0.0290, 0.2290)
CAD_FINGERTIP = 0.1253  # jaw_upper mesh tip, tool_roll_link Y
JAW_BAND = (0.045, 0.075)  # finger shank between the gripper body and the pads (pads start at 0.081)
LIDAR_BOX = ((0.100, 0.160), (0.030, 0.080), (-1.0, 0.056))  # front LiDAR, not part of the LM


def read_stl(data):
    if data[:5] == b'solid' and b'facet' in data[:300]:
        v = [list(map(float, l.split()[1:4])) for l in data.decode().splitlines()
             if l.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return a['v'].astype(float)


def write_stl(path, tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    buf = io.BytesIO()
    buf.write(b'fit_measured.py'.ljust(80, b' '))
    buf.write(struct.pack('<I', len(tris)))
    buf.write(rec.tobytes())
    path.write_bytes(buf.getvalue())


def original(name):
    rel = f'src/craft_planning/v75_ws/src/v75_robot_description/meshes/{name}.stl'
    return read_stl(subprocess.run(['git', 'show', f'{ORIG_REV}:{rel}'], cwd=PKG, check=True,
                                   capture_output=True).stdout)


def components(tris):
    v = np.round(tris.reshape(-1, 3), 5)
    _, inv = np.unique(v, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    parent = np.arange(inv.max() + 1)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a, b, c in inv:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    roots = np.array([find(i) for i in inv[:, 0]])
    return [np.flatnonzero(roots == r) for r in np.unique(roots)]


def stretch(tris, vec, new_len):
    """Stretch the middle BAND of a link along vec (child joint origin) to new_len."""
    length = np.linalg.norm(vec)
    d = vec / length
    t = tris @ d
    t1, t2 = BAND[0] * length, BAND[1] * length
    k = (t2 - t1 + new_len - length) / (t2 - t1)
    new_t = np.where(t <= t1, t, np.where(t >= t2, t + new_len - length, t1 + (t - t1) * k))
    return tris + (new_t - t)[..., None] * d, d * new_len


def x_map(x, src, dst):
    return dst[0] + (x - src[0]) * (dst[1] - dst[0]) / (src[1] - src[0])


def is_lm_part(lo, hi):
    if lo[0] < -0.015 or hi[0] > 0.275 or hi[1] > 0.136 or lo[2] < 0.025:
        return False
    in_lidar = all(LIDAR_BOX[i][0] <= lo[i] and hi[i] <= LIDAR_BOX[i][1] for i in range(3))
    return not in_lidar


def main():
    m = MEASURED
    origins = {}
    for names, vec, new_len, joint in (
            (['shoulder_link_orange', 'shoulder_link_black', 'shoulder_link_green'], SHOULDER_ELBOW,
             m['shoulder_elbow'], 'elbow_joint'),
            (['elbow_link_black'], ELBOW_WRIST, m['elbow_wrist'], 'wrist_pitch_joint'),
            (['wrist_pitch_link_orange', 'wrist_pitch_link_black'], WRIST_TOOL, m['wrist_tool_horn'],
             'tool_roll_joint')):
        for name in names:
            tris, origin = stretch(original(name), vec, new_len)
            write_stl(PKG / 'meshes' / f'{name}.stl', tris)
        origins[joint] = origin

    # Fingers: stretch the shank so the pads, tips and TCP move out together; body and camera stay.
    extra = m['horn_fingertip'] - CAD_FINGERTIP
    for name in ('jaw_upper_link_black', 'jaw_upper_link_orange', 'jaw_lower_link_black', 'jaw_lower_link_orange'):
        tris = original(name)
        y = tris[..., 1]
        t1, t2 = JAW_BAND
        tris[..., 1] = np.where(y <= t1, y, np.where(y >= t2, y + extra, t1 + (y - t1) * (t2 - t1 + extra) / (t2 - t1)))
        write_stl(PKG / 'meshes' / f'{name}.stl', tris)
    origins['tool_tcp_joint_y'] = m['horn_fingertip'] - m['fingertip_tcp']

    plate = (m['waffle_left_x'] + m['plate_start_from_waffle'],
             m['waffle_left_x'] + m['plate_start_from_waffle'] + m['plate_length'])
    centre = (plate[0] + plate[1]) / 2
    rail = (centre - m['rail_length'] / 2, centre + m['rail_length'] / 2)
    for name in ('base_link_black', 'base_link_green', 'base_link_orange'):
        tris = original(name)
        for idx in components(tris):
            part = tris[idx]
            lo, hi = part.reshape(-1, 3).min(0), part.reshape(-1, 3).max(0)
            if not is_lm_part(lo, hi):
                continue
            part[..., 0] = x_map(part[..., 0], CAD_PLATE, plate)
            is_rail = (name == 'base_link_green' and abs(lo[0] - CAD_RAIL[0]) < 0.002
                       and abs(hi[0] - CAD_RAIL[1]) < 0.002)
            if is_rail:
                part[..., 0] = x_map(part[..., 0], (part[..., 0].min(), part[..., 0].max()), rail)
            tris[idx] = part
        write_stl(PKG / 'meshes' / f'{name}.stl', tris)
    origins['idler_joint_x'] = x_map(0.234, CAD_PLATE, plate)

    print(f'plate x {plate[0]:.4f}..{plate[1]:.4f}  rail x {rail[0]:.4f}..{rail[1]:.4f}')
    for k, v in origins.items():
        print(k, np.round(v, 6))


if __name__ == '__main__':
    main()
