#!/usr/bin/env python3
"""팔 링크끼리 부딪히는 각도를 찾아 관절 한계를 좁힌다.

- 판정은 실제 형상(visual 메시)을 1 mm 복셀로 채워서 한다. 볼록 껍질은 ㄷ자 힌지 브라켓 안쪽을 메워
  모터와 겹친 것으로 잘못 나와서 쓰지 않는다.
- 관절 하나씩, 나머지 관절은 0에 두고 1° 단위로 돌린다 (조합 자세는 보지 않음).
- 관절 축에서 HUB_R(14 mm) 안의 부모-자식 겹침은 뺀다 (혼·혼 볼트는 자식과 같이 돌지만 메시는 모터 쪽).
- 두 링크의 겹친 복셀 수가 0자세 때보다 MARGIN개 넘게 늘면 충돌 (0자세의 접촉·체결부는 허용).
  충돌 직전 각도까지를 한계로 둔다 (URDF 한계보다 좁힐 때만).
- 몸체(base_link 등)와의 충돌은 보지 않는다. 팔 링크끼리만 본다.

사용: python3 collision_limits.py <urdf> [관절 ...]   (URDF를 그 자리에서 고침)
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
ARM = ['shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link', 'jaw_upper_link', 'jaw_lower_link']
JOINTS = ['elbow', 'wrist_pitch']
VOXEL = 0.001          # 1 mm
MARGIN = 5             # 0자세보다 이만큼(복셀 개수) 넘게 더 겹치면 충돌
HUB_R = 0.014          # 관절 축에서 이 반지름 안(혼·혼 볼트)의 부모-자식 겹침은 뺀다: 혼은 자식과 같이 도는데 메시는 모터(부모) 쪽
STEP = np.radians(1.0)


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rot(axis, q):
    a = np.asarray(axis) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


def vec(el, attr, default='0 0 0'):
    return np.array(list(map(float, (el.get(attr) if el is not None and el.get(attr) else default).split())))


def read_stl(p):
    b = Path(p).read_bytes()
    n = int(np.frombuffer(b, '<u4', 1, 80)[0])
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(b, dt, n, 84)['v'].astype(float)


class Voxels:
    """닫힌 메시를 z방향 광선 홀짝으로 채운 복셀 격자 (링크 좌표)"""

    def __init__(self, tri):
        lo = tri.reshape(-1, 3).min(0) - VOXEL
        hi = tri.reshape(-1, 3).max(0) + VOXEL
        self.lo = lo
        self.shape = np.ceil((hi - lo) / VOXEL).astype(int) + 1
        nx, ny, nz = self.shape
        hits = [[] for _ in range(nx * ny)]
        for t in tri:
            (x0, y0), (x1, y1) = t[:, :2].min(0), t[:, :2].max(0)
            ix = np.arange(int(np.ceil((x0 - lo[0]) / VOXEL - 0.5)), int(np.floor((x1 - lo[0]) / VOXEL - 0.5)) + 1)
            iy = np.arange(int(np.ceil((y0 - lo[1]) / VOXEL - 0.5)), int(np.floor((y1 - lo[1]) / VOXEL - 0.5)) + 1)
            if not len(ix) or not len(iy):
                continue
            X, Y = np.meshgrid(lo[0] + (ix + 0.5) * VOXEL, lo[1] + (iy + 0.5) * VOXEL, indexing='ij')
            a, b, c = t
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-18:
                continue
            l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
            l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
            l3 = 1 - l1 - l2
            inside = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            Z = l1 * a[2] + l2 * b[2] + l3 * c[2]
            for i, j in zip(*np.nonzero(inside)):
                hits[ix[i] * ny + iy[j]].append(Z[i, j])
        occ = np.zeros(self.shape, bool)
        for k, zs in enumerate(hits):
            if len(zs) < 2:
                continue
            zs = np.sort(zs)
            i, j = divmod(k, ny)
            for z0, z1 in zip(zs[0::2], zs[1::2]):
                k0 = int(np.ceil((z0 - lo[2]) / VOXEL - 0.5))
                k1 = int(np.floor((z1 - lo[2]) / VOXEL - 0.5))
                occ[i, j, max(k0, 0):k1 + 1] = True
        self.occ = occ
        self.centers = lo + (np.argwhere(occ) + 0.5) * VOXEL

    def contains(self, pts):
        idx = np.floor((pts - self.lo) / VOXEL).astype(int)
        ok = ((idx >= 0) & (idx < self.shape)).all(1)
        out = np.zeros(len(pts), bool)
        out[ok] = self.occ[tuple(idx[ok].T)]
        return out


def load(urdf):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        joints[j.find('child').get('link')] = dict(
            name=j.get('name'), parent=j.find('parent').get('link'), type=j.get('type'),
            T=tf(vec(o, 'xyz'), vec(o, 'rpy')), axis=vec(j.find('axis'), 'xyz', '1 0 0'), limit=j.find('limit'))
    for child, j in joints.items():
        if j['parent'] in ARM and child in ARM and j['type'] in ('revolute', 'continuous'):
            HUBS[(j['parent'], child)] = j['axis'] / np.linalg.norm(j['axis'])
    vox = {}
    for link in root.findall('link'):
        name = link.get('name')
        if name not in ARM:
            continue
        vis = link.find('visual')
        mesh = vis.find('geometry/mesh').get('filename').replace('package://craft_description/', '')
        T = tf(vec(vis.find('origin'), 'xyz'), vec(vis.find('origin'), 'rpy'))
        tri = read_stl(PKG / mesh)
        tri = (tri.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]).reshape(-1, 3, 3)
        vox[name] = Voxels(tri)
    return joints, vox


def fk(joints, q):
    out = {}

    def get(link):
        if link in out:
            return out[link]
        j = joints[link]
        T = (np.eye(4) if j['parent'] not in ARM else get(j['parent'])) @ j['T']
        if j['type'] in ('revolute', 'continuous'):
            T = T @ axis_rot(j['axis'], q.get(j['name'], 0.0))
        elif j['type'] == 'prismatic':
            T[:3, 3] += T[:3, :3] @ (j['axis'] * q.get(j['name'], 0.0))
        out[link] = T
        return T
    for link in ARM:
        get(link)
    return out


HUBS = {}   # (부모, 자식): 자식 좌표의 관절 축 (load에서 채움)


def overlaps(vox, T):
    """링크 쌍마다 겹친 복셀 수 (부모-자식 쌍은 관절 축 둘레 HUB_R 안을 뺌)"""
    res = {}
    for i, a in enumerate(ARM):
        for b in ARM[i + 1:]:
            M = np.linalg.inv(T[b]) @ T[a]
            pts = vox[a].centers @ M[:3, :3].T + M[:3, 3]
            hit = vox[b].contains(pts)
            for key in ((a, b), (b, a)):
                if key in HUBS:        # 축은 자식(key[1]) 좌표, 원점 지남
                    child = key[1]
                    Mc = np.linalg.inv(T[child]) @ T[b]
                    q = pts @ Mc[:3, :3].T + Mc[:3, 3]
                    ax = HUBS[key]
                    r = np.linalg.norm(q - np.outer(q @ ax, ax), axis=1)
                    hit &= r > HUB_R
            res[(a, b)] = int(hit.sum())
    return res


def sweep(urdf, names=JOINTS):
    joints, vox = load(urdf)
    base = overlaps(vox, fk(joints, {}))
    print('0자세 겹친 복셀(체결·접촉부):', {f'{a}-{b}': n for (a, b), n in base.items() if n})
    result = {}
    for name in names:
        lim = next(j['limit'] for j in joints.values() if j['name'] == name)
        lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        found = []
        for sign, bound in ((+1, hi), (-1, lo)):
            q, last = 0.0, 0.0
            while abs(q + sign * STEP) <= abs(bound) + 1e-9:
                q += sign * STEP
                ov = overlaps(vox, fk(joints, {name: q}))
                hit = [(k, n - base[k]) for k, n in ov.items() if n - base[k] > MARGIN]
                if hit:
                    (a, b), n = max(hit, key=lambda h: h[1])
                    print(f'  {name} {np.degrees(q):+.0f}°: {a} ↔ {b} ({n} mm³ 더 겹침)')
                    break
                last = q
            else:
                last = bound
            found.append(last)
        result[name] = (max(lo, found[1]), min(hi, found[0]))
        print(f'{name}: URDF {np.degrees(lo):+.1f} ~ {np.degrees(hi):+.1f}° → 충돌 없는 범위 '
              f'{np.degrees(result[name][0]):+.1f} ~ {np.degrees(result[name][1]):+.1f}°')
    return result


def apply(urdf, result):
    text = Path(urdf).read_text()
    for name, (lo, hi) in result.items():
        text = re.sub(rf'(<joint name="{name}".*?<limit lower=")[^"]*(" upper=")[^"]*',
                      lambda m: f'{m.group(1)}{lo:.9g}{m.group(2)}{hi:.9g}', text, count=1, flags=re.S)
    Path(urdf).write_text(text)


if __name__ == '__main__':
    urdf = sys.argv[1]
    apply(urdf, sweep(urdf, sys.argv[2:] or JOINTS))
