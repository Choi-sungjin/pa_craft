#!/usr/bin/env python3
"""V307 몸체 + ROBOTIS OpenMANIPULATOR-X 팔 링크 URDF를 '추가'로 만든다 (원본은 그대로).

- shoulder_link, elbow_link: OMX link3(LINK FRAME LONG + XM430), link4(LINK FRAME SHORT + XM430)
  메시와 질량·관성(ROBOTIS open_manipulator, Apache-2.0)으로 바꾼다.
  CRAFT 팔은 0자세에서 앞팔이 -x로 가므로 OMX 메시를 z축 180° 돌려 붙인다.
  shoulder→elbow = (-0.024, 0, 0.128), elbow→wrist_pitch = (-0.124, 0, 0) (OMX joint3/joint4 원점)
- wrist_pitch_link(tool_roll 모터 포함), tool_roll_link, 집게, 몸체, LM, 바퀴는 V307 그대로.
- 카메라 삭제: camera_* 프레임을 빼고, 상단·손목 D435 몸체를 메시에서 지우고 질량 72 g(Intel 사양)을 뺀다.
  손목은 카메라 거치대도 지우고 PLA 부피 질량을 뺀다.

- 손목 브라켓: OMX link5의 옆판 2장 + 연결판으로 바꾸고, tool_roll 모터·관절·집게·TCP를 연결판에 붙게 당긴다.
  질량 = link5 143.3 g - XM430 82 g - 집게 레일(부피 x 1.24). 지운 우리 브라켓은 PLA로 보고 뺀다.

사용: python3 make_robotis_arm_urdf.py <omx 메시 폴더(link3.stl, link4.stl, link5.stl)>
"""
import re
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
SRC = PKG / 'urdf/v307_full_robot.urdf'
OUT = PKG / 'urdf/v307_robotis_arm.urdf'
SUB = 'robotis_arm'
D435_KG = 0.072
# D435 몸체 바운딩 박스 (링크 좌표, m). 메시 연결 성분 중 이 상자 안에 다 들어가는 것을 지운다
D435_BOX = {'base_link': ([-0.0885, -0.046, 0.584], [-0.054, 0.046, 0.619]),
            'wrist_pitch_link': ([-0.0731, 0.097, -0.0286], [-0.0346, 0.1303, 0.0633])}
# 손목 카메라 거치대(+y 쪽 브래킷·연결 암·볼트): y 하한이 이 값 이상인 성분을 지운다. PLA 질량을 부피로 빼다
MOUNT_MIN_Y = {'wrist_pitch_link': 0.035}
PLA_KG_M3 = 1240 * 0.85  # LINKS.json과 같은 가정 (PLA 1.24, 채움 70% 추정 x 0.85)

OMX = {  # OMX URDF 그대로 (원점: 그 링크의 관절)
    'shoulder_link': dict(mesh='link3', mass=1.3850917e-01, com=(1.0308393e-02, 3.7743363e-04, 1.0170197e-01),
                          I=(3.3055381e-04, -9.7940978e-08, -3.8505711e-05, 3.4290447e-04, -1.5717516e-06, 6.0346498e-05)),
    'elbow_link': dict(mesh='link4', mass=1.3274562e-01, com=(9.0909590e-02, 3.8929816e-04, 2.2413279e-04),
                       I=(3.0654178e-05, -1.2764155e-06, -2.6874417e-07, 2.4230292e-04, 1.1559550e-08, 2.5155057e-04)),
}
JOINT_XYZ = {'elbow': '-0.024 0 0.128', 'wrist_pitch': '-0.124 0 0'}
FLIP_ELBOW = False  # 엘보 모터만 pitch 180° 뒤집기 (L1 24 mm 꺾임과 안 맞아 끔, 대신 FLIP_UPPER)
# 구간마다 링크(프레임)·브라켓 경계 [m], 관절 원점 방향 좌표 s (0 = 부모 관절, 끝 = 이 관절).
# box_reach가 <urdf 이름>.segments.json을 읽어 링크 길이·브라켓 길이 슬라이더를 따로 띄운다.
SEGMENTS = {
    'elbow': ('L1 어깨-팔꿈치', [(0.0276, 'bracket', '어깨 힌지 브라켓'), (0.1200, 'link', 'LINK FRAME(LONG)'),
                                  (None, 'bracket', '엘보 모터 자리')]),
    'wrist_pitch': ('L2 팔꿈치-손목', [(0.0273, 'bracket', '엘보 힌지 브라켓'), (0.0950, 'link', 'LINK FRAME(SHORT)'),
                                     (None, 'bracket', '손목 모터 자리')]),
    'tool_roll': ('L3 손목-비틀기', [(0.0280, 'bracket', 'OMX 손목 브라켓'), (None, 'link', 'tool_roll 모터 오프셋')]),
    'tcp_joint': ('L4 비틀기-TCP', [(0.0490, 'bracket', 'roll-집게 연결 브라켓'), (None, 'link', 'OMX 집게')]),
}
FLIP_UPPER = True   # 어깨 관절 위쪽 전체를 어깨 수직축 기준 180° 돌림: 어깨 모터·축·0점·한계는 그대로, 앞팔이 앞(+x)
UPPER_LINKS = ['shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link', 'jaw_upper_link', 'jaw_lower_link']
UPPER_JOINTS = ['elbow', 'wrist_pitch', 'tool_roll', 'jaw_upper', 'jaw_lower', 'tcp_joint']
# 손목 브라켓 → OMX link5의 옆판 2장 + 모터 연결판 (+ 볼트). OMX 모터·집게 레일은 쓰지 않는다
OWN_BRACKET_MAX_X = -0.041   # 손목 메시에서 x 상한이 이보다 큰 성분 = 우리 옆판 2장 + 연결판 + 옆판 볼트 → 지움
OMX_LINK5_KG = 1.4327573e-01  # OMX URDF link5 전체
XM430_KG = 0.082              # XM430-W350 사양
OMX_RAIL_KG_M3 = 1240         # OMX 집게 레일 질량 추정용 (플라스틱 가정, 부피 x 1.24 g/cm3)

HULL_LINKS = ('shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link', 'jaw_upper_link', 'jaw_lower_link')

DT = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])


def read_stl(p):
    b = Path(p).read_bytes()
    n = int(np.frombuffer(b, '<u4', 1, 80)[0])
    return np.frombuffer(b, DT, n, 84)['v'].astype(float)


def write_stl(p, tri):
    tri = np.asarray(tri, np.float32)
    a = np.zeros(len(tri), DT)
    a['v'] = tri
    nr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(nr, axis=1, keepdims=True)
    ln[ln == 0] = 1
    a['n'] = nr / ln
    Path(p).parent.mkdir(parents=True, exist_ok=True)
    Path(p).write_bytes(b'\0' * 80 + np.uint32(len(tri)).tobytes() + a.tobytes())


def components(tri):
    _, idx = np.unique(np.round(tri.reshape(-1, 3), 6), axis=0, return_inverse=True)
    idx = idx.reshape(-1, 3)
    par = np.arange(idx.max() + 1)

    def find(x):
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x
    for a, b, c in idx:
        r = find(a)
        par[find(b)] = r
        par[find(c)] = r
    return np.array([find(i) for i in idx[:, 0]])


def drop_box(tri, lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    comp = components(tri)
    keep = np.ones(len(tri), bool)
    for k in np.unique(comp):
        m = comp == k
        p = tri[m].reshape(-1, 3)
        if (p.min(0) >= lo).all() and (p.max(0) <= hi).all():
            keep[m] = False
    return tri[keep], (~keep).sum()


def drop_mount(tri, min_y):
    comp = components(tri)
    keep = np.ones(len(tri), bool)
    for k in np.unique(comp):
        m = comp == k
        if tri[m].reshape(-1, 3)[:, 1].min() >= min_y:
            keep[m] = False
    return tri[keep], tri[~keep]


def solid(tri, density):
    """닫힌 메시를 균일 밀도 고체로 보고 (질량, 무게중심, 무게중심 기준 관성)을 낸다."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    det = np.einsum('ij,ij->i', a, np.cross(b, c))
    vol = det.sum() / 6
    sgn = np.sign(vol) or 1.0
    m = abs(vol) * density
    com = (det[:, None] * (a + b + c)).sum(0) / 24 / vol
    A = np.stack([a, b, c], axis=2)  # 열 = 꼭짓점
    canon = (np.ones((3, 3)) + np.eye(3)) / 120
    C = np.einsum('n,nij,jk,nlk->il', det, A, canon, A) * sgn * density  # 원점 기준 2차 모멘트
    I_o = np.trace(C) * np.eye(3) - C
    I_c = I_o - m * (com @ com * np.eye(3) - np.outer(com, com))
    return m, com, I_c


def box_inertia(m, size):
    x, y, z = size
    return m / 12 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])


def convex_hull(pts):
    from scipy.spatial import ConvexHull
    h = ConvexHull(pts)
    tri = pts[h.simplices]
    c = pts[h.vertices].mean(0)
    nr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    flip = np.einsum('ij,ij->i', nr, tri[:, 0] - c) < 0   # 바깥을 보게
    tri[flip] = tri[flip][:, ::-1]
    # 조각끼리 꼭짓점을 공유하면 읽을 때 한 덩어리로 합쳐지므로 중심 쪽으로 0.1 mm 줄인다
    d = tri - c
    n = np.linalg.norm(d, axis=2, keepdims=True)
    return c + d * np.maximum(n - 1e-4, 0) / np.where(n == 0, 1, n)


def hull_vol(pts):
    from scipy.spatial import ConvexHull
    try:
        return ConvexHull(pts).volume
    except Exception:
        return 0.0


def decompose(part, ratio=1.3, max_pieces=12, min_tris=24):
    """오목한 부품(ㄷ자 브라켓 등)을 볼록 조각 여러 개로 근사한다.
    조각들의 볼록 껍질 부피 합이 부품 부피의 ratio배 이하가 될 때까지,
    껍질이 가장 큰 조각을 삼각형 중심 2-means로 둘로 나눈다."""
    V = solid(part, 1.0)[0]
    pieces = [part]
    rng = np.random.default_rng(0)
    while len(pieces) < max_pieces:
        vols = [hull_vol(p.reshape(-1, 3)) for p in pieces]
        if sum(vols) <= ratio * V:
            break
        order = np.argsort(vols)[::-1]
        for i in order:
            p = pieces[i]
            if len(p) < 2 * min_tris:
                continue
            c = p.mean(1)
            # 가장 긴 축 방향 양 끝에서 시작하는 2-means
            ax = np.linalg.svd(c - c.mean(0), full_matrices=False)[2][0]
            proj = (c - c.mean(0)) @ ax
            cent = np.array([c[proj.argmin()], c[proj.argmax()]])
            for _ in range(20):
                lab = np.linalg.norm(c[:, None] - cent[None], axis=2).argmin(1)
                if lab.min() == lab.max():
                    break
                cent = np.array([c[lab == 0].mean(0), c[lab == 1].mean(0)])
            if min((lab == 0).sum(), (lab == 1).sum()) < min_tris:
                continue
            pieces[i:i + 1] = [p[lab == 0], p[lab == 1]]
            break
        else:
            break
    return [p.reshape(-1, 3) for p in pieces]


def _rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _mat_rpy(R):
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) < 1e-9:
        return 0.0, p, np.arctan2(-R[0, 1], R[1, 1])
    return np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])


def flip_upper(text):
    """어깨 관절 위쪽(shoulder_link 이하)을 어깨 관절 수직축 기준 180° 돌린 것과 같게 고친다.
    관절 원점에 회전을 넣지 않고(box_reach가 관절 rpy를 안 읽음), 각 링크 좌표 안의 값을
    Rz(180°) = diag(-1,-1,1)로 바꾼다: 메시(visual/collision 원점), 무게중심·관성, 아래 관절 원점·축·rpy.
    어깨 관절 자체(원점, 축 +y, 한계)는 그대로라 어깨 모터 기준 방향은 바뀌지 않는다."""
    Rz = np.diag([-1.0, -1.0, 1.0])

    def fix_origin(m):  # <origin xyz rpy> → Rz 적용
        xyz = np.array(list(map(float, m.group(1).split())))
        R = _rpy_mat(*map(float, m.group(2).split()))
        return f'<origin xyz="{fmt(Rz @ xyz)}" rpy="{fmt(_mat_rpy(Rz @ R @ Rz.T if m.group(3) else Rz @ R))}"/>'

    for link in UPPER_LINKS:
        blk = re.search(rf'<link name="{link}">.*?</link>', text, re.S).group(0)
        new = blk
        # 메시: 정점 v → Rz (R v + p)
        for tag in ('visual', 'collision'):
            new = re.sub(rf'(<{tag}>)<origin xyz="([^"]*)" rpy="([^"]*)"/>',
                         lambda m: m.group(1) + fix_origin(type('M', (), {'group': lambda self, i: (None, m.group(2), m.group(3), None)[i]})()),
                         new, count=1)
        # 관성: 무게중심 xy 부호, 관성 ixz·iyz 부호
        m = re.search(r'<inertial><origin xyz="([^"]*)"', new)
        com = np.array(list(map(float, m.group(1).split())))
        new = new.replace(m.group(0), f'<inertial><origin xyz="{fmt(Rz @ com)}"', 1)
        for k in ('ixz', 'iyz'):
            new = re.sub(rf'{k}="([^"]*)"', lambda mm: f'{k}="{-float(mm.group(1)):.9g}"', new, count=1)
        text = text.replace(blk, new)
    for joint in UPPER_JOINTS:
        blk = re.search(rf'<joint name="{joint}".*?(?:</joint>|/>\s*$)', text, re.S | re.M).group(0)
        new = re.sub(r'<origin xyz="([^"]*)" rpy="([^"]*)"/>',
                     lambda m: fix_origin(type('M', (), {'group': lambda self, i: (None, m.group(1), m.group(2), True)[i]})()),
                     blk, count=1)
        new = re.sub(r'<axis xyz="([^"]*)"/>', lambda m: f'<axis xyz="{fmt(Rz @ np.array(list(map(float, m.group(1).split()))))}"/>',
                     new, count=1)
        text = text.replace(blk, new)
    return text


def write_segments(text):
    import json
    out = {}
    for joint, (label, parts) in SEGMENTS.items():
        xyz = re.search(rf'<joint name="{joint}".*?<origin xyz="([^"]*)"', text, re.S).group(1)
        L = float(np.linalg.norm(list(map(float, xyz.split()))))
        knots, kinds, names = [0.0], [], []
        for end, kind, name in parts:
            knots.append(L if end is None else end)
            kinds.append(kind)
            names.append(name)
        out[joint] = {'label': label, 'knots': knots, 'kinds': kinds, 'names': names}
        lk = sum(b - a for a, b, k in zip(knots, knots[1:], kinds) if k == 'link')
        print(f'{label}: 링크 {lk * 1000:.1f} + 브라켓 {(L - lk) * 1000:.1f} = {L * 1000:.1f} mm')
    f = OUT.with_suffix('.segments.json')
    f.write_text(json.dumps(out, ensure_ascii=False, indent=1))


def rot_z180(com, I):
    x, y, z = com
    ixx, ixy, ixz, iyy, iyz, izz = I
    return (-x, -y, z), (ixx, ixy, -ixz, iyy, -iyz, izz)


def fmt(v):
    return ' '.join(f'{x:.9g}' for x in v)


def inertial(mass, com, I):
    ixx, ixy, ixz, iyy, iyz, izz = I
    return (f'<inertial><origin xyz="{fmt(com)}" rpy="0 0 0"/><mass value="{mass:.6f}"/>'
            f'<inertia ixx="{ixx:.9g}" ixy="{ixy:.9g}" ixz="{ixz:.9g}" iyy="{iyy:.9g}" iyz="{iyz:.9g}" izz="{izz:.9g}"/></inertial>')


def remove_body(block, m_rm, c_rm, I_rm):
    """inertial 블록에서 강체(질량 m_rm, 무게중심 c_rm, 그 점 기준 관성 I_rm)를 뺀다 (평행축 정리)."""
    mass = float(re.search(r'<mass value="([^"]+)"', block).group(1))
    com = np.array(list(map(float, re.search(r'<inertial><origin xyz="([^"]+)"', block).group(1).split())))
    g = {k: float(re.search(rf'{k}="([^"]+)"', block).group(1)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    I = np.array([[g['ixx'], g['ixy'], g['ixz']], [g['ixy'], g['iyy'], g['iyz']], [g['ixz'], g['iyz'], g['izz']]])
    c_rm = np.array(c_rm)

    def steiner(m, d):
        return m * (d @ d * np.eye(3) - np.outer(d, d))
    I_o = I + steiner(mass, com) - (I_rm + steiner(m_rm, c_rm))   # 링크 원점 기준
    m2 = mass - m_rm
    c2 = (mass * com - m_rm * c_rm) / m2
    I2 = I_o - steiner(m2, c2)
    ev = np.sort(np.linalg.eigvalsh(I2))
    if ev[0] <= 0 or ev[0] + ev[1] < ev[2]:
        # 원본 관성이 지운 부품을 다른 재질/모양으로 잡았으면 빼고 남은 값이 물리적으로 불가능해진다.
        # 그때는 원본 관성(무게중심 기준)을 질량 비율로 줄여 쓴다. 토크 계산은 질량·무게중심만 쓴다
        print(f'  관성 빼기 결과가 물리적으로 불가능 → 원본 관성 x {m2 / mass:.3f}로 대신함')
        I2 = I * (m2 / mass)
    return inertial(m2, c2, (I2[0, 0], I2[0, 1], I2[0, 2], I2[1, 1], I2[1, 2], I2[2, 2]))


def main(omx_dir):
    text = SRC.read_text()

    # 카메라 프레임 삭제
    text = re.sub(r'\s*<link name="camera_[^"]*"/>', '', text)
    text = re.sub(r'\s*<joint name="camera_[^"]*".*?</joint>', '', text)

    # D435 몸체 삭제 (메시 + 질량)
    for link, (lo, hi) in D435_BOX.items():
        mount = None
        for kind in ('visual', 'collision'):
            tri, n = drop_box(read_stl(PKG / f'meshes/{kind}/{link}.stl'), lo, hi)
            print(f'{kind}/{link}: D435 삼각형 {n}개 삭제')
            if link in MOUNT_MIN_Y:
                tri, gone = drop_mount(tri, MOUNT_MIN_Y[link])
                print(f'{kind}/{link}: 카메라 거치대 삼각형 {len(gone)}개 삭제')
                if kind == 'visual':
                    mount = gone
            write_stl(PKG / f'meshes/{kind}/{SUB}/{link}.stl', tri)
        text = text.replace(f'meshes/visual/{link}.stl', f'meshes/visual/{SUB}/{link}.stl')
        text = text.replace(f'meshes/collision/{link}.stl', f'meshes/collision/{SUB}/{link}.stl')
        c_rm = (np.array(lo) + np.array(hi)) / 2
        pat = re.compile(rf'(<link name="{link}">\s*)(<inertial>.*?</inertial>)', re.S)
        I_cam = box_inertia(D435_KG, np.array(hi) - np.array(lo))
        text = pat.sub(lambda m: m.group(1) + remove_body(m.group(2), D435_KG, c_rm, I_cam), text)
        if mount is not None:
            comp = components(mount)
            for k in np.unique(comp):
                kg, com, I_c = solid(mount[comp == k], PLA_KG_M3)
                print(f'  거치대 성분 {kg * 1000:.1f} g @ {np.round(com, 4)}')
                text = pat.sub(lambda m: m.group(1) + remove_body(m.group(2), kg, com, I_c), text)

    # 손목 브라켓 → OMX link5 브라켓, tool_roll 모터 이하를 연결판에 붙게 당김
    l5 = read_stl(Path(omx_dir) / 'link5.stl') * 1e-3
    comp = components(l5)
    bracket, rail = [], None
    for k in np.unique(comp):
        part = l5[comp == k]
        n, lo = len(part), part.reshape(-1, 3).min(0)
        if n == 3346:            # XM430 (x 28~68 mm) - 우리 tool_roll 모터를 씀. 이 자리에 맞춘다
            motor_face, motor_floor = lo[0], lo[2]
        elif n == 2532:          # 집게 레일
            rail = part
        elif lo[0] < 0.05:       # 옆판·연결판·연결 볼트 (x 54 mm 넘는 건 레일 볼트)
            bracket.append(part)
    bracket = np.concatenate(bracket) * np.array([-1, -1, 1])   # z축 180°
    rail_kg = solid(rail, OMX_RAIL_KG_M3)[0]
    bracket_kg = OMX_LINK5_KG - XM430_KG - rail_kg
    vol = solid(bracket, 1.0)[0]
    b_kg, b_com, b_I = solid(bracket, bracket_kg / vol)
    print(f'OMX 손목 브라켓 {b_kg * 1000:.1f} g = link5 {OMX_LINK5_KG * 1000:.1f} - XM430 {XM430_KG * 1000:.0f} '
          f'- 레일 {rail_kg * 1000:.1f} (부피 {vol * 1e6:.2f} cm3)')

    link = 'wrist_pitch_link'
    pat = re.compile(rf'(<link name="{link}">\s*)(<inertial>.*?</inertial>)', re.S)
    for kind in ('visual', 'collision'):
        f = PKG / f'meshes/{kind}/{SUB}/{link}.stl'
        tri = read_stl(f)
        comp = components(tri)
        own = np.zeros(len(tri), bool)
        for k in np.unique(comp):
            m = comp == k
            if tri[m].reshape(-1, 3)[:, 0].max() > OWN_BRACKET_MAX_X:
                own[m] = True
        keep = tri[~own]
        kc = components(keep)
        mot = keep[kc == max(np.unique(kc), key=lambda k: (kc == k).sum())].reshape(-1, 3)
        # 모터 앞끝 → 연결판 끝(x -28 mm), 모터 바닥 → OMX 모터 바닥(z)
        shift = np.array([-motor_face - mot[:, 0].max(), 0, motor_floor - mot[:, 2].min()])
        if kind == 'collision':   # 충돌 메시는 거칠어 bbox가 달라지므로 visual 이동량을 그대로 쓴다
            shift = vis_shift
        print(f'{kind}/{link}: 우리 브라켓 삼각형 {own.sum()}개 삭제, tool_roll 쪽 이동 {np.round(shift * 1000, 1)} mm')
        if kind == 'visual':   # 관절·질량은 visual 기준
            vis_keep, vis_shift = keep + shift, shift
        write_stl(f, np.concatenate([keep + shift, bracket]))
    shift = vis_shift

    # 손목 질량은 부품별로 다시 합산한다. 남는 건 tool_roll XM430 + 혼·볼트뿐이고,
    # 원본 577 g에서 빼는 방식은 원본의 재질 가정을 몰라 무게중심이 틀어진다.
    bodies = [(b_kg, b_com, b_I)]
    kc = components(vis_keep)
    big = max(np.unique(kc), key=lambda k: (kc == k).sum())
    for k in np.unique(kc):
        part = vis_keep[kc == k]
        if k == big:   # tool_roll 모터
            bodies.append(solid(part, XM430_KG / solid(part, 1.0)[0]))
        else:          # 혼·볼트 (강철)
            bodies.append(solid(part, 7850))
    M_ = sum(b[0] for b in bodies)
    C_ = sum(b[0] * b[1] for b in bodies) / M_
    I_ = sum(b[2] + b[0] * ((b[1] - C_) @ (b[1] - C_) * np.eye(3) - np.outer(b[1] - C_, b[1] - C_)) for b in bodies)
    print(f'{link}: 다시 합산 {M_ * 1000:.1f} g (OMX 브라켓 {b_kg * 1000:.1f} + XM430 {XM430_KG * 1000:.0f} + 혼·볼트)')
    new_in = inertial(M_, C_, (I_[0, 0], I_[0, 1], I_[0, 2], I_[1, 1], I_[1, 2], I_[2, 2]))
    text = pat.sub(lambda m: m.group(1) + new_in, text)
    text = re.sub(r'(<joint name="tool_roll"[^>]*>.*?<origin xyz=")([^"]*)',
                  lambda m: m.group(1) + fmt(np.array(list(map(float, m.group(2).split()))) + shift),
                  text, count=1, flags=re.S)

    # 집게 → OMX. 우리 tool_roll_link의 집게 모터·레일은 OMX link5의 것과 같은 메시라
    # 둘을 맞춰(Kabsch) OMX link5 좌표 → tool_roll 좌표 변환을 구하고, 손가락·끝점을 OMX URDF대로 둔다
    tr_tri = read_stl(PKG / 'meshes/visual/tool_roll_link.stl')
    tc = components(tr_tri)
    lc = components(l5)
    pick = lambda t, c, n: t[c == [k for k in np.unique(c) if (c == k).sum() == n][0]].reshape(-1, 3)
    A, B = pick(l5, lc, 3346), pick(tr_tri, tc, 3346)
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    t0 = cb - R @ ca
    print(f'집게 모터 맞춤 오차 {np.abs(A @ R.T + t0 - B).max() * 1000:.4f} mm')
    rpy = (np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0]), np.arctan2(R[1, 0], R[0, 0]))

    # tool_roll_link 질량: 부품별 합산 (집게 모터 XM430, 레일은 OMX 레일과 같은 추정, 연결 브라켓 PLA, 나머지 강철)
    bodies = []
    for k in np.unique(tc):
        part = tr_tri[tc == k]
        n, v = (tc == k).sum(), solid(part, 1.0)[0]
        if n == 3346:
            bodies.append(solid(part, XM430_KG / v))
        elif n == 2628:
            bodies.append(solid(part, OMX_RAIL_KG_M3))
        elif v > 1e-6:
            bodies.append(solid(part, PLA_KG_M3))
        else:
            bodies.append(solid(part, 7850))

    def combine(bodies):
        m = sum(b[0] for b in bodies)
        c = sum(b[0] * b[1] for b in bodies) / m
        I = sum(b[2] + b[0] * ((b[1] - c) @ (b[1] - c) * np.eye(3) - np.outer(b[1] - c, b[1] - c)) for b in bodies)
        return inertial(m, c, (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])), m
    blk, m_roll = combine(bodies)
    text = re.sub(r'(<link name="tool_roll_link">\s*)<inertial>.*?</inertial>', lambda m: m.group(1) + blk, text, flags=re.S)
    print(f'tool_roll_link: 다시 합산 {m_roll * 1000:.1f} g')

    # 손가락: OMX gripper_left/right_palm (left = jaw_upper, right = jaw_lower). OMX URDF 질량은 1 g 자리표시라 부피 x 1.24로
    for jaw, palm, y, ax in (('jaw_upper', 'gripper_left_palm', 0.021, '0 1 0'), ('jaw_lower', 'gripper_right_palm', -0.021, '0 -1 0')):
        tri = read_stl(Path(omx_dir) / f'{palm}.stl') * 1e-3
        for kind in ('visual', 'collision'):
            write_stl(PKG / f'meshes/{kind}/{SUB}/{jaw}_link.stl', tri)
        blk, mj = combine([solid(tri, OMX_RAIL_KG_M3)])
        print(f'{jaw}: OMX {palm} {mj * 1000:.1f} g')
        new_link = (f'<link name="{jaw}_link">\n    {blk}\n'
                    f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="package://craft_description/meshes/visual/{SUB}/{jaw}_link.stl"/></geometry>'
                    f'<material name="{jaw}_link_mat"><color rgba="0.6 0.6 0.6 1"/></material></visual>\n'
                    f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="package://craft_description/meshes/collision/{SUB}/{jaw}_link.stl"/></geometry></collision>\n  </link>')
        text = re.sub(rf'<link name="{jaw}_link">.*?</link>', lambda _: new_link, text, flags=re.S)
        pos = R @ np.array([0.0817, y, 0]) + t0
        new_joint = (f'<joint name="{jaw}" type="prismatic"><parent link="tool_roll_link"/><child link="{jaw}_link"/>'
                     f'<origin xyz="{fmt(pos)}" rpy="{fmt(rpy)}"/><axis xyz="{ax}"/>\n'
                     f'    <limit lower="-0.011" upper="0.02" effort="20.0" velocity="0.08"/>\n  </joint>')
        text = re.sub(rf'<joint name="{jaw}".*?</joint>', lambda _: new_joint, text, flags=re.S)

    # TCP = OMX end_effector (link5 x 126 mm). 자세(rpy)는 기존 tcp 그대로 (접근축이 OMX x축과 같음)
    ee = R @ np.array([0.126, 0, 0]) + t0
    print(f'TCP: tool_roll에서 {np.round(ee * 1000, 1)} mm')
    text = re.sub(r'(<joint name="tcp_joint"[^>]*>.*?<origin xyz=")[^"]*', lambda m: m.group(1) + fmt(ee), text, count=1, flags=re.S)

    # 팔 링크 → OMX
    for link, s in OMX.items():
        tri = read_stl(Path(omx_dir) / f"{s['mesh']}.stl") * 1e-3
        for kind in ('visual', 'collision'):
            write_stl(PKG / f'meshes/{kind}/{SUB}/{link}.stl', tri)
        com, I = rot_z180(s['com'], s['I'])
        new = (f'<link name="{link}">\n    {inertial(s["mass"], com, I)}\n'
               f'    <visual><origin xyz="0 0 0" rpy="0 0 3.14159265"/><geometry><mesh filename="package://craft_description/meshes/visual/{SUB}/{link}.stl"/></geometry>'
               f'<material name="robotis_frame_mat"><color rgba="0.75 0.75 0.78 1"/></material></visual>\n'
               f'    <collision><origin xyz="0 0 0" rpy="0 0 3.14159265"/><geometry><mesh filename="package://craft_description/meshes/collision/{SUB}/{link}.stl"/></geometry></collision>\n  </link>')
        text = re.sub(rf'<link name="{link}">.*?</link>', lambda _: new, text, flags=re.S)
    for joint, xyz in JOINT_XYZ.items():
        text = re.sub(rf'(<joint name="{joint}"[^>]*>.*?<origin xyz=")[^"]*', lambda m: m.group(1) + xyz, text, count=1, flags=re.S)

    if FLIP_ELBOW:
        # 엘보 모터를 혼 축(y) 기준 180° 뒤집어 단다: 모터 몸체를 elbow 관절 둘레로 180° 돌리고,
        # elbow 관절 원점에 pitch 180°를 넣어 L2부터 끝까지도 돌린다 → 0°에서 앞팔이 앞(+x)을 향함
        f = PKG / f'meshes/visual/{SUB}/shoulder_link.stl'
        tri = read_stl(f)                       # OMX link3 좌표 (visual 원점 yaw 180°)
        comp = components(tri)
        mk = max(np.unique(comp), key=lambda k: (comp == k).sum())
        mot = tri[comp == mk]
        J = np.array([0.024, 0, 0.128])         # OMX link3 좌표의 joint3
        flipped = (mot - J) * np.array([-1, 1, -1]) + J
        write_stl(f, np.concatenate([tri[comp != mk], flipped]))
        yaw = np.array([-1, -1, 1])             # OMX link3 좌표 → URDF shoulder_link 좌표
        v = solid(mot, 1.0)[0]
        old = solid(mot * yaw, XM430_KG / v)
        new = solid(flipped * yaw, XM430_KG / v)
        pat = re.compile(r'(<link name="shoulder_link">\s*)(<inertial>.*?</inertial>)', re.S)
        text = pat.sub(lambda m: m.group(1) + remove_body(m.group(2), old[0], old[1], old[2]), text)
        text = pat.sub(lambda m: m.group(1) + remove_body(m.group(2), -new[0], new[1], -new[2]), text)
        text = re.sub(r'(<joint name="elbow"[^>]*>.*?<origin xyz="[^"]*" rpy=")[^"]*', lambda m: m.group(1) + '0 3.14159265 0',
                      text, count=1, flags=re.S)
        print(f'엘보 모터 pitch 180° 뒤집음: 모터 무게중심 {np.round(old[1] * 1000, 1)} → {np.round(new[1] * 1000, 1)} mm')

    if FLIP_UPPER:
        text = flip_upper(text)
        print('어깨 위쪽 전체를 z축 180° 돌림 (어깨 관절은 그대로)')

    # 충돌형상: 바꾼 팔 링크는 부품(연결 성분)마다 볼록 껍질. 0.05 cm3 미만(볼트 등)은 뺀다
    for link in HULL_LINKS:
        vis = PKG / f'meshes/visual/{SUB}/{link}.stl'
        if not vis.exists():
            vis = PKG / f'meshes/visual/{link}.stl'
        tri = read_stl(vis)
        comp = components(tri)
        hulls = []
        for k in np.unique(comp):
            part = tri[comp == k]
            if solid(part, 1.0)[0] < 5e-8:
                continue
            hulls += [convex_hull(p) for p in decompose(part)]
        out = np.concatenate(hulls)
        write_stl(PKG / f'meshes/collision/{SUB}/{link}.stl', out)
        text = re.sub(rf'(<link name="{link}">.*?<collision>.*?filename=")[^"]*', lambda m: m.group(1) +
                      f'package://craft_description/meshes/collision/{SUB}/{link}.stl', text, count=1, flags=re.S)
        print(f'collision/{link}: 볼록 조각 {len(hulls)}개, 삼각형 {len(tri)} → {len(out)}')

    text = text.replace('<robot name="', '<!-- V307 몸체 + ROBOTIS OpenMANIPULATOR-X 팔 링크, 카메라 없음. '
                        'scripts/make_robotis_arm_urdf.py로 생성 -->\n<robot name="', 1)
    # elbow 한계는 V307 값 대신 충돌로 정한다: 양쪽 다 열어 두고 아래 충돌 검사가 좁힌다
    text = re.sub(r'(<joint name="elbow".*?<limit lower=")[^"]*(" upper=")[^"]*',
                  lambda m: m.group(1) + '-3.12413936' + m.group(2) + '3.12413936', text, count=1, flags=re.S)
    OUT.write_text(text)
    write_segments(text)
    # 팔 링크끼리 부딪히는 각도로 elbow·wrist_pitch 한계를 좁힌다 (collision_limits.py)
    import collision_limits
    collision_limits.apply(OUT, collision_limits.sweep(OUT))
    print('->', OUT)


if __name__ == '__main__':
    main(sys.argv[1])
