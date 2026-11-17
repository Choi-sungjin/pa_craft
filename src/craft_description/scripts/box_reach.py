#!/usr/bin/env python3
"""로봇이 전방에 놓인 박스 윗면을 TCP로 다 커버하는지 보는 도구 (기본값은 V307).

슬라이더
  관절: LM 가이드와 팔 관절(어깨, 팔꿈치, 손목, 비틀기). 나머지 움직이는 관절은 0으로 고정
  리밋 체크박스: 관절마다 한계 ON/OFF. OFF면 그 관절 한계를 무시하고 판정한다
  숫자 칸: 슬라이더마다 값을 직접 넣고 Enter (관절은 한계 안으로 자르고, 나머지는 범위를 넓힌다)
  LM 스트로크: 현재 범위의 중앙을 기준으로 양쪽 대칭으로 늘이거나 줄인다
  링크 길이: 어깨-팔꿈치, 팔꿈치-손목, 손목-비틀기, 비틀기-TCP. 바꾸면 URDF의 관절
    원점을 같은 방향으로 늘이거나 줄여 robot_state_publisher에 다시 넣는다.
    메시도 링크 방향으로 같이 늘인다: 양 끝 관절 허브(링크 길이의 30%, 최대 35 mm)는
    그대로 두고 그 사이만 늘이거나 줄이며, 끝 허브 너머는 늘어난 만큼 평행 이동한다.
    L4는 그리퍼 몸체와 두 집게를 함께 늘인다. 늘인 메시는 임시 폴더에 STL로 써서 쓴다
  박스: 로봇 앞면과 박스 사이 거리, 박스 좌우 중심, 검사 높이.
    로봇 앞면은 박스 높이대(바닥~박스 높이)와 박스 좌우 폭 안에 있는 몸체 메시(base_link와
    거기 바로 붙은 링크, LM 가이드 쪽은 제외)의 가장 앞 점이다. 그래서 거리 0이면 박스가 몸체에 닿는다
  TCP 각도 허용: 집게가 수직 아래에서 몇 도까지 기울어도 되는지 (180 = 아무 방향)
  높이 여유: 박스 윗면 점 바로 위 0~여유 높이 중 어디서든 TCP가 오면 그 점은 도달로 친다

토크 (창 오른쪽)
  현재 자세에서 무게(중력)만으로 shoulder/elbow/wrist_pitch에 걸리는 토크. URDF 링크 질량과
  무게중심(+ 집게 하중)을 쓰고, 가속·마찰은 뺀다. 링크 길이를 바꾸면 무게중심도 메시와 같은
  방식으로 옮기지만 질량은 그대로 둔다. 관절-모터 직결(감속 없음)이라 관절 토크 = 모터 토크.
  모터 기본값 XM430-W350 @12 V: 스톨 4.1 N·m(ROBOTIS e-manual), 정격 0.82 N·m(스톨의 20%, 추정).

판정
  어깨/팔꿈치/손목 축이 서로 평행하고 LM 가이드가 그 축 방향으로 움직이는 구조를 가정한다
  (V307: 축 y, 팔은 x-z 평면). TCP는 비틀기 축 위에 있어 비틀기와 무관하다.
  박스 윗면 격자점마다 좌우는 LM 가이드 범위, 평면 안은 3링크 IK + 관절 한계로 판정.
  평면 판정은 그 점 위 0~높이 여유를 HEIGHT_STEP 간격으로 올려 가며 하나라도 되면 도달.
  충돌(로봇 몸체, 박스)은 보지 않는다.
"""
import atexit
import math
import os
import re
import shutil
import struct
import tempfile
import threading
import tkinter as tk
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

GRID_STEP = 0.005               # 커버 판정 격자 [m]
THETA_STEP = math.radians(2.0)  # TCP 각도 표본 간격
HEIGHT_STEP = 0.002             # 높이 여유 표본 간격 [m]
AXES = {'x': np.array([1.0, 0, 0]), 'y': np.array([0, 1.0, 0]), 'z': np.array([0, 0, 1.0])}


def vec(el, attr, default):
    return np.array([float(v) for v in el.get(attr, default).split()])


def unit(v):
    return v / np.linalg.norm(v)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def resolve_uri(uri):
    if uri.startswith('package://'):
        pkg, rel = uri[len('package://'):].split('/', 1)
        return os.path.join(get_package_share_directory(pkg), rel)
    return uri[len('file://'):] if uri.startswith('file://') else uri


def read_stl(path):
    """삼각형 꼭짓점 (n, 3, 3). 바이너리/아스키 둘 다."""
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:
        rec = np.frombuffer(data[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
        return rec['v'].reshape(-1, 3, 3).astype(float)
    v = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)
    return np.array(v, dtype=float).reshape(-1, 3, 3)


def write_stl(path, tris):
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'], rec['v'] = nrm, tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'box_reach stretched mesh'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def stretch_map(s, L, L_new):
    """링크 방향 좌표 s를 길이 L -> L_new에 맞게 옮긴다 (양 끝 허브는 모양 유지)."""
    a = min(0.3 * L, 0.035)
    out = np.where(s >= L - a, s + (L_new - L), s)
    if L_new - 2 * a > 1e-4:  # 가운데 구간만 늘이거나 줄임
        mid = (s > a) & (s < L - a)
        out = np.where(mid, a + (s - a) * (L_new - 2 * a) / (L - 2 * a), out)
    else:                     # 너무 짧으면 0..L 전체를 비례 축소
        inside = (s > 0) & (s < L)
        out = np.where(inside, s * L_new / L, out)
    return out


class BoxReach(Node):
    def __init__(self):
        super().__init__('box_reach')
        urdf = self.declare_parameter('robot_description', '').value
        self.slide = self.declare_parameter('slide_joint', 'lm_slide').value
        self.arm = list(self.declare_parameter(
            'arm_joints', ['shoulder', 'elbow', 'wrist_pitch']).value)
        self.roll = self.declare_parameter('roll_joint', 'tool_roll').value
        self.tcp = self.declare_parameter('tcp_joint', 'tcp_joint').value
        front = self.declare_parameter('front_axis', '+x').value
        # 로봇 앞면(전방축 좌표)과 바닥 높이 [m, base_link 기준]. V307: base_footprint.
        # auto_front면 앞면을 몸체 메시에서 박스 높이대·좌우 폭 안의 가장 앞 점으로 잡고,
        # 메시를 못 읽거나 그 구간에 몸체가 없으면 front_face(V307 base 메시 전체 앞끝)를 쓴다
        self.auto_front = self.declare_parameter('auto_front', True).value
        self.front_face = self.declare_parameter('front_face', 0.273).value
        self.floor_z = self.declare_parameter('floor_z', -0.034346).value
        self.box_size = list(self.declare_parameter('box_size', [0.205, 0.205, 0.070]).value)
        # 모터 토크 [N·m]: XM430-W350 @12 V. 정격(연속)은 공식 표에 없어 스톨의 20%로 추정
        self.motor_name = self.declare_parameter('motor_name', 'XM430-W350 @12V').value
        self.stall_nm = self.declare_parameter('motor_stall_nm', 4.1).value
        self.rated_nm = self.declare_parameter('motor_rated_nm', 0.82).value
        stroke = self.declare_parameter('stroke', 0.0).value  # 시작 LM 스트로크 [m], 0이면 URDF 값
        # 관절 한계 바꾸기: ['관절:하한:상한', ...], 회전은 deg, 직동은 mm. 예: ['shoulder:0:45']
        overrides = [o for o in self.declare_parameter('limit_overrides', ['']).value if o]
        if not urdf:
            raise RuntimeError('robot_description 파라미터가 비어 있음')

        self.tree = ET.ElementTree(ET.fromstring(urdf))
        self.joint_el = {j.get('name'): j for j in self.tree.getroot().iter('joint')}
        self.base_origin = {}
        for n, j in self.joint_el.items():
            o = j.find('origin')
            self.base_origin[n] = vec(o, 'xyz', '0 0 0') if o is not None else np.zeros(3)

        self.limits = {}
        for n, j in self.joint_el.items():
            if j.get('type') in ('fixed', 'floating', 'planar') or j.find('mimic') is not None:
                continue
            lim = j.find('limit')
            if j.get('type') == 'continuous' or lim is None:
                self.limits[n] = (-math.pi, math.pi)
            else:
                self.limits[n] = (float(lim.get('lower', 0)), float(lim.get('upper', 0)))
        for o in overrides:
            name, lo, hi = o.split(':')
            k = 1000.0 if self.joint_el[name].get('type') == 'prismatic' else math.degrees(1.0)
            self.limits[name] = (float(lo) / k, float(hi) / k)
            self.get_logger().info(f'{name} 한계를 {lo}~{hi}로 바꿈 (URDF 파일은 그대로)')
        self.limit_on = {n: True for n in self.limits}
        self.positions = {n: min(max(0.0, lo), hi) for n, (lo, hi) in self.limits.items()}
        self.slider_joints = [self.slide, *self.arm, self.roll]

        # 팔 평면: 팔 관절 축 a, 평면 기저 (e1, e2), e1 x e2 = a 라서 a축 +회전이 반시계
        axes = [unit(vec(self.joint_el[j].find('axis'), 'xyz', '1 0 0')) for j in self.arm]
        self.a = axes[0]
        self.s = unit(vec(self.joint_el[self.slide].find('axis'), 'xyz', '1 0 0'))
        if any(abs(abs(ax @ self.a) - 1) > 1e-3 for ax in axes) or abs(abs(self.s @ self.a) - 1) > 1e-3:
            self.get_logger().warn('팔 관절 축이 평행하지 않거나 LM 가이드가 팔 축 방향이 아님: 판정이 틀릴 수 있음')
        up = AXES['z'] - (AXES['z'] @ self.a) * self.a
        self.e1 = unit(up) if np.linalg.norm(up) > 1e-6 else unit(np.cross(self.a, AXES['x']))
        self.e2 = np.cross(self.a, self.e1)
        # 팔 관절 축이 a와 반대 방향이면 회전 부호를 뒤집는다
        self.sign = {j: float(np.sign(ax @ self.a)) for j, ax in zip(self.arm, axes)}

        self.f = AXES[front[-1]] * (-1.0 if front.startswith('-') else 1.0)  # 전방
        self.lat = self.s * np.sign(self.s @ self.a)                         # 좌우 = LM 가이드 방향

        self.links = [('L1 어깨-팔꿈치', self.arm[1]), ('L2 팔꿈치-손목', self.arm[2]),
                      ('L3 손목-비틀기', self.roll), ('L4 비틀기-TCP', self.tcp)]
        self.lengths = {j: float(np.linalg.norm(self.base_origin[j])) for _, j in self.links}
        self.mesh_dir = tempfile.mkdtemp(prefix='box_reach_meshes_')
        atexit.register(shutil.rmtree, self.mesh_dir, True)
        self.stretch = {j: self.stretch_targets(j) for _, j in self.links}
        self.body = self.body_vertices() if self.auto_front else np.zeros((0, 3))
        self.payload = 0.0  # 집게 하중 [kg], TCP에 점질량
        self.inertial = {}  # 링크: (질량, 링크 좌표 무게중심)
        for l in self.tree.getroot().iter('link'):
            i = l.find('inertial')
            if i is not None and i.find('mass') is not None:
                o = i.find('origin')
                self.inertial[l.get('name')] = (float(i.find('mass').get('value')),
                                                vec(o, 'xyz', '0 0 0') if o is not None else np.zeros(3))
        self.stretched_by = {l: j for _, j in self.links for l in self.stretched_links(j)}
        lo, hi = self.limits[self.slide]
        self.slide_center, self.stroke0 = 0.5 * (lo + hi), hi - lo
        self.box = {'distance': 0.0,
                    'lateral': self.arm_lateral(0.5 * (lo + hi)),  # LM 가이드 중앙
                    'height': self.box_size[2]}
        self.tol_deg = 0.0
        self.clearance = 0.030  # 높이 여유 [m]

        self.lock = threading.Lock()
        if stroke > 0:
            self.set_stroke(stroke)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'box_reach/markers', latched)
        self.rsp_client = self.create_client(
            SetParameters, '/robot_state_publisher/set_parameters')
        self.create_timer(1.0 / 30.0, self.publish_joints)

    # ---- 기하 ----
    def origin(self, joint):
        """링크 길이를 반영한 관절 원점 (방향은 원래 URDF 그대로)."""
        v = self.base_origin[joint]
        if joint in self.lengths:
            n = np.linalg.norm(v)
            v = v / n * self.lengths[joint] if n > 0 else v
        return v

    def stretched_links(self, joint):
        """joint 길이를 바꿀 때 같이 늘이는 링크: 부모 링크와 원점 0으로 붙은 형제 링크."""
        parent = self.joint_el[joint].find('parent').get('link')
        return [parent] + [
            j.find('child').get('link') for n, j in self.joint_el.items()
            if n != joint and j.find('parent').get('link') == parent
            and np.linalg.norm(self.base_origin[n]) < 1e-9]

    def stretch_targets(self, joint):
        """링크 길이 joint를 바꿀 때 같이 늘일 visual 메시들.

        joint의 부모 링크와, 그 부모에 원점 0으로 붙은 형제 링크(V307 L4: 두 집게)의
        메시를 부모 링크 좌표의 삼각형으로 읽어 둔다.
        """
        links = self.stretched_links(joint)
        link_el = {l.get('name'): l for l in self.tree.getroot().iter('link')}
        targets = []
        for name in links:
            for i, vis in enumerate(link_el[name].findall('visual')):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                o = vis.find('origin')
                R = rpy_matrix(*vec(o, 'rpy', '0 0 0')) if o is not None else np.eye(3)
                t = vec(o, 'xyz', '0 0 0') if o is not None else np.zeros(3)
                scale = vec(mesh, 'scale', '1 1 1')
                try:
                    tris = read_stl(resolve_uri(mesh.get('filename'))) * scale
                except (OSError, ValueError) as e:
                    self.get_logger().warn(f'{name} 메시를 못 읽어 늘이지 않음: {e}')
                    continue
                targets.append({'name': f'{name}_{i}', 'mesh': mesh, 'vis': vis,
                                'uri': mesh.get('filename'), 'scale': mesh.get('scale'),
                                'origin': (o.get('xyz'), o.get('rpy')) if o is not None else None,
                                'tris': tris @ R.T + t})  # 링크 좌표
        return targets

    def body_vertices(self):
        """base_link와 거기 바로 붙은 링크(LM 가이드 제외)의 visual 메시 꼭짓점 (base_link 기준)."""
        link_el = {l.get('name'): l for l in self.tree.getroot().iter('link')}
        root = self.joint_el[self.slide].find('parent').get('link')
        frames = [(root, np.eye(3), np.zeros(3))]
        for n, j in self.joint_el.items():
            if n != self.slide and j.find('parent').get('link') == root:
                o = j.find('origin')
                R = rpy_matrix(*vec(o, 'rpy', '0 0 0')) if o is not None else np.eye(3)
                frames.append((j.find('child').get('link'), R, self.base_origin[n]))
        pts = [np.zeros((0, 3))]
        for name, Rj, tj in frames:
            for vis in link_el[name].findall('visual'):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                o = vis.find('origin')
                Rv = rpy_matrix(*vec(o, 'rpy', '0 0 0')) if o is not None else np.eye(3)
                tv = vec(o, 'xyz', '0 0 0') if o is not None else np.zeros(3)
                try:
                    v = read_stl(resolve_uri(mesh.get('filename'))).reshape(-1, 3)
                except (OSError, ValueError) as e:
                    self.get_logger().warn(f'{name} 메시를 못 읽어 앞면 계산에서 뺌: {e}')
                    continue
                v = (v * vec(mesh, 'scale', '1 1 1')) @ Rv.T + tv
                pts.append(v @ Rj.T + tj)
        return np.vstack(pts)

    def face(self):
        """박스 쪽을 향한 로봇 앞면의 전방 좌표 [m]."""
        _, width, height = self.box_size
        c = self.box['lateral']
        z = self.body[:, 2] - self.floor_z
        lat = self.body @ self.lat
        m = (z >= 0) & (z <= height) & (np.abs(lat - c) <= width / 2)
        return float((self.body[m] @ self.f).max()) if m.any() else self.front_face

    def update_meshes(self, joint):
        """joint 길이에 맞게 늘인 메시를 써서 URDF visual을 바꾼다."""
        L = float(np.linalg.norm(self.base_origin[joint]))
        L_new = self.lengths[joint]
        d = self.base_origin[joint] / L
        for t in self.stretch[joint]:
            mesh, vis = t['mesh'], t['vis']
            if abs(L_new - L) < 1e-6:  # 원래 메시로 되돌림
                mesh.set('filename', t['uri'])
                if t['scale'] is not None:
                    mesh.set('scale', t['scale'])
                if t['origin'] is not None:
                    o = vis.find('origin')
                    for k, v in zip(('xyz', 'rpy'), t['origin']):
                        if v is None:
                            o.attrib.pop(k, None)
                        else:
                            o.set(k, v)
                continue
            path = os.path.join(self.mesh_dir, f"{t['name']}_{L_new * 1000:.1f}mm.stl")
            if not os.path.exists(path):  # RViz는 URI로 캐시하므로 길이마다 다른 파일
                tris = t['tris']
                s = tris @ d
                write_stl(path, tris + ((stretch_map(s, L, L_new) - s)[..., None] * d))
            mesh.set('filename', 'file://' + path)
            mesh.attrib.pop('scale', None)
            o = vis.find('origin')  # 링크 좌표로 썼으므로 visual 원점은 0
            if o is not None:
                o.set('xyz', '0 0 0')
                o.set('rpy', '0 0 0')

    def lim(self, joint):
        """판정과 슬라이더에 쓰는 한계. 리밋 OFF면 회전 ±180°, 직동 ±300 mm."""
        if self.limit_on[joint]:
            return self.limits[joint]
        free = 0.3 if self.joint_el[joint].get('type') == 'prismatic' else math.pi
        return (-free, free)

    def set_stroke(self, stroke):
        """LM 가이드 범위를 중앙 기준 대칭으로 바꾼다. 현재 위치는 새 범위 안으로 자른다."""
        lo, hi = self.slide_center - stroke / 2, self.slide_center + stroke / 2
        self.limits[self.slide] = (lo, hi)
        with self.lock:
            self.positions[self.slide] = min(max(self.positions[self.slide], lo), hi)
        lim = self.joint_el[self.slide].find('limit')
        if lim is not None:
            lim.set('lower', f'{lo:.6g}')
            lim.set('upper', f'{hi:.6g}')

    def plane(self, v):
        return np.array([v @ self.e1, v @ self.e2])

    def shoulder(self):
        """LM 가이드 0일 때 어깨 축 위치 (base_link 기준)."""
        return self.origin(self.slide) + self.origin(self.arm[0])

    def arm_lateral(self, q_slide):
        """LM 가이드 값에서 팔 평면의 좌우 위치 (링크의 좌우 성분 포함)."""
        chain = [self.arm[1], self.arm[2], self.roll, self.tcp]
        off = sum(self.origin(j) @ self.lat for j in chain)
        return float(self.shoulder() @ self.lat + q_slide * (self.s @ self.lat) + off)

    def segments(self):
        """평면 안의 링크 벡터: 어깨-팔꿈치, 팔꿈치-손목, 손목-TCP, 그리고 집게 방향각."""
        v1 = self.plane(self.origin(self.arm[1]))
        v2 = self.plane(self.origin(self.arm[2]))
        tcp = self.origin(self.tcp)
        w = self.plane(self.origin(self.roll) + tcp)
        d = self.plane(tcp)  # TCP는 비틀기 축 위: 비틀기 원점 -> TCP 방향 = 집게 방향
        return v1, v2, w, math.atan2(d[1], d[0])

    def compute(self):
        """박스 윗면(검사 높이) 격자점마다 도달 여부 (그 위 0~높이 여유 중 한 곳이라도 되면 도달)."""
        depth, width, _ = self.box_size
        d, c, h = self.box['distance'], self.box['lateral'], self.box['height']
        us = np.arange(0.0, depth + 1e-9, GRID_STEP) + self.face() + d       # 전방 좌표
        vs = np.arange(-width / 2, width / 2 + 1e-9, GRID_STEP) + c          # 좌우 좌표
        U, V = np.meshgrid(us, vs, indexing='ij')
        P = (U[..., None] * self.f + V[..., None] * self.lat
             + (self.floor_z + h) * AXES['z'])

        # 좌우: LM 가이드
        q_s = (V - self.arm_lateral(0.0)) / (self.s @ self.lat)
        slo, shi = self.lim(self.slide)
        ok_lat = (q_s >= slo - 1e-9) & (q_s <= shi + 1e-9)

        # 평면: 어깨 기준 3링크 IK. 집게 방향 φ = Θ + β, 수직 아래 = -e1 방향 = π
        v1, v2, w, beta = self.segments()
        a, b, lw = np.linalg.norm(v1), np.linalg.norm(v2), np.linalg.norm(w)
        al1, al2, gw = (math.atan2(v[1], v[0]) for v in (v1, v2, w))
        tol = math.radians(self.tol_deg)
        n = max(1, int(round(2 * tol / THETA_STEP)) + 1)
        Th = (math.pi + np.linspace(-tol, tol, n) - beta)[None, None, :]   # Θ = q1+q2+q3

        lims = [self.lim(j) for j in self.arm]
        sg = [self.sign[j] for j in self.arm]
        ok_plane = np.zeros(U.shape, bool)
        for dz in np.arange(0.0, self.clearance + 1e-9, HEIGHT_STEP):
            R = P + dz * AXES['z'] - self.shoulder()
            Py, Pz = (R @ self.e1)[..., None], (R @ self.e2)[..., None]
            Wy = Py - lw * np.cos(Th + gw)
            Wz = Pz - lw * np.sin(Th + gw)
            cc = (Wy ** 2 + Wz ** 2 - a ** 2 - b ** 2) / (2 * a * b)
            reach = np.abs(cc) <= 1.0
            cc = np.clip(cc, -1.0, 1.0)
            for elbow in (1.0, -1.0):  # 팔꿈치 위/아래 두 해
                dl = elbow * np.arccos(cc)
                phi1 = np.arctan2(Wz, Wy) - np.arctan2(b * np.sin(dl), a + b * np.cos(dl))
                q1 = wrap(phi1 - al1)
                q2 = wrap(dl + al1 - al2)
                q3 = wrap(Th - q1 - q2)
                ok = reach
                for q, s, (lo, hi) in zip((q1, q2, q3), sg, lims):
                    ok = ok & (s * q >= lo) & (s * q <= hi)  # 평면 각 -> 관절 값
                ok_plane |= ok.any(-1)

        return P, ok_lat, ok_plane

    def link_frames(self):
        """현재 관절값·링크 길이로 링크마다 (R, p) (base_link 기준), 그리고 관절마다 (축, 원점)."""
        root = self.joint_el[self.slide].find('parent').get('link')
        frames, joints = {root: (np.eye(3), np.zeros(3))}, {}
        todo = [root]
        while todo:
            parent = todo.pop()
            Rp, pp = frames[parent]
            for n, j in self.joint_el.items():
                if j.find('parent').get('link') != parent:
                    continue
                o = j.find('origin')
                Ro = rpy_matrix(*vec(o, 'rpy', '0 0 0')) if o is not None else np.eye(3)
                R, p = Rp @ Ro, pp + Rp @ self.origin(n)
                ax = j.find('axis')
                a = unit(vec(ax, 'xyz', '1 0 0')) if ax is not None else np.array([1.0, 0, 0])
                joints[n] = (R @ a, p)
                q = self.positions.get(n, 0.0)
                if j.get('type') in ('revolute', 'continuous'):
                    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    R = R @ (np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K)
                elif j.get('type') == 'prismatic':
                    p = p + R @ a * q
                child = j.find('child').get('link')
                frames[child] = (R, p)
                todo.append(child)
        return frames, joints

    def gravity_torques(self):
        """팔 관절마다 무게를 버티는 데 필요한 모터 토크 [N·m] (부호: 관절 + 방향)."""
        frames, joints = self.link_frames()
        g = np.array([0, 0, -9.81])
        masses = []  # (링크, 질량, 무게중심 base_link 기준)
        for link, (m, c) in self.inertial.items():
            if link not in frames:
                continue
            j = self.stretched_by.get(link)
            if j is not None:  # 늘인 링크: 무게중심도 메시와 같은 방식으로 옮김
                L = float(np.linalg.norm(self.base_origin[j]))
                d = self.base_origin[j] / L
                s = c @ d
                c = c + (stretch_map(np.array(s), L, self.lengths[j]) - s) * d
            R, p = frames[link]
            masses.append((link, m, p + R @ c))
        tcp_link = self.joint_el[self.tcp].find('child').get('link')
        if self.payload > 0 and tcp_link in frames:
            masses.append((tcp_link, self.payload, frames[tcp_link][1]))
        out = {}
        for jn in self.arm:
            below = self.subtree(self.joint_el[jn].find('child').get('link'))
            a, o = joints[jn]
            tau_g = sum(float(a @ np.cross(c - o, m * g)) for l, m, c in masses if l in below)
            out[jn] = -tau_g  # 모터가 내야 하는 토크
        return out

    def subtree(self, link):
        out, todo = set(), [link]
        while todo:
            l = todo.pop()
            out.add(l)
            todo += [j.find('child').get('link') for j in self.joint_el.values()
                     if j.find('parent').get('link') == l]
        return out

    def tcp_position(self):
        """현재 관절값의 TCP 위치 (base_link 기준)."""
        q = self.positions
        v1, v2, w, _ = self.segments()
        p = self.plane(self.shoulder())
        ang = 0.0
        for j, v in zip(self.arm, (v1, v2, w)):
            ang += self.sign[j] * q[j]
            c, s = math.cos(ang), math.sin(ang)
            p = p + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
        lat = self.arm_lateral(q[self.slide])
        # 좌우 성분은 lat, 평면 성분은 e1/e2. a와 lat는 같은 축(부호만 다를 수 있음)
        return p[0] * self.e1 + p[1] * self.e2 + lat * self.lat

    # ---- 발행 ----
    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.positions)
            msg.position = list(self.positions.values())
        self.js_pub.publish(msg)

    def update_markers(self, P, ok):
        stamp = self.get_clock().now().to_msg()
        depth, width, height = self.box_size
        box = Marker(type=Marker.CUBE, ns='box', id=0, action=Marker.ADD)
        box.header.frame_id = 'base_link'
        box.header.stamp = stamp
        center = ((self.face() + self.box['distance'] + depth / 2) * self.f
                  + self.box['lateral'] * self.lat + (self.floor_z + height / 2) * AXES['z'])
        box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, center)
        box.pose.orientation.w = 1.0
        size = depth * np.abs(self.f) + width * np.abs(self.lat) + height * AXES['z']
        box.scale.x, box.scale.y, box.scale.z = map(float, size)
        box.color.r, box.color.g, box.color.b, box.color.a = 0.85, 0.7, 0.45, 0.35

        pts = Marker(type=Marker.POINTS, ns='coverage', id=1, action=Marker.ADD)
        pts.header = box.header
        pts.pose.orientation.w = 1.0
        pts.scale.x = pts.scale.y = GRID_STEP * 0.8
        green = ColorRGBA(r=0.1, g=0.75, b=0.2, a=1.0)
        red = ColorRGBA(r=0.9, g=0.15, b=0.1, a=1.0)
        for p, o in zip(P.reshape(-1, 3), ok.ravel()):
            pts.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.001))
            pts.colors.append(green if o else red)

        tcp = Marker(type=Marker.SPHERE, ns='tcp', id=2, action=Marker.ADD)
        tcp.header = box.header
        tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z = map(float, self.tcp_position())
        tcp.pose.orientation.w = 1.0
        tcp.scale.x = tcp.scale.y = tcp.scale.z = 0.012
        tcp.color.r, tcp.color.g, tcp.color.b, tcp.color.a = 0.2, 0.3, 1.0, 1.0

        self.marker_pub.publish(MarkerArray(markers=[box, pts, tcp]))

    def push_description(self):
        """링크 길이와 LM 스트로크를 반영한 URDF를 robot_state_publisher에 넣는다."""
        for joint in self.lengths:
            v = self.origin(joint)
            self.joint_el[joint].find('origin').set('xyz', ' '.join(f'{c:.6g}' for c in v))
            self.update_meshes(joint)
        if not self.rsp_client.service_is_ready():
            self.get_logger().warn('robot_state_publisher 파라미터 서비스가 아직 없음')
            return
        p = Parameter(name='robot_description')
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = ET.tostring(self.tree.getroot(), encoding='unicode')
        self.rsp_client.call_async(SetParameters.Request(parameters=[p]))


def build_gui(node):
    root = tk.Tk()
    root.title('Box reach')
    root.geometry('+60+60')  # 주 모니터 왼쪽 위 (안 그러면 다른 모니터 구석에 뜨기도 함)
    row = [0]
    pending = {}

    def section(title):
        tk.Label(root, text=title, font=('TkDefaultFont', 10, 'bold')).grid(
            row=row[0], column=0, columnspan=3, sticky='w', padx=8, pady=(10, 0))
        row[0] += 1

    def slider(label, lo, hi, init, res, on_change, clamp=False):
        """슬라이더 + 숫자 칸. clamp면 칸에 넣은 값을 범위 안으로 자르고, 아니면 범위를 넓힌다."""
        tk.Label(root, text=label, anchor='w').grid(row=row[0], column=0, sticky='w', padx=8)
        var = tk.StringVar()
        digits = 1 if res < 1 else 0

        def on_scale(v):
            var.set(f'{float(v):.{digits}f}')
            on_change(float(v))

        s = tk.Scale(root, from_=lo, to=hi, orient=tk.HORIZONTAL, length=420,
                     resolution=res, showvalue=False, command=on_scale)
        s.grid(row=row[0], column=1, padx=8)
        entry = tk.Entry(root, textvariable=var, width=8, justify='right')
        entry.grid(row=row[0], column=2, padx=(0, 8))

        def on_entry(_=None):
            try:
                v = float(var.get())
            except ValueError:
                var.set(f'{s.get():.{digits}f}')
                return
            a, b = float(s.cget('from')), float(s.cget('to'))
            if not clamp and not a <= v <= b:
                s.configure(from_=min(a, v), to=max(b, v))
            s.set(v)
            on_scale(s.get())  # 같은 값이면 Scale이 command를 안 부르므로 직접

        entry.bind('<Return>', on_entry)
        entry.bind('<KP_Enter>', on_entry)
        entry.bind('<FocusOut>', on_entry)
        s.set(init)
        var.set(f'{float(s.get()):.{digits}f}')
        row[0] += 1
        return s

    def debounce(key, ms, fn):
        if key in pending:
            root.after_cancel(pending[key])
        pending[key] = root.after(ms, fn)

    status = tk.StringVar()
    tcp_var = tk.StringVar()

    def refresh():
        P, ok_lat, ok_plane = node.compute()
        ok = ok_lat & ok_plane
        node.update_markers(P, ok)
        full = '다 커버함' if ok.all() else '못 닿는 곳 있음 (RViz 빨간 점)'
        status.set(f'전체 {ok.mean() * 100:.1f}%  — {full}   (로봇 앞면 {node.face() * 1000:.1f} mm)\n'
                   f'좌우(LM 가이드) 조건 {ok_lat.mean() * 100:.1f}%, '
                   f'전후·높이(팔) 조건 {ok_plane.mean() * 100:.1f}%')

    panel = tk.LabelFrame(root, text=f'무게 토크 (중력만)  — {node.motor_name}', padx=8, pady=6)
    panel.grid(row=0, column=4, rowspan=40, sticky='n', padx=(4, 10), pady=10)
    for c, t in enumerate(('관절', '토크 [N·m]', '정격 대비', '스톨 대비')):
        tk.Label(panel, text=t, font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=c, padx=4)
    torque_cells = {}
    for r, jn in enumerate(node.arm, start=1):
        tk.Label(panel, text=jn, anchor='w').grid(row=r, column=0, sticky='w', padx=4)
        torque_cells[jn] = [tk.Label(panel, width=w, anchor='e') for w in (8, 9, 9)]
        for c, lab in enumerate(torque_cells[jn], start=1):
            lab.grid(row=r, column=c, padx=4)
    n_arm = len(node.arm)
    tk.Label(panel, text=f'정격 {node.rated_nm:.2f} N·m (스톨의 20%, 추정)\n'
                         f'스톨 {node.stall_nm:.2f} N·m (ROBOTIS e-manual)',
             justify='left', anchor='w').grid(row=n_arm + 1, column=0, columnspan=4,
                                             sticky='w', pady=(8, 0))
    tk.Label(panel, text='초록 ≤ 정격 · 주황 ≤ 스톨 · 빨강 > 스톨\n'
                         'URDF 질량·무게중심 기준. 가속·마찰 제외.\n'
                         '링크 길이를 바꿔도 질량은 그대로 둠.',
             justify='left', anchor='w', fg='gray30').grid(row=n_arm + 2, column=0,
                                                         columnspan=4, sticky='w', pady=(4, 0))
    pay = tk.Frame(panel)
    pay.grid(row=n_arm + 3, column=0, columnspan=4, sticky='w', pady=(8, 0))
    tk.Label(pay, text='집게 하중 [g]').pack(side='left')
    pay_var = tk.StringVar(value='0')
    pay_entry = tk.Entry(pay, textvariable=pay_var, width=7, justify='right')
    pay_entry.pack(side='left', padx=4)

    def set_payload(_=None):
        try:
            node.payload = max(0.0, float(pay_var.get())) / 1000.0
        except ValueError:
            pay_var.set(f'{node.payload * 1000:.0f}')
        refresh_tcp()

    pay_entry.bind('<Return>', set_payload)
    pay_entry.bind('<KP_Enter>', set_payload)
    pay_entry.bind('<FocusOut>', set_payload)

    def refresh_torque():
        for jn, tau in node.gravity_torques().items():
            mag = abs(tau)
            color = '#1a8f3c' if mag <= node.rated_nm else '#d9822b' if mag <= node.stall_nm else '#d0302a'
            cells = torque_cells[jn]
            cells[0].config(text=f'{tau:+.3f}', fg=color)
            cells[1].config(text=f'{mag / node.rated_nm * 100:.0f} %', fg=color)
            cells[2].config(text=f'{mag / node.stall_nm * 100:.0f} %', fg=color)

    def refresh_tcp():
        p = node.tcp_position() * 1000
        tcp_var.set(f'TCP (base_link) x {p[0]:.1f}  y {p[1]:.1f}  z {p[2]:.1f} mm')
        refresh_torque()
        debounce('markers', 50, refresh)

    def set_joint(name, k):
        def f(v):
            with node.lock:
                node.positions[name] = v / k
            refresh_tcp()
        return f

    section('관절  (오른쪽 체크 = 리밋 ON)')
    scales, kfac = {}, {}

    def apply_range(name):
        """리밋/스트로크가 바뀐 관절 슬라이더 범위를 맞추고 현재 값을 범위 안으로 자른다."""
        lo, hi = node.lim(name)
        k = kfac[name]
        with node.lock:
            node.positions[name] = min(max(node.positions[name], lo), hi)
        scales[name].configure(from_=lo * k, to=hi * k)
        scales[name].set(node.positions[name] * k)

    def toggle_limit(name, var):
        def f():
            node.limit_on[name] = var.get()
            apply_range(name)
            refresh_tcp()
        return f

    for name in node.slider_joints:
        lo, hi = node.lim(name)
        prismatic = node.joint_el[name].get('type') == 'prismatic'
        k, unit_ = (1000.0, 'mm') if prismatic else (math.degrees(1.0), 'deg')
        kfac[name] = k
        r = row[0]
        scales[name] = slider(f'{name} [{unit_}]  {lo * k:.0f}~{hi * k:.0f}', lo * k, hi * k,
                              node.positions[name] * k, 0.5, set_joint(name, k), clamp=True)
        on = tk.BooleanVar(value=True)
        tk.Checkbutton(root, text='리밋', variable=on,
                       command=toggle_limit(name, on)).grid(row=r, column=3, padx=(0, 8))

    section('LM 가이드 [mm]')

    def set_stroke(v):
        node.set_stroke(v / 1000.0)
        apply_range(node.slide)
        debounce('rsp', 300, node.push_description)
        refresh_tcp()

    S0 = node.stroke0 * 1000
    lo, hi = node.limits[node.slide]
    slider(f'스트로크 (원래 {S0:.1f}, 중앙 기준 대칭)', 0, max(400, 2 * S0), (hi - lo) * 1000, 1,
           set_stroke)

    section('링크 길이 [mm]')

    def set_length(joint):
        def f(v):
            node.lengths[joint] = v / 1000.0
            debounce('rsp', 300, node.push_description)
            refresh_tcp()
        return f

    for label, joint in node.links:
        L = node.lengths[joint] * 1000
        slider(f'{label} (원래 {L:.1f})', 10, max(400, 2 * L), L, 0.1, set_length(joint))

    depth, width, height = (v * 1000 for v in node.box_size)
    section(f'박스 {depth:.0f}×{width:.0f}×{height:.0f} mm')

    def set_box(key):
        def f(v):
            node.box[key] = v / 1000.0
            debounce('markers', 50, refresh)
        return f

    slider('로봇 앞면-박스 거리 [mm]', 0, 400, 0, 1, set_box('distance'))
    slider('박스 좌우 중심 [mm]', -300, 300, node.box['lateral'] * 1000, 1, set_box('lateral'))
    slider('검사 높이 (바닥 기준) [mm]', 0, 300, height, 1, set_box('height'))

    def set_tol(v):
        node.tol_deg = v
        debounce('markers', 50, refresh)

    slider('TCP 각도 허용 (수직 아래 기준) [deg]', 0, 180, 0, 1, set_tol)

    def set_clearance(v):
        node.clearance = v / 1000.0
        debounce('markers', 50, refresh)

    slider('높이 여유 (윗면 위 이 높이까지 OK) [mm]', 0, 100, node.clearance * 1000, 1,
           set_clearance)

    tk.Label(root, textvariable=tcp_var, anchor='w').grid(
        row=row[0], column=0, columnspan=4, sticky='w', padx=8, pady=(10, 0))
    row[0] += 1
    tk.Label(root, textvariable=status, anchor='w', justify='left',
             font=('TkDefaultFont', 11, 'bold')).grid(
        row=row[0], column=0, columnspan=4, sticky='w', padx=8, pady=(4, 10))

    root.after(500, refresh_tcp)
    return root


def main():
    rclpy.init()
    node = BoxReach()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = build_gui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
