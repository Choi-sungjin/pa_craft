"""craft 2D 시뮬레이터의 세계 모델 (ROS 없이 동작).

- 맵: ROS map_server 형식 yaml + pgm(trinary). 플래닝이 쓰는 같은 파일을 읽는다.
- 차체: 이상적 차동구동. 명령한 바퀴 각속도가 그대로 실제 속도가 된다.
- 센서: 참값. LiDAR 스캔은 맵 + 동적 장애물에 대한 광선 투사, 박스 관측은 참 박스 자세로 계산한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def wrap(a: float) -> float:
    return math.remainder(a, 2 * math.pi)


# ---------------------------------------------------------------------------------------------------------------
# 맵
# ---------------------------------------------------------------------------------------------------------------
class GridMap:
    """occupied[row, col], row 0 = 가장 작은 y (이미지를 위아래 뒤집어 저장)."""

    def __init__(self, yaml_path: str | Path):
        yaml_path = Path(yaml_path).expanduser()
        meta = yaml.safe_load(yaml_path.read_text())
        img_path = Path(meta['image'])
        if not img_path.is_absolute():
            img_path = yaml_path.parent / img_path
        img = np.asarray(Image.open(img_path).convert('L'), dtype=np.float64)
        self.yaml_path = yaml_path
        self.res = float(meta['resolution'])
        ox, oy = meta['origin'][0], meta['origin'][1]
        self.origin = (float(ox), float(oy))
        negate = int(meta.get('negate', 0))
        occ_p = img / 255.0 if negate else (255.0 - img) / 255.0
        occ_t, free_t = float(meta.get('occupied_thresh', 0.65)), float(meta.get('free_thresh', 0.196))
        occ_p = occ_p[::-1, :]  # row 0 = 아래쪽(y 최소)
        self.occupied = occ_p >= occ_t
        self.free = occ_p <= free_t
        self.unknown = ~(self.occupied | self.free)
        self.h, self.w = self.occupied.shape
        # 동적 장애물 격자(같은 해상도). 장애물이 바뀔 때만 다시 그린다.
        self.dynamic = np.zeros_like(self.occupied)

    @property
    def extent(self):
        x0, y0 = self.origin
        return x0, x0 + self.w * self.res, y0, y0 + self.h * self.res

    def to_cell(self, x, y):
        c = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        r = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        return r, c

    def inside(self, r, c):
        return (r >= 0) & (r < self.h) & (c >= 0) & (c < self.w)

    def occupancy_msg_data(self):
        """nav_msgs/OccupancyGrid data (row-major, row 0 = origin row)."""
        d = np.full(self.occupied.shape, -1, dtype=np.int8)
        d[self.free] = 0
        d[self.occupied] = 100
        return d.ravel().tolist()

    def render_dynamic(self, obstacles, boxes=()):
        """맵에 없는 물체(동적 장애물 + 박스)를 동적 격자에 그린다."""
        self.dynamic[:] = False
        for ob in obstacles:
            if ob.active:
                ob.rasterize(self)
        for b in boxes:
            b.rasterize(self)

    @property
    def wall_dist(self):
        """각 칸에서 가장 가까운 점유·미지 칸까지 거리 [m] (실제 LiDAR 장애물 필터의 벽 판정과 같은 기준)."""
        if not hasattr(self, '_wall_dist'):
            from scipy.ndimage import distance_transform_edt
            self._wall_dist = distance_transform_edt(~(self.occupied | self.unknown)) * self.res
        return self._wall_dist

    def wall_distance(self, x, y):
        r, c = self.to_cell(x, y)
        ins = self.inside(r, c)
        out = np.zeros(np.shape(r))
        out[ins] = self.wall_dist[r[ins], c[ins]]
        return out

    def raycast(self, ox, oy, angles, max_range, step=None):
        """원점에서 각도별 광선. 맵 점유 칸 또는 동적 장애물 칸에서 멈춘다.
        반환: ranges(np.inf = 안 맞음), hit_dynamic(bool)."""
        step = step or self.res
        n = int(max_range / step) + 1
        d = np.arange(1, n + 1) * step  # [n]
        xs = ox + np.cos(angles)[:, None] * d[None, :]
        ys = oy + np.sin(angles)[:, None] * d[None, :]
        r, c = self.to_cell(xs, ys)
        ins = self.inside(r, c)
        rr, cc = np.where(ins, r, 0), np.where(ins, c, 0)
        stat = self.occupied[rr, cc] & ins
        dyn = self.dynamic[rr, cc] & ins
        hit = stat | dyn
        any_hit = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        ranges = np.where(any_hit, d[first], np.inf)
        k = np.arange(len(angles))
        hit_dyn = any_hit & dyn[k, first] & ~stat[k, first]
        return ranges, hit_dyn

    def exclude_boxes_from_collision(self, boxes, margin=0.06):
        """맵에 그려진 박스 외곽은 벽이 아니다: 설정 박스 둘레(margin 포함) 칸을 맵 충돌 검사에서 뺀다.
        실제 박스와의 접촉은 동적 격자(참 박스)로 따로 판정한다."""
        mask = self.occupied.copy()
        for b in boxes:
            h = b.side / 2 + margin
            r0, c0 = self.to_cell(b.x - 1.5 * h, b.y - 1.5 * h)
            r1, c1 = self.to_cell(b.x + 1.5 * h, b.y + 1.5 * h)
            r0, c0, r1, c1 = max(int(r0), 0), max(int(c0), 0), min(int(r1), self.h - 1), min(int(c1), self.w - 1)
            rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
            xs, ys = self.origin[0] + (cc + 0.5) * self.res, self.origin[1] + (rr + 0.5) * self.res
            co, so = math.cos(b.yaw), math.sin(b.yaw)
            lx, ly = co * (xs - b.x) + so * (ys - b.y), -so * (xs - b.x) + co * (ys - b.y)
            mask[rr, cc] &= ~((np.abs(lx) <= h) & (np.abs(ly) <= h))
        self.collision_static = mask

    def footprint_hits(self, pose, rect):
        """차체 사각형(base_link 기준 [xmin, xmax, ymin, ymax]) 안에 점유 칸이 있는지: (맵 충돌, 동적 충돌)."""
        x, y, th = pose
        xmin, xmax, ymin, ymax = rect
        s = self.res
        gx, gy = np.meshgrid(np.arange(xmin, xmax + 1e-9, s), np.arange(ymin, ymax + 1e-9, s))
        c, sn = math.cos(th), math.sin(th)
        wx = x + c * gx - sn * gy
        wy = y + sn * gx + c * gy
        r, cc = self.to_cell(wx, wy)
        ins = self.inside(r, cc)
        rr, ccc = np.where(ins, r, 0), np.where(ins, cc, 0)
        static = getattr(self, 'collision_static', self.occupied)
        return bool((static[rr, ccc] & ins).any()), bool((self.dynamic[rr, ccc] & ins).any())


# ---------------------------------------------------------------------------------------------------------------
# 박스·장애물
# ---------------------------------------------------------------------------------------------------------------
@dataclass
class Box:
    name: str
    x: float
    y: float
    yaw: float      # 한 면의 법선 방향 [rad] (planning.yaml 과 같은 정의)
    side: float = 0.205

    def corners(self):
        h = self.side / 2
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = [(h, h), (-h, h), (-h, -h), (h, -h)]
        return [(self.x + c * a - s * b, self.y + s * a + c * b) for a, b in pts]

    def contains(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(x) - self.x, np.asarray(y) - self.y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        h = self.side / 2
        return (np.abs(lx) <= h) & (np.abs(ly) <= h)

    def rasterize(self, m: 'GridMap'):
        h = self.side * 0.75
        r0, c0 = m.to_cell(self.x - h, self.y - h)
        r1, c1 = m.to_cell(self.x + h, self.y + h)
        r0, c0, r1, c1 = max(int(r0), 0), max(int(c0), 0), min(int(r1), m.h - 1), min(int(c1), m.w - 1)
        if r1 < r0 or c1 < c0:
            return
        rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
        m.dynamic[rr, cc] |= self.contains(m.origin[0] + (cc + 0.5) * m.res, m.origin[1] + (rr + 0.5) * m.res)

    def near_face(self, rx, ry):
        """로봇 쪽을 향한 면: (면 중점, 바깥 법선 각)."""
        best = None
        for k in range(4):
            n = self.yaw + k * math.pi / 2
            mx, my = self.x + math.cos(n) * self.side / 2, self.y + math.sin(n) * self.side / 2
            score = math.cos(n) * (rx - self.x) + math.sin(n) * (ry - self.y)
            if best is None or score > best[0]:
                best = (score, (mx, my), n)
        return best[1], best[2]


def box_geometry(pose, box: Box, front_face_x: float):
    """참값 박스 관측 (센서 브리지와 같은 정의).

    gap_m: 앞면 기준(base x = front_face_x)에서 박스 앞면 중점까지의 base x 거리
    lateral_m: 앞면 중점의 base y
    yaw_rad: 로봇 진행 방향에서 본 박스 안쪽 법선(로봇→박스)의 각
    """
    x, y, th = pose
    (mx, my), n_out = box.near_face(x, y)
    c, s = math.cos(th), math.sin(th)
    dx, dy = mx - x, my - y
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    yaw_rel = wrap((n_out + math.pi) - th)
    return {'gap_m': bx - front_face_x, 'lateral_m': by, 'yaw_rad': yaw_rel,
            'face_mid': (mx, my), 'face_normal_out': n_out}


def lidar_box_fit(pose, box: Box, expected, side: float, front_face_x: float, lidar_x: float = 0.155,
                  range_min: float = 0.10, range_max: float = 1.0, beam_deg: float = 1.0):
    """실제 센서 브리지 fit_box_face(uppercam_lidar_geometry.py)와 같은 계약의 참값 LiDAR 박스 관측.

    - 기대 면: 요청의 expected_box(설정 위치)에서 로봇 쪽 면. 접선 방향 중심은 이 prior 를 그대로 쓴다
      ("a face line alone cannot identify" the lateral centre).
    - 측정: 참 박스의 대응 면 직선(기울기 a, 법선 방향 offset b).
    - gap: 로봇 전방축이 면 평면과 만나는 거리 - front_face_x. box_yaw: 바깥 법선.
    - 보이는지: 앞 라이다(x=lidar_x)에서 1° 간격 광선이 원점 거리 range_min~range_max 로 면에 닿는 점이
      ROI(법선 ±0.10, 접선 ±(side/2+0.05)) 안에 6개 이상이고 길이 0.08 m 이상, 기울기 20° 이하, |denom|>=0.3.
    반환: dict 또는 (None, 이유)."""
    rx, ry, th = pose
    bx, by, byaw = expected
    robot = np.array([rx, ry])
    center = np.array([bx, by])
    cand = np.array([[math.cos(byaw + k * math.pi / 2), math.sin(byaw + k * math.pi / 2)] for k in range(4)])
    normal = cand[np.argmax(cand @ (robot - center))]
    tangent = np.array([-normal[1], normal[0]])
    e = center + normal * side / 2
    # 참 박스에서 기대 법선과 가장 가까운 면
    tc = np.array([[math.cos(box.yaw + k * math.pi / 2), math.sin(box.yaw + k * math.pi / 2)] for k in range(4)])
    n_t = tc[np.argmax(tc @ normal)]
    t_t = np.array([-n_t[1], n_t[0]])
    m_t = np.array([box.x, box.y]) + n_t * box.side / 2
    # 앞 라이다에서 보이는 면 점 (가림 없음 가정)
    lx, ly = rx + lidar_x * math.cos(th), ry + lidar_x * math.sin(th)
    L = np.array([lx, ly])
    vis = []
    if (L - m_t) @ n_t > 0:  # 면 바깥쪽에서 볼 때만
        for a in np.radians(np.arange(-90, 90 + 1e-9, beam_deg)):  # 앞 라이다 180° 광선
            u = np.array([math.cos(th + a), math.sin(th + a)])
            den = u @ n_t
            if abs(den) < 1e-9:
                continue
            t = (m_t - L) @ n_t / den
            if t <= 0 or not (range_min <= t <= range_max):
                continue
            p = L + t * u
            if abs((p - m_t) @ t_t) <= box.side / 2:
                vis.append(p)
    vis = np.array(vis).reshape(-1, 2)
    if len(vis):
        cn, ct = (vis - e) @ normal, (vis - e) @ tangent
        roi = (np.abs(cn) <= 0.10) & (np.abs(ct) <= side / 2 + 0.05)
        vis, ct = vis[roi], ct[roi]
    if len(vis) < 6 or np.ptp(ct) < 0.08:
        return None, f'no unambiguous LiDAR box face (visible {len(vis)} points)'
    a = float((t_t @ normal) / (t_t @ tangent))
    if abs(a) > math.tan(math.radians(20)):
        return None, 'face slope over 20 deg'
    b = float((m_t - e) @ normal - a * ((m_t - e) @ tangent))
    fit_normal = (normal - a * tangent) / math.hypot(1, a)
    fit_face = e + b * normal
    fitted_center = fit_face - fit_normal * side / 2
    fwd = np.array([math.cos(th), math.sin(th)])
    left = np.array([-fwd[1], fwd[0]])
    denom = float(fit_normal @ fwd)
    if abs(denom) < 0.3:
        return None, 'face too oblique'
    forward_plane = float(fit_normal @ (fit_face - robot) / denom)
    inward = math.atan2(-fit_normal[1], -fit_normal[0])
    return {'box_center_map': (float(fitted_center[0]), float(fitted_center[1])),
            'box_yaw': math.atan2(fit_normal[1], fit_normal[0]),
            'gap_m': forward_plane - front_face_x, 'lateral_m': float((fit_face - robot) @ left),
            'yaw_rad': wrap(inward - th), 'points': len(vis)}, ''


@dataclass
class Obstacle:
    name: str
    shape: str                   # circle | rect
    center: tuple
    radius: float = 0.1
    size: tuple = (0.2, 0.2)
    yaw: float = 0.0
    appear: dict = field(default_factory=dict)     # {} = 처음부터 / {after_s: t} / {robot_within: [x, y, r]}
    disappear: dict = field(default_factory=dict)  # {} = 계속 / {after_s: t} (나타난 뒤부터)
    active: bool = False
    appeared_at: float | None = None
    gone: bool = False

    def contains(self, x, y):
        cx, cy = self.center
        if self.shape == 'circle':
            return (np.asarray(x) - cx) ** 2 + (np.asarray(y) - cy) ** 2 <= self.radius ** 2
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(x) - cx, np.asarray(y) - cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return (np.abs(lx) <= self.size[0] / 2) & (np.abs(ly) <= self.size[1] / 2)

    def bbox(self):
        cx, cy = self.center
        r = self.radius if self.shape == 'circle' else math.hypot(*self.size) / 2
        return cx - r, cx + r, cy - r, cy + r

    def rasterize(self, m: GridMap):
        x0, x1, y0, y1 = self.bbox()
        r0, c0 = m.to_cell(x0, y0)
        r1, c1 = m.to_cell(x1, y1)
        r0, c0 = max(int(r0), 0), max(int(c0), 0)
        r1, c1 = min(int(r1), m.h - 1), min(int(c1), m.w - 1)
        if r1 < r0 or c1 < c0:
            return
        rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
        xs = m.origin[0] + (cc + 0.5) * m.res
        ys = m.origin[1] + (rr + 0.5) * m.res
        m.dynamic[rr, cc] |= self.contains(xs, ys)

    def update(self, t_run: float | None, robot_xy) -> bool:
        """활성 상태 갱신. 바뀌었으면 True. t_run: 첫 목표를 보낸 뒤 경과 시간(None = 아직)."""
        if self.gone:
            return False
        before = self.active
        if not self.active:
            ok = False
            if not self.appear:
                ok = True
            elif 'after_s' in self.appear:
                ok = t_run is not None and t_run >= float(self.appear['after_s'])
            elif 'robot_within' in self.appear:
                px, py, r = self.appear['robot_within']
                ok = t_run is not None and math.hypot(robot_xy[0] - px, robot_xy[1] - py) <= r
            if ok:
                self.active, self.appeared_at = True, (t_run or 0.0)
        elif self.disappear and 'after_s' in self.disappear and t_run is not None:
            if t_run - self.appeared_at >= float(self.disappear['after_s']):
                self.active, self.gone = False, True
        return self.active != before


class ObstacleFilter:
    """실제 LidarObstacleFilter(perception.yaml lidar_obstacle.filter)의 단계를 따른다.
    입력 점은 이미 맵 벽 wall_margin 밖, 거리 3 m 이내로 거른 것.
      2. 지속: 점의 칸(cell_m)이 직전 persist-1 프레임마다 neighbor 칸 안에 살아남은 점을 가졌어야 함
      3. 묶음: 8-연결 칸 묶음의 이번 프레임 점 수가 min_cluster 미만이면 버림
      4. 기억: 남은 칸은 시야 밖에서도 유지, 광선이 persist 프레임 연속 지나가면 삭제(새 Navigate 에서 초기화)"""

    def __init__(self, cell=0.05, persist=3, neighbor=2, min_cluster=3):
        self.cell, self.persist, self.neighbor, self.min_cluster = cell, persist, neighbor, min_cluster
        self.history = []          # 직전 프레임들의 살아남은 칸 집합
        self.memory = {}           # 칸 -> 연속 통과 횟수

    def clear(self):
        self.memory.clear()

    def _near(self, cell, s):
        cx, cy = cell
        n = self.neighbor
        return any((cx + dx, cy + dy) in s for dx in range(-n, n + 1) for dy in range(-n, n + 1))

    def step(self, pts, rays):
        """pts (N,2) map 좌표, rays: [(ox, oy, angles(world), ranges)] → 발행할 점 (M,2)."""
        cells = [tuple(c) for c in np.floor(pts / self.cell).astype(int)] if len(pts) else []
        keep = np.zeros(len(cells), bool)
        if len(self.history) >= self.persist - 1:
            for i, c in enumerate(cells):
                keep[i] = all(self._near(c, s) for s in self.history[-(self.persist - 1):])
        # 묶음
        kept_cells = {}
        for i in np.where(keep)[0]:
            kept_cells.setdefault(cells[i], []).append(i)
        seen, groups = set(), []
        for c in kept_cells:
            if c in seen:
                continue
            stack, g = [c], []
            seen.add(c)
            while stack:
                q = stack.pop()
                g.append(q)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        r = (q[0] + dx, q[1] + dy)
                        if r in kept_cells and r not in seen:
                            seen.add(r)
                            stack.append(r)
            groups.append(g)
        out_idx, out_cells = [], set()
        for g in groups:
            idx = [i for c in g for i in kept_cells[c]]
            if len(idx) >= self.min_cluster:
                out_idx += idx
                out_cells.update(g)
        self.history = (self.history + [set(cells)])[-max(self.persist - 1, 1):]
        # 기억
        for c in out_cells:
            self.memory[c] = 0
        for c in list(self.memory):
            if c in out_cells:
                continue
            cx, cy = (c[0] + 0.5) * self.cell, (c[1] + 0.5) * self.cell
            passed = False
            for ox, oy, ang, rng in rays:
                d = math.hypot(cx - ox, cy - oy)
                b = math.atan2(cy - oy, cx - ox)
                diff = np.abs((ang - b + np.pi) % (2 * np.pi) - np.pi)
                k = int(np.argmin(diff))
                if diff[k] <= math.radians(1.0) and rng[k] > d + self.cell:
                    passed = True
                    break
            self.memory[c] = self.memory[c] + 1 if passed else 0
            if self.memory[c] >= self.persist:
                del self.memory[c]
        mem = [((c[0] + 0.5) * self.cell, (c[1] + 0.5) * self.cell) for c in self.memory if c not in out_cells]
        kept = pts[out_idx] if out_idx else np.zeros((0, 2))
        return np.vstack((kept, np.array(mem).reshape(-1, 2))) if mem else kept


def obstacles_from_scenario(items):
    out = []
    for i, it in enumerate(items or []):
        out.append(Obstacle(
            name=it.get('name', f'ob{i}'), shape=it.get('shape', 'circle'), center=tuple(it['center']),
            radius=float(it.get('radius', 0.1)), size=tuple(it.get('size', (0.2, 0.2))),
            yaw=math.radians(float(it.get('yaw_deg', 0.0))), appear=dict(it.get('appear', {}) or {}),
            disappear=dict(it.get('disappear', {}) or {})))
    return out


# ---------------------------------------------------------------------------------------------------------------
# 설정 읽기 (로봇 코드의 yaml 그대로)
# ---------------------------------------------------------------------------------------------------------------
def load_params(craft_src: Path):
    planning = yaml.safe_load((craft_src / 'craft_planning/config/planning.yaml').read_text())['planning_master']['ros__parameters']
    control = yaml.safe_load((craft_src / 'craft_control/config/control.yaml').read_text())['control_master']['ros__parameters']
    return planning, control


def boxes_from_planning(planning: dict, offsets: dict | None = None):
    ba = planning['box_approach']
    side = float(ba.get('side', 0.205))
    out = {}
    for name in ba.get('names', []):
        x, y, yaw_deg = ba[name]
        dx, dy, dyaw = (offsets or {}).get(name, (0.0, 0.0, 0.0))
        out[name] = Box(name, float(x) + dx, float(y) + dy, math.radians(float(yaw_deg) + dyaw), side)
    return out


class DiffDrive:
    """이상적 차동구동: v = r(wl+wr)/2, w = r(wr-wl)/sep (sep = 제어기와 같은 유효 바퀴 간격)."""

    def __init__(self, wheel_radius: float, separation: float, pose=(0.0, 0.0, 0.0)):
        self.r, self.sep = wheel_radius, separation
        self.x, self.y, self.th = pose
        self.v = self.w = 0.0

    def step(self, wl: float, wr: float, dt: float):
        v = self.r * (wl + wr) / 2.0
        w = self.r * (wr - wl) / self.sep
        if abs(w) < 1e-9:
            self.x += v * dt * math.cos(self.th)
            self.y += v * dt * math.sin(self.th)
        else:  # 정확한 원호 적분
            th1 = self.th + w * dt
            self.x += v / w * (math.sin(th1) - math.sin(self.th))
            self.y -= v / w * (math.cos(th1) - math.cos(self.th))
            self.th = th1
        self.th = wrap(self.th)
        self.v, self.w = v, w

    @property
    def pose(self):
        return self.x, self.y, self.th
