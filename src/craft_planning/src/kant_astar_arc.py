# Imported from JunmyeongPark/KANT_path_planner, commit 9af403b68bc4378516132201e9f215cea4518ea1.
# Planning core only; standalone output/visualization helpers were omitted. See third_party/.
# STL input/origin support adapted from the user-supplied KANT_path_planner.zip.
"""
4방향 A*(회전 비용) + 코너별 최대 원호(fillet) 삽입 Path Planner

설계 요약
  1) 20x20 블록 맵 -> 0.05 m 격자로 래스터화
  2) C-space 변환: 장애물을 (로봇 외접 반경 + margin)만큼 padding
     -> 이후 모든 충돌 검사는 로봇을 "질점"으로 보고 수행
  3) 상태 (ix, iy, heading∈{E,N,W,S}) A*, 전진 1칸 = res(+벽 근접 비용), 90° 회전 = λ
  4) 방향이 바뀌는 지점만 코너 waypoint로 압축
  5) 코너마다 들어갈 수 있는 최대 R을 이분 탐색 (구간 길이 / C-space 충돌 / 전역 상한)
  6) R >= R_MIN -> PASS, 아니면(또는 목표점/180° 반전) STOP_TURN
  7) waypoint = (x, y, θ, mode, R, v_max)
  8) 미션 목표(맵의 r/l/u/d): 진입점 E = 목표 - approach_len·u(dir) 까지 A* (E 에서 heading = dir 강제)
     -> E→목표 직진 구간 보존 (원호가 침범 못 함) -> 도착 mode = ALIGN

구성
  run(map)      : 맵 받아서 경로(PlanResult) 리턴 - print/저장/시각화 없음
  이 프로젝트에서는 path_planner.py가 ROS 입력과 Path 출력을 연결한다.
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field, asdict

import numpy as np
from scipy import ndimage

# ---------------------------------------------------------------------------
# ★ 맵 데이터 - 여기만 수정   첫 줄 = 위쪽(+y), 왼쪽 = -x
#   '#' = 장애물, '.' = 빈칸
#   'r' 'l' 'u' 'd' = 미션 목표 (puck / knob 앞 정지 위치) + 진입 방향
#        r: 오른쪽(+x)을 보며 진입, l: 왼쪽(-x), u: 위(+y), d: 아래(-y)
#        -> 글자 칸 = 로봇 중심 정지 위치, 로봇은 그 방향을 바라본 채 approach_len 만큼 직진해서 도착
#        -> 목표물('#')과 글자 사이에 빈칸 1칸 이상 필요 (padding 0.24 m)
#        -> 여러 개면 읽는 순서(위->아래, 왼->오)대로 방문. 순서를 바꾸려면 run(..., targets=[...])
#   'S' = 시작 (없으면 START 사용), 'G' = 방향 무관 목표 (r/l/u/d 가 없을 때만 사용)
#   기존 20x20 예제는 block_size=0.25로 5 m x 5 m; STL 수정본 기본값은 0.05 m
# ---------------------------------------------------------------------------
MAP = """
####################
#..................#
#..................#
#..................#
#.....######....####
#.....######....####
#..................#
#.......u..........#
#..................#
#...........###....#
#...........###..r.#
#...........###....#
#.....d............#
#..................#
#...#####.....##...#
#...#####.....##...#
#..................#
#.l................#
#..................#
####################
"""
START = (2, 2)     # (col, row)  row는 위에서부터 0
GOAL = (17, 17)    # (col, row)  맵에 r/l/u/d 가 하나도 없을 때만 사용 (방향 무관)


# heading index: 0=E(+x), 1=N(+y), 2=W(-x), 3=S(-y)
DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)
HEADING_RAD = (0.0, math.pi / 2, math.pi, -math.pi / 2)
HEADING_NAME = ("E", "N", "W", "S")
DIR_CHAR = {"r": 0, "u": 1, "l": 2, "d": 3}      # 맵 글자 -> heading index
DIR_NAME = ("r", "u", "l", "d")

STOP_TURN = "STOP_TURN"
PASS = "PASS"
ALIGN = "ALIGN"          # 진입 방향이 지정된 미션 목표 도착점 (Motion FSM 에서 ALIGN 으로 연결)


# ---------------------------------------------------------------------------
# 파라미터
# ---------------------------------------------------------------------------
@dataclass
class Params:
    block_size: float = 0.05      # 수정본 기본 격자 크기 [m]; 기존 블록 예제는 0.25를 지정
    res: float = 0.05             # A* 격자 해상도 [m]
    r_robot: float = 0.21         # 로봇 외접원 반경 [m] (실측 후 확정)
    margin: float = 0.03          # 추가 여유 [m]  -> padding = r_robot + margin
    lam: float = 0.60             # 90° 회전 1회 비용 [m 환산]
    w_prox: float = 1.0           # 벽 근접 비용 가중치
    prox_band: float = 0.15       # padding 경계에서 이 거리 안쪽이면 근접 비용 부과 [m]
    r_min: float = 0.08           # 이 값 미만이면 STOP_TURN [m]
    r_hi_global: float = 0.40     # 원호 반경 전역 상한 [m]
    r_tol: float = 0.01           # 이분 탐색 허용 오차 [m]
    arc_sample: float = 0.02      # 원호 충돌 검사 샘플 간격 [m]
    v_ref: float = 0.26           # 직선 최대 속도 [m/s] (Waffle Pi 스펙)
    w_max: float = 1.82           # 최대 각속도 [rad/s] (Waffle Pi 스펙)
    a_lat_max: float = 0.50       # 최대 횡가속도 [m/s^2]
    approach_len: float = 0.30    # 미션 목표 직전 직진 구간 길이 [m] (원호/회전 금지 구간)
    a_lon_max: float = 0.50       # 종방향 가감속 [m/s^2] - 정지 1회당 시간 손해(v_ref/a) 계산용

    @property
    def pad(self) -> float:
        return self.r_robot + self.margin


# ---------------------------------------------------------------------------
# 맵
# ---------------------------------------------------------------------------
@dataclass
class GridMap:
    occ: np.ndarray        # [iy, ix] bool, True=장애물 (원본)
    cspace: np.ndarray     # [iy, ix] bool, True=padding 포함 점유 (질점 기준 충돌)
    esdf: np.ndarray       # [iy, ix] 셀 중심 -> 가장 가까운 장애물 경계 거리 [m]
    res: float
    start: tuple[int, int]
    goal: tuple[int, int] | None
    block_size: float
    n_blocks: tuple[int, int]
    targets: list = field(default_factory=list)   # [((ix, iy), heading), ...] 맵의 r/l/u/d
    origin: tuple[float, float] = (0.0, 0.0)  # cell (0, 0) corner in world metres

    def cell_to_block(self, ix, iy):
        k = int(round(self.block_size / self.res))
        return (ix // k, self.n_blocks[1] - 1 - iy // k)

    def block_to_cell(self, col, row):
        k = int(round(self.block_size / self.res))
        return (col * k + k // 2, (self.n_blocks[1] - 1 - row) * k + k // 2)

    @property
    def shape(self):
        return self.occ.shape

    def cell_center(self, ix, iy):
        return (self.origin[0] + (ix + 0.5) * self.res,
                self.origin[1] + (iy + 0.5) * self.res)

    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.res)),
                int(math.floor((y - self.origin[1]) / self.res)))

    def point_free(self, x, y) -> bool:
        """질점 충돌 검사 (C-space)"""
        ix, iy = self.world_to_cell(x, y)
        h, w = self.shape
        if not (0 <= ix < w and 0 <= iy < h):
            return False
        return not self.cspace[iy, ix]

    def esdf_at(self, x, y) -> float:
        """연속 좌표에서 장애물 거리 (bilinear) - 검증/리포트용"""
        fx = (x - self.origin[0]) / self.res - 0.5
        fy = (y - self.origin[1]) / self.res - 0.5
        return float(ndimage.map_coordinates(self.esdf, [[fy], [fx]], order=1, mode="nearest")[0])


def parse_map(text: str) -> list[str]:
    """문자열 -> 맵 행 리스트. '#', '.', 'S', 'G', 'r', 'l', 'u', 'd' 로만 된 줄만 맵으로 인식"""
    rows = []
    for line in text.splitlines():
        line = line.strip()
        if line and set(line) <= set("#.SGrlud"):
            rows.append(line)
    if not rows:
        raise ValueError("맵 데이터가 비어 있습니다")
    return rows


def gen_random_map(n: int = 20, n_obs: int = 9, seed: int = 0,
                   start=START, goal=GOAL, p: "Params | None" = None, max_trial: int = 500) -> list[str]:
    """랜덤 n x n 맵 생성 ('#'/'.'). 시작-목표가 padding 후에도 연결될 때까지 재시도"""
    import random
    rng = random.Random(seed)
    p = p or Params()
    for _ in range(max_trial):
        g = [["#" if r in (0, n - 1) or c in (0, n - 1) else "." for c in range(n)] for r in range(n)]
        for _ in range(n_obs):
            h, w = rng.randint(1, 5), rng.randint(1, 5)
            r, c = rng.randint(1, n - 1 - h), rng.randint(1, n - 1 - w)
            for i in range(r, r + h):
                for j in range(c, c + w):
                    g[i][j] = "#"
        # 시작/목표 주변 3x3 은 비움
        for (c0, r0) in (start, goal):
            for i in range(r0 - 1, r0 + 2):
                for j in range(c0 - 1, c0 + 2):
                    if 0 < i < n - 1 and 0 < j < n - 1:
                        g[i][j] = "."
        rows = ["".join(row) for row in g]
        try:
            run(rows, start, goal, p)
            return rows
        except (ValueError, RuntimeError):
            continue
    raise RuntimeError("연결된 랜덤 맵 생성 실패 - n_obs 를 줄여보세요")


def build_map(rows: list[str], p: Params, start=START, goal=GOAL) -> GridMap:
    """'#'/'.' 블록 맵 -> 0.05 m 격자 + ESDF + C-space(padding)"""
    nb_y = len(rows)
    nb_x = max(len(r) for r in rows)
    k = int(round(p.block_size / p.res))           # 블록당 셀 수
    H, W = nb_y * k, nb_x * k
    occ = np.zeros((H, W), dtype=bool)

    def block_to_cell(col, row):
        by = nb_y - 1 - row                        # 첫 줄 = 위쪽(+y)
        return (col * k + k // 2, by * k + k // 2)

    s_cell = block_to_cell(*start) if start is not None else None
    g_cell = block_to_cell(*goal) if goal is not None else None
    targets = []
    for r, line in enumerate(rows):
        by = nb_y - 1 - r
        for bx, ch in enumerate(line.ljust(nb_x, "#")):
            if ch == "#":
                occ[by * k:(by + 1) * k, bx * k:(bx + 1) * k] = True
            elif ch == "S":
                s_cell = block_to_cell(bx, r)
            elif ch == "G":
                g_cell = block_to_cell(bx, r)
            elif ch in DIR_CHAR:
                targets.append((block_to_cell(bx, r), DIR_CHAR[ch]))
    if s_cell is None or (g_cell is None and not targets):
        raise ValueError("시작/목표가 없습니다 (START/GOAL 지정 또는 맵에 S/G/r/l/u/d 표시)")

    # 맵 바깥도 벽으로 취급하기 위해 1셀 패딩 후 거리장 계산
    padded = np.pad(occ, 1, constant_values=True)
    edt = ndimage.distance_transform_edt(~padded)[1:-1, 1:-1]
    esdf = np.maximum(edt * p.res - 0.5 * p.res, 0.0)   # 셀 중심 -> 장애물 셀 경계
    cspace = esdf < p.pad                                 # padding = r_robot + margin
    return GridMap(occ, cspace, esdf, p.res, s_cell, g_cell, p.block_size, (nb_x, nb_y), targets)


def load_block_map(path: str, p: Params, start=START, goal=GOAL) -> GridMap:
    """텍스트 파일 맵 로드 (파일 안에 S/G 가 있으면 그 위치 사용)"""
    with open(path, encoding="utf-8") as f:
        return build_map(parse_map(f.read()), p, start, goal)


def load_stl_map(path: str, p: Params, start, goal=None, targets=None,
                 heights=(0.10, 0.20, 0.30)) -> GridMap:
    """Binary STL (metres, Z-up) -> horizontal wall slices -> occupancy/ESDF/C-space.

    start, goal and targets are Gazebo/world coordinates in metres, not block indices.
    The returned GridMap and run() results retain the mesh's XY world origin.
    """
    heights = np.asarray(heights, dtype=float)
    if heights.ndim != 1 or not heights.size or not np.isfinite(heights).all():
        raise ValueError('STL slice heights must be a nonempty list of finite metres')
    if not math.isfinite(p.res) or p.res <= 0 or not math.isfinite(p.pad) or p.pad <= 0:
        raise ValueError('STL resolution and robot radius plus margin must be positive')
    dtype = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
    with open(path, 'rb') as stream:
        header = stream.read(84)
        if len(header) != 84:
            raise ValueError('Invalid binary STL header')
        count = int.from_bytes(header[80:84], 'little')
        stream.seek(0, 2)
        if not count or stream.tell() != 84 + count * dtype.itemsize:
            raise ValueError('Expected binary STL with a valid triangle count (ASCII STL is unsupported)')
        stream.seek(84)
        tri = np.fromfile(stream, dtype=dtype, count=count)['v'].astype(float)
    if not np.isfinite(tri).all():
        raise ValueError('STL contains non-finite vertices')
    segments = []
    for height in heights:
        t = tri[(tri[:, :, 2].min(1) < height) & (tri[:, :, 2].max(1) > height)]
        if not len(t):
            continue
        a, b = t, np.roll(t, -1, axis=1)
        dz = b[:, :, 2] - a[:, :, 2]
        hit = ((np.minimum(a[:, :, 2], b[:, :, 2]) <= height) &
               (np.maximum(a[:, :, 2], b[:, :, 2]) >= height) & (dz != 0))
        fraction = np.where(hit, (height - a[:, :, 2]) / np.where(dz == 0, 1, dz), 0)
        points = a[:, :, :2] + fraction[:, :, None] * (b[:, :, :2] - a[:, :, :2])
        first_two = np.argsort(~hit, axis=1, kind='stable')[:, :2]
        segments.append(np.take_along_axis(points, first_two[:, :, None], axis=1))
    if not segments:
        raise ValueError('STL has no wall intersections at the configured slice heights')
    segments = np.concatenate(segments)
    # Match the supplied loader: sample each intersection at resolution/4 and union the slices.
    samples = np.ceil(np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1) /
                      (p.res / 4)).astype(int) + 1
    indices = np.repeat(np.arange(len(segments)), samples)
    fraction = ((np.arange(samples.sum()) - np.repeat(np.cumsum(samples) - samples, samples)) /
                np.maximum(np.repeat(samples, samples) - 1, 1))
    points = segments[indices, 0] + fraction[:, None] * (segments[indices, 1] - segments[indices, 0])
    origin = points.min(0)
    ix, iy = np.floor((points - origin) / p.res).astype(int).T
    occupied = np.zeros((iy.max() + 1, ix.max() + 1), dtype=bool)
    occupied[iy, ix] = True
    padded = np.pad(occupied, 1, constant_values=True)
    distance = ndimage.distance_transform_edt(~padded)[1:-1, 1:-1]
    esdf = np.maximum(distance * p.res - 0.5 * p.res, 0.0)

    def cell(x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('STL start/goal/targets must be finite world coordinates')
        return (int(math.floor((x - origin[0]) / p.res)),
                int(math.floor((y - origin[1]) / p.res)))

    marked_targets = [(cell(x, y), DIR_CHAR[direction] if direction else None)
                      for x, y, direction in targets or []]
    height, width = occupied.shape
    return GridMap(occupied, esdf < p.pad, esdf, p.res, cell(*start),
                   cell(*goal) if goal is not None else None, p.res, (width, height),
                   marked_targets, (float(origin[0]), float(origin[1])))


# ---------------------------------------------------------------------------
# A* (x, y, heading)
# ---------------------------------------------------------------------------
def _min_turns(h: int, dx: int, dy: int) -> int:
    need = set()
    if dx > 0: need.add(0)
    if dx < 0: need.add(2)
    if dy > 0: need.add(1)
    if dy < 0: need.add(3)
    if not need:
        return 0
    if h in need:
        return len(need) - 1
    if len(need) == 1:
        (d,) = need
        return 2 if (d - h) % 4 == 2 else 1
    return 2


def astar(m: GridMap, p: Params, start, start_h: int, goal, goal_h: int | None = None):
    """(start, start_h) -> goal. goal_h 지정 시 goal 에서 그 heading 이어야 도착으로 인정"""
    H, W = m.shape
    free = ~m.cspace
    sx, sy = start
    gx, gy = goal
    if not free[sy, sx]:
        raise ValueError(f"시작점이 padding 영역 안에 있습니다 (장애물 거리 {m.esdf[sy, sx]:.3f} m < {p.pad:.3f} m)")
    if not free[gy, gx]:
        raise ValueError(f"목표점이 padding 영역 안에 있습니다 (장애물 거리 {m.esdf[gy, gx]:.3f} m < {p.pad:.3f} m)")

    # 벽 근접 비용: padding 경계에서 prox_band 이내일수록 증가 (0~w_prox)
    prox = np.clip((p.pad + p.prox_band - m.esdf) / p.prox_band, 0.0, 1.0) * p.w_prox
    step_cost = p.res * (1.0 + prox)

    def hfun(ix, iy, h):
        dx, dy = gx - ix, gy - iy
        t = _min_turns(h, dx, dy)
        if goal_h is not None and dx == 0 and dy == 0 and h != goal_h:
            t = 2 if (goal_h - h) % 4 == 2 else 1      # 목표 셀에서 제자리 회전 필요
        return p.res * (abs(dx) + abs(dy)) + p.lam * t

    g = np.full((H, W, 4), np.inf)
    parent = {}
    g[sy, sx, start_h] = 0.0
    heap = [(hfun(sx, sy, start_h), 0.0, sx, sy, start_h)]
    closed = np.zeros((H, W, 4), dtype=bool)
    expanded = 0
    goal_state = None
    while heap:
        f, gc, ix, iy, h = heapq.heappop(heap)
        if closed[iy, ix, h]:
            continue
        closed[iy, ix, h] = True
        expanded += 1
        if (ix, iy) == (gx, gy) and (goal_h is None or h == goal_h):
            goal_state = (ix, iy, h)
            break
        # 전진
        nx, ny = ix + DX[h], iy + DY[h]
        succ = []
        if 0 <= nx < W and 0 <= ny < H and free[ny, nx]:
            succ.append((nx, ny, h, step_cost[ny, nx]))
        # 제자리 90° 회전
        succ.append((ix, iy, (h + 1) % 4, p.lam))
        succ.append((ix, iy, (h - 1) % 4, p.lam))
        for nx, ny, nh, c in succ:
            ng = gc + c
            if ng < g[ny, nx, nh]:
                g[ny, nx, nh] = ng
                parent[(nx, ny, nh)] = (ix, iy, h)
                heapq.heappush(heap, (ng + hfun(nx, ny, nh), ng, nx, ny, nh))
    if goal_state is None:
        raise RuntimeError(f"경로 없음 {start} -> {goal} (padding 적용 후 연결되지 않음)")
    states = [goal_state]
    while states[-1] in parent:
        states.append(parent[states[-1]])
    states.reverse()
    return states, float(g[goal_state[1], goal_state[0], goal_state[2]]), expanded


# ---------------------------------------------------------------------------
# 코너 압축 + fillet
# ---------------------------------------------------------------------------
@dataclass
class Waypoint:
    x: float
    y: float
    theta: float          # 이 waypoint 이후 진행 heading [rad]
    mode: str
    R: float = 0.0
    v_max: float = 0.0
    reason: str = ""      # mode 결정 이유 (디버깅용)
    # 내부용
    h_in: int | None = field(default=None, repr=False)
    h_out: int | None = field(default=None, repr=False)
    leg: int = 0          # 몇 번째 목표로 가는 구간인지 (0부터)

    def to_dict(self):
        d = asdict(self)
        d.pop("h_in"); d.pop("h_out")
        d["theta_deg"] = round(math.degrees(self.theta), 1)
        return d


def compress_corners(states, m: GridMap):
    """A* 상태열 -> [start, corner..., goal] (셀 좌표, h_in, h_out)"""
    pts = []  # (ix, iy, h_in, h_out)
    ix0, iy0, h0 = states[0]
    first_move_h = None
    for (ix, iy, h), (nx, ny, nh) in zip(states, states[1:]):
        if (nx, ny) != (ix, iy):
            first_move_h = h
            break
    pts.append((ix0, iy0, h0, first_move_h if first_move_h is not None else h0))
    # 위치가 바뀌는 전진 동작만 추림
    moves = []  # (ix, iy, h) : h 방향으로 (ix,iy)에서 출발
    for (ix, iy, h), (nx, ny, nh) in zip(states, states[1:]):
        if (nx, ny) != (ix, iy):
            moves.append((ix, iy, h))
    for (ix, iy, h), (jx, jy, jh) in zip(moves, moves[1:]):
        if jh != h:
            pts.append((jx, jy, h, jh))
    gx, gy, gh = states[-1]
    last_h = moves[-1][2] if moves else h0
    pts.append((gx, gy, last_h, gh))
    return pts


def make_fillet(C, u_in, u_out, R, ds):
    """90° 코너 C에 반경 R 원호. 반환: 샘플점 배열, (T1, T2, O, 시작각, sweep)"""
    C = np.asarray(C, float)
    u_in = np.asarray(u_in, float)
    u_out = np.asarray(u_out, float)
    T1 = C - u_in * R
    T2 = C + u_out * R
    O = C - u_in * R + u_out * R
    turn = np.sign(u_in[0] * u_out[1] - u_in[1] * u_out[0])  # +1 좌회전, -1 우회전
    a0 = math.atan2(T1[1] - O[1], T1[0] - O[0])
    sweep = turn * math.pi / 2
    n = max(2, int(math.ceil(abs(sweep) * R / ds)) + 1)
    a = a0 + np.linspace(0.0, sweep, n)
    pts = np.stack([O[0] + R * np.cos(a), O[1] + R * np.sin(a)], axis=1)
    return pts, (T1, T2, O, a0, sweep)


def arc_free(m: GridMap, pts) -> bool:
    return all(m.point_free(x, y) for x, y in pts)


def max_fillet_radius(m: GridMap, p: Params, C, u_in, u_out, len_in_avail, len_out_avail):
    R_hi = min(p.r_hi_global, len_in_avail, len_out_avail)
    R_lo = 0.0
    if R_hi <= 0:
        return 0.0, 0
    # 상한 자체가 가능하면 바로 채택
    if arc_free(m, make_fillet(C, u_in, u_out, R_hi, p.arc_sample)[0]):
        return R_hi, 1
    it = 1
    while R_hi - R_lo > p.r_tol:
        R = 0.5 * (R_lo + R_hi)
        it += 1
        if arc_free(m, make_fillet(C, u_in, u_out, R, p.arc_sample)[0]):
            R_lo = R
        else:
            R_hi = R
    return R_lo, it


def v_for_radius(p: Params, R: float) -> float:
    if R <= 0:
        return 0.0
    return min(p.v_ref, p.w_max * R, math.sqrt(p.a_lat_max * R))


def build_waypoints(states, m: GridMap, p: Params, goal_heading: int | None = None,
                    approach: float = 0.0, leg: int = 0):
    """approach > 0 이면 목표 직전 approach [m] 는 직선으로 보존 (원호가 침범 못 함), 도착 mode = ALIGN"""
    pts = compress_corners(states, m)
    xy = [np.array(m.cell_center(ix, iy)) for ix, iy, _, _ in pts]
    n = len(pts)
    wps: list[Waypoint] = []
    stats = []

    # 1단계: 의미상 강제 STOP_TURN 판정
    forced = [None] * n
    forced[0] = "start"
    forced[-1] = f"goal ALIGN ({DIR_NAME[goal_heading]})" if approach > 0 else "goal"
    for i in range(1, n - 1):
        h_in, h_out = pts[i][2], pts[i][3]
        if (h_out - h_in) % 4 == 2:
            forced[i] = "180° reversal"

    R_used = [0.0] * n
    for i in range(n):
        ix, iy, h_in, h_out = pts[i]
        x, y = xy[i]
        if forced[i] is not None:
            th = h_out
            if i == n - 1 and goal_heading is not None:
                th = goal_heading
            mode = ALIGN if (i == n - 1 and approach > 0) else STOP_TURN
            wps.append(Waypoint(x, y, HEADING_RAD[th], mode, 0.0, 0.0, forced[i], h_in, th, leg))
            continue
        # 2단계: R 상한 (구간 길이)
        len_in = float(np.abs(xy[i] - xy[i - 1]).sum())
        len_out = float(np.abs(xy[i + 1] - xy[i]).sum())
        in_avail = len_in - R_used[i - 1]
        next_is_candidate = forced[i + 1] is None
        out_avail = 0.5 * len_out if next_is_candidate else len_out
        if i == n - 2 and approach > 0:
            out_avail = len_out - approach        # 진입 직선 구간 보존
        if out_avail <= 1e-9:
            wps.append(Waypoint(x, y, HEADING_RAD[h_out], STOP_TURN, 0.0, 0.0,
                                "approach entry (turn in place)", h_in, h_out, leg))
            continue
        u_in = (DX[h_in], DY[h_in])
        u_out = (DX[h_out], DY[h_out])
        # 3단계: 이분 탐색
        R, iters = max_fillet_radius(m, p, (x, y), u_in, u_out, in_avail, out_avail)
        limit = "global" if R >= p.r_hi_global - 1e-9 else (
            "segment" if R >= min(in_avail, out_avail) - 1e-9 else "obstacle")
        if R >= p.r_min:
            R_used[i] = R
            if i == n - 2 and approach > 0 and R >= out_avail - 1e-9:
                limit = "approach"
            wps.append(Waypoint(x, y, HEADING_RAD[h_out], PASS, R, v_for_radius(p, R),
                                f"R limited by {limit}", h_in, h_out, leg))
        else:
            wps.append(Waypoint(x, y, HEADING_RAD[h_out], STOP_TURN, 0.0, 0.0,
                                f"R={R:.3f} < R_MIN ({limit})", h_in, h_out, leg))
        stats.append(dict(i=i, in_avail=in_avail, out_avail=out_avail, R=R, iters=iters))
    return wps, stats


# ---------------------------------------------------------------------------
# 최종 경로 (직선 + 원호) 샘플링
# ---------------------------------------------------------------------------
def build_path(wps: list[Waypoint], p: Params, ds: float = 0.02):
    """반환: (N,5) 배열 [x, y, yaw, kappa, v_limit]"""
    out = []
    cur = np.array([wps[0].x, wps[0].y])

    def add_line(a, b, yaw, v):
        L = float(np.linalg.norm(b - a))
        if L < 1e-9:
            return
        k = max(1, int(math.ceil(L / ds)))
        for t in np.linspace(0, 1, k, endpoint=False):
            q = a + (b - a) * t
            out.append((q[0], q[1], yaw, 0.0, v))

    for i in range(1, len(wps)):
        w = wps[i]
        prev = wps[i - 1]
        C = np.array([w.x, w.y])
        yaw_in = HEADING_RAD[w.h_in] if w.h_in is not None else prev.theta
        if w.mode == PASS:
            u_in = np.array([DX[w.h_in], DY[w.h_in]])
            u_out = np.array([DX[w.h_out], DY[w.h_out]])
            arc, (T1, T2, O, a0, sweep) = make_fillet(C, u_in, u_out, w.R, ds)
            add_line(cur, T1, yaw_in, p.v_ref)
            turn = np.sign(sweep)
            for j, (qx, qy) in enumerate(arc[:-1]):
                a = a0 + sweep * j / (len(arc) - 1)
                yaw = a + turn * math.pi / 2
                out.append((qx, qy, math.atan2(math.sin(yaw), math.cos(yaw)), turn / w.R, w.v_max))
            cur = T2
        else:
            add_line(cur, C, yaw_in, p.v_ref)
            cur = C
    out.append((wps[-1].x, wps[-1].y, wps[-1].theta, 0.0, 0.0))
    return np.array(out)


def path_length(path) -> float:
    return float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())


# ===========================================================================
# 실행부 (API) - 다른 모듈에서 import 해서 사용
#   from astar_arc import run, MAP
#   res = run(MAP, start=(2, 2), goal=(17, 17))
#   res.waypoints -> [Waypoint(x, y, theta, mode, R, v_max), ...]
#   res.path      -> (N,5) [x, y, yaw, kappa, v_limit]
# ===========================================================================
@dataclass
class Leg:
    """목표 하나까지의 구간"""
    start: tuple[int, int]
    start_h: int
    goal: tuple[int, int]
    goal_h: int | None            # None 이면 방향 무관
    states: list                  # A* 상태열 [(ix, iy, h), ...] (진입 직선 포함)
    cost: float
    expanded: int
    waypoints: list[Waypoint]
    path: np.ndarray              # (N,5) [x, y, yaw, kappa, v_limit]


@dataclass
class PlanResult:
    map: GridMap
    params: Params
    legs: list[Leg]
    t_astar_ms: float
    t_arc_ms: float
    raw_length: float            # A* 계단 경로 길이 [m]
    length: float                # 원호 삽입 후 길이 [m]
    min_clearance: float         # 최종 경로 최소 장애물 거리 [m]
    t_drive: float               # 예상 주행 시간 (가감속 무시) [s]
    t_turn: float                # 예상 제자리 회전 시간 [s]

    # 전체 구간을 이어 붙인 값 (구간별은 legs[i] 사용)
    @property
    def waypoints(self) -> list[Waypoint]:
        return [w for leg in self.legs for w in leg.waypoints]

    @property
    def path(self) -> np.ndarray:
        return np.vstack([leg.path for leg in self.legs])

    @property
    def states(self) -> list:
        return [s for leg in self.legs for s in leg.states]

    @property
    def cost(self) -> float:
        return sum(leg.cost for leg in self.legs)

    @property
    def expanded(self) -> int:
        return sum(leg.expanded for leg in self.legs)

    @property
    def n_pass(self) -> int:
        return sum(w.mode == PASS for w in self.waypoints)

    @property
    def n_stop(self) -> int:
        return sum(w.mode == STOP_TURN for w in self.waypoints)

    @property
    def n_align(self) -> int:
        return sum(w.mode == ALIGN for w in self.waypoints)

    @property
    def ok(self) -> bool:
        """최종 경로가 padding 조건을 만족하는지 (격자 양자화 res/2 허용)"""
        return self.min_clearance >= self.params.pad - self.params.res / 2


def _leg_time(wps: list[Waypoint], path: np.ndarray, p: Params) -> float:
    """예상 소요 시간 = 주행 + 제자리 회전 + 정지 가감속 손해"""
    t_drive, t_turn = _time_parts(wps, path, p)
    return t_drive + t_turn


def _time_parts(wps: list[Waypoint], path: np.ndarray, p: Params):
    seg = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    t_drive = float((seg / np.maximum(path[:-1, 4], 1e-3)).sum())
    n90 = sum(min((w.h_out - w.h_in) % 4, (w.h_in - w.h_out) % 4)
              for w in wps if w.mode == STOP_TURN and w.h_in is not None and w.h_out is not None)
    # 중간 정지(STOP_TURN, 시작 제외) 1회당 감속+가속 손해 ≈ v_ref / a_lon_max
    n_stop = sum(1 for w in wps if w.mode == STOP_TURN and w.reason != "start")
    return t_drive, n90 * (math.pi / 2) / p.w_max + n_stop * p.v_ref / p.a_lon_max


def plan_leg(m: GridMap, p: Params, start, start_h: int, goal, goal_h: int | None, leg: int = 0) -> Leg:
    """한 목표까지 계획.
    goal_h 지정 시 (미션 목표):
      진입점 E = goal - L·u(goal_h) 까지 A* (E 에서 heading = goal_h 필수) + E→goal 직진
      approach_len 구간에는 원호/회전 금지. L 은 approach_len + {R_hi, R_hi/2, 0} 을 시도해서
      (진입 전 코너에 원호가 들어갈 여유를 주기 위함) 예상 시간이 가장 짧은 것을 채택
    """
    if goal_h is None:
        states, cost, expanded = astar(m, p, start, start_h, goal)
        wps, _ = build_waypoints(states, m, p, None, 0.0, leg)
        return Leg(start, start_h, goal, None, states, cost, expanded, wps, build_path(wps, p))

    n_app = max(1, int(round(p.approach_len / p.res)))
    gx, gy = goal
    dx, dy = DX[goal_h], DY[goal_h]
    H, W = m.shape

    def cells(n):
        return [(gx - (n - i) * dx, gy - (n - i) * dy) for i in range(n + 1)]   # E ... goal

    def clear(cs):
        return all(0 <= cx < W and 0 <= cy < H and not m.cspace[cy, cx] for cx, cy in cs)

    name = f"목표 '{DIR_NAME[goal_h]}' (col, row)={m.cell_to_block(*goal)}"
    if not clear([goal]):
        raise ValueError(f"{name}: 목표점이 padding 안 (장애물 거리 {m.esdf[gy, gx]:.3f} m < {p.pad:.3f} m) "
                         f"-> 장애물/벽에서 한 칸 더 떨어뜨릴 것")
    if not clear(cells(n_app)):
        raise ValueError(f"{name}: 진입 직선 {p.approach_len} m 가 padding 에 걸림 "
                         f"-> 목표 뒤쪽(진입 방향 반대편)에 공간 확보 필요")
    best, expanded_total = None, 0
    extras = sorted({int(round(p.r_hi_global / p.res)), int(round(0.5 * p.r_hi_global / p.res)), 0}, reverse=True)
    for extra in extras:
        cs = cells(n_app + extra)
        if not clear(cs):
            continue
        try:
            states, cost, expanded = astar(m, p, start, start_h, cs[0], goal_h)
        except RuntimeError:
            continue
        expanded_total += expanded
        states = states + [(cx, cy, goal_h) for cx, cy in cs[1:]]
        cost += (len(cs) - 1) * p.res
        wps, _ = build_waypoints(states, m, p, goal_h, n_app * p.res, leg)
        path = build_path(wps, p)
        t = _leg_time(wps, path, p)
        if best is None or t < best[0] - 1e-6:
            best = (t, states, cost, wps, path)
    if best is None:
        raise RuntimeError(f"{name}: 경로 없음")
    _, states, cost, wps, path = best
    return Leg(start, start_h, goal, goal_h, states, cost, expanded_total, wps, path)


def run(map_data=MAP, start=START, goal=GOAL, p: Params | None = None,
        start_heading: int = 0, goal_heading: int | None = None, targets=None) -> PlanResult:
    """맵을 받아 경로를 리턴 (print / 저장 / 시각화 없음)

    map_data : 맵 문자열, 행 리스트, 또는 build_map() 으로 만든 GridMap
    start    : (col, row) 블록 좌표 (맵에 S 가 있으면 S 우선)
    targets  : [(col, row, 'r'|'l'|'u'|'d'|None), ...] 방문 순서 지정. None 이면 맵의 r/l/u/d 를 읽는 순서대로
               (맵에 r/l/u/d 가 없으면 goal / goal_heading 하나)
    start_heading / goal_heading : 0=E(r), 1=N(u), 2=W(l), 3=S(d)
    """
    p = p or Params()
    if isinstance(map_data, GridMap):
        m = map_data
    else:
        rows = parse_map(map_data) if isinstance(map_data, str) else list(map_data)
        m = build_map(rows, p, start, goal)

    if targets is not None:
        tgt = [(m.block_to_cell(c, r), DIR_CHAR[d] if d else None) for c, r, d in targets]
    elif m.targets:
        tgt = list(m.targets)
    else:
        tgt = [(m.goal, goal_heading)]

    t0 = time.perf_counter()
    legs = []
    cur, cur_h = m.start, start_heading
    for i, (g, gh) in enumerate(tgt):
        leg = plan_leg(m, p, cur, cur_h, g, gh, i)
        legs.append(leg)
        cur, cur_h = g, leg.states[-1][2]
    t1 = time.perf_counter()

    path = np.vstack([leg.path for leg in legs])
    wps = [w for leg in legs for w in leg.waypoints]
    clear = min(m.esdf_at(x, y) for x, y in path[:, :2])
    raw_len = sum(sum(1 for (a1, b1, _), (a2, b2, _) in zip(leg.states, leg.states[1:]) if (a1, b1) != (a2, b2))
                  for leg in legs) * p.res
    t_drive, t_turn = _time_parts(wps, path, p)
    # A* 와 원호 계산이 구간마다 섞여 있으므로 전체 시간을 t_astar_ms 에 기록
    return PlanResult(m, p, legs, 1e3 * (t1 - t0), 0.0,
                      raw_len, path_length(path), clear, t_drive, t_turn)
