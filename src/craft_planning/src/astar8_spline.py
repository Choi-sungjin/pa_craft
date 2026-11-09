"""8-connected A* with spline smoothing, for comparison with A*+arc (astar_arc.py) on the same C-space.

Both variants search the GridMap that astar_arc builds (cspace = ESDF < r_robot + margin, same proximity cost
res * (1 + prox) per cell length). Coordinates are the grid's local metres.

  smooth_spline   8-connected A* cells -> cubic B-spline through them (smoothing s halved until collision free)
  smooth_curve    8-connected A* cells -> line-of-sight pruning (any-angle polyline) -> iterative smoothing
                  with clearance and curvature terms (points stay in free C-space) -> cubic spline

Goal heading (as A*+arc's ALIGN): with goal_h (0 E, 1 N, 2 W, 3 S) the search ends at the entry point
E = goal - approach_len * u(goal_h); the last approach_len is the exact straight E -> goal and the smoothed part
is pulled onto that line (weighted spline / fixed points), so the robot arrives along the goal heading.

profile() gives the same speed/time model for every path: v <= v_ref, sqrt(a_lat_max / k), w_max / k,
accelerate/brake at a_lon_max, v = 0 at in-place turns (their time |dyaw| / w_max is added).
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import interpolate

SQ2 = math.sqrt(2.0)
MOVES = [(1, 0, 1.0), (1, 1, SQ2), (0, 1, 1.0), (-1, 1, SQ2), (-1, 0, 1.0), (-1, -1, SQ2), (0, -1, 1.0), (1, -1, SQ2)]
HEADING = {0: (1, 0), 1: (0, 1), 2: (-1, 0), 3: (0, -1)}


@dataclass
class SmoothPath:
    xy: np.ndarray                  # (N, 2) local metres, ~ds spacing
    kappa: np.ndarray               # (N,) signed curvature [1/m]
    stops: list = field(default_factory=list)   # [(index, turn angle rad)] in-place turns
    cells: list = field(default_factory=list)   # raw A* cells (to the entry point)
    key: np.ndarray | None = None   # pruned / smoothed control points (curve variant)
    expanded: int = 0


def _prox_cost(m, p):
    prox = np.clip((p.pad + p.prox_band - m.esdf) / p.prox_band, 0.0, 1.0) * p.w_prox
    return m.res * (1.0 + prox)          # the map's cell, as the C++ planner (Params.res is the STL/text grid)


def astar8(m, p, start, goal):
    """8-connected A* over free C-space cells; diagonal moves may not cut a blocked corner."""
    H, W = m.shape
    free = ~m.cspace
    (sx, sy), (gx, gy) = start, goal
    if not free[sy, sx] or not free[gy, gx]:
        raise ValueError('start or goal inside the padding')
    step = _prox_cost(m, p)

    def h(ix, iy):
        dx, dy = abs(gx - ix), abs(gy - iy)
        return m.res * (max(dx, dy) + (SQ2 - 1) * min(dx, dy))

    g = np.full((H, W), np.inf); g[sy, sx] = 0.0
    parent = {}
    closed = np.zeros((H, W), bool)
    heap = [(h(sx, sy), 0.0, sx, sy)]
    expanded = 0
    while heap:
        _, gc, ix, iy = heapq.heappop(heap)
        if closed[iy, ix]:
            continue
        closed[iy, ix] = True; expanded += 1
        if (ix, iy) == (gx, gy):
            break
        for dx, dy, length in MOVES:
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < W and 0 <= ny < H) or not free[ny, nx]:
                continue
            if dx and dy and not (free[iy, nx] and free[ny, ix]):
                continue
            ng = gc + length * step[ny, nx]
            if ng < g[ny, nx]:
                g[ny, nx] = ng; parent[(nx, ny)] = (ix, iy)
                heapq.heappush(heap, (ng + h(nx, ny), ng, nx, ny))
    if not closed[gy, gx]:
        raise RuntimeError('no route through the inflated map')
    cells = [(gx, gy)]
    while cells[-1] in parent:
        cells.append(parent[cells[-1]])
    return cells[::-1], expanded


def _centres(m, cells):
    return np.array([m.cell_center(ix, iy) for ix, iy in cells])


def _cells(m, pts):
    return (np.floor((pts[:, 0] - m.origin[0]) / m.res).astype(int),
            np.floor((pts[:, 1] - m.origin[1]) / m.res).astype(int))


def _free(m, pts):
    H, W = m.shape
    ix, iy = _cells(m, pts)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    out = np.zeros(len(pts), bool)
    out[ok] = ~m.cspace[iy[ok], ix[ok]]
    return out


def _esdf(m, pts):
    H, W = m.shape
    ix, iy = _cells(m, pts)
    return m.esdf[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)]


def _segment_free(m, a, b):
    n = max(2, int(math.ceil(np.linalg.norm(b - a) / (m.res / 2))) + 1)
    return bool(_free(m, np.linspace(a, b, n)).all())


def _resample(xy, ds):
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(0, s[-1], max(2, int(math.ceil(s[-1] / ds)) + 1))
    return np.column_stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])])


def _poly_kappa(xy):
    """Curvature of a polyline sampled at ~constant ds: turning angle / ds (0 on straight runs)."""
    k = np.zeros(len(xy))
    if len(xy) > 2:
        d = np.diff(xy, axis=0)
        a = np.arctan2(d[:, 1], d[:, 0])
        da = (np.diff(a) + math.pi) % (2 * math.pi) - math.pi
        k[1:-1] = da / np.maximum(0.5 * (np.linalg.norm(d[1:], axis=1) + np.linalg.norm(d[:-1], axis=1)), 1e-9)
    return k


def _spline(m, pts, s, ds, w=None):
    """Cubic B-spline through pts, sampled every ds of arc length with its analytic curvature.

    Returns (xy, kappa) or None when it leaves free C-space.
    """
    keep = np.concatenate([[True], np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9])
    pts = pts[keep]
    w = None if w is None else np.asarray(w)[keep]
    if len(pts) < 4:
        xy = _resample(pts, ds)
        return xy, np.zeros(len(xy))
    tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1]], w=w, s=s, k=3)
    u = np.linspace(0, 1, 40 * len(pts))
    dense = np.column_stack(interpolate.splev(u, tck))
    sd = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
    us = np.interp(np.linspace(0, sd[-1], max(2, int(math.ceil(sd[-1] / ds)) + 1)), sd, u)
    x, y = interpolate.splev(us, tck)
    dx, dy = interpolate.splev(us, tck, der=1)
    ddx, ddy = interpolate.splev(us, tck, der=2)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-12)
    xy = np.column_stack([x, y])
    return (xy, kappa) if _free(m, xy).all() else None


def _approach(m, p, goal, goal_h):
    """(search goal cell, straight approach points E..goal or None)."""
    if goal_h is None:
        return goal, None
    n = max(1, int(round(p.approach_len / m.res)))
    dx, dy = HEADING[goal_h]
    cells = [(goal[0] - (n - i) * dx, goal[1] - (n - i) * dy) for i in range(n + 1)]
    H, W = m.shape
    if not all(0 <= cx < W and 0 <= cy < H and not m.cspace[cy, cx] for cx, cy in cells):
        raise ValueError(f'approach straight {p.approach_len} m before the goal is inside the padding')
    return cells[0], _centres(m, cells)


def _join_approach(xy, kappa, line, ds):
    """Cut the smoothed path where it reaches the entry point and continue on the exact approach line.

    Returns (xy, kappa, stop): stop = (index, turn) when the path meets the line at more than 10 degrees, an
    in-place turn at the entry point (A*+arc's STOP_TURN there), else None.
    """
    i = max(2, int(np.argmin(np.linalg.norm(xy - line[0], axis=1))))
    tail = _resample(line, ds)
    d_in = xy[i - 1] - xy[max(0, i - 6)]
    d_ln = line[-1] - line[0]
    turn = abs((math.atan2(d_ln[1], d_ln[0]) - math.atan2(d_in[1], d_in[0]) + math.pi) % (2 * math.pi) - math.pi)
    out_k = np.concatenate([kappa[:i], np.zeros(len(tail))])
    stop = None
    if turn > math.radians(10):
        stop = (i, turn); out_k[i - 3:i + 1] = 0.0          # turned in place there, not driven
    return np.vstack([xy[:i], tail]), out_k, stop


def _initial_turn(xy, start_yaw):
    k = min(len(xy) - 1, 5)
    yaw = math.atan2(xy[k, 1] - xy[0, 1], xy[k, 0] - xy[0, 0])
    turn = (yaw - start_yaw + math.pi) % (2 * math.pi) - math.pi
    return [(0, abs(turn))] if abs(turn) > math.radians(5) else []


def smooth_spline(m, p, start, goal, start_yaw=0.0, goal_h=None, ds=0.02):
    entry, line = _approach(m, p, goal, goal_h)
    cells, expanded = astar8(m, p, start, entry)
    pts = _centres(m, cells)
    w = np.ones(len(pts))
    if line is not None:                          # pull the spline onto the approach line
        pts = np.vstack([pts, line[1:]]); w = np.concatenate([w, np.full(len(line) - 1, 20.0)])
    s = len(pts) * (0.5 * m.res) ** 2             # ~half a cell RMS deviation allowed
    res = None
    while res is None and s > 1e-6:
        res = _spline(m, pts, s, ds, w); s *= 0.5
    res = res or _spline(m, pts, 0.0, ds, w)
    if res is None:                               # interpolating spline overshoots: give the polyline
        xy = _resample(pts, ds); res = (xy, _poly_kappa(xy))
    xy, kappa = res
    stops = []
    if line is not None:
        xy, kappa, stop = _join_approach(xy, kappa, line, ds)
        stops = [stop] if stop else []
    return SmoothPath(xy, kappa, _initial_turn(xy, start_yaw) + stops, cells, None, expanded)


def prune_los(m, pts):
    """Any-angle polyline: keep a point only when the straight segment to the next kept one is blocked."""
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _segment_free(m, pts[i], pts[j]):
            j -= 1
        out.append(pts[j]); i = j
    return np.array(out)


def smooth_curve(m, p, start, goal, start_yaw=0.0, goal_h=None, ds=0.02, step=0.05, iters=600,
                 w_smooth=0.3, w_kappa=0.1, w_obs=0.3, clear_band=0.05, max_push=0.01, kappa_max=None):
    """Clearance- and curvature-aware smoothing of the pruned path (Dolgov-style gradient steps).

    kappa_max defaults to the curvature at which the speed limit falls below v_ref:
    min(a_lat_max / v_ref^2, w_max / v_ref). The approach line (goal heading) and the start stay fixed.
    Stability: w_smooth + w_kappa < 0.5 (explicit Laplacian step) and the clearance push is capped at max_push
    per iteration with a two-cell ESDF gradient, so points do not ping-pong across a corridor's medial axis.
    """
    entry, line = _approach(m, p, goal, goal_h)
    cells, expanded = astar8(m, p, start, entry)
    key = prune_los(m, _centres(m, cells))
    kmax = kappa_max or min(p.a_lat_max / p.v_ref ** 2, p.w_max / p.v_ref)
    P = _resample(key, step)
    fixed = np.zeros(len(P), bool); fixed[0] = True
    if line is not None:
        L = _resample(line, step)
        P = np.vstack([P, L[1:]]); fixed = np.concatenate([fixed, np.ones(len(L) - 1, bool)])
        fixed[len(fixed) - len(L)] = True        # the entry point itself
    else:
        fixed[-1] = True
    d_target = p.pad + clear_band
    for _ in range(iters):
        upd = np.zeros_like(P)
        upd[1:-1] = w_smooth * (P[:-2] + P[2:] - 2 * P[1:-1])
        d = _esdf(m, P)                           # clearance: along the ESDF gradient inside pad + clear_band
        near = d < d_target
        if near.any():
            q = P[near]
            h2 = 2 * m.res
            gx = (_esdf(m, q + [h2, 0]) - _esdf(m, q - [h2, 0])) / (2 * h2)
            gy = (_esdf(m, q + [0, h2]) - _esdf(m, q - [0, h2])) / (2 * h2)
            gn = np.column_stack([gx, gy]); gn /= np.maximum(np.linalg.norm(gn, axis=1, keepdims=True), 1e-3)
            push = np.minimum(w_obs * (d_target - d[near]), max_push)
            upd[near] += push[:, None] * gn
        k = np.abs(_poly_kappa(P))                # curvature: sharp points move toward their neighbours' midpoint
        sharp = np.zeros(len(P), bool); sharp[1:-1] = k[1:-1] > kmax
        idx = np.where(sharp)[0]
        upd[idx] += w_kappa * (P[idx - 1] + P[idx + 1] - 2 * P[idx])
        upd[fixed] = 0
        new = P + upd
        P = np.where(_free(m, new)[:, None], new, P)
    w = np.where(fixed, 20.0, 1.0)
    res = None
    s = len(P) * (0.4 * m.res) ** 2                # smooth out the residual sub-cell ripple
    while res is None and s > 1e-7:
        res = _spline(m, P, s, ds, w); s *= 0.5
    if res is None:
        xy = _resample(P, ds); res = (xy, _poly_kappa(xy))
    xy, kappa = res
    stops = []
    if line is not None:
        xy, kappa, stop = _join_approach(xy, kappa, line, ds)
        stops = [stop] if stop else []
    return SmoothPath(xy, kappa, _initial_turn(xy, start_yaw) + stops, cells, P, expanded)


def profile(xy, kappa, stops, p):
    """Speed limit, forward/backward acceleration pass, time. stops: [(index, turn rad)] with v = 0 there."""
    ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    k = np.abs(kappa)
    v = np.minimum.reduce([np.full(len(xy), p.v_ref), np.sqrt(p.a_lat_max / np.maximum(k, 1e-9)),
                           p.w_max / np.maximum(k, 1e-9)])
    v[0] = v[-1] = 0.0
    for i, _ in stops:
        v[i] = 0.0
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * p.a_lon_max * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * p.a_lon_max * ds[i]))
    vm = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3)
    t_drive = float((ds / vm).sum())
    t_turn = sum(a for _, a in stops) / p.w_max
    return v, t_drive, t_turn
