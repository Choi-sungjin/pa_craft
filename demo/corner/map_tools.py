"""ROS map (YAML + PGM) helpers for the corner demo: occupancy, clearance, global scan-to-map search, goal.

No ROS imports, so the search can be tested on synthetic scans (test_map_tools.py).
Poses are map-frame (x, y, yaw); scan points are base_link (x, y) metres.
"""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class MapGrid:
    """A nav2/slam_toolbox map: trinary PGM, row 0 at the top, origin = lower-left corner of the image."""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(Path(yaml_path).read_text())
        img = cv2.imread(str(Path(yaml_path).parent / meta['image']), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(meta['image'])
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = img.shape
        occupancy = (255.0 - img) / 255.0  # negate 0
        self.occupied = occupancy >= float(meta.get('occupied_thresh', 0.65))
        self.free = occupancy <= float(meta.get('free_thresh', 0.196)) - 1e-6
        # Distance [m] from each cell to the nearest occupied cell.
        self.dist = cv2.distanceTransform((~self.occupied).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * self.res

    def to_cell(self, x, y):
        """map (x, y) [m] -> (col, row) floats."""
        return (np.asarray(x) - self.ox) / self.res, self.h - (np.asarray(y) - self.oy) / self.res

    def to_xy(self, col, row):
        """cell centre -> map (x, y)."""
        return self.ox + (np.asarray(col) + 0.5) * self.res, self.oy + (self.h - np.asarray(row) - 0.5) * self.res

    def free_cells(self, clearance, step):
        """map (x, y) of free cells at least `clearance` m from any wall, on a `step` m grid."""
        k = max(1, int(round(step / self.res)))
        ok = self.free & (self.dist >= clearance)
        rows, cols = np.nonzero(ok[::k, ::k])
        x, y = self.to_xy(cols * k, rows * k)
        return np.stack([x, y], 1)


class ScanMatcher:
    """Likelihood-field matcher: a scan point scores exp(-d^2 / 2 sigma^2), d = distance to the nearest wall."""

    def __init__(self, grid, sigma=0.05):
        self.g = grid
        self.lik = np.exp(-grid.dist ** 2 / (2 * sigma ** 2)).astype(np.float32)

    def score(self, poses, pts):
        """Mean point score of `pts` (P,2, base_link) at each pose (N,3) -> (N,)."""
        out = np.empty(len(poses), np.float32)
        for yaw in np.unique(poses[:, 2]):
            sel = poses[:, 2] == yaw
            c, s = math.cos(yaw), math.sin(yaw)
            rx, ry = pts[:, 0] * c - pts[:, 1] * s, pts[:, 0] * s + pts[:, 1] * c
            col, row = self.g.to_cell(poses[sel, 0, None] + rx[None], poses[sel, 1, None] + ry[None])
            col, row = np.floor(col).astype(np.int32), np.floor(row).astype(np.int32)
            inside = (col >= 0) & (row >= 0) & (col < self.g.w) & (row < self.g.h)
            v = np.zeros(col.shape, np.float32)
            v[inside] = self.lik[row[inside], col[inside]]
            out[sel] = v.mean(1)
        return out

    def global_search(self, pts, clearance=0.18, step=0.06, angle_step_deg=3.0, top=6, near=None, radius=0.5):
        # 6 cm / 3 deg: 6 s on the Jetson for arena3_clean, synthetic scans <= 1.1 cm after refine (4 cm / 2 deg: 19 s)
        """Best distinct poses over the free map (or only within radius of near=(x, y)): [(score, x, y, yaw)], best first."""
        cells = self.g.free_cells(clearance, step)
        if near is not None:
            cells = cells[np.hypot(cells[:, 0] - near[0], cells[:, 1] - near[1]) <= radius]
            if not len(cells):
                return []
        yaws = np.radians(np.arange(0.0, 360.0, angle_step_deg))
        scores = np.empty((len(yaws), len(cells)), np.float32)
        for i, yaw in enumerate(yaws):
            scores[i] = self.score(np.c_[cells, np.full(len(cells), yaw)], pts)
        order = np.argsort(scores, axis=None)[::-1]
        picks = []
        for flat in order:  # non-maximum suppression: distinct by 0.3 m or 20 deg
            i, j = divmod(int(flat), len(cells))
            x, y, yaw = cells[j, 0], cells[j, 1], yaws[i]
            if all(math.hypot(x - p[1], y - p[2]) > 0.3 or abs(math.remainder(yaw - p[3], 2 * math.pi)) > math.radians(20)
                   for p in picks):
                picks.append((float(scores[i, j]), float(x), float(y), float(yaw)))
                if len(picks) == top:
                    break
        return picks

    def refine(self, pose, span=0.06, step=0.01, span_deg=4.0, step_deg=0.5):
        """Local grid search around pose (x, y, yaw), coarse then fine -> (score, x, y, yaw)."""
        x0, y0, yaw0 = pose
        best = None
        for sp, st, spd, std in ((span, step, span_deg, step_deg), (step, step / 4, step_deg, step_deg / 4)):
            d = np.arange(-sp, sp + 1e-9, st)
            a = np.radians(np.arange(-spd, spd + 1e-9, std))
            X, Y, A = np.meshgrid(x0 + d, y0 + d, yaw0 + a, indexing='ij')
            poses = np.stack([X.ravel(), Y.ravel(), A.ravel()], 1)
            s = self.score(poses, self.pts)
            k = int(np.argmax(s))
            best = (float(s[k]), *map(float, poses[k]))
            x0, y0, yaw0 = best[1:]
        return best

    def localize(self, pts, clearance=0.18, ambiguity=0.92, near=None, radius=0.5):
        """Global search + refinement. Returns dict(pose, score, second, second_pose, ambiguous, candidates).
        near=(x, y): only candidates within radius of it compete (the robot was placed there roughly)."""
        self.pts = pts
        # near: search only there (filtering the whole-map top candidates afterwards could drop the true pose)
        cands = self.global_search(pts, clearance, near=near, radius=radius)
        if near is not None:
            if not cands:
                return {'pose': None, 'score': 0.0, 'second': 0.0, 'second_pose': None, 'ambiguous': True,
                        'candidates': []}
        refined = sorted((self.refine(c[1:]) for c in cands[:3]), reverse=True)
        best = refined[0]
        # the runner-up must be a different place, not the best one found again after refinement
        others = [r for r in refined[1:] + [tuple(c) for c in cands[3:]]
                  if math.hypot(r[1] - best[1], r[2] - best[2]) > 0.3
                  or abs(math.remainder(r[3] - best[3], 2 * math.pi)) > math.radians(20)]
        runner = max(others, default=None)
        second = runner[0] if runner else 0.0
        return {'pose': best[1:], 'score': best[0], 'second': second, 'second_pose': runner[1:] if runner else None,
                'ambiguous': second > ambiguity * best[0], 'candidates': cands}


def corner_goal(grid, start_xy, corner=(1, -1), clearance=0.30, reach_clearance=0.24):
    """Free cell nearest the map's corner direction `corner` (+x, -y = bottom right) that is at least
    `clearance` from walls and reachable from start_xy through cells `reach_clearance` from walls."""
    reach = (grid.free & (grid.dist >= reach_clearance)).astype(np.uint8)
    n, labels = cv2.connectedComponents(reach, connectivity=8)
    col, row = grid.to_cell(*start_xy)
    col, row = int(col), int(row)
    if not (0 <= row < grid.h and 0 <= col < grid.w):
        raise ValueError('start outside the map')
    lab = labels[row, col]
    if lab == 0:  # start cell itself too close to a wall: take the nearest reachable component
        rows, cols = np.nonzero(labels)
        k = int(np.argmin((rows - row) ** 2 + (cols - col) ** 2))
        lab = labels[rows[k], cols[k]]
    ok = (labels == lab) & (grid.dist >= clearance)
    rows, cols = np.nonzero(ok)
    if not len(rows):
        raise ValueError(f'no cell {clearance} m from walls reachable from the start')
    x, y = grid.to_xy(cols, rows)
    k = int(np.argmax(corner[0] * x + corner[1] * y))
    return float(x[k]), float(y[k])
