"""Find the 205 x 205 mm arena boxes in the A* map (ROS trinary YAML, run at that map's resolution).

The A* planner uses the YAML resolution as its grid (no resampling), so every length here is in metres and
converted with that resolution. LiDAR sees a box only from the free side: one to four faces, never the inside.

  1. Cells: hit (black), free (white), unknown (gray), read with the same thresholds as the planner.
  2. Walls: hits on straight runs longer than wall_min_len (horizontal or vertical opening of the hit mask,
     dilated by one cell to bridge gaps). A box face is only 0.205 m, so it survives; a box against a wall
     keeps its other faces.
  3. Clusters: the remaining hits, grown by cluster_gap, extent at most side + extent_slack.
     Fragments (e.g. two opposite faces 0.2 m apart) merge while their union fits one footprint.
  4. Square fit per cluster: centre and yaw (+-max_yaw_deg) maximising the hits on the square's outline.
     Box insides show as free in these maps (the scan passes over the 70 mm top), so free space says nothing
     about which side an unseen face is on: with one face seen, the nearest layout box breaks the tie.
  5. Match to the arena layout (boxes from the project2 brief, mm in the maze-dice frame).
  6. Optional --write: the map with every box completed to its full 205 mm square outline (unseen faces too).

  /usr/bin/python3 box_detector.py ~/maps/arena5_legend_arena.yaml [--png out.png] [--write out_base]
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# Project2 brief (maze-dice frame, mm -> m): start boxes 1-6 and destinations L1-L3, all 205 x 205 x 70 mm.
# L2 sits on the small box like L1/L3 (the drawing's large centre pedestal is not used).
ARENA_BOXES = {
    'S1': (1.468, 0.950), 'S2': (3.670, 3.152), 'S3': (2.936, 5.354),
    'S4': (0.734, 5.354), 'S5': (2.202, 5.354), 'S6': (3.670, 3.886),
    'L1': (3.670, 0.950), 'L2': (2.202, 3.700), 'L3': (3.670, 5.354),
}
ARENA_SIZE = (4.464, 6.364)  # [m] outer post centres, origin at the lower-left post


@dataclass
class BoxParams:
    side: float = 0.205          # [m] box footprint
    wall_min_len: float = 0.32   # [m] straight hit runs at least this long are walls (box face 0.205)
    wall_clear: float = 0.03     # [m] hits this close to a wall belong to it
    cluster_gap: float = 0.05    # [m] hits closer than this form one cluster
    extent_slack: float = 0.08   # [m] cluster bounding box at most side + slack
    min_hits: int = 4            # hit cells in a cluster
    min_coverage: float = 0.20   # outline fraction seen (one full face = 0.25)
    max_yaw_deg: float = 15.0
    yaw_step_deg: float = 5.0
    yaw_cost: float = 0.2        # hit-equivalents per degree: one or two faces barely fix the yaw, keep the arena axes
    match_radius: float = 0.35   # [m] detection <-> layout box
    prior_weight: float = 2.0    # one-face tie-break toward the nearest layout box (~3 hits one width away)
    arena_margin: float = 0.05   # [m] clusters outside the arena (scan noise behind the outer wall) are dropped


@dataclass
class Box:
    x: float
    y: float
    yaw: float
    coverage: float
    faces: str                   # seen faces of the square in its own frame: +x, +y, -x, -y -> 'E', 'N', 'W', 'S'
    hits: int


@dataclass
class GridMap:
    hit: np.ndarray              # [iy, ix] bool, iy = 0 at origin y (planner orientation)
    free: np.ndarray
    res: float
    origin: tuple[float, float]

    def centre(self, ix, iy):
        return self.origin[0] + (np.asarray(ix) + 0.5) * self.res, self.origin[1] + (np.asarray(iy) + 0.5) * self.res


def load_map(yaml_file) -> GridMap:
    """Same YAML rules as PathPlanner.load_map; occupied/free split by the YAML thresholds."""
    yaml_file = Path(yaml_file).expanduser()
    data = yaml.safe_load(yaml_file.read_text(encoding='utf-8'))
    if data.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only ROS trinary map YAML is supported')
    origin = [float(v) for v in data['origin']]
    if abs(origin[2]) > 1e-9:
        raise ValueError('Rotated map origins are not supported (render the map in the arena frame)')
    free_t, occ_t = float(data.get('free_thresh', 0.196)), float(data.get('occupied_thresh', 0.65))
    with Image.open(yaml_file.parent / data['image']) as image:
        pixels = np.asarray(image.convert('L'), dtype=float) / 255.0
    p = pixels if int(data.get('negate', 0)) else 1.0 - pixels
    p = np.flipud(p)
    return GridMap(p >= occ_t, p < free_t, float(data['resolution']), (origin[0], origin[1]))


def cells(length, res):
    return max(1, int(round(length / res)))


def wall_mask(hit, res, prm: BoxParams):
    grown = ndimage.binary_dilation(hit, np.ones((3, 3), bool))
    n = cells(prm.wall_min_len, res)
    wall = ndimage.binary_opening(grown, np.ones((1, n), bool)) | ndimage.binary_opening(grown, np.ones((n, 1), bool))
    return ndimage.binary_dilation(wall & hit, iterations=cells(prm.wall_clear, res))


def _outline(side, res):
    """Points every half cell around a square of this side, local frame, by face."""
    h, n = side / 2, max(4, int(math.ceil(side / (res / 2))))
    t = np.linspace(-h, h, n)
    return {'E': (np.full(n, h), t), 'N': (t, np.full(n, h)), 'W': (np.full(n, -h), t), 'S': (t, np.full(n, -h))}


def fit_square(points, grid: GridMap, prm: BoxParams, prior=None):
    """Best square for one cluster of hit centres (N x 2, metres).

    Score = sum over hits of exp(-(d / res)^2), d = Chebyshev distance to the square outline, at every centre
    on a quarter-cell lattice and every yaw. With one face seen the two placements score about the same (the
    lattice alone can tip it by ~1 hit); the prior -prior_weight * (dist / side)^2 to the nearest layout box
    (~3 hits one box width away) decides. Two or more seen faces differ by >10 hits and are not overridden.
    """
    res, h = grid.res, prm.side / 2
    lo, hi = points.min(0) - h, points.max(0) + h
    step = res / 4
    CX, CY = [a.ravel() for a in np.meshgrid(np.arange(lo[0], hi[0] + step, step), np.arange(lo[1], hi[1] + step, step))]
    best = (-np.inf, 0.0, 0.0, 0.0)
    for yaw in np.radians(np.arange(-prm.max_yaw_deg, prm.max_yaw_deg + 1e-9, prm.yaw_step_deg)):
        c, s = math.cos(yaw), math.sin(yaw)
        du, dv = points[:, 0][None] - CX[:, None], points[:, 1][None] - CY[:, None]   # [centre, point]
        u, v = c * du + s * dv, -s * du + c * dv
        d = np.maximum(np.abs(u), np.abs(v)) - h
        score = np.exp(-(d / res) ** 2).sum(1) - prm.yaw_cost * abs(math.degrees(yaw))
        if prior is not None:
            score -= prm.prior_weight * ((CX - prior[0]) ** 2 + (CY - prior[1]) ** 2) / prm.side ** 2
        i = int(np.argmax(score))
        if score[i] > best[0]:
            best = (score[i], CX[i], CY[i], yaw)
    _, cx, cy, yaw = best
    c, s = math.cos(yaw), math.sin(yaw)
    hits = set(map(tuple, np.floor((points - grid.origin) / res).astype(int)))
    seen, total, faces = 0, 0, ''
    for name, (fu, fv) in _outline(prm.side, res).items():
        ix = np.floor((cx + c * fu - s * fv - grid.origin[0]) / res).astype(int)
        iy = np.floor((cy + s * fu + c * fv - grid.origin[1]) / res).astype(int)
        near = sum(any((a + i, b + j) in hits for i in (-1, 0, 1) for j in (-1, 0, 1)) for a, b in zip(ix, iy))
        seen += near; total += len(ix)
        faces += name if near >= 0.5 * len(ix) else ''
    return Box(float(cx), float(cy), float(yaw), seen / total, faces, len(points))


def _merge_fragments(clusters, limit):
    """Join hit fragments whose union still fits in one box footprint, closest pair first."""
    clusters = [c for c in clusters]
    while True:
        best = None
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                u = np.vstack([clusters[i], clusters[j]])
                if np.ptp(u, 0).max() <= limit:
                    gap = np.linalg.norm(clusters[i].mean(0) - clusters[j].mean(0))
                    if best is None or gap < best[0]:
                        best = (gap, i, j)
        if best is None:
            return clusters
        _, i, j = best
        clusters[i] = np.vstack([clusters[i], clusters[j]]); del clusters[j]


def detect_boxes(grid: GridMap, prm: BoxParams | None = None, layout=ARENA_BOXES):
    """Boxes in the map. layout (name -> (x, y) m, or None) is only the tie-break prior and the arena bounds."""
    prm = prm or BoxParams()
    res = grid.res
    cand = grid.hit & ~wall_mask(grid.hit, res, prm)
    joined = ndimage.binary_dilation(cand, iterations=max(1, cells(prm.cluster_gap, res) // 2))
    labels, n = ndimage.label(joined, np.ones((3, 3), bool))
    limit = prm.side + prm.extent_slack
    frags = []
    for i in range(1, n + 1):
        iy, ix = np.nonzero((labels == i) & cand)
        pts = np.column_stack(grid.centre(ix, iy))
        if len(pts) and np.ptp(pts, 0).max() <= limit:
            frags.append(pts)
    boxes = []
    for pts in _merge_fragments(frags, limit):
        if len(pts) < prm.min_hits:
            continue
        m = pts.mean(0)
        if layout and not (-prm.arena_margin <= m[0] <= ARENA_SIZE[0] + prm.arena_margin and
                           -prm.arena_margin <= m[1] <= ARENA_SIZE[1] + prm.arena_margin):
            continue
        prior = min(layout.values(), key=lambda q: math.hypot(q[0] - m[0], q[1] - m[1])) if layout else None
        box = fit_square(pts, grid, prm, prior)
        if box.coverage >= prm.min_coverage:
            boxes.append(box)
    return boxes, cand


def match_layout(boxes, layout=ARENA_BOXES, radius=BoxParams.match_radius):
    """Greedy nearest pairs within radius: {name: Box | None}, unmatched detections."""
    pairs = sorted((math.hypot(b.x - x, b.y - y), name, j) for name, (x, y) in layout.items()
                   for j, b in enumerate(boxes))
    used_n, used_b, out = set(), set(), {name: None for name in layout}
    for d, name, j in pairs:
        if d <= radius and name not in used_n and j not in used_b:
            out[name] = boxes[j]; used_n.add(name); used_b.add(j)
    return out, [b for j, b in enumerate(boxes) if j not in used_b]


def box_corners(box: Box, side=BoxParams.side):
    """Four corners of the full side x side square (counter-clockwise from +x +y), metres."""
    h, c, s = side / 2, math.cos(box.yaw), math.sin(box.yaw)
    return [(box.x + c * u - s * v, box.y + s * u + c * v) for u, v in ((h, h), (-h, h), (-h, -h), (h, -h))]


def write_completed_map(yaml_file, out_base, boxes, side=BoxParams.side):
    """Copy the map with every box drawn as its full square outline (occupied), unseen faces included.

    Only the outline: the faces are real (LiDAR matching agrees with them) and r_robot + margin inflation
    blocks the 0.205 m inside anyway. Writes <out_base>.pgm + .yaml with the source resolution and origin.
    """
    yaml_file, out_base = Path(yaml_file).expanduser(), Path(out_base).expanduser()
    for ext in ('.pgm', '.yaml'):
        if out_base.with_name(out_base.name + ext).exists():
            raise FileExistsError(f'{out_base}{ext} exists')
    data = yaml.safe_load(yaml_file.read_text(encoding='utf-8'))
    with Image.open(yaml_file.parent / data['image']) as image:
        pixels = np.array(image.convert('L'))
    res, (ox, oy) = float(data['resolution']), (float(data['origin'][0]), float(data['origin'][1]))
    occupied = 255 if int(data.get('negate', 0)) else 0
    H, W = pixels.shape
    for box in boxes:
        corners = box_corners(box, side)
        for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
            n = int(math.ceil(math.hypot(x1 - x0, y1 - y0) / (res / 4))) + 1
            ix = np.floor((np.linspace(x0, x1, n) - ox) / res).astype(int)
            iy = np.floor((np.linspace(y0, y1, n) - oy) / res).astype(int)
            ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
            pixels[H - 1 - iy[ok], ix[ok]] = occupied
    pgm = out_base.with_name(out_base.name + '.pgm')
    Image.fromarray(pixels).save(pgm)
    out = dict(data, image=pgm.name)
    out_base.with_name(out_base.name + '.yaml').write_text(yaml.safe_dump(out, sort_keys=False), encoding='utf-8')
    return pgm


def _png(grid, cand, boxes, matched, prm, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for f in fm.findSystemFonts():
        if 'NotoSansCJK' in f:
            fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
    img = np.full(grid.hit.shape, 0.8); img[grid.free] = 1.0; img[grid.hit] = 0.0
    rgb = np.dstack([img] * 3); rgb[cand] = (0.85, 0.1, 0.1)
    H, W = grid.hit.shape
    ext = [grid.origin[0], grid.origin[0] + W * grid.res, grid.origin[1], grid.origin[1] + H * grid.res]
    fig, ax = plt.subplots(figsize=(9, 12), dpi=120)
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
    h = prm.side / 2
    for name, (x, y) in ARENA_BOXES.items():
        ax.add_patch(plt.Rectangle((x - h, y - h), prm.side, prm.side, fill=False, ec='tab:orange', lw=1.5, ls='--'))
        b = matched[name]
        lab = f'{name}' + (f'\nΔ{math.hypot(b.x - x, b.y - y) * 1000:.0f}mm' if b else '\n미검출')
        ax.text(x, y + h + 0.05, lab, ha='center', va='bottom', fontsize=8, color='tab:orange')
    for b in boxes:
        corners = np.array(box_corners(b, prm.side) + box_corners(b, prm.side)[:1])
        ax.plot(corners[:, 0], corners[:, 1], color='tab:green', lw=1.8)
        ax.plot(b.x, b.y, '+', color='tab:green')
    ax.set_xlim(-0.2, 4.66); ax.set_ylim(-0.2, 6.56); ax.set_aspect('equal')
    ax.set_title(f'박스 검출 (A* 격자 {grid.res * 100:.0f} cm) — 주황 점선: 발제 위치, 초록: 검출, 빨강: 벽 제외 후 후보 셀')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map', nargs='?', default=str(Path.home() / 'maps/arena5_legend_arena.yaml'))
    ap.add_argument('--png', help='write an overlay image here')
    ap.add_argument('--write', metavar='OUT_BASE',
                    help='write <OUT_BASE>.pgm/.yaml: the map with every box completed to its full square')
    args = ap.parse_args()
    prm = BoxParams()
    grid = load_map(args.map)
    boxes, cand = detect_boxes(grid, prm)
    matched, extra = match_layout(boxes, radius=prm.match_radius)
    print(f'map {args.map}: {grid.hit.shape[1]}x{grid.hit.shape[0]} cells, res {grid.res} m, {len(boxes)} boxes')
    print(f'{"box":4} {"layout x,y [mm]":>17} {"detected x,y [mm]":>19} {"dx":>6} {"dy":>6} {"dist":>6} '
          f'{"yaw":>5} {"seen":>5} faces')
    for name, (x, y) in ARENA_BOXES.items():
        b = matched[name]
        if b is None:
            print(f'{name:4} {x * 1000:8.0f},{y * 1000:7.0f}   not found'); continue
        print(f'{name:4} {x * 1000:8.0f},{y * 1000:7.0f} {b.x * 1000:10.0f},{b.y * 1000:7.0f} '
              f'{(b.x - x) * 1000:6.0f} {(b.y - y) * 1000:6.0f} {math.hypot(b.x - x, b.y - y) * 1000:6.0f} '
              f'{math.degrees(b.yaw):5.0f} {b.coverage:5.2f} {b.faces}')
    for b in extra:
        print(f'extra  detected {b.x * 1000:.0f},{b.y * 1000:.0f} yaw {math.degrees(b.yaw):.0f} '
              f'seen {b.coverage:.2f} {b.faces}')
    if args.png:
        _png(grid, cand, boxes, matched, prm, args.png)
        print('wrote', args.png)
    if args.write:
        print('wrote', write_completed_map(args.map, args.write, boxes, prm.side))


if __name__ == '__main__':
    main()
