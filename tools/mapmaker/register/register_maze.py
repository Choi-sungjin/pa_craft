#!/usr/bin/python3
"""Register a ROS map to the maze-dice wall drawing (arena frame of the project2 brief).

  /usr/bin/python3 register_maze.py ~/maps/arena6.yaml            # transform map -> arena: x y [m], theta [deg]
  /usr/bin/python3 register_maze.py --check ~/maps/arena6_arena_1cm.yaml   # an arena-frame map: should be ~0

Cost: mean distance [mm] of the map's occupied pixels to the nearest drawn wall, truncated at 150 mm (walls
the real maze lacks or adds do not dominate). Search: rotations every 2 deg x translations +-0.4 m every
0.1 m around the centroid match, then Nelder-Mead. Arena frame: origin at the lower-left outer post centre,
+X right, +Y up. The drawing is reference/maze-dice.png.
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage, optimize

MAPMAKER = Path(__file__).resolve().parents[1]
MAZE = MAPMAKER / 'reference/maze-dice.png'
# maze-dice.png pixel <-> mm: outer wall lines x=0 / 4464 at px 220 / 1458, y=0 / 6364 at px 1969 / 198
X0, X1, Y0, Y1 = 220, 1458, 1969, 198
ARENA = (4464.0, 6364.0)
G, OFF, TRUNC = 10, 150, 150      # wall distance grid [mm], border [mm], truncation [mm]


def wall_distance():
    """Distance [mm] to the nearest drawn wall on a 10 mm grid covering the arena +- OFF."""
    a = np.array(Image.open(MAZE).convert('RGB')).astype(int)
    m = (abs(a[..., 0] - 0x2c) < 20) & (abs(a[..., 1] - 0x3e) < 20) & (abs(a[..., 2] - 0x50) < 25)
    m[:, 1500:] = False; m[:180] = False; m[1990:] = False; m[:, :200] = False   # plot frame only
    ys, xs = np.nonzero(m)
    wx, wy = (xs - X0) / (X1 - X0) * ARENA[0], (ys - Y0) / (Y1 - Y0) * ARENA[1]
    W, H = int((ARENA[0] + 2 * OFF) / G), int((ARENA[1] + 2 * OFF) / G)
    grid = np.ones((H, W), bool)
    grid[((wy + OFF) / G).astype(int), ((wx + OFF) / G).astype(int)] = False
    return ndimage.distance_transform_edt(grid) * G


def map_hits(yaml_file):
    """Occupied pixel centres of a ROS map in mm (world frame of that map)."""
    d = yaml.safe_load(Path(yaml_file).read_text())
    if abs(float(d['origin'][2])) > 1e-9:
        raise SystemExit(f'{yaml_file}: rotated map origin not supported')
    p = np.array(Image.open(Path(yaml_file).parent / d['image']))
    res, (ox, oy), h = float(d['resolution']), map(float, d['origin'][:2]), p.shape[0]
    r, c = np.nonzero(p < 50)
    return (ox + (c + 0.5) * res) * 1000, (oy + (h - 1 - r + 0.5) * res) * 1000


def make_cost(dt, rx, ry):
    H, W = dt.shape

    def cost(q):
        th, tx, ty = q
        c, s = np.cos(th), np.sin(th)
        ax, ay = c * rx - s * ry + tx, s * rx + c * ry + ty
        i = np.clip(((ax + OFF) / G).astype(int), 0, W - 1)
        j = np.clip(((ay + OFF) / G).astype(int), 0, H - 1)
        return np.minimum(dt[j, i], TRUNC).mean()
    return cost


def _refine(cost, x0):
    # explicit simplex: from zeros Nelder-Mead would only probe 0.00025 (rad / mm) and never move
    x0 = np.asarray(x0, float)
    simplex = [x0, x0 + [np.radians(1), 0, 0], x0 + [0, 20, 0], x0 + [0, 0, 20]]
    return optimize.minimize(cost, x0, method='Nelder-Mead',
                             options=dict(initial_simplex=simplex, xatol=1e-5, fatol=1e-4))


def register(yaml_file, dt=None):
    """(theta rad, x m, y m, cost mm) taking the map frame to the arena frame."""
    dt = wall_distance() if dt is None else dt
    rx, ry = map_hits(yaml_file)
    cost = make_cost(dt, rx, ry)
    cx, cy = rx.mean(), ry.mean()
    best = None
    for deg in np.arange(-180, 180, 2):
        th = np.radians(deg); c, s = np.cos(th), np.sin(th)
        tx, ty = ARENA[0] / 2 - (c * cx - s * cy), ARENA[1] / 2 - (s * cx + c * cy)
        for dx in range(-400, 401, 100):
            for dy in range(-400, 401, 100):
                v = cost((th, tx + dx, ty + dy))
                if best is None or v < best[0]:
                    best = (v, (th, tx + dx, ty + dy))
    o = _refine(cost, best[1])
    th = (o.x[0] + np.pi) % (2 * np.pi) - np.pi
    return th, o.x[1] / 1000, o.x[2] / 1000, o.fun


def check(yaml_file, dt=None):
    """For an arena-frame map: (identity cost, best cost, best theta deg, dx mm, dy mm); best should be ~0."""
    dt = wall_distance() if dt is None else dt
    cost = make_cost(dt, *map_hits(yaml_file))
    o = _refine(cost, (0.0, 0.0, 0.0))
    return cost((0, 0, 0)), o.fun, float(np.degrees(o.x[0])), o.x[1], o.x[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map', help='ROS map yaml')
    ap.add_argument('--check', action='store_true', help='the map is already in the arena frame: report residual')
    args = ap.parse_args()
    if args.check:
        ident, best, deg, dx, dy = check(args.map)
        print(f'identity {ident:.1f} mm, best {best:.1f} mm at {deg:+.3f} deg ({dx:+.1f}, {dy:+.1f}) mm')
    else:
        th, x, y, c = register(args.map)
        print(f'x {x:.5f} m  y {y:.5f} m  theta {np.degrees(th):.4f} deg  (mean wall distance {c:.1f} mm)')


if __name__ == '__main__':
    main()
