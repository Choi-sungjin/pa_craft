#!/usr/bin/python3
"""SLAM save -> arena-frame map for localization + A*, with the boxes completed.

  /usr/bin/python3 ~/craft/tools/mapmaker/pipeline/make_arena_map.py ~/maps/arena5_legend   # -> ~/maps/arena5_legend_arena.*

Steps (nothing existing is overwritten; every output name is refused if present):
  1. register/register_maze.py: <base>.pgm onto the maze-dice walls -> (x, y, theta)
  2. posegraph/posegraph_tool transform <base> <base>_arena x y theta  (SLAM pose graph in the arena frame)
  3. posegraph_tool render  -> <base>_arena_raw (0.02 m, the A* grid) and <base>_arena_1cm (0.01 m)
  4. craft_planning box_detector --write -> <base>_arena.pgm/.yaml: raw + every 205 mm box as its full square
  5. checks: register_maze --check on the result (best fit must be ~0); <base>_arena_boxes.png and
     figure/compare_figure.py -> <base>_arena_vs_maze-dice.png

Use with: ros2 launch craft_bringup corner_demo.launch.py map:=<base>_arena
Needs the system python (/usr/bin/python3: scipy matches its numpy; the repo venv's numpy 2 does not).
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

MAPMAKER = Path(__file__).resolve().parents[1]
REPO = MAPMAKER.parents[1]
sys.path.insert(0, str(MAPMAKER / 'register'))
sys.path.insert(0, str(MAPMAKER / 'figure'))
from compare_figure import figure  # noqa: E402
from register_maze import check, register, wall_distance  # noqa: E402

POSEGRAPH_TOOL = MAPMAKER / 'posegraph/build/posegraph_tool'
BOX_DETECTOR = REPO / 'src/craft_planning/src/box_detector.py'
NOISE = ('Mapper', 'MapperGraph', 'Dataset', '**', 'Load From', 'Save To', 'Graph')


def run(*cmd):
    print('$', ' '.join(map(str, cmd)), flush=True)
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    lines = [l for l in (r.stdout + r.stderr).splitlines() if l.strip() and not l.startswith(NOISE)]
    print('\n'.join('  ' + l for l in lines))
    if r.returncode:
        raise SystemExit(f'failed ({r.returncode})')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('base', help='slam_toolbox save base without extension (.posegraph/.data/.yaml/.pgm)')
    ap.add_argument('--out', help='output base (default <base>_arena)')
    ap.add_argument('--transform', nargs=3, type=float, metavar=('X', 'Y', 'THETA_DEG'),
                    help='skip registration and use this transform [m, m, deg]')
    args = ap.parse_args()
    base = Path(args.base).expanduser()
    out = Path(args.out).expanduser() if args.out else base.with_name(base.name + '_arena')
    for ext in ('.posegraph', '.data', '.yaml'):
        if not base.with_name(base.name + ext).is_file():
            raise SystemExit(f'{base}{ext} missing')
    for suffix in ('.posegraph', '.data', '.pgm', '.yaml', '_raw.pgm', '_raw.yaml', '_1cm.pgm', '_1cm.yaml'):
        if out.with_name(out.name + suffix).exists():
            raise SystemExit(f'{out}{suffix} exists, refusing to overwrite')
    if not POSEGRAPH_TOOL.is_file():
        raise SystemExit(f'build posegraph_tool first: cd {POSEGRAPH_TOOL.parent.parent} && '
                         'cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4')

    dt = wall_distance()
    if args.transform:
        x, y, deg = args.transform
    else:
        th, x, y, fit = register(base.with_name(base.name + '.yaml'), dt)
        deg = float(np.degrees(th))
        print(f'1. registration: x {x:.5f} m, y {y:.5f} m, theta {deg:.4f} deg, mean wall distance {fit:.1f} mm')
    run(POSEGRAPH_TOOL, 'transform', base, out, f'{x:.6f}', f'{y:.6f}', f'{deg:.6f}')
    raw, cm1 = out.with_name(out.name + '_raw'), out.with_name(out.name + '_1cm')
    run(POSEGRAPH_TOOL, 'render', out, raw, '0.02')
    run(POSEGRAPH_TOOL, 'render', out, cm1, '0.01')
    run(sys.executable, BOX_DETECTOR, f'{raw}.yaml', '--write', out, '--png', out.with_name(out.name + '_boxes.png'))
    ident, best, bdeg, dx, dy = check(f'{cm1}.yaml', dt)
    print(f'5. check on {cm1.name}: identity {ident:.1f} mm, best {best:.1f} mm at {bdeg:+.3f} deg, '
          f'({dx:+.1f}, {dy:+.1f}) mm  (registration accuracy ~1 cm / 0.1 deg: the cost is flat near the optimum)')
    print('wrote', figure(f'{cm1}.yaml', out.with_name(out.name + '_vs_maze-dice.png'),
                          f'{base.name} → arena frame (회전 {deg:.1f}°)'))
    print(f'done: ros2 launch craft_bringup corner_demo.launch.py map:={out}')


if __name__ == '__main__':
    main()
