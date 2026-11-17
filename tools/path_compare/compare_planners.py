#!/usr/bin/python3
"""Compare A*+arc (current planner) with 8-connected A* + spline and 8-connected A* + spline + curve smoothing.

  /usr/bin/python3 ~/craft/tools/path_compare/compare_planners.py [--map ~/maps/arena5_legend_arena.yaml] [--out DIR]

Same map, same C-space (planning.yaml path_planner: r_robot + margin, proximity cost), same legs, same speed
model (astar8_spline.profile). Planning time is the Python implementations' (median of --reps runs) so the
three are comparable; the C++ A*+arc rate on the Jetson is measured separately (tools/path_compare/README.md).
Writes one figure per leg and summary.csv / summary.md.
"""
import argparse
import dataclasses
import math
import statistics
import sys
import time
import types
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / 'src/craft_planning/src'
pkg = types.ModuleType('craft_planning'); pkg.__path__ = [str(SRC)]   # import the sources, not the install
sys.modules['craft_planning'] = pkg
from craft_planning import astar_arc as aa  # noqa: E402
from craft_planning import astar8_spline as a8  # noqa: E402
from craft_planning.path_planner import PathPlanner  # noqa: E402

# (name, start xy, start yaw, goal xy, goal heading 0 E / 1 N / 2 W / 3 S). World (arena) metres; goals are the
# approach points 0.45-0.55 m from the detected L boxes (box_detector), facing the box: the last approach_len is
# straight along that heading (A*+arc ALIGN), for all three methods.
LEGS = [
    ('start → L2 접근 (남쪽 보며)', (0.734, 0.950), 0.0, (2.348, 4.281), 3),
    ('start → L3 접근 (북쪽 보며)', (0.734, 0.950), 0.0, (3.778, 4.867), 1),
    ('L3 접근 → L1 접근 (동쪽 보며)', (3.778, 4.867), math.pi / 2, (3.108, 1.066), 0),
]
METHODS = ['A*+arc (기존)', '8방향+Spline', '8방향+Spline+곡선경로']
COLORS = ['tab:blue', 'tab:orange', 'tab:green']


def params():
    cfg = yaml.safe_load((REPO / 'src/craft_planning/config/planning.yaml').read_text())
    pp = cfg['planning_master']['ros__parameters']['path_planner']
    return aa.Params(**{k: v for k, v in pp.items() if k in aa.Params.__dataclass_fields__})


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run_arc(m, p, s, g, start_yaw, goal_h):
    leg = aa.plan_leg(m, p, s, int(round(start_yaw / (math.pi / 2))) % 4, g, goal_h)
    path = leg.path
    xy, kappa = path[:, :2], path[:, 3]
    stops = []
    for w in leg.waypoints:
        if w.mode != aa.STOP_TURN:
            continue
        i = int(np.argmin(np.hypot(xy[:, 0] - w.x, xy[:, 1] - w.y)))
        if w.h_in is not None and w.h_out is not None:
            turn = abs(wrap((w.h_out - w.h_in) * math.pi / 2))
        elif w.h_out is not None:
            turn = abs(wrap(w.h_out * math.pi / 2 - start_yaw))
        else:
            turn = 0.0
        if turn > 1e-6 or w.reason != 'start':
            stops.append((i, turn))
    return a8.SmoothPath(xy, kappa, stops, [(ix, iy) for ix, iy, _ in leg.states], None, leg.expanded)


def run(method, m, p, s, g, start_yaw, goal_h):
    if method == 0:
        return run_arc(m, p, s, g, start_yaw, goal_h)
    if method == 1:
        return a8.smooth_spline(m, p, s, g, start_yaw, goal_h)
    return a8.smooth_curve(m, p, s, g, start_yaw, goal_h)


def metrics(path, m, p, goal_h):
    xy, k = path.xy, path.kappa
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    length = float(seg.sum())
    v, t_drive, t_turn = a8.profile(xy, k, path.stops, p)
    stops_mid = [st for st in path.stops if 0 < st[0] < len(xy) - 1 or st[1] > 0]
    # curvature continuity: largest jump between samples (A*+arc lines/arcs jump to 1/R)
    dk = float(np.max(np.abs(np.diff(k)))) if len(k) > 1 else 0.0
    clear = float(a8._esdf(m, xy).min())
    d = xy[-1] - xy[-6]
    arrive = math.degrees(abs(wrap(math.atan2(d[1], d[0]) - goal_h * math.pi / 2))) if goal_h is not None else 0.0
    return dict(arrive=arrive, length=length, t=t_drive + t_turn, t_turn=t_turn, n_stop=len(stops_mid),
                kmax=float(np.abs(k).max()), dk=dk, clear=clear, v=v)


def figure(name, m, origin, results, rows, p, png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for f in fm.findSystemFonts():
        if 'NotoSansCJK' in f:
            fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
    fig = plt.figure(figsize=(18, 11), dpi=110)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.25], height_ratios=[1, 1, 0.9])
    ax = fig.add_subplot(gs[:, 0])
    H, W = m.shape
    img = np.ones((H, W, 3))
    img[m.cspace] = (0.87, 0.87, 0.92)
    img[m.occ] = (0.1, 0.1, 0.1)
    ext = [origin[0], origin[0] + W * m.res, origin[1], origin[1] + H * m.res]
    ax.imshow(img, origin='lower', extent=ext, interpolation='nearest')
    allxy = np.vstack([r.xy for r in results]) + origin
    lo, hi = allxy.min(0) - 0.4, allxy.max(0) + 0.4
    for r, c, lab in zip(results, COLORS, METHODS):
        xy = r.xy + origin
        ax.plot(xy[:, 0], xy[:, 1], color=c, lw=2.2, label=lab)
        for i, _ in r.stops:
            ax.plot(*xy[i], 'o', ms=7, mfc='none', mec=c, mew=1.8)
        if r.key is not None:
            ax.plot(*(r.key + origin).T, '.', color=c, ms=3, alpha=.5)
    ax.plot(*allxy[0], 'k^', ms=10); ax.plot(*(results[0].xy[-1] + origin), 'k*', ms=14)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal')
    ax.set_title(f'{name}  (연회색: padding {p.pad:.3f} m, ○: 제자리 정지·회전)')
    ax.legend(loc='best', fontsize=9); ax.grid(alpha=.25); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    axk, axv = fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
    for r, c, lab in zip(results, COLORS, METHODS):
        s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(r.xy, axis=0), axis=1))])
        axk.plot(s, r.kappa, color=c, lw=1.4, label=lab)
        v, _, _ = a8.profile(r.xy, r.kappa, r.stops, p)
        axv.plot(s, v, color=c, lw=1.4, label=lab)
    axk.set_ylabel('곡률 κ (1/m)'); axk.grid(alpha=.3); axk.set_ylim(-14, 14); axk.legend(fontsize=8)
    axv.set_ylabel('속도 상한 v (m/s)'); axv.set_xlabel('경로 길이 s (m)'); axv.grid(alpha=.3)
    axt = fig.add_subplot(gs[2, 1]); axt.axis('off')
    cols = ['방법', '길이 m', '예상시간 s', '정지', '최대|κ|', '최대Δκ', '최소여유 m', '도착각 오차°', '계획 ms', 'Hz']
    tb = axt.table(cellText=[[r[c] for c in cols] for r in rows], colLabels=cols, loc='center', cellLoc='center',
                   colWidths=[0.22] + [0.0867] * (len(cols) - 1))
    tb.auto_set_font_size(False); tb.set_fontsize(9.5); tb.scale(1, 1.6)
    for j, c in enumerate(COLORS):
        tb[j + 1, 0].set_text_props(color=c, weight='bold')
    fig.tight_layout(); fig.savefig(png); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', default=str(Path.home() / 'maps/arena5_legend_arena.yaml'))
    ap.add_argument('--out', default=str(Path(__file__).resolve().parent / 'out'))
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--approach-len', type=float, default=0.25,
                    help='straight along the goal heading [m]; L3 from the south has 0.28 m free at pad 0.293')
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    p = params()
    pl = PathPlanner(args.map, p)
    m, origin = pl.grid, np.array(pl.origin[:2])
    # The C++ planner counts cells, approach and turn cost with the map's resolution; the Python reference uses
    # Params.res (0.05, meant for STL/text maps). Match the C++ planner on this 0.02 m map.
    p = dataclasses.replace(p, res=m.res, block_size=m.res, approach_len=args.approach_len)
    print(f'map {args.map}: res {m.res} m, pad {p.pad:.3f} m (r_robot {p.r_robot} + margin {p.margin})')
    table = []
    for li, (name, s_w, yaw0, g_w, goal_h) in enumerate(LEGS):
        s = m.world_to_cell(s_w[0] - origin[0], s_w[1] - origin[1])
        g = m.world_to_cell(g_w[0] - origin[0], g_w[1] - origin[1])
        results, rows = [], []
        for k, meth in enumerate(METHODS):
            times = []
            for _ in range(args.reps):
                t0 = time.perf_counter(); r = run(k, m, p, s, g, yaw0, goal_h); times.append(time.perf_counter() - t0)
            ms = statistics.median(times) * 1000
            mt = metrics(r, m, p, goal_h)
            row = {'구간': name, '방법': meth, '길이 m': f'{mt["length"]:.2f}', '예상시간 s': f'{mt["t"]:.1f}',
                   '정지': f'{mt["n_stop"]}', '최대|κ|': f'{mt["kmax"]:.1f}', '최대Δκ': f'{mt["dk"]:.2f}',
                   '최소여유 m': f'{mt["clear"]:.3f}', '도착각 오차°': f'{mt["arrive"]:.1f}', '계획 ms': f'{ms:.0f}', 'Hz': f'{1000 / ms:.1f} Hz'}
            results.append(r); rows.append(row); table.append(row)
            print(' | '.join(f'{v}' for v in row.values()))
        figure(name, m, origin, results, rows, p, out / f'leg{li + 1}.png')
    cols = list(table[0].keys())
    (out / 'summary.csv').write_text('\n'.join([','.join(cols)] + [','.join(r[c] for c in cols) for r in table]) + '\n')
    md = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)] + ['| ' + ' | '.join(r[c] for c in cols) + ' |' for r in table]
    (out / 'summary.md').write_text('\n'.join(md) + '\n')
    print('wrote', out)


if __name__ == '__main__':
    main()
