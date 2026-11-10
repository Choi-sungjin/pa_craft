"""시뮬레이션 기록 → 지표·그림·HTML 보고서."""
from __future__ import annotations

import base64
import html
import io
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from craft_sim.world import GridMap, box_geometry, boxes_from_planning, load_params, obstacles_from_scenario, wrap  # noqa: E402

for _f in ('NanumGothic', 'Noto Sans CJK KR', 'Noto Sans KR', 'UnDotum'):
    if any(_f in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = _f
        break
plt.rcParams['axes.unicode_minus'] = False

PHASE_COLORS = {
    'stage': '#cfe8ff', 'stage_stop': '#e0e0e0', 'lidar_perceive': '#fff2b3', 'lidar_release': '#ffe08a',
    'lidar_dock': '#ffd166', 'handoff_stop': '#e0e0e0', 'camera_perceive': '#d9f2d9', 'camera_release': '#b8e6b8',
    'camera_dock': '#8fd68f', 'final_stop': '#e0e0e0', 'camera_verify': '#c7b8f5'}
RESULT_TEXT = {'complete': '완료', 'fault': 'Fault', 'timeout': '시간 초과', 'not_ready': 'Ready 안 됨',
               'not_started': '시작 안 됨'}


# ---------------------------------------------------------------------------------------------------------------
def polyline_signed_distance(px, py, pts):
    """점들(px, py)에서 꺾은선 pts 까지 부호 있는 거리(+ = 진행 방향 왼쪽)와 접선 각."""
    P = np.asarray(pts, dtype=float)
    if len(P) < 2:
        return np.full(len(px), np.nan), np.full(len(px), np.nan)
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    Q = np.column_stack((px, py))
    t = np.clip(((Q[:, None, :] - A[None]) * AB[None]).sum(2) / L2[None], 0, 1)
    C = A[None] + t[..., None] * AB[None]
    d2 = ((Q[:, None, :] - C) ** 2).sum(2)
    k = d2.argmin(1)
    ab = AB[k]
    aq = Q - A[k]
    cross = ab[:, 0] * aq[:, 1] - ab[:, 1] * aq[:, 0]
    dist = np.sqrt(d2[np.arange(len(Q)), k]) * np.sign(cross)
    return dist, np.arctan2(ab[:, 1], ab[:, 0])


def obstacle_clearance(pose, footprint, ob):
    x, y, th = pose
    fx0, fx1, fy0, fy1 = footprint
    c, s = math.cos(th), math.sin(th)
    u = np.linspace(0, 1, 9)
    edge = np.vstack([np.column_stack((fx0 + (fx1 - fx0) * u, np.full_like(u, fy0))),
                      np.column_stack((fx0 + (fx1 - fx0) * u, np.full_like(u, fy1))),
                      np.column_stack((np.full_like(u, fx0), fy0 + (fy1 - fy0) * u)),
                      np.column_stack((np.full_like(u, fx1), fy0 + (fy1 - fy0) * u))])
    wx, wy = x + c * edge[:, 0] - s * edge[:, 1], y + s * edge[:, 0] + c * edge[:, 1]
    cx, cy = ob.center
    if ob.shape == 'circle':
        return float((np.hypot(wx - cx, wy - cy) - ob.radius).min())
    co, so = math.cos(ob.yaw), math.sin(ob.yaw)
    lx, ly = co * (wx - cx) + so * (wy - cy), -so * (wx - cx) + co * (wy - cy)
    dx = np.maximum(np.abs(lx) - ob.size[0] / 2, 0)
    dy = np.maximum(np.abs(ly) - ob.size[1] / 2, 0)
    return float(np.hypot(dx, dy).min())


def fig_png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return buf.getvalue()


# ---------------------------------------------------------------------------------------------------------------
def build(run_dir: Path, sc: dict, summary: dict, rec: dict, craft_src: Path, map_yaml: Path):
    planning, control = load_params(craft_src)
    ba = planning['box_approach']
    boxes = boxes_from_planning(planning, {k: tuple(v) for k, v in (sc.get('box_offsets') or {}).items()})
    obstacles = obstacles_from_scenario(sc.get('obstacles'))
    footprint = (sc.get('sim', {}) or {}).get('footprint', [-0.20, 0.165, -0.165, 0.165])
    m = GridMap(map_yaml)
    T = rec['truth']
    tt = np.array([s['t'] for s in T]) if T else np.zeros(0)
    X = np.array([[s['x'], s['y'], s['yaw'], s['v'], s['w']] for s in T]) if T else np.zeros((0, 5))

    # 장애물 활성 구간 (sim 이벤트)
    active = {ob.name: [] for ob in obstacles}
    for e in rec['events']:
        if e.get('type') == 'obstacle' and e.get('name') in active:
            if e['active']:
                active[e['name']].append([e['t'], math.inf])
            elif active[e['name']]:
                active[e['name']][-1][1] = e['t']
    collisions = [e for e in rec['events'] if e.get('type') == 'collision']

    # 추종 경로 버전별 횡오차
    paths = [p for p in rec['paths'] if len(p['pts']) >= 2]
    xte = np.full(len(tt), np.nan)
    herr = np.full(len(tt), np.nan)
    if paths and len(tt):
        pt = np.array([p['t'] for p in paths])
        idx = np.searchsorted(pt, tt, side='right') - 1
        # 경로가 비면(해제) 그 뒤는 계산 안 함
        cleared = [p['t'] for p in rec['paths'] if len(p['pts']) < 2]
        for k in np.unique(idx[idx >= 0]):
            sel = np.where(idx == k)[0]
            nxt_clear = [c for c in cleared if c > pt[k]]
            if nxt_clear:
                sel = sel[tt[sel] < min(nxt_clear)]
            if not len(sel):
                continue
            d, tang = polyline_signed_distance(X[sel, 0], X[sel, 1], paths[k]['pts'])
            xte[sel] = d
            herr[sel] = [wrap(a) for a in (X[sel, 2] - tang)]
        # 후진 구간: 진행 방향 반대
        rev = X[:, 3] < -0.005
        herr[rev] = [wrap(a + math.pi) for a in herr[rev]]
    moving = np.abs(X[:, 3]) > 0.01 if len(tt) else np.zeros(0, bool)

    actions = []
    for res in summary['actions']:
        t0, t1 = res['t_start'], res['t_end']
        w = (tt >= t0) & (tt <= t1 + 0.6)
        a = {'type': res['type'], 'result': res['result'], 'expect': res['expect'], 'pass': res['pass'],
             'elapsed_s': round(res['elapsed_s'], 2), 'action': res['action']}
        if w.any():
            xs = X[w]
            a['path_length_m'] = round(float(np.hypot(np.diff(xs[:, 0]), np.diff(xs[:, 1])).sum()), 3)
            a['max_speed_mps'] = round(float(np.abs(xs[:, 3]).max()), 3)
            a['max_yaw_rate_dps'] = round(math.degrees(float(np.abs(xs[:, 4]).max())), 1)
            mw = w & moving & np.isfinite(xte)
            if mw.any():
                e = xte[mw]
                a['xte_rmse_mm'] = round(1000 * float(np.sqrt(np.mean(e ** 2))), 1)
                a['xte_max_mm'] = round(1000 * float(np.abs(e).max()), 1)
                a['xte_p95_mm'] = round(1000 * float(np.percentile(np.abs(e), 95)), 1)
                h = herr[mw]
                a['heading_err_rmse_deg'] = round(math.degrees(float(np.sqrt(np.nanmean(h ** 2)))), 2)
            final = xs[-1]
            a['final_pose'] = [round(float(final[0]), 4), round(float(final[1]), 4), round(math.degrees(final[2]), 2)]
        cmds = [c for c in rec['commands'] if t0 <= c['t'] <= t1 + 0.6]
        a['commands'] = {op: sum(1 for c in cmds if c['op'] == op) for op in ('EXECUTE', 'UPDATE_PATH', 'STOP')}
        a['tasks'] = len({c['task_id'] for c in cmds if c['op'] == 'EXECUTE'})
        a['path_versions'] = sum(1 for p in paths if t0 <= p['t'] <= t1)
        a['states'] = [(round(s['t'] - t0, 2), s['v']) for s in rec['state'] if t0 - 0.1 <= s['t'] <= t1 + 0.6]
        ph = [s for s in rec['box_phase'] if t0 - 0.1 <= s['t'] <= t1 + 0.6]
        a['box_phases'] = [(round(s['t'] - t0, 2), s['v']) for s in ph]
        durs = {}
        for i, s in enumerate(ph):
            end = ph[i + 1]['t'] if i + 1 < len(ph) else t1
            durs[s['v']] = round(durs.get(s['v'], 0) + end - s['t'], 2)
        a['phase_durations_s'] = durs
        a['box_status'] = [(round(s['t'] - t0, 2), s['text']) for s in rec['box_status'] if t0 - 0.1 <= s['t'] <= t1 + 0.6]
        if w.any() and res['type'] == 'goal':
            act = res['action']
            gx, gy = act['pose'][:2]
            a['final_goal_dist_mm'] = round(1000 * math.hypot(final[0] - gx, final[1] - gy), 1)
            if len(act['pose']) > 2:
                a['final_heading_err_deg'] = round(math.degrees(wrap(final[2] - math.radians(act['pose'][2]))), 2)
        if w.any() and res['type'] in ('box', 'box_near'):
            box = boxes[res['action']['name']]
            g = box_geometry(tuple(final[:3]), box, float(ba['front_face_x']))
            heading = g['face_normal_out'] + math.pi
            dx, dy = final[0] - box.x, final[1] - box.y
            lat_planning = -math.sin(heading) * dx + math.cos(heading) * dy
            a['final_box'] = {
                'gap_mm': round(1000 * g['gap_m'], 2),
                'gap_target_mm': round(1000 * float(ba['camera_final_gap']), 1),
                'gap_tolerance_mm': round(1000 * float(ba['camera_gap_tolerance']), 1),
                'lateral_mm': round(1000 * lat_planning, 2),
                'lateral_tolerance_mm': round(1000 * float(ba['lateral_tolerance']), 1),
                'yaw_err_deg': round(math.degrees(wrap(final[2] - heading)), 3),
                'yaw_tolerance_deg': float(ba.get('yaw_tolerance_deg', 1.0)),
            }
            fb = a['final_box']
            fb['truth_ok'] = bool(abs(fb['gap_mm'] - fb['gap_target_mm']) <= fb['gap_tolerance_mm'] + 0.05 and
                                  abs(fb['lateral_mm']) <= fb['lateral_tolerance_mm'] and
                                  abs(fb['yaw_err_deg']) <= fb['yaw_tolerance_deg'])
            gaps = []
            for s, xv in zip(tt[w], xs):
                gg = box_geometry(tuple(xv[:3]), box, float(ba['front_face_x']))
                gaps.append((s - t0, gg['gap_m']))
            a['_gap_series'] = gaps
        # 장애물 최소 간격
        clear = {}
        for ob in obstacles:
            for (ta, tb) in active.get(ob.name, []):
                sel = w & (tt >= ta) & (tt <= tb)
                if sel.any():
                    v = min(obstacle_clearance(tuple(p[:3]), footprint, ob) for p in X[sel][::5])
                    clear[ob.name] = round(1000 * min(v, clear.get(ob.name, 1e9) / 1000), 1)
        a['min_obstacle_clearance_mm'] = clear
        a['collisions'] = sum(1 for c in collisions if t0 <= c['t'] <= t1 + 0.6)
        actions.append(a)

    # ------------------------------------------------------------------ 그림
    imgs = {}
    if len(tt):
        imgs['map'] = plot_map(m, X, paths, boxes, obstacles, active, rec, summary, sc)
        imgs['series'] = plot_series(tt, X, xte, rec, summary)
        for i, a in enumerate(actions):
            if '_gap_series' in a:
                imgs[f'box{i}'] = plot_box(a, ba)
    for k, v in imgs.items():
        (run_dir / f'{k}.png').write_bytes(v)

    planning_log = (run_dir / 'planning.log').read_text(errors='replace').splitlines() if (run_dir / 'planning.log').exists() else []
    control_log = (run_dir / 'control.log').read_text(errors='replace').splitlines() if (run_dir / 'control.log').exists() else []
    metrics = {'actions': [{k: v for k, v in a.items() if not k.startswith('_')} for a in actions],
               'collisions_total': len(collisions)}
    write_html(run_dir, sc, summary, metrics, actions, imgs, planning_log, control_log, rec)
    return metrics


# ---------------------------------------------------------------------------------------------------------------
def plot_map(m, X, paths, boxes, obstacles, active, rec, summary, sc):
    xs, ys = X[:, 0], X[:, 1]
    pts = [(b.x, b.y) for b in boxes.values() if any(a['action'].get('name') == b.name for a in summary['actions'])]
    for p in paths:
        pts += p['pts'][::10]
    allx = np.concatenate([xs, [p[0] for p in pts]]) if pts else xs
    ally = np.concatenate([ys, [p[1] for p in pts]]) if pts else ys
    x0, x1 = allx.min() - 0.6, allx.max() + 0.6
    y0, y1 = ally.min() - 0.6, ally.max() + 0.6
    ex = m.extent
    img = np.full(m.occupied.shape, 1.0)
    img[m.unknown] = 0.8
    img[m.occupied] = 0.15
    fig, ax = plt.subplots(figsize=(8, 8 * (y1 - y0) / max(x1 - x0, 0.5)))
    ax.imshow(img, origin='lower', extent=ex, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
    for b in boxes.values():
        c = b.corners() + [b.corners()[0]]
        ax.fill([p[0] for p in c], [p[1] for p in c], color='#d9963a', alpha=0.6)
        ax.text(b.x, b.y, b.name, ha='center', va='center', fontsize=8, clip_on=True)
    for ob in obstacles:
        col = '#d62728' if active.get(ob.name) else '#999999'
        if ob.shape == 'circle':
            ax.add_patch(plt.Circle(ob.center, ob.radius, color=col, alpha=0.6))
        else:
            co, so = math.cos(ob.yaw), math.sin(ob.yaw)
            hw, hh = ob.size[0] / 2, ob.size[1] / 2
            c = [(ob.center[0] + co * a - so * b, ob.center[1] + so * a + co * b)
                 for a, b in ((hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh), (hw, hh))]
            ax.fill([p[0] for p in c], [p[1] for p in c], color=col, alpha=0.6)
        ax.text(ob.center[0], ob.center[1], ob.name, ha='center', va='center', fontsize=7, color='white', clip_on=True)
    for i, p in enumerate(paths):
        P = np.asarray(p['pts'])
        ax.plot(P[:, 0], P[:, 1], color='#1f77b4', alpha=0.3 if i < len(paths) - 1 else 0.9, lw=0.8, ls='--', zorder=4,
                label='추종 경로' if i == len(paths) - 1 else None)
    sc_ = ax.scatter(xs, ys, c=np.abs(X[:, 3]), cmap='viridis', s=3, zorder=3)
    fig.colorbar(sc_, ax=ax, fraction=0.03, label='속도 [m/s]')
    la = rec['lookahead']
    if la:
        L = np.array([[q['x'], q['y']] for q in la[::4]])
        ax.scatter(L[:, 0], L[:, 1], s=2, color='#ff7f0e', alpha=0.5, zorder=2, label='룩어헤드 점')
    ax.plot(xs[0], ys[0], 'o', color='#2ca02c', ms=8, label='출발')
    ax.plot(xs[-1], ys[-1], 's', color='#d62728', ms=7, label='끝')
    for act in rec['actions']:
        if act.get('type') == 'goal':
            ax.plot(act['x'], act['y'], '*', color='#9467bd', ms=12, label='목표')
    for g in rec['approach_goal'][-1:]:
        ax.arrow(g['x'], g['y'], 0.12 * math.cos(g['yaw']), 0.12 * math.sin(g['yaw']), width=0.008,
                 color='#9467bd', label='면 목표')
    for e in rec['events']:
        if e.get('type') == 'collision':
            ax.plot(e['x'], e['y'], 'x', color='red', ms=8)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map)')
    ax.set_ylabel('y [m]')
    h, l = ax.get_legend_handles_labels()
    uniq = dict(zip(l, h))
    ax.legend(uniq.values(), uniq.keys(), loc='best', fontsize=8)
    ax.set_title(f'{summary["name"]}: 경로(파랑)·궤적(속도 색)')
    return fig_png(fig)


def phase_spans(ax, rec, t0, t_end):
    ph = rec['box_phase']
    for i, s in enumerate(ph):
        a = s['t'] - t0
        b = (ph[i + 1]['t'] if i + 1 < len(ph) else t_end) - t0
        col = PHASE_COLORS.get(s['v'])
        if col:
            ax.axvspan(a, b, color=col, alpha=0.6, lw=0)


def plot_series(tt, X, xte, rec, summary):
    t0 = summary['actions'][0]['t_start'] if summary['actions'] else tt[0]
    t = tt - t0
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax in axs:
        phase_spans(ax, rec, t0, tt[-1])
    axs[0].plot(t, X[:, 3], lw=1)
    axs[0].set_ylabel('v [m/s]')
    axs[1].plot(t, np.degrees(X[:, 4]), lw=1, color='#ff7f0e')
    axs[1].set_ylabel('ω [deg/s]')
    axs[2].plot(t, 1000 * xte, lw=1, color='#2ca02c')
    axs[2].axhline(0, color='k', lw=0.5)
    axs[2].set_ylabel('횡오차 [mm]')
    axs[2].set_xlabel('첫 요청 후 시간 [s]')
    for s in rec['state']:
        if s['v'] in ('Run.Complete', 'Fault'):
            for ax in axs:
                ax.axvline(s['t'] - t0, color='red' if s['v'] == 'Fault' else 'green', lw=1, ls='--')
    names = sorted({s['v'] for s in rec['box_phase'] if s['v'] in PHASE_COLORS},
                   key=list(PHASE_COLORS).index)
    if names:
        from matplotlib.patches import Patch
        axs[0].legend([Patch(color=PHASE_COLORS[n]) for n in names], names, fontsize=7, ncol=4, loc='upper right')
    return fig_png(fig)


def plot_box(a, ba):
    g = np.array(a['_gap_series'])
    tgt, tol = 1000 * float(ba['camera_final_gap']), 1000 * float(ba['camera_gap_tolerance'])
    hand = 1000 * float(ba['handoff_gap'])
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4), gridspec_kw={'width_ratios': [1, 1.3]})
    near = g[1000 * g[:, 1] <= 150]
    for ax, data, title in ((axs[0], g, '전체 (400 mm 이하)'), (axs[1], near if len(near) else g, '도킹 구간 확대 (150 mm 이하)')):
        ax.plot(data[:, 0], 1000 * data[:, 1], lw=1.2)
        ax.axhline(hand, color='#ff7f0e', ls='--', lw=1, label='LiDAR→카메라 전환 간격')
        ax.axhspan(tgt - tol, tgt + tol, color='#2ca02c', alpha=0.3, label='최종 간격 허용')
        ax.set_xlabel('요청 후 시간 [s]')
        ax.set_title(title, fontsize=10)
    axs[0].set_ylim(min(0, 1000 * g[:, 1].min() - 3), min(400, 1000 * g[:, 1].max() + 10))
    axs[1].set_ylim(min(0, 1000 * g[:, 1].min() - 3), 150)
    axs[0].set_ylabel('참값 간격 [mm]')
    axs[1].legend(fontsize=8)
    fig.suptitle('박스 앞면 간격 (LiDAR 앞면 → 박스 면, 참값)', fontsize=11)
    return fig_png(fig)


# ---------------------------------------------------------------------------------------------------------------
CSS = """
:root{--bg:#fbfaf7;--fg:#1d1d1b;--mut:#6b6a66;--card:#fff;--line:#e3e1db;--ok:#1f7a3f;--bad:#b42318;--warn:#a15c00}
@media (prefers-color-scheme:dark){:root{--bg:#161614;--fg:#ecebe6;--mut:#a3a29c;--card:#1f1f1c;--line:#34332f}}
body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,'Noto Sans KR',sans-serif;margin:0}
main{max-width:1100px;margin:0 auto;padding:24px 16px}
h1{font-size:24px;margin:0 0 4px} h2{font-size:18px;margin:28px 0 10px;border-bottom:1px solid var(--line);padding-bottom:4px}
.sub{color:var(--mut);margin:0 0 16px} .pill{display:inline-block;padding:2px 10px;border-radius:99px;font-weight:600;color:#fff}
.ok{background:var(--ok)} .bad{background:var(--bad)} .warn{background:var(--warn)}
table{border-collapse:collapse;width:100%;background:var(--card);margin:8px 0}
td,th{border:1px solid var(--line);padding:5px 8px;text-align:left;vertical-align:top;font-size:14px}
th{background:rgba(127,127,127,.08)} img{max-width:100%;border:1px solid var(--line);border-radius:6px;background:#fff}
pre{background:var(--card);border:1px solid var(--line);padding:8px;overflow:auto;font-size:12px;max-height:360px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:8px}
.kpi{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:8px 10px}
.kpi b{display:block;font-size:20px} .kpi span{color:var(--mut);font-size:12px}
details summary{cursor:pointer;color:var(--mut)}
"""


def kpi(label, value):
    return f'<div class="kpi"><span>{html.escape(label)}</span><b>{html.escape(str(value))}</b></div>'


def write_html(run_dir, sc, summary, metrics, actions, imgs, planning_log, control_log, rec):
    e = html.escape
    passed = summary.get('pass')
    if passed is None:
        passed = bool(summary['actions']) and all(a['pass'] for a in summary['actions']) and \
            len(summary['actions']) == len(sc.get('actions', []))
    parts = [f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
             f'<title>시뮬레이션 {e(summary["name"])}</title><style>{CSS}</style></head><body><main>',
             f'<h1>{e(summary["name"])} <span class="pill {"ok" if passed else "bad"}">{"통과" if passed else "실패"}</span></h1>',
             f'<p class="sub">{e(sc.get("description", ""))}<br>맵 {e(Path(summary["map"]).name)} · 로봇 코드 '
             f'{e(summary["craft_src"])} · 실행 폴더 {e(str(run_dir))}</p>']
    for i, a in enumerate(actions):
        act = a['action']
        label = f'목표 {act["pose"]}' if a['type'] == 'goal' else f'박스 {act.get("name")} ({a["type"]})'
        ok = a['pass']
        parts.append(f'<h2>요청 {i + 1}: {e(label)} <span class="pill {"ok" if ok else "bad"}">'
                     f'{e(RESULT_TEXT.get(a["result"], a["result"]))}</span> <small>(기대: {e(RESULT_TEXT.get(a["expect"], a["expect"]))})</small></h2>')
        k = [kpi('소요 시간', f'{a["elapsed_s"]} s')]
        if 'path_length_m' in a:
            k += [kpi('주행 거리', f'{a["path_length_m"]} m'), kpi('최고 속도', f'{a["max_speed_mps"]} m/s')]
        if 'xte_rmse_mm' in a:
            k += [kpi('횡오차 RMSE', f'{a["xte_rmse_mm"]} mm'), kpi('횡오차 최대', f'{a["xte_max_mm"]} mm'),
                  kpi('방향 오차 RMSE', f'{a["heading_err_rmse_deg"]}°')]
        if 'final_goal_dist_mm' in a:
            k.append(kpi('목표까지 남은 거리', f'{a["final_goal_dist_mm"]} mm'))
        if 'final_box' in a:
            fb = a['final_box']
            k.append(kpi('참값 기준 최종 검사', '허용 안' if fb['truth_ok'] else '허용 초과 (플래닝 판정과 다를 수 있음)'))
            k += [kpi('최종 간격', f'{fb["gap_mm"]} mm (목표 {fb["gap_target_mm"]}±{fb["gap_tolerance_mm"]})'),
                  kpi('최종 횡오차', f'{fb["lateral_mm"]} mm (허용 {fb["lateral_tolerance_mm"]})'),
                  kpi('최종 방향 오차', f'{fb["yaw_err_deg"]}° (허용 {fb["yaw_tolerance_deg"]})')]
        k += [kpi('EXECUTE / UPDATE / STOP', f'{a["commands"]["EXECUTE"]} / {a["commands"]["UPDATE_PATH"]} / {a["commands"]["STOP"]}'),
              kpi('추종 경로 갱신', a['path_versions']), kpi('충돌', a['collisions'])]
        for name, v in a['min_obstacle_clearance_mm'].items():
            k.append(kpi(f'{name} 최소 간격', f'{v} mm'))
        parts.append('<div class="grid">' + ''.join(k) + '</div>')
        if a['phase_durations_s']:
            parts.append('<table><tr><th>박스 단계</th><th>시간 [s]</th></tr>' + ''.join(
                f'<tr><td>{e(p)}</td><td>{d}</td></tr>' for p, d in a['phase_durations_s'].items() if p != 'off') + '</table>')
        rows = [(t, '상태', v) for t, v in a['states']] + [(t, '박스 단계', v) for t, v in a['box_phases']] + \
               [(t, 'box_status', v) for t, v in a['box_status']]
        rows.sort(key=lambda r: r[0])
        parts.append('<details><summary>시간순 단계·상태 기록</summary><table><tr><th>t [s]</th><th>종류</th><th>내용</th></tr>' +
                     ''.join(f'<tr><td>{t}</td><td>{e(kd)}</td><td>{e(v)}</td></tr>' for t, kd, v in rows) + '</table></details>')
        if f'box{i}' in imgs:
            parts.append(f'<img alt="박스 간격" src="data:image/png;base64,{base64.b64encode(imgs[f"box{i}"]).decode()}">')
    parts.append('<h2>지도</h2>')
    if 'map' in imgs:
        parts.append(f'<img alt="지도와 궤적" src="data:image/png;base64,{base64.b64encode(imgs["map"]).decode()}">')
    parts.append('<h2>시간 그래프</h2><p class="sub">배경색은 박스 단계, 녹색 점선은 Run.Complete, 빨간 점선은 Fault. '
                 '횡오차는 control 이 추종 중인 경로(/control/tracking_path) 기준, 왼쪽이 +.</p>')
    if 'series' in imgs:
        parts.append(f'<img alt="시간 그래프" src="data:image/png;base64,{base64.b64encode(imgs["series"]).decode()}">')
    ev = [x for x in rec['events'] if x.get('type') in ('obstacle', 'collision', 'request', 't0')]
    if ev:
        t0 = summary['actions'][0]['t_start'] if summary['actions'] else summary['t_begin']
        parts.append('<h2>시뮬레이터 이벤트</h2><table><tr><th>t [s]</th><th>내용</th></tr>' + ''.join(
            f'<tr><td>{x["t"] - t0:.2f}</td><td>{e(x["text"])}</td></tr>' for x in ev[:200]) + '</table>')
    errs = [ln for ln in planning_log if 'ERROR' in ln or 'WARN' in ln]
    parts.append('<h2>planning_master 로그</h2>')
    parts.append(f'<details><summary>WARN/ERROR {len(errs)}줄</summary><pre>{e(chr(10).join(errs[-80:]))}</pre></details>')
    parts.append(f'<details><summary>마지막 80줄</summary><pre>{e(chr(10).join(planning_log[-80:]))}</pre></details>')
    cerr = [ln for ln in control_log if 'ERROR' in ln or 'WARN' in ln]
    parts.append('<h2>control_master 로그</h2>')
    parts.append(f'<details><summary>WARN/ERROR {len(cerr)}줄</summary><pre>{e(chr(10).join(cerr[-80:]))}</pre></details>')
    parts.append(f'<details><summary>지표 JSON</summary><pre>{e(json.dumps(metrics, ensure_ascii=False, indent=1))}</pre></details>')
    parts.append('<p class="sub">시뮬레이터 가정: 이상적 차동구동(명령 = 실제), 위치·LiDAR·카메라 참값, 지연·잡음·미끄럼 없음. '
                 '실제 planning_master·control_master 바이너리를 그대로 실행.</p></main></body></html>')
    (run_dir / 'report.html').write_text(''.join(parts))
