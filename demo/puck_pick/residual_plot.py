#!/usr/bin/python3
"""Residual time per gain run: several sis1 bags side by side -> residual.png (next to the newest bag).

Residual = time a joint is out of its completion tolerance (0.5 deg, LM 0.06 mm) but within RESIDUAL
tolerances (bag_plot.py), i.e. the 1-tick I-gain creep, per MANIPULATOR step and per joint.
  top:    per step, one stacked bar per run (shoulder / elbow / wrist_pitch)
  bottom: per joint total over the steps every run has (move 1 .. ascend by default), one bar per run

Usage:  source ~/craft/install/setup.bash && /usr/bin/python3 demo/puck_pick/residual_plot.py \\
            ~/bags/sis_1103_1008=sh100/wr100 ~/bags/sis_1103_1029=sh150/wr150 ...
"""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from bag_plot import LANES, RESIDUAL, read

STEPS = ['move 1', 'move 2', 'descend', 'ascend', 'load 1', 'load 2', 'load 3', 'back 1', 'back 2']  # sis1 MANIPULATOR order
COMMON = STEPS[:4]  # same targets in every run so far (the loading pose changed on 2026-11-03)
JOINTS = [1, 2, 3]  # shoulder, elbow, wrist_pitch (LM and tool_roll stay ~0)
COLORS = ['#2a78d6', '#eb6834', '#1baf7a']


def residuals(bag):
    """{step: [s per LANES joint]} for the MANIPULATOR steps that finished."""
    b = read(bag)
    J = b.get('/control/arm_joints', [])
    if not J:
        return {}
    t0 = J[0][0]
    ts = np.array([t for t, _ in J]) - t0
    pos = np.array([list(m.position) for _, m in J])
    scale = np.array([s for _, _, s, _ in LANES])
    tol = np.array([x for _, _, _, x in LANES])
    done = {}
    for t, m in b['/control/task_result']:
        if m.status == 1 and m.task_id not in done:
            done[m.task_id] = t - t0
    out = {}
    manip = [(t - t0, c) for t, c in b['/planning/task_command'] if c.kind == 1]
    for (t, c), name in zip(manip, STEPS):
        if c.task_id not in done:
            break
        m = (ts >= t) & (ts <= done[c.task_id])
        e = np.abs((pos[m] - np.array(c.joints.position[:5])) * scale)  # move 1 may carry a 6th, the gripper
        dt = np.diff(ts[m], append=ts[m][-1])
        out[name] = [float(dt[(e[:, j] > tol[j]) & (e[:, j] <= RESIDUAL * tol[j])].sum()) for j in range(len(LANES))]
    return out


def main():
    runs = []
    for arg in sys.argv[1:]:
        path, _, label = arg.partition('=')
        path = Path(path).expanduser()
        r = residuals(path)
        if not r:
            print(f'skip {path.name}: no arm feedback / no finished step')
            continue
        runs.append((label or path.name, path, r))
    if not runs:
        sys.exit('no usable bags')
    steps = [s for s in STEPS if any(s in r for _, _, r in runs)]

    print(f'{"step":10s}' + ''.join(f'{lab:>22s}' for lab, _, _ in runs) + '   (s: shoulder/elbow/wrist)')
    for s in steps:
        cells = [('/'.join(f'{r[s][j]:.1f}' for j in JOINTS) if s in r else '-') for _, _, r in runs]
        print(f'{s:10s}' + ''.join(f'{c:>22s}' for c in cells))
    print(f'{"common sum":10s}' + ''.join(
        f'{"/".join(f"{sum(r[s][j] for s in COMMON if s in r):.1f}" for j in JOINTS):>22s}' for _, _, r in runs))

    n = len(runs)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(max(10, 1.6 * len(steps) * n / 2), 9),
                                 gridspec_kw={'height_ratios': [3, 2]}, constrained_layout=True)
    w = 0.8 / n
    for i, (lab, _, r) in enumerate(runs):
        x = np.arange(len(steps)) - 0.4 + w * (i + 0.5)
        bottom = np.zeros(len(steps))
        for c, j in zip(COLORS, JOINTS):
            v = np.array([r[s][j] if s in r else 0.0 for s in steps])
            a1.bar(x, v, w * 0.9, bottom=bottom, color=c, label=LANES[j][0] if i == 0 else None)
            bottom += v
        for xi, s in zip(x, steps):
            a1.text(xi, -0.15, lab if s in r else '', rotation=90, ha='center', va='top', fontsize=7)
    a1.set_xticks(np.arange(len(steps)), steps)
    a1.tick_params(axis='x', pad=62)
    a1.set_ylabel('residual [s]')
    a1.set_title(f'residual per step (out of 0.5 deg but within {RESIDUAL * 0.5:g} deg), one bar per run',
                 loc='left', fontsize=10)
    a1.legend(frameon=False, fontsize=9)

    x = np.arange(len(JOINTS))
    for i, (lab, _, r) in enumerate(runs):
        v = [sum(r[s][j] for s in COMMON if s in r) for j in JOINTS]
        a2.bar(x - 0.4 + w * (i + 0.5), v, w * 0.9, color='#9aa1a8' if i < n - 1 else '#52514e')
        for xi, vi in zip(x - 0.4 + w * (i + 0.5), v):
            a2.text(xi, vi + 0.15, f'{vi:.1f}', ha='center', fontsize=8)
            a2.text(xi, -0.3, lab, rotation=90, ha='center', va='top', fontsize=7)
    a2.set_xticks(x, [LANES[j][0] for j in JOINTS])
    a2.tick_params(axis='x', pad=62)
    a2.set_ylabel('residual [s]')
    a2.set_title(f'per joint, sum over {", ".join(COMMON)} (same targets in every run); newest run dark',
                 loc='left', fontsize=10)
    for a in (a1, a2):
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
    out = max((p for _, p, _ in runs), key=lambda p: p.stat().st_mtime) / 'residual.png'
    fig.savefig(out, dpi=110)
    print(f'-> {out}')


if __name__ == '__main__':
    main()
