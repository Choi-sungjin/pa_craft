#!/usr/bin/python3
"""sis bag (brobag) -> where each step's time goes, as a PNG next to the bag.

Per MANIPULATOR step, from its /planning/task_command to its SUCCEEDED /control/task_result:
  move     until every joint is within NEAR (1 deg, LM 1 mm) of the target for good
  residual until every joint is within its completion tolerance (0.5 deg, LM 0.06 mm) for good
  judge    from there to SUCCEEDED (control/board completion latency)
Then per joint: residual time and overshoot past the tolerance (I gain steps: up while residual is long, back
when overshoot shows). Below: per joint error to the active target with the tolerance band, the board state strip and step bands.

Usage:  source ~/craft/install/setup.bash && /usr/bin/python3 demo/puck_pick/bag_plot.py [~/bags/<name>]
        (default: newest bag in ~/bags; system python: the venv numpy 2 breaks the system matplotlib)
"""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LANES = [('LM', 'mm', 1000.0, 0.06), ('shoulder', 'deg', 180 / math.pi, 0.5), ('elbow', 'deg', 180 / math.pi, 0.5),
         ('wrist_pitch', 'deg', 180 / math.pi, 0.5), ('tool_roll', 'deg', 180 / math.pi, 0.5)]
NEAR = 1.0  # [deg | mm] "arrived" for the move phase
RESIDUAL = 3.0  # [tolerances] out of tolerance but this close counts as residual (1.5 deg, 0.18 mm)
KIND = {0: 'NAVIGATE', 1: 'MANIPULATOR', 2: 'GRIPPER', 3: 'CLOCK', 4: 'WAIT'}
BLUE, ORANGE, AQUA, GREY, RED = '#2a78d6', '#eb6834', '#1baf7a', '#9aa1a8', '#d03b3b'


def read(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, t = r.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def last_out(ts, err, limit):
    """Time of the last sample with |err| > limit in any column (ts[0] if none)."""
    bad = np.where((np.abs(err) > limit).any(axis=1))[0]
    return ts[bad[-1]] if len(bad) else ts[0]


def main():
    bag = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else max(Path('~/bags').expanduser().iterdir(),
                                                                        key=lambda p: p.stat().st_mtime)
    b = read(bag)
    J = b['/control/arm_joints']
    t0 = J[0][0]
    ts = np.array([t for t, _ in J]) - t0
    pos = np.array([list(m.position) for _, m in J])
    scale = np.array([s for _, _, s, _ in LANES])
    tol = np.array([x for _, _, _, x in LANES])
    done = {}
    for t, m in b['/control/task_result']:
        if m.status not in (0, 3) and m.task_id not in done:  # first terminal result (not RUNNING/STOPPING)
            done[m.task_id] = (t - t0, m.status, m.reason)

    steps, target, err = [], None, np.full(pos.shape, np.nan)
    for t, c in b['/planning/task_command']:
        t, end = t - t0, done.get(c.task_id, (ts[-1], None, 'no result'))
        s = dict(kind=KIND.get(c.kind, c.kind), t0=t, t1=end[0], status=end[1], reason=end[2])
        if c.kind == 1:
            target = np.array(c.joints.position[:5])  # move 1 may carry a 6th, the gripper
            m = (ts >= t) & (ts <= end[0])
            e = (pos[m] - target) * scale
            s['near'] = last_out(ts[m], e / NEAR, 1.0)
            s['tol'] = last_out(ts[m], e / tol, 1.0)
            err[ts >= t] = (pos[ts >= t] - target) * scale
            # largest residual (in tolerances) when the move phase ends
            i = np.searchsorted(ts, s['near'] + 0.05)
            k = int(np.argmax(np.abs(err[i]) / tol))
            s['worst'] = (LANES[k][0], err[i][k], LANES[k][1])
            # Per joint: residual = time out of tolerance but within RESIDUAL tolerances (the 1-tick creep);
            # overshoot = after first entering the tolerance, the largest excursion past it on the opposite side
            # of where it came from (I gain too high: crosses the target and comes back).
            s['residual'], s['over'] = {}, {}
            tt, dt = ts[m], np.diff(ts[m], append=ts[m][-1])
            for j, (name, unit, _, tl) in enumerate(LANES):
                a = np.abs(e[:, j])
                s['residual'][name] = float(dt[(a > tl) & (a <= RESIDUAL * tl)].sum())
                inside = np.where(a <= tl)[0]
                if len(inside) and inside[0] > 0:
                    side = np.sign(e[inside[0] - 1, j])
                    past = -side * e[inside[0]:, j]
                    s['over'][name] = float(max(past.max() - tl, 0.0))
        steps.append(s)
    states = [(t - t0, m.data) for t, m in b['/control/arm_lm_state']]
    faults = [t - t0 for t, m in b['/control/status'] if m.fault]

    print(f'{bag}  ({len(ts)} arm feedback samples, {ts[-1]:.1f} s)')
    print(f'{"step":4s} {"kind":12s} {"total":>7s} {"move":>7s} {"residual":>9s} {"judge":>7s}  worst residual at arrival')
    for n, s in enumerate(steps, 1):
        tot = s['t1'] - s['t0']
        if 'near' in s:
            mv, rs, jd = s['near'] - s['t0'], s['tol'] - s['near'], s['t1'] - s['tol']
            w = f"{s['worst'][0]} {s['worst'][1]:+.2f} {s['worst'][2]}"
            print(f'{n:<4d} {s["kind"]:12s} {tot:7.2f} {mv:7.2f} {rs:9.2f} {jd:7.2f}  {w}')
        else:
            print(f'{n:<4d} {s["kind"]:12s} {tot:7.2f}')
        if s['status'] != 1:
            print(f'     -> status {s["status"]} {s["reason"]}')
    print(f'\nper joint over all MANIPULATOR steps: residual = s out of tolerance but within {RESIDUAL:g} tolerances,'
          '\n  overshoot = worst crossing past the tolerance after first reaching it (> 0: lower that I gain)')
    for name, unit, _, _ in LANES:
        res = sum(s['residual'][name] for s in steps if 'residual' in s)
        over = max([s['over'].get(name, 0.0) for s in steps if 'over' in s] or [0.0])
        print(f'  {name:12s} residual {res:5.1f} s   overshoot {over:5.2f} {unit}')
    if faults:
        print(f'fault at {faults[0]:.2f} s (last arm feedback {ts[-1]:.2f} s)')

    fig = plt.figure(figsize=(14, 13), constrained_layout=True)
    gs = fig.add_gridspec(2, 1, height_ratios=[len(steps) * 0.42 + 0.8, 9])
    ax = fig.add_subplot(gs[0])
    for n, s in enumerate(steps):
        y, tot = len(steps) - 1 - n, s['t1'] - s['t0']
        label = f'{n + 1}. {s["kind"].lower()}'
        if 'near' in s:
            mv, rs, jd = s['near'] - s['t0'], s['tol'] - s['near'], s['t1'] - s['tol']
            ax.barh(y, mv, color=BLUE, height=0.6)
            ax.barh(y, rs, left=mv, color=ORANGE, height=0.6)
            ax.barh(y, jd, left=mv + rs, color=AQUA, height=0.6)
            ax.text(tot + 0.08, y, f'{tot:.2f} s  (move {mv:.2f} / residual {rs:.2f} / judge {jd:.2f})  '
                    f'{s["worst"][0]} {s["worst"][1]:+.2f} {s["worst"][2]}', va='center', fontsize=9)
        else:
            ax.barh(y, tot, color=GREY, height=0.6)
            ax.text(tot + 0.08, y, f'{tot:.2f} s', va='center', fontsize=9)
        ax.text(-0.1, y, label, va='center', ha='right', fontsize=9)
    ax.set_yticks([])
    ax.set_xlim(0, max(s['t1'] - s['t0'] for s in steps) * 1.9)
    ax.set_xlabel('s from the command')
    ax.set_title(f'{bag.name}: time per step  (move: all joints within {NEAR:g} deg / mm,  residual: until within '
                 f'0.5 deg / 0.06 mm,  judge: then to SUCCEEDED)', fontsize=10, loc='left')
    ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=c) for c in (BLUE, ORANGE, AQUA, GREY)],
              labels=['move', 'residual (I gain, 1 tick steps)', 'judge', 'gripper / clock'], loc='lower right',
              fontsize=9, frameon=False)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)

    sub = gs[1].subgridspec(len(LANES) + 1, 1, height_ratios=[0.35] + [1] * len(LANES), hspace=0.08)
    axs = [fig.add_subplot(sub[0])]
    axs += [fig.add_subplot(sub[i + 1], sharex=axs[0]) for i in range(len(LANES))]
    for (t, v), nxt in zip(states, states[1:] + [(ts[-1], None)]):
        if v:
            axs[0].axvspan(t, nxt[0], color=RED if v == 2 else ORANGE, lw=0)
    axs[0].set_yticks([])
    axs[0].set_ylabel('board\nmoving', rotation=0, ha='right', va='center', fontsize=9)
    for k, (name, unit, _, tl) in enumerate(LANES):
        a = axs[k + 1]
        lim = 3 * tl / 0.5 if unit == 'deg' else 1.0
        for n, s in enumerate(steps):
            a.axvspan(s['t0'], s['t1'], color='#eef1f4' if n % 2 else '#e3e8ed', lw=0, zorder=0)
        a.axhspan(-tl, tl, color='#d9ecdf', lw=0, zorder=1)
        a.plot(ts, np.clip(err[:, k], -lim, lim), color=BLUE, lw=1.2, zorder=2)
        a.set_ylim(-lim * 1.05, lim * 1.05)
        a.set_ylabel(f'{name}\n[{unit}]', rotation=0, ha='right', va='center', fontsize=9)
        for t in faults[:1]:
            a.axvline(t, color=RED, ls='--', lw=1)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
    for n, s in enumerate(steps):
        axs[0].text(s['t0'], 1.15, str(n + 1), fontsize=8, va='bottom', transform=axs[0].get_xaxis_transform())
    axs[0].set_xlim(0, ts[-1] + 2)  # the bag runs on after the last arm feedback (fault)
    axs[-1].set_xlabel('s from the bag start (error clipped to +-3 deg / +-1 mm; green = completion tolerance)')
    for a in axs[:-1]:
        plt.setp(a.get_xticklabels(), visible=False)
    out = bag / 'steps.png'
    fig.savefig(out, dpi=110)
    print(f'-> {out}')


if __name__ == '__main__':
    main()
