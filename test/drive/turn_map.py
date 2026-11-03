#!/usr/bin/env python3
"""In-place turn map: commanded wheel speed difference -> real chassis yaw rate, same experiment run 3 times.

Same board link as rotate.py (wheel_clock firmware, 50 Hz, clock held). Each trial commands an in-place
turn wl = -s*D/2, wr = +s*D/2 (D = wr - wl [rad/s], s = +1 CCW / -1 CW), waits out the firmware profile
ramp plus SETTLE, then averages the IMU yaw rate (gz minus the offset measured standing still right
before the trial) over HOLD seconds. No wheel geometry is assumed: the table maps D straight to the
measured yaw rate, and `fit` inverts it (desired yaw rate -> D to command). Scenario: README.md.

  turn_map.py run --spot 1 [--note "tile 12.1V"]   run (or resume) run 1/2/3; the first call makes the plan
  turn_map.py fit                                  spot-dependence verdict + monotone lookup table
  turn_map.py plan [--seed N] [--reps 3] --force   start over with a new plan (new trial order)
Stop control_master first; run `run` through _craft_locked wheel (board lock).
"""
import argparse
import csv
import math
import random
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rotate import GZ, WHEEL_ACCEL, Board  # noqa: E402
from slip_test import CONTROL_YAML, HZ, PORT, params  # noqa: E402  (rotate.py put slip/ on sys.path)
import yaml  # noqa: E402

LOGS = HERE / 'logs'
PLAN = LOGS / 'turn_map_plan.csv'
RESULTS = LOGS / 'turn_map_results.csv'
TABLE = LOGS / 'turn_map_table.yaml'
SPOTS = 3
# D = wr - wl [rad/s]. Dense at the low end, where static friction makes a dead band; 14 keeps each
# wheel (7 rad/s) under max_wheel 7.8. Geometric yaw rate r * D / track = 0.114 * D rad/s.
DELTAS = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0]
SETTLE = 0.5    # [s] after the profile ramp before averaging
HOLD = 3.0      # [s] averaging window
STILL_FB = 0.02  # [rad/s] both wheels below: stopped
OFFSET_S = 1.0  # [s] gyro offset sampling, robot stopped, before every trial
MOVING = 0.05   # [rad/s] |yaw rate| below this counts as not turning (dead band)
# Spot dependence, per (D, dir) with a moving median: the spread of the spot medians is over SPREAD_MAX of
# the overall median AND over NOISE_X times the largest within-spot spread (repeat noise cannot explain
# it). DISCARD_SHARE of such levels means one table cannot serve the floor: the method is dropped.
SPREAD_MAX = 0.15
NOISE_X = 2.0
DISCARD_SHARE = 0.25
PLAN_FIELDS = ['seed', 'spot_order', 'spot_id', 'pass', 'trial', 'delta', 'dir']
RESULT_FIELDS = ['seed', 'stamp', 'spot_order', 'spot_id', 'pass', 'trial', 'delta', 'dir', 'delta_fb',
                 'omega_imu', 'omega_std', 'omega_geo', 'geo_over_imu', 'track_l', 'track_r', 'gz_offset',
                 'aborted', 'note']


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def plan(args):
    if args.reps < 1:
        sys.exit('reps must be >= 1')
    if PLAN.exists() and not args.force:
        sys.exit(f'{PLAN.relative_to(HERE)} exists (results refer to its seed): --force to replace it')
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(10 ** 6)
    rng = random.Random(seed)
    rows = []
    for spot in range(1, SPOTS + 1):  # the same experiment SPOTS times, each with its own trial order
        trial = 0
        for rep in range(1, args.reps + 1):
            # Each pass covers every level once in both directions, in a new random order, so battery
            # drop and motor heating spread over all levels instead of biasing the large ones.
            trials = [(d, s) for d in DELTAS for s in ('ccw', 'cw')]
            rng.shuffle(trials)
            for d, s in trials:
                trial += 1
                rows.append([seed, spot, spot, rep, trial, d, s])
    LOGS.mkdir(exist_ok=True)
    with open(PLAN, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(PLAN_FIELDS)
        w.writerows(rows)
    print(f'[map] plan seed {seed}: {SPOTS} runs x {len(rows) // SPOTS} trials ({len(DELTAS)} levels x 2 dirs '
          f'x {args.reps} passes, shuffled per pass) -> {PLAN.relative_to(HERE)}')
    return 0


def offset(board):
    """Wait until both wheels stop, then the mean gz over OFFSET_S; None on fault or timeout."""
    t0, still_since = time.monotonic(), None
    while True:
        now = time.monotonic()
        if board.step(0.0, 0.0) is not None and board.fault():
            return None
        still = abs(board.fb[0]) < STILL_FB and abs(board.fb[1]) < STILL_FB
        still_since = (still_since or now) if still else None
        if still_since and now - still_since > 0.3:
            break
        if now - t0 > 5.0:
            return None
        time.sleep(1 / HZ)
    gz, t0 = [], time.monotonic()
    while time.monotonic() - t0 < OFFSET_S:
        if board.step(0.0, 0.0) is not None:
            gz.append(board.fb[GZ])
        time.sleep(1 / HZ)
    return sum(gz) / len(gz) if gz else None


def trial(board, delta, sign, gz0, raw, label):
    """One in-place turn; returns (delta_fb, omega, omega_std, track_l, track_r) or None if aborted."""
    half = sign * delta / 2
    start = half / sign / WHEEL_ACCEL + SETTLE  # firmware profile ramp, then settle
    t0 = time.monotonic()
    d_fb, om, tl, tr = [], [], [], []
    yaw = 0.0
    while (t := time.monotonic() - t0) < start + HOLD:
        now = time.monotonic()
        dt = board.step(-half, half)
        if dt is not None:
            fb = board.fb
            if board.fault():
                print('[map] board fault during the turn'); return None
            rate = fb[GZ] - gz0
            yaw += rate * dt
            if t >= start:
                d_fb.append(fb[1] - fb[0]); om.append(rate)
                tl.append(fb[0] / -half); tr.append(fb[1] / half)
            raw.append(label + [round(t, 4), round(-half, 4), round(half, 4), fb[0], fb[1], fb[GZ],
                                round(math.degrees(yaw), 3)])
        elif now - board.last > 0.5:
            print('[map] feedback lost'); return None
        time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
    if len(om) < 2:
        print('[map] no feedback in the averaging window'); return None
    return (statistics.fmean(d_fb), statistics.fmean(om), statistics.stdev(om),
            statistics.fmean(tl), statistics.fmean(tr))


def run(args):
    if not PLAN.exists():
        plan(argparse.Namespace(seed=None, reps=3, force=False))
    r, track, max_wheel = params()
    todo = [p for p in read_csv(PLAN) if int(p['spot_order']) == args.spot]
    if not todo:
        sys.exit(f'--spot must be 1..{SPOTS}')
    seed, spot_id = todo[0]['seed'], todo[0]['spot_id']
    if max(float(p['delta']) for p in todo) / 2 > max_wheel:
        sys.exit(f'plan exceeds max_wheel {max_wheel} rad/s')
    done = set()
    if RESULTS.exists():
        done = {int(x['trial']) for x in read_csv(RESULTS) if x['seed'] == seed and
                int(x['spot_order']) == args.spot and x['aborted'] == '0'}
    todo = [p for p in todo if int(p['trial']) not in done]
    if not todo:
        print(f'[map] run {args.spot} already complete'); return 0
    print(f'[map] run {args.spot}: {len(todo)} trials left ({len(done)} done). Clear 0.5 m around the spot.')
    input('[map] put the robot center on this run\'s spot mark, Enter to start ')

    board = Board(args.port)
    # An idle board reports state 2 (250 ms command-timeout hold) until a command arrives, and the first
    # line read here predates our first command: send stops before judging faults.
    board.stop(0.5)
    if board.fault():
        board.port.close()
        sys.exit('[map] state 2 after 0.5 s of stop commands: latched fault, checkwheel / resetwheel')
    if int(board.fb[5]) != 1:
        sys.exit('[map] imu_ok=0: resetwheel with the robot still')
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    raw, aborted, last_pass = [], False, int(todo[0]['pass'])
    LOGS.mkdir(exist_ok=True)
    new = not RESULTS.exists()
    with open(RESULTS, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(RESULT_FIELDS)
        try:
            for i, p in enumerate(todo, 1):
                if int(p['pass']) != last_pass:  # turns wander off the mark: re-center between passes
                    last_pass = int(p['pass'])
                    board.stop(0.5)
                    input(f'[map] pass {last_pass}: re-center the robot on the mark, Enter ')
                    board.stop(0.3)  # re-prime the 250 ms command watchdog after the prompt
                delta, sign = float(p['delta']), 1 if p['dir'] == 'ccw' else -1
                gz0 = offset(board)
                if gz0 is None:
                    print('[map] robot did not stop / board fault (checkwheel)'); aborted = True; break
                label = [p['trial'], delta, p['dir']]
                res = trial(board, delta, sign, gz0, raw, label)
                if res is None:
                    w.writerow([seed, stamp, args.spot, spot_id, p['pass'], p['trial'], delta, p['dir'],
                                '', '', '', '', '', '', '', round(gz0, 5), 1, args.note])
                    aborted = True; break
                d_fb, om, sd, tl, tr = res
                geo = r * d_fb / track
                ratio = geo / om if abs(om) > MOVING else float('inf')
                w.writerow([seed, stamp, args.spot, spot_id, p['pass'], p['trial'], delta, p['dir'],
                            round(d_fb, 4), round(om, 4), round(sd, 4), round(geo, 4), round(ratio, 3),
                            round(tl, 3), round(tr, 3), round(gz0, 5), 0, args.note])
                f.flush()
                print(f'[map] {i}/{len(todo)} D {delta:4.1f} {p["dir"]:>3}: imu {om:+.3f} +-{sd:.3f} rad/s, '
                      f'geo/imu {ratio:.2f}, tracking L {tl:.2f} R {tr:.2f}')
        except KeyboardInterrupt:
            print('\n[map] Ctrl+C (finished trials are saved; run again to resume)'); aborted = True
        finally:
            board.stop(0.5)
            board.port.close()
    raw_path = LOGS / f'{stamp}_turn_map_spot{args.spot}.csv'
    with open(raw_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial', 'delta', 'dir', 't', 'cmd_wl', 'cmd_wr', 'wheel_l', 'wheel_r', 'gz', 'yaw_imu_deg'])
        w.writerows(raw)
    print(f'[map] raw log {raw_path.relative_to(HERE)}; results {RESULTS.relative_to(HERE)}')
    return 1 if aborted else 0


def monotone(xs):
    """Pool-adjacent-violators: the closest non-decreasing sequence (equal weights)."""
    blocks = [[x, 1] for x in xs]
    i = 0
    while i < len(blocks) - 1:
        if blocks[i][0] > blocks[i + 1][0]:
            (a, n), (b, m) = blocks[i], blocks[i + 1]
            blocks[i:i + 2] = [[(a * n + b * m) / (n + m), n + m]]
            i = max(i - 1, 0)
        else:
            i += 1
    return [v for v, n in blocks for _ in range(n)]


def interp(x, xs, ys):
    """Piecewise-linear y(x) on ascending xs (clamped); used for yaw rate -> D."""
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1e-9)
    return ys[-1]


def fit(args):
    if not RESULTS.exists():
        sys.exit('no results yet')
    rows = [x for x in read_csv(RESULTS) if x['aborted'] == '0']
    seed = args.seed or (read_csv(PLAN)[0]['seed'] if PLAN.exists() else rows[-1]['seed'])
    rows = [x for x in rows if x['seed'] == seed]
    if not rows:
        sys.exit(f'no finished trials for seed {seed}')
    spots = sorted({x['spot_id'] for x in rows}, key=int)
    print(f'[map] seed {seed}, spots {spots}, {len(rows)} trials. |yaw rate| [rad/s], median per spot:')
    print(f'{"D":>5} {"dir":>4} ' + ' '.join(f'#{s:>5}' for s in spots) +
          f' {"all":>6} {"spread":>7} {"repeat":>7} {"geo/imu":>8} {"track":>6}  spot')
    table, judged, dependent = {}, 0, []
    for d in DELTAS:
        for s in ('ccw', 'cw'):
            g = [x for x in rows if float(x['delta']) == d and x['dir'] == s]
            if not g:
                continue
            vals = {sp: [abs(float(x['omega_imu'])) for x in g if x['spot_id'] == sp] for sp in spots}
            vals = {sp: v for sp, v in vals.items() if v}
            per = {sp: statistics.median(v) for sp, v in vals.items()}
            allv = statistics.median(abs(float(x['omega_imu'])) for x in g)
            between = max(per.values()) - min(per.values())
            within = max(max(v) - min(v) for v in vals.values())  # repeat noise on the same spot
            ratio = statistics.median(float(x['geo_over_imu']) for x in g)
            track = min(min(float(x['track_l']), float(x['track_r'])) for x in g)
            table[(d, s)] = allv
            verdict = ''
            if allv > MOVING and len(per) == SPOTS and min(len(v) for v in vals.values()) >= 2:
                judged += 1
                if between > SPREAD_MAX * allv and between > NOISE_X * within:
                    dependent.append(f'{d:g} {s}'); verdict = 'DEPENDENT'
                else:
                    verdict = 'same'
            pct = (lambda v: f'{v / allv:7.0%}' if allv > MOVING else f'{"-":>7}')
            print(f'{d:5.1f} {s:>4} ' + ' '.join(f'{per[sp]:6.3f}' if sp in per else f'{"-":>6}' for sp in spots) +
                  f' {allv:6.3f} {pct(between)} {pct(within)} {ratio:8.2f} {track:6.2f}  {verdict}')

    if judged:
        share = len(dependent) / judged
        if not dependent:
            print(f'[map] KEEP: no spot dependence on {judged} moving levels')
        elif share >= DISCARD_SHARE:
            print(f'[map] DISCARD: the yaw rate depends on the spot at {len(dependent)}/{judged} moving levels '
                  f'({", ".join(dependent)}); one D -> yaw rate table does not hold on this floor')
        else:
            print(f'[map] BORDERLINE: spot dependence at {len(dependent)}/{judged} levels ({", ".join(dependent)}); '
                  'rerun those levels before deciding')
    else:
        print(f'[map] no verdict yet: needs all {SPOTS} spots with >= 2 passes each')

    out = ['# turn_map.py fit: in-place yaw rate [rad/s] reached for D = wr - wl [rad/s], median of all',
           f'# spots (seed {seed}, spots {", ".join(spots)}), made non-decreasing (PAVA). Invert with linear',
           '# interpolation: desired |yaw rate| -> D; below the first moving level the floor holds the robot.',
           'turn_map:']
    for s in ('ccw', 'cw'):
        ds = [d for d in DELTAS if (d, s) in table]
        if len(ds) < 2:
            continue
        om = monotone([table[(d, s)] for d in ds])
        out.append(f'  {s}:')
        out.append(f'    delta: [{", ".join(f"{d:g}" for d in ds)}]')
        out.append(f'    omega: [{", ".join(f"{o:.4f}" for o in om)}]')
        moving = [d for d, o in zip(ds, om) if o > MOVING]
        print(f'[map] {s}: turns (> {MOVING} rad/s) from D {moving[0] if moving else "?"} rad/s; '
              'command for |yaw rate| '
              + ', '.join(f'{w:.1f}->D {interp(w, om, ds):.2f}' for w in (0.2, 0.4, 0.6, 0.8, 1.0)
                          if om[-1] >= w))
    TABLE.write_text('\n'.join(out) + '\n')
    print(f'[map] table {TABLE.relative_to(HERE)}')
    return 0


def plot(args):
    """|yaw rate| vs commanded D: one color per spot, marker per direction, trials as points, medians joined."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not RESULTS.exists():
        sys.exit('no results yet')
    rows = [x for x in read_csv(RESULTS) if x['aborted'] == '0']
    seed = args.seed or (read_csv(PLAN)[0]['seed'] if PLAN.exists() else rows[-1]['seed'])
    rows = [x for x in rows if x['seed'] == seed]
    if not rows:
        sys.exit(f'no finished trials for seed {seed}')
    r, track, _ = params()
    colors = ['#2a78d6', '#eb6834', '#1baf7a']  # categorical slots 1-3, fixed by spot order
    markers = {'ccw': ('o', 'CCW'), 'cw': ('^', 'CW')}
    ink, muted, grid = '#0b0b0b', '#52514e', '#e4e3df'
    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=130)
    fig.patch.set_facecolor('#fcfcfb'); ax.set_facecolor('#fcfcfb')
    dmax = max(DELTAS)
    ax.plot([0, dmax], [0, r * dmax / track], ls='--', lw=1.2, color=muted, zorder=1,
            label=f'no slip: r·D/W (r {r} m, W {track} m)')
    wheelbase = args.wheelbase if args.wheelbase is not None else \
        yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']['wheelbase']
    skid = 1 + (wheelbase / track) ** 2  # control's geometric estimate when skid_steer_factor is 0
    ax.plot([0, dmax], [0, r * dmax / track / skid], ls='-.', lw=1.2, color=muted, zorder=1,
            label=f'skid geometry: r·D/(W·(1+(L/W)²)), L {wheelbase} m -> ×1/{skid:.3f}')
    for sp in sorted({x['spot_order'] for x in rows}, key=int):
        c = colors[(int(sp) - 1) % len(colors)]
        for s, (m, name) in markers.items():
            g = [x for x in rows if x['spot_order'] == sp and x['dir'] == s]
            if not g:
                continue
            ax.scatter([float(x['delta']) for x in g], [abs(float(x['omega_imu'])) for x in g], s=28, marker=m,
                       facecolor=c if s == 'ccw' else 'none', edgecolor=c, lw=1.2, alpha=0.8, zorder=3)
            ds = sorted({float(x['delta']) for x in g})
            med = [statistics.median(abs(float(x['omega_imu'])) for x in g if float(x['delta']) == d) for d in ds]
            ax.plot(ds, med, color=c, lw=2, ls='-' if s == 'ccw' else ':', marker=m, ms=0, zorder=2,
                    label=f'run {sp} {name} (median)')
    xs = [float(x['delta']) for x in rows]
    ys = [abs(float(x['omega_imu'])) for x in rows]
    k = sum(a * b for a, b in zip(xs, ys)) / sum(a * a for a in xs)  # least squares through the origin
    ax.plot([0, dmax], [0, k * dmax], lw=1, color=ink, alpha=0.5, zorder=1,
            label=f'fit through 0: |yaw rate| = {k:.4f} · D')
    ax.set_xlim(0, dmax * 1.03); ax.set_ylim(0, None)
    ax.set_xlabel('commanded wheel speed difference D = wr − wl [rad/s]', color=ink)
    ax.set_ylabel('chassis |yaw rate| from IMU [rad/s]', color=ink)
    ax.set_title(f'In-place turn map (seed {seed}, {len(rows)} trials; filled = CCW, hollow = CW)',
                 color=ink, fontsize=11, loc='left')
    ax.grid(color=grid, lw=0.8); ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(muted)
    ax.tick_params(colors=muted)
    ax.legend(frameon=False, fontsize=9, labelcolor=ink, loc='upper left')
    out = LOGS / 'turn_map_plot.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'[map] plot {out.relative_to(HERE)}; fit through 0: {k:.4f} rad/s per rad/s of D '
          f'(no slip {r / track:.4f}, ratio {r / track / k:.3f}; skid geometry L {wheelbase} m: '
          f'{r / track / skid:.4f}, ratio {skid:.3f})')
    return 0


def poly(args):
    """Representative values (mean of every spot x direction series, each the mean of its passes) and
    w = sum(c_k * |D_fb|^k), k = 1..n, through the origin; n by leave-one-out error unless --degree."""
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not RESULTS.exists():
        sys.exit('no results yet')
    rows = [x for x in read_csv(RESULTS) if x['aborted'] == '0']
    seed = args.seed or (read_csv(PLAN)[0]['seed'] if PLAN.exists() else rows[-1]['seed'])
    rows = [x for x in rows if x['seed'] == seed]
    series = sorted({(x['spot_order'], x['dir']) for x in rows})
    # Odometry integrates the wheel feedback, so x is the measured D_fb, not the command.
    xs, ys = [], []
    for d in DELTAS:
        px, py = [], []
        for sp, s in series:
            g = [x for x in rows if float(x['delta']) == d and (x['spot_order'], x['dir']) == (sp, s)]
            if g:
                k = args.gyro_scale if s == 'ccw' or args.gyro_scale_cw is None else args.gyro_scale_cw
                px.append(statistics.fmean(abs(float(x['delta_fb'])) for x in g))
                py.append(k * statistics.fmean(abs(float(x['omega_imu'])) for x in g))
        if len(px) == len(series):  # only levels every series has
            xs.append(statistics.fmean(px)); ys.append(statistics.fmean(py))
    X, Y = np.array(xs), np.array(ys)
    if len(X) < 3:
        sys.exit('need >= 3 levels measured in every series')

    def solve(x, y, n):
        return np.linalg.lstsq(np.vstack([x ** k for k in range(1, n + 1)]).T, y, rcond=None)[0]

    def value(c, x):
        return sum(ck * x ** k for k, ck in enumerate(c, 1))

    k_cw = args.gyro_scale if args.gyro_scale_cw is None else args.gyro_scale_cw
    print(f'[map] seed {seed}: {len(series)} series ({", ".join(f"run {a} {b}" for a, b in series)}), '
          f'{len(X)} representative points'
          + (f', IMU x {args.gyro_scale:g} CCW / {k_cw:g} CW' if (args.gyro_scale, k_cw) != (1.0, 1.0) else ''))
    fits = {}
    for n in range(1, min(args.max_degree, len(X) - 2) + 1):
        c = solve(X, Y, n)
        loo = [Y[i] - value(solve(np.delete(X, i), np.delete(Y, i), n), X[i]) for i in range(len(X))]
        fits[n] = (c, float(np.sqrt(np.mean((Y - value(c, X)) ** 2))), float(np.sqrt(np.mean(np.square(loo)))))
        print(f'[map]   n={n}: rms {fits[n][1]:.4f}, leave-one-out rms {fits[n][2]:.4f} rad/s, '
              f'c = [{", ".join(f"{v:.6g}" for v in c)}]')
    n = args.degree or min(fits, key=lambda k: fits[k][2])
    if n not in fits:
        sys.exit(f'--degree must be 1..{max(fits)}')
    c = fits[n][0]
    print(f'[map] degree {n}{" (lowest leave-one-out)" if not args.degree else ""}; control.yaml:')
    print(f'    wheel_yaw_rate_coeffs: [{", ".join(f"{v:.6g}" for v in c)}]')

    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=130)
    fig.patch.set_facecolor('#fcfcfb'); ax.set_facecolor('#fcfcfb')
    xf = np.linspace(0, max(DELTAS), 200)
    ax.plot(xf, value(c, xf), color='#2a78d6', lw=2, zorder=2,
            label=f'degree {n}: ' + ' + '.join(f'{v:.4g}·D^{k}' if k > 1 else f'{v:.4g}·D' for k, v in enumerate(c, 1)))
    ax.scatter(X, Y, s=36, color='#0b0b0b', zorder=3, label=f'representative (mean of {len(series)} series)')
    ax.set_xlim(0, max(DELTAS) * 1.03); ax.set_ylim(0, None)
    ax.set_xlabel('measured wheel speed difference D_fb = wr − wl [rad/s]', color='#0b0b0b')
    ax.set_ylabel('chassis |yaw rate| from IMU [rad/s]', color='#0b0b0b')
    ax.set_title(f'Wheel odometry yaw rate map (seed {seed})', color='#0b0b0b', fontsize=11, loc='left')
    ax.grid(color='#e4e3df', lw=0.8); ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors='#52514e')
    ax.legend(frameon=False, fontsize=9, loc='upper left')
    out = LOGS / 'turn_map_poly.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'[map] plot {out.relative_to(HERE)}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('plan', help='make a new plan (run makes one if none exists)')
    p.add_argument('--seed', type=int, help='default: random (printed and stored in the plan)')
    p.add_argument('--reps', type=int, default=3,
                   help='passes per spot (every level, both directions); >= 2 for the spot-dependence verdict')
    p.add_argument('--force', action='store_true', help='replace an existing plan')
    p = sub.add_parser('run', help='run or resume the trials of one planned spot')
    p.add_argument('--spot', type=int, required=True, help=f'1..{SPOTS}: which run of the same experiment')
    p.add_argument('--note', default='', help='free text (floor, battery V, ...)')
    p.add_argument('--port', default=PORT)
    p = sub.add_parser('fit', help='summarize results and write the lookup table')
    p.add_argument('--seed', help='plan seed to fit (default: current plan)')
    p = sub.add_parser('plot', help='yaw rate vs D chart -> logs/turn_map_plot.png')
    p.add_argument('--seed', help='plan seed to plot (default: current plan)')
    p.add_argument('--wheelbase', type=float, help='front/rear axle distance L [m] (default: control.yaml)')
    p = sub.add_parser('poly', help='representative values + degree-n odometry yaw rate map')
    p.add_argument('--seed', help='plan seed (default: current plan)')
    p.add_argument('--degree', type=int, help='force n (default: lowest leave-one-out error)')
    p.add_argument('--max-degree', type=int, default=5)
    p.add_argument('--gyro-scale', type=float, default=1.0,
                   help='IMU rate multiplier (CCW, or both) from odom_compare.py --summary')
    p.add_argument('--gyro-scale-cw', type=float, help='IMU rate multiplier for CW (default: --gyro-scale)')
    args = ap.parse_args()
    return {'plan': plan, 'run': run, 'fit': fit, 'plot': plot, 'poly': poly}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
