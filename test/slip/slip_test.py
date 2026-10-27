#!/usr/bin/env python3
"""Wheel slip test: drive straight until wheel odometry reads DISTANCE, then compare with the tape.

Talks to the wheel/clock OpenCR directly, in control_master's format: 'wheel_l,wheel_r,clock\\n'
(%.4f, wheel output shaft rad/s, clock held at its start angle) at 50 Hz; odometry integrates the
newest wheel feedback over host time like WheelOdometry. The trapezoid ramps at --accel up to the
cruise speed and decelerates on the remaining odometry distance (v <= sqrt(2 a d)), so only the slope
changes between runs. Stop control_master first; run through the 'slip' shell helper (board lock).

  slip_test.py run --speed high|low|<m/s> --accel <m/s^2>   one run, then asks for the tape distance
  slip_test.py plan                                         run order (baseline first, rest shuffled)
  slip_test.py summary                                      slip per speed/accel vs the baseline
"""
import argparse
import csv
import math
import random
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
import yaml

HERE = Path(__file__).resolve().parent
CONTROL_YAML = HERE.parents[1] / 'src/craft_control/config/control.yaml'
RESULTS = HERE / 'results.csv'
LOGS = HERE / 'logs'
PORT = '/dev/opencr_wheel_clock'
HZ = 50
LOW_SPEED = 0.05             # [m/s] parking correction cruise speed
ACCELS = [0.05, 0.1, 0.2, 0.4, 0.8]  # [m/s^2] trapezoid slopes; 0.1 ~ the old firmware profile (0.124)
BASELINE = ('low', 0.05)     # least slip: its tape/odom ratio calibrates the effective wheel radius
REPEATS, BASELINE_REPEATS = 3, 5
CREEP = 0.01                 # [m/s] floor near the end so the last millimetres do not stall
FIELDS = ['time', 'speed_mode', 'v_cruise', 'accel', 'target', 'odom', 'real', 'lateral',
          'yaw_wheel_deg', 'yaw_imu_deg', 'peak_v', 'duration', 'log']


def params():
    p = yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']
    return p['wheel_radius'], p['wheel_separation'], p['max_wheel']


def newest(port, buf):
    """Newest complete 7-field feedback line: wheel_l, wheel_r, clock, wheel_state, clock_state, imu_ok, gz."""
    buf += port.read(port.in_waiting or 1)
    *lines, rest = buf.split(b'\n')
    for line in reversed(lines):
        try:
            v = [float(x) for x in line.decode(errors='replace').split(',')]
        except ValueError:
            continue
        if len(v) == 7 and all(math.isfinite(x) for x in v):
            return v, rest
    return None, rest


def run(args):
    r, sep, max_wheel = params()
    v_max = max_wheel * r if args.speed == 'high' else LOW_SPEED if args.speed == 'low' else float(args.speed)
    mode = args.speed if args.speed in ('high', 'low') else 'custom'
    if not 0 < v_max <= max_wheel * r + 1e-9 or not args.accel > 0:
        sys.exit(f'speed must be in (0, {max_wheel * r:.4f}] m/s and accel > 0')
    print(f'[slip] {mode} {v_max:.4f} m/s, accel {args.accel} m/s^2, odometry target {args.target} m '
          f'(r {r} m, separation {sep} m)')
    port = serial.Serial(args.port, 115200, timeout=0.005)
    try:
        port.dtr = True  # OpenCR USB CDC sends only with DTR set
    except OSError:
        pass  # a pty (offline test) has no modem lines
    buf, fb, t0 = b'', None, time.monotonic()
    while fb is None and time.monotonic() - t0 < 1.0:
        fb, buf = newest(port, buf)
    if fb is None:
        sys.exit(f'[slip] no feedback from {args.port} (control_master running? wrong firmware?)')
    clock = fb[2]
    send = lambda v: port.write(f'{v / r:.4f},{v / r:.4f},{clock:.4f}\n'.encode())
    if not args.yes:
        input('[slip] clear 1.5 m ahead, Enter to drive (Ctrl+C stops) ')
    # Prime the 250 ms command watchdog with a stop only now: waiting at the prompt above lets it expire
    # (state 2 until commands resume), which the first feedback of the run would read as a fault.
    for _ in range(HZ // 2):
        send(0.0); time.sleep(1 / HZ)
        fresh, buf = newest(port, buf)
        fb = fresh or fb
    if fb is None or 2 in (int(fb[3]), int(fb[4])):
        sys.exit('[slip] board reports state 2 (fault): checkwheel / resetwheel first')

    LOGS.mkdir(exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    log_path = LOGS / f'{stamp}_{mode}_{args.accel}.csv'
    rows, s, yaw_w, yaw_i, peak = [], 0.0, 0.0, 0.0, 0.0
    t_start = last = time.monotonic()
    still_since, cmd, aborted = None, 0.0, False
    try:
        while True:
            now = time.monotonic()
            fresh, buf = newest(port, buf)
            if fresh is not None:
                fb = fresh
                dt = now - last
                last = now
                v_meas = r * (fb[0] + fb[1]) / 2
                s += v_meas * dt
                yaw_w += r * (fb[1] - fb[0]) / sep * dt
                yaw_i += fb[6] * dt
                peak = max(peak, v_meas)
                if 2 in (int(fb[3]), int(fb[4])):
                    print('[slip] board fault during the run'); aborted = True; break
            elif now - last > 0.5:
                print('[slip] feedback lost'); aborted = True; break
            t = now - t_start
            remaining = args.target - s
            if remaining > 0:
                cmd = min(v_max, args.accel * t, max(CREEP, math.sqrt(2 * args.accel * remaining)))
            else:
                cmd = 0.0
                still = abs(fb[0]) < 0.02 and abs(fb[1]) < 0.02
                still_since = (still_since or now) if still else None
                if still_since and now - still_since > 0.5:
                    break
            send(cmd)
            rows.append([round(t, 4), round(cmd, 4), fb[0], fb[1], round(s, 5), round(math.degrees(yaw_w), 3),
                         round(math.degrees(yaw_i), 3), fb[6]])
            if t > 30:
                print('[slip] 30 s timeout'); aborted = True; break
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print('\n[slip] Ctrl+C'); aborted = True
    finally:
        for _ in range(HZ // 2):
            send(0.0); time.sleep(1 / HZ)
        port.close()
    with open(log_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'cmd_v', 'wheel_l', 'wheel_r', 'odom', 'yaw_wheel_deg', 'yaw_imu_deg', 'gz'])
        w.writerows(rows)
    duration = rows[-1][0] if rows else 0.0
    print(f'[slip] odometry {s * 1000:.1f} mm, peak {peak:.3f} m/s, {duration:.1f} s, '
          f'yaw wheel {math.degrees(yaw_w):+.2f} deg / IMU {math.degrees(yaw_i):+.2f} deg. log {log_path.name}')
    if aborted:
        print('[slip] aborted run: not added to results'); return 1
    real = input('[slip] tape: start line -> reference mark, mm (Enter = discard run): ').strip()
    if not real:
        print('[slip] discarded'); return 0
    lateral = input('[slip] sideways offset, mm (+ left, Enter = skip): ').strip()
    new = not RESULTS.exists()
    with open(RESULTS, 'a', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        if new:
            w.writeheader()
        w.writerow(dict(time=stamp, speed_mode=mode, v_cruise=round(v_max, 4), accel=args.accel,
                        target=args.target, odom=round(s, 5), real=float(real) / 1000,
                        lateral=float(lateral) / 1000 if lateral else '', yaw_wheel_deg=round(math.degrees(yaw_w), 3),
                        yaw_imu_deg=round(math.degrees(yaw_i), 3), peak_v=round(peak, 4),
                        duration=duration, log=log_path.name))
    print(f'[slip] tape/odometry {float(real) / 1000 / s:.4f}. Saved to {RESULTS.name}')
    return 0


def plan(args):
    rng = random.Random(args.seed)
    rest = [(m, a) for m in ('high', 'low') for a in ACCELS if (m, a) != BASELINE] * REPEATS
    rng.shuffle(rest)
    order = [BASELINE] * BASELINE_REPEATS + rest
    for i, (m, a) in enumerate(order, 1):
        print(f'{i:2d}. slip run --speed {m} --accel {a}')
    return 0


def summary(args):
    if not RESULTS.exists():
        sys.exit('no results yet')
    groups = {}
    for row in csv.DictReader(open(RESULTS)):
        ratio = float(row['real']) / float(row['odom'])
        groups.setdefault((row['speed_mode'], float(row['accel'])), []).append((ratio, row))
    base = groups.get(BASELINE)
    if not base:
        sys.exit(f'no baseline runs yet ({BASELINE[0]} speed, accel {BASELINE[1]})')
    k = sum(x for x, _ in base) / len(base)
    r, _, _ = params()
    print(f'baseline {BASELINE}: tape/odometry {k:.4f} over {len(base)} runs -> effective wheel radius '
          f'{r * k * 1000:.2f} mm (nominal {r * 1000:.1f})')
    print('slip = distance lost per metre against the baseline (+ = wheels turned more than the robot moved)')
    print(f'{"speed":6s} {"accel":>6s} {"n":>2s} {"slip mm/m":>10s} {"std":>6s} {"yaw IMU deg":>12s} {"lateral mm":>11s}')
    for (m, a) in sorted(groups):
        xs = [1000 * (1 - x / k) for x, _ in groups[(m, a)]]
        mean = sum(xs) / len(xs)
        std = math.sqrt(sum((x - mean) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else float('nan')
        yaw = sum(float(rw['yaw_imu_deg']) for _, rw in groups[(m, a)]) / len(xs)
        lats = [float(rw['lateral']) * 1000 for _, rw in groups[(m, a)] if rw['lateral']]
        lat = f'{sum(lats) / len(lats):+.1f}' if lats else '-'
        print(f'{m:6s} {a:6.2f} {len(xs):2d} {mean:+10.1f} {std:6.1f} {yaw:+12.2f} {lat:>11s}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('run')
    p.add_argument('--speed', default='high', help="high (max_wheel x radius), low, or m/s")
    p.add_argument('--accel', type=float, required=True, help='trapezoid slope [m/s^2]')
    p.add_argument('--target', type=float, default=1.0, help='odometry distance [m]')
    p.add_argument('--port', default=PORT)
    p.add_argument('--yes', action='store_true', help='no Enter before driving')
    p = sub.add_parser('plan')
    p.add_argument('--seed', type=int, default=1)
    sub.add_parser('summary')
    args = ap.parse_args()
    return {'run': run, 'plan': plan, 'summary': summary}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
