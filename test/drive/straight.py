#!/usr/bin/env python3
"""Straight drive with heading hold: drive DISTANCE on wheel odometry while steering yaw back to 0.

Same board link and trapezoid as test/slip/slip_test.py (wheel_clock firmware, 50 Hz, clock held); on 4WD
firmware the rear wheels follow their side and the feedback is the side mean (see rotate.py). Yaw is
integrated from the start of the run: IMU (gz minus the offset measured standing still first; the board is
upright) or wheel odometry. Steering is P on yaw: omega = -k * yaw, clamped; wheels = (v -/+ omega * sep / 2) / r.
Keep --accel below the firmware wheel ramp (0.124 m/s^2) so the host trapezoid, not the firmware, shapes
the stop. Tracking is the side's mean feedback / mean command at cruise. After the run the tape distance and
sideways offset can be typed in; every run is appended to logs/straight_results.csv.
Stop control_master first; run through _craft_locked wheel (board lock). Scenario: README.md.

  straight.py [--target 2.0] [--speed 0.1] [--accel 0.1] [--yaw imu|wheel] [--k 2.0] [--note text]
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'slip'))
from slip_test import CREEP, HZ, PORT, params  # noqa: E402
from rotate import GZ, Board  # noqa: E402

LOGS = HERE / 'logs'
RESULTS = LOGS / 'straight_results.csv'


def number(prompt):
    while True:
        a = input(prompt).strip()
        if not a:
            return None
        try:
            return float(a)
        except ValueError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--target', type=float, default=2.0, help='odometry distance [m]')
    ap.add_argument('--speed', type=float, default=0.1, help='cruise speed [m/s]')
    ap.add_argument('--accel', type=float, default=0.1, help='trapezoid slope [m/s^2]')
    ap.add_argument('--yaw', choices=['imu', 'wheel'], default='imu', help='heading source')
    ap.add_argument('--k', type=float, default=2.0, help='yaw P gain [1/s]')
    ap.add_argument('--max-omega', type=float, default=0.5, help='steering clamp [rad/s]')
    ap.add_argument('--note', default='', help='free text for straight_results.csv (4wd/2wd, floor, battery V)')
    ap.add_argument('--port', default=PORT)
    args = ap.parse_args()

    r, sep, max_wheel = params()
    if not 0 < args.speed <= max_wheel * r or not args.accel > 0 or not args.target > 0:
        sys.exit(f'speed must be in (0, {max_wheel * r:.4f}] m/s, accel and target > 0')
    print(f'[drive] {args.target} m at {args.speed} m/s, accel {args.accel} m/s^2, yaw hold on {args.yaw} '
          f'(k {args.k}, |omega| <= {args.max_omega} rad/s)')
    board = Board(args.port)
    if int(board.fb[5]) != 1:
        sys.exit('[drive] imu_ok=0: resetwheel with the robot still')
    input(f'[drive] clear {args.target + 0.5:g} m ahead, Enter to drive (Ctrl+C stops) ')
    print('[drive] measuring the gyro offset, keep the robot still 2 s ...')
    gz, t0 = [], time.monotonic()
    while time.monotonic() - t0 < 2.0:
        if board.step(0.0, 0.0) is not None:  # also re-primes the 250 ms command watchdog after the prompt
            gz.append(board.fb[GZ])
        time.sleep(1 / HZ)
    if board.fault() or not gz:
        sys.exit('[drive] board reports state 2 (fault) or no feedback: checkwheel / resetwheel first')
    offset = sum(gz) / len(gz)

    LOGS.mkdir(exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    log_path = LOGS / f'{stamp}_straight_{args.yaw}.csv'
    rows, s, yaw_w, yaw_i = [], 0.0, 0.0, 0.0
    cruise = [0.0, 0.0, 0.0, 0.0]  # sum of fb_l, fb_r, cmd_l, cmd_r while at cruise speed
    t_start = time.monotonic()
    board.last = t_start
    timeout = args.target / args.speed + args.speed / args.accel + 10
    still_since, aborted = None, False
    try:
        wl = wr = 0.0
        while True:
            now = time.monotonic()
            t = now - t_start
            fb = board.fb
            yaw = yaw_i if args.yaw == 'imu' else yaw_w
            remaining = args.target - s
            if remaining > 0:
                v = min(args.speed, args.accel * t, max(CREEP, math.sqrt(2 * args.accel * remaining)))
                omega = max(-args.max_omega, min(args.max_omega, -args.k * yaw))
            else:
                v = omega = 0.0  # no pivoting in place at the end
                still = abs(fb[0]) < 0.02 and abs(fb[1]) < 0.02
                still_since = (still_since or now) if still else None
                if still_since and now - still_since > 0.5:
                    break
            wl, wr = (v - omega * sep / 2) / r, (v + omega * sep / 2) / r
            over = max(abs(wl), abs(wr)) / max_wheel
            if over > 1:  # keep the steering, give up speed
                wl, wr = (v / over - omega * sep / 2) / r, (v / over + omega * sep / 2) / r
                wl, wr = max(-max_wheel, min(max_wheel, wl)), max(-max_wheel, min(max_wheel, wr))
            dt = board.step(wl, wr)
            fb = board.fb
            if dt is not None:
                s += r * (fb[0] + fb[1]) / 2 * dt
                yaw_w += r * (fb[1] - fb[0]) / sep * dt
                yaw_i += (fb[GZ] - offset) * dt
                if board.fault():
                    print('[drive] board fault during the run'); aborted = True; break
                if v == args.speed:
                    for k, x in enumerate((fb[0], fb[1], wl, wr)):
                        cruise[k] += x
            elif now - board.last > 0.5:
                print('[drive] feedback lost'); aborted = True; break
            rows.append([round(t, 4), round(v, 4), round(omega, 4), round(wl, 4), round(wr, 4), fb[0], fb[1],
                         round(s, 5), round(math.degrees(yaw_w), 3), round(math.degrees(yaw_i), 3), fb[GZ]])
            if t > timeout:
                print(f'[drive] {timeout:.0f} s timeout'); aborted = True; break
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print('\n[drive] Ctrl+C'); aborted = True
    finally:
        board.stop(0.5)
        board.port.close()
    with open(log_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'cmd_v', 'cmd_omega', 'cmd_wl', 'cmd_wr', 'wheel_l', 'wheel_r', 'odom',
                    'yaw_wheel_deg', 'yaw_imu_deg', 'gz'])
        w.writerows(rows)
    tl, tr = (cruise[k] / cruise[k + 2] if cruise[k + 2] else float('nan') for k in (0, 1))
    print(f'[drive] odometry {s * 1000:.1f} mm, {rows[-1][0] if rows else 0:.1f} s, '
          f'yaw wheel {math.degrees(yaw_w):+.2f} deg / IMU {math.degrees(yaw_i):+.2f} deg, '
          f'tracking L {tl:.2f} R {tr:.2f}{" (aborted)" if aborted else ""}. '
          f'log {log_path.relative_to(HERE.parents[1])}')
    if not rows:
        return 1
    real = number('[drive] tape: start mark -> end mark along the line, mm (Enter = skip): ')
    lateral = number('[drive] sideways offset at the end, mm (+ left, Enter = skip): ') if real is not None else None
    slip = (s * 1000 - real) / s if real is not None and s > 0 else None  # mm lost per m of odometry
    if slip is not None:
        print(f'[drive] slip {slip:+.1f} mm/m (+ = wheels turned further than the robot moved)')
    new = not RESULTS.exists()
    with open(RESULTS, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['stamp', 'target', 'speed', 'accel', 'yaw', 'k', 'odom_mm', 'real_mm', 'slip_mm_per_m',
                        'lateral_mm', 'yaw_imu_deg', 'yaw_wheel_deg', 'track_l', 'track_r', 'aborted', 'note'])
        w.writerow([stamp, args.target, args.speed, args.accel, args.yaw, args.k, round(s * 1000, 1),
                    '' if real is None else real, '' if slip is None else round(slip, 2),
                    '' if lateral is None else lateral, round(math.degrees(yaw_i), 2),
                    round(math.degrees(yaw_w), 2), round(tl, 3), round(tr, 3), int(aborted), args.note])
    return 1 if aborted else 0


if __name__ == '__main__':
    sys.exit(main())
