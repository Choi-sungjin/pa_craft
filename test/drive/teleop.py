#!/usr/bin/env python3
"""Keyboard driving (WASD) for drive tests: steer the robot by hand on the wheel board.

Same board link as straight.py / rotate.py (wheel_clock firmware, 50 Hz, clock held). Keys set a target
(v, omega) in steps, like turtlebot3_teleop; the command ramps toward it at --accel / --alpha so a key press
never jumps the wheels. Wheels = (v -/+ omega * sep / 2) / r, scaled down together past max_wheel so the
turn is kept. Odometry and yaw (wheel and IMU, gz minus the offset measured standing still first) are
integrated from the start and shown live; every run is logged to logs/<stamp>_teleop.csv.
Stop control_master first; run through _craft_locked wheel (board lock).

  w/s   faster forward / backward (v step)      a/d   turn left / right (omega step)
  space stop (targets 0, ramps down)            x     zero omega only (drive straight)
  r     reset odometry and yaw                  q     quit (Ctrl+C too; both stop the wheels first)

  teleop.py [--max-speed 0.2] [--max-omega 1.0] [--v-step 0.02] [--w-step 0.1] [--accel 0.1] [--alpha 1.0]
"""
import argparse
import csv
import math
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'slip'))
from slip_test import HZ, PORT, params  # noqa: E402
from rotate import GZ, Board  # noqa: E402

LOGS = HERE / 'logs'


def toward(x, target, step):
    return min(target, x + step) if target > x else max(target, x - step)


def keys():
    """All keys typed since the last loop (non-blocking)."""
    out = ''
    while select.select([sys.stdin], [], [], 0)[0]:
        out += sys.stdin.read(1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--max-speed', type=float, default=0.2, help='|v| cap [m/s]')
    ap.add_argument('--max-omega', type=float, default=1.0, help='|omega| cap [rad/s]')
    ap.add_argument('--v-step', type=float, default=0.02, help='v change per w/s press [m/s]')
    ap.add_argument('--w-step', type=float, default=0.1, help='omega change per a/d press [rad/s]')
    ap.add_argument('--accel', type=float, default=0.1, help='v ramp [m/s^2]')
    ap.add_argument('--alpha', type=float, default=1.0, help='omega ramp [rad/s^2]')
    ap.add_argument('--port', default=PORT)
    args = ap.parse_args()

    r, sep, max_wheel = params()
    if not 0 < args.max_speed <= max_wheel * r:
        sys.exit(f'--max-speed must be in (0, {max_wheel * r:.4f}] m/s')
    if not sys.stdin.isatty():
        sys.exit('[tele] needs a terminal for the keys')
    board = Board(args.port)
    print('[tele] measuring the gyro offset, keep the robot still 2 s ...')
    gz, t0 = [], time.monotonic()
    while time.monotonic() - t0 < 2.0:
        if board.step(0.0, 0.0) is not None:
            gz.append(board.fb[GZ])
        time.sleep(1 / HZ)
    if board.fault() or not gz:
        sys.exit('[tele] board reports state 2 (fault) or no feedback: checkwheel / resetwheel first')
    imu = int(board.fb[5]) == 1
    if not imu:
        print('[tele] imu_ok=0: IMU yaw is not shown (resetwheel with the robot still to fix)')
    offset = sum(gz) / len(gz)
    print(__doc__.split('\n\n')[2])

    LOGS.mkdir(exist_ok=True)
    log_path = LOGS / f'{datetime.now():%Y%m%d-%H%M%S}_teleop.csv'
    rows, s, yaw_w, yaw_i = [], 0.0, 0.0, 0.0
    tv = tw = v = w = 0.0
    t_start = board.last = time.monotonic()
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            now = time.monotonic()
            quit_ = False
            for k in keys().lower():
                if k == 'w':
                    tv += args.v_step
                elif k == 's':
                    tv -= args.v_step
                elif k == 'a':
                    tw += args.w_step
                elif k == 'd':
                    tw -= args.w_step
                elif k == ' ':
                    tv = tw = 0.0
                elif k == 'x':
                    tw = 0.0
                elif k == 'r':
                    s = yaw_w = yaw_i = 0.0
                elif k == 'q':
                    quit_ = True
            if quit_:
                break
            tv = round(max(-args.max_speed, min(args.max_speed, tv)), 4)
            tw = round(max(-args.max_omega, min(args.max_omega, tw)), 4)
            v = toward(v, tv, args.accel / HZ)
            w = toward(w, tw, args.alpha / HZ)
            wl, wr = (v - w * sep / 2) / r, (v + w * sep / 2) / r
            over = max(abs(wl), abs(wr)) / max_wheel
            if over > 1:
                wl, wr = wl / over, wr / over
            dt = board.step(wl, wr)
            fb = board.fb
            if dt is not None:
                s += r * (fb[0] + fb[1]) / 2 * dt
                yaw_w += r * (fb[1] - fb[0]) / sep * dt
                yaw_i += (fb[GZ] - offset) * dt
                if board.fault():
                    print('\n[tele] board fault'); break
            elif now - board.last > 0.5:
                print('\n[tele] feedback lost'); break
            t = now - t_start
            rows.append([round(t, 4), tv, tw, round(v, 4), round(w, 4), round(wl, 4), round(wr, 4), fb[0], fb[1],
                         round(s, 5), round(math.degrees(yaw_w), 3), round(math.degrees(yaw_i), 3), fb[GZ]])
            imu_txt = f'{math.degrees(yaw_i):+7.1f}' if imu else '    n/a'
            sys.stdout.write(f'\r[tele] target v {tv:+.2f} w {tw:+.2f} | cmd v {v:+.3f} w {w:+.2f} | '
                             f'fb L {fb[0]:+5.2f} R {fb[1]:+5.2f} | odom {s:+6.3f} m yaw wheel '
                             f'{math.degrees(yaw_w):+7.1f} IMU {imu_txt} deg ')
            sys.stdout.flush()
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        board.stop(0.5)
        board.port.close()
    print()
    with open(log_path, 'w', newline='') as f:
        wtr = csv.writer(f)
        wtr.writerow(['t', 'target_v', 'target_omega', 'cmd_v', 'cmd_omega', 'cmd_wl', 'cmd_wr', 'wheel_l',
                      'wheel_r', 'odom', 'yaw_wheel_deg', 'yaw_imu_deg', 'gz'])
        wtr.writerows(rows)
    print(f'[tele] odometry {s:+.3f} m, yaw wheel {math.degrees(yaw_w):+.1f} deg / IMU '
          f'{math.degrees(yaw_i):+.1f} deg. log {log_path.relative_to(HERE.parents[1])}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
