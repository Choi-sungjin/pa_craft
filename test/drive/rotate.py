#!/usr/bin/env python3
"""In-place turns on 4WD: do all four wheels drive, and does the robot turn as far as the wheels say?

Same board link as test/slip/slip_test.py (wheel_clock firmware, 50 Hz, clock held). The firmware drives
the rear wheels (ID 3, 4) as followers of wheel_l / wheel_r (ID 1, 2), and each side's feedback is the
mean of its front and rear motor. Scenario: README.md next to this file.

--lifted (wheels off the ground): runs each side forward/backward and one in-place turn, 2 s each, checks
  that the side's mean feedback tracks the command, and asks whether both wheels of that side turned the
  right way (the feedback cannot see a follower with the wrong direction sign).
default (on the floor): each turn commands +-omega in place (wheels -/+ omega * sep / 2 / r; the firmware
  profile acceleration ramps it) until wheel odometry yaw reaches ANGLE, then stops. The IMU (gz minus the
  offset measured standing still first) gives the real turn. wheel/IMU > 1 means the wheels turned further
  than the chassis (skid steer scrub); sep x wheel/IMU is the effective wheel separation. Tracking is the
  side's mean feedback / command after the ramp: well below 1 means the motors cannot keep up (stall).
Stop control_master first; run through _craft_locked wheel (board lock).

  rotate.py [--angle 90] [--turns 4] [--omega 0.6] [--ccw] [--note text]
  rotate.py --lifted [--omega 0.6]
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'slip'))
from slip_test import HZ, PORT, newest, params  # noqa: E402

LOGS = HERE / 'logs'
RESULTS = LOGS / 'rotate_results.csv'
GZ = 8  # feedback index: wheel_l, wheel_r, clock, wheel_state, clock_state, imu_ok, gx, gy, gz, ...
# Wheel profile acceleration in firmware/wheel_clock/config.h: 10 x 214.577 rev/min^2 [rad/s^2].
WHEEL_ACCEL = 10 * 214.577 * 2 * math.pi / 3600
TRACK_OK = (0.9, 1.1)  # mean feedback / command after the ramp
STILL = 0.05  # [rad/s] uncommanded side while lifted


class Board:
    """wheel_clock link: send wheel_l, wheel_r (clock held) and keep the newest feedback line."""

    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 115200, timeout=0.005)
        try:
            self.port.dtr = True  # OpenCR USB CDC sends only with DTR set
        except OSError:
            pass
        self.buf, self.fb, t0 = b'', None, time.monotonic()
        while self.fb is None and time.monotonic() - t0 < 1.0:
            self.fb, self.buf = newest(self.port, self.buf)
        if self.fb is None:
            sys.exit(f'[rot] no feedback from {port_name} (control_master running? resetwheel)')
        self.clock = self.fb[2]
        self.last = time.monotonic()

    def step(self, wl, wr):
        """Send one command, read; returns dt since the previous fresh line, or None if none arrived."""
        self.port.write(f'{wl:.4f},{wr:.4f},{self.clock:.4f}\n'.encode())
        fresh, self.buf = newest(self.port, self.buf)
        if fresh is None:
            return None
        now = time.monotonic()
        self.fb, dt, self.last = fresh, now - self.last, now
        return dt

    def fault(self):
        return 2 in (int(self.fb[3]), int(self.fb[4]))

    def stop(self, sec):
        """Command zero for sec (also re-primes the 250 ms command watchdog after a prompt)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            self.step(0.0, 0.0)
            time.sleep(1 / HZ)


def ask(prompt):
    while True:
        a = input(prompt).strip().lower()
        if a in ('y', 'n'):
            return a == 'y'


def lifted(board, wheel):
    """Each side alone, then one in-place turn; feedback tracking plus what the operator saw."""
    steps = [('left forward', wheel, 0.0), ('left backward', -wheel, 0.0),
             ('right forward', 0.0, wheel), ('right backward', 0.0, -wheel),
             ('turn left', -wheel, wheel)]
    seen = {'left forward': 'both LEFT wheels (ID 1 front, ID 3 rear) roll FORWARD',
            'left backward': 'both LEFT wheels roll BACKWARD',
            'right forward': 'both RIGHT wheels (ID 2 front, ID 4 rear) roll FORWARD',
            'right backward': 'both RIGHT wheels roll BACKWARD',
            'turn left': 'left pair BACKWARD and right pair FORWARD (robot would turn CCW from above)'}
    rows, ok_all = [], True
    for name, cl, cr in steps:
        input(f'[rot] {name} ({cl:+.2f}, {cr:+.2f} rad/s, 2 s): watch all 4 wheels, Enter ')
        board.stop(0.5)
        if board.fault():
            sys.exit('[rot] board reports state 2 (fault): checkwheel / resetwheel first')
        fl, fr, t0 = [], [], time.monotonic()
        while (t := time.monotonic() - t0) < 2.0:
            if board.step(cl, cr) is not None and t > 1.0:
                fl.append(board.fb[0]); fr.append(board.fb[1])
            time.sleep(1 / HZ)
        board.stop(1.0)
        if board.fault() or not fl:
            sys.exit('[rot] board fault or no feedback during the step: checkwheel')
        ml, mr = sum(fl) / len(fl), sum(fr) / len(fr)
        fb_ok = all((TRACK_OK[0] <= m / c <= TRACK_OK[1]) if c else abs(m) < STILL
                    for c, m in ((cl, ml), (cr, mr)))
        print(f'[rot]   feedback mean L {ml:+.3f}, R {mr:+.3f} rad/s -> {"ok" if fb_ok else "CHECK"}')
        eye_ok = ask(f'[rot]   did {seen[name]}? [y/n] ')
        ok_all &= fb_ok and eye_ok
        rows.append([name, cl, cr, round(ml, 4), round(mr, 4), int(fb_ok), int(eye_ok)])
    return rows, ok_all


def turns(board, args, r, sep, wheel, offset):
    sign = 1 if args.ccw else -1
    target = math.radians(args.angle)
    ramp = wheel / WHEEL_ACCEL + 0.3  # tracking counts only after the profile ramp
    rows, results, aborted = [], [], False
    t_start = time.monotonic()
    for turn in range(1, args.turns + 1):
        yaw_w = yaw_i = 0.0
        t_turn = time.monotonic()
        turning, still_since, track = True, None, ([], [])
        while True:
            now = time.monotonic()
            cmd = sign * wheel if turning else 0.0
            dt = board.step(-cmd, cmd)
            fb = board.fb
            if dt is not None:
                yaw_w += r * (fb[1] - fb[0]) / sep * dt
                yaw_i += (fb[GZ] - offset) * dt
                if board.fault():
                    print('[rot] board fault during the turn'); aborted = True; break
                if turning and now - t_turn > ramp:
                    track[0].append(fb[0] / -cmd); track[1].append(fb[1] / cmd)
            elif now - board.last > 0.5:
                print('[rot] feedback lost'); aborted = True; break
            if turning and abs(yaw_w) >= target:
                turning = False
            if not turning:
                still = abs(fb[0]) < 0.02 and abs(fb[1]) < 0.02
                still_since = (still_since or now) if still else None
                if still_since and now - still_since > 1.0:  # IMU keeps integrating the coast-down
                    break
            rows.append([turn, round(now - t_start, 4), round(-cmd, 4), round(cmd, 4), fb[0], fb[1],
                         round(math.degrees(yaw_w), 3), round(math.degrees(yaw_i), 3), fb[GZ]])
            if now - t_turn > target / args.omega * 3 + 15:  # a stalled turn never reaches ANGLE
                print('[rot] timeout (wheels not reaching the angle: stall?)'); aborted = True; break
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
        if aborted:
            break
        w, i = math.degrees(yaw_w), math.degrees(yaw_i)
        tl, tr = (sum(x) / len(x) if x else float('nan') for x in track)
        results.append((turn, w, i, tl, tr, time.monotonic() - t_turn))
        ratio = w / i if abs(i) > 1 else float('inf')
        print(f'[rot] turn {turn}: wheel {w:+.1f} deg, IMU {i:+.1f} deg, wheel/IMU {ratio:.2f}, '
              f'tracking L {tl:.2f} R {tr:.2f}')
    return rows, results, aborted


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--angle', type=float, default=90.0, help='wheel-odometry turn per step [deg]')
    ap.add_argument('--turns', type=int, default=4)
    ap.add_argument('--omega', type=float, default=0.6, help='turn rate [rad/s] (control wheel_rotate_angular_speed)')
    ap.add_argument('--ccw', action='store_true', help='turn left (default: right, clockwise)')
    ap.add_argument('--lifted', action='store_true', help='wheels off the ground: per-side direction check')
    ap.add_argument('--note', default='', help='free text for rotate_results.csv (floor, battery V, ...)')
    ap.add_argument('--port', default=PORT)
    args = ap.parse_args()

    r, sep, max_wheel = params()
    wheel = args.omega * sep / 2 / r
    if not 0 < wheel <= max_wheel or not args.angle > 0 or args.turns < 1:
        sys.exit(f'omega must be in (0, {max_wheel * 2 * r / sep:.2f}] rad/s, angle > 0, turns >= 1')
    board = Board(args.port)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    LOGS.mkdir(exist_ok=True)

    if args.lifted:
        print(f'[rot] LIFTED check at {wheel:.2f} rad/s per wheel. All 4 wheels must be off the ground.')
        try:
            rows, ok = lifted(board, wheel)
        except KeyboardInterrupt:
            print('\n[rot] Ctrl+C'); rows, ok = [], False
        finally:
            board.stop(0.5)
            board.port.close()
        log_path = LOGS / f'{stamp}_lifted_{args.omega}.csv'
        with open(log_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['step', 'cmd_wl', 'cmd_wr', 'fb_l', 'fb_r', 'feedback_ok', 'seen_ok'])
            w.writerows(rows)
        print(f'[rot] lifted check {"PASSED" if ok else "FAILED"}; log {log_path.relative_to(HERE.parents[1])}')
        return 0 if ok else 1

    if int(board.fb[5]) != 1:
        sys.exit('[rot] imu_ok=0: resetwheel with the robot still')
    print('[rot] measuring the gyro offset, keep the robot still 2 s ...')
    gz, t0 = [], time.monotonic()
    while time.monotonic() - t0 < 2.0:
        if board.step(0.0, 0.0) is not None:  # also primes the 250 ms command watchdog
            gz.append(board.fb[GZ])
        time.sleep(1 / HZ)
    if board.fault():
        sys.exit('[rot] board reports state 2 (fault): checkwheel / resetwheel first')
    offset = sum(gz) / len(gz)
    direction = 'ccw' if args.ccw else 'cw'
    print(f'[rot] gz offset {offset:+.5f} rad/s ({len(gz)} samples). {args.turns} x {args.angle:g} deg '
          f'{"left" if args.ccw else "right"} at {args.omega} rad/s (wheels {wheel:.2f} rad/s)')

    try:
        rows, results, aborted = turns(board, args, r, sep, wheel, offset)
    except KeyboardInterrupt:
        print('\n[rot] Ctrl+C'); rows, results, aborted = [], [], True
    finally:
        board.stop(0.5)
        board.port.close()
    log_path = LOGS / f'{stamp}_rotate_{args.omega}.csv'
    with open(log_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['turn', 't', 'cmd_wl', 'cmd_wr', 'wheel_l', 'wheel_r', 'yaw_wheel_deg', 'yaw_imu_deg', 'gz'])
        w.writerows(rows)
    if results:
        new = not RESULTS.exists()
        with open(RESULTS, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['stamp', 'omega', 'dir', 'angle', 'turn', 'wheel_deg', 'imu_deg', 'ratio',
                            'sep_eff', 'track_l', 'track_r', 'time_s', 'aborted', 'note'])
            for turn, wd, idg, tl, tr, dur in results:
                ratio = wd / idg if abs(idg) > 1 else float('inf')
                w.writerow([stamp, args.omega, direction, args.angle, turn, round(wd, 2), round(idg, 2),
                            round(ratio, 3), round(sep * ratio, 4), round(tl, 3), round(tr, 3), round(dur, 2),
                            int(aborted), args.note])
        sw, si = sum(x[1] for x in results), sum(x[2] for x in results)
        ratio = sw / si if abs(si) > 1 else float('inf')
        print(f'[rot] total: wheel {sw:+.1f} deg, IMU {si:+.1f} deg, wheel/IMU {ratio:.2f} '
              f'(effective separation {sep * ratio:.3f} m vs {sep} m){" (aborted)" if aborted else ""}')
    print(f'[rot] log {log_path.relative_to(HERE.parents[1])}')
    return 1 if aborted else 0


if __name__ == '__main__':
    sys.exit(main())
