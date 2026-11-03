#!/usr/bin/env python3
"""Gyro scale + in-place wheel odometry check: N full turns from a floor tape line back onto it.

Drives through control_master (/control/teleop/cmd_vel, so its wheel odometry with wheel_yaw_rate_coeffs
is what gets checked) and reads /control/wheel/odom and both raw board IMUs, /control/imu/{wheel_clock,lm_arm}
(each scaled on its own: gz minus its offset, both boards upright). The turn stops on wheel_clock. Per run:
  1. 3 s still: gyro offset.                      2. turn until the IMU says 360 N - MARGIN, then stop.
  3. you finish the turn onto the tape line with a/d (slow nudges), then c.   4. 3 s still: offset again.
The true turn is then exactly 360 N, so gyro scale = true / IMU and wheel odometry / true need no angle
measurement. One row per board and run in test/gyro_scale/results.csv (odom_compare.py --summary).
Runs alternate CCW / CW by default; after each run the robot already sits on the line for the next one.

  gyro_spin.py [--turns 5] [--runs 3] [--dir both|ccw|cw] [--omega 0.4] [--note text]
Needs control_master up (bro) and nothing else sending teleop.
"""
import argparse
import csv
import math
import select
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from odom_compare import SCALE_CSV, SCALE_FIELDS  # noqa: E402

LOGS = HERE / 'logs'
BOARDS = ('wheel_clock', 'lm_arm')
LEAD = 'wheel_clock'  # stop criterion
HZ = 20
STILL_S = 3.0
MARGIN = 45.0      # [deg] stop short of the line: covers a 3 % IMU scale error at 1800 deg plus coasting
ALPHA = 1.0        # [rad/s^2] command ramp, as teleop_ros.py
NUDGE = 0.12       # [rad/s] command while a/d is held during the finish
NUDGE_S = 0.25     # [s] one key press keeps nudging this long (key repeat fills the gaps)
STILL_W = 1e-3     # [rad/s] wheel odometry yaw rate counted as stopped


def keys():
    out = ''
    while select.select([sys.stdin], [], [], 0)[0]:
        out += sys.stdin.read(1)
    return out


def toward(x, target, step):
    return min(target, x + step) if target > x else max(target, x - step)


class Spin:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/control/teleop/cmd_vel', 10)
        node.create_subscription(Odometry, '/control/wheel/odom', self.on_odom, qos_profile_sensor_data)
        self.imu = {b: [] for b in BOARDS}  # board -> [(t, gz)]
        for b in BOARDS:
            node.create_subscription(Imu, f'/control/imu/{b}', lambda m, b=b: self.on_imu(b, m),
                                     qos_profile_sensor_data)
        self.odom = []  # (t, v, w, unwrapped yaw)
        self.yaw_w, self.last_q = 0.0, None

    def on_odom(self, m):
        q = 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
        if self.last_q is not None:
            self.yaw_w += math.remainder(q - self.last_q, 2 * math.pi)
        self.last_q = q
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z, self.yaw_w))

    def on_imu(self, board, m):
        self.imu[board].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z))

    def tick(self, w, until):
        """Send omega w at HZ (control drops teleop after 0.3 s without input) until until() is true.
        Messages arrive on the executor thread (150/s: one spin_once per tick would drop most of them)."""
        while rclpy.ok():
            now = time.monotonic()
            msg = Twist(); msg.angular.z = w if not callable(w) else w()
            self.pub.publish(msg)
            if until():
                return
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))

    def stopped(self):
        return bool(self.odom) and abs(self.odom[-1][2]) < STILL_W and abs(self.odom[-1][1]) < 1e-4

    def imu_yaw(self, board, t0, offset, t1=math.inf):
        """Integrated IMU yaw [deg] over [t0, t1] (trapezoid on the message stamps)."""
        s = [(t, g - offset) for t, g in self.imu[board] if t0 <= t <= t1]
        return math.degrees(sum((b[0] - a[0]) * (a[1] + b[1]) / 2 for a, b in zip(s, s[1:])))

    def mean_gz(self, board, t0, t1):
        g = [gz for t, gz in self.imu[board] if t0 <= t <= t1]
        return sum(g) / len(g) if g else None


def stamp(node):
    return node.get_clock().now().nanoseconds * 1e-9


def run_once(s, node, sign, args):
    """One run; returns one result row per board with data, or None when aborted."""
    name = 'CCW' if sign > 0 else 'CW'
    target = 360.0 * args.turns
    # 1. still: offset
    s.tick(0.0, lambda: s.stopped())
    t_still = stamp(node); end = time.monotonic() + STILL_S
    s.tick(0.0, lambda: time.monotonic() > end)
    t_start = stamp(node)
    off0 = {b: s.mean_gz(b, t_still, t_start) for b in BOARDS}
    if off0[LEAD] is None:
        print(f'[spin] no {LEAD} IMU samples: is control_master up?'); return None
    yaw_w0 = s.yaw_w
    print(f'[spin] {name}: offsets ' + ', '.join(f'{b} {o:+.5f}' if o is not None else f'{b} (no data)'
                                                 for b, o in off0.items()) + f' rad/s, turning {args.turns} turns ...')
    lead = (lambda: s.imu_yaw(LEAD, t_start, off0[LEAD]))
    # 2. turn to target - MARGIN on the IMU, then ramp down
    cmd = [0.0]
    deadline = time.monotonic() + math.radians(target) / (args.omega * 1.2) * 2 + 20

    def spin_cmd():
        done = abs(lead()) >= target - MARGIN
        cmd[0] = toward(cmd[0], 0.0 if done else sign * args.omega, ALPHA / HZ)
        sys.stdout.write(f'\r[spin] IMU {lead():+8.1f} deg  wheel '
                         f'{math.degrees(s.yaw_w - yaw_w0):+8.1f} deg ')
        sys.stdout.flush()
        return cmd[0]

    s.tick(spin_cmd, lambda: (cmd[0] == 0.0 and abs(lead()) > 10 and s.stopped())
           or time.monotonic() > deadline)
    print()
    if time.monotonic() > deadline:
        print('[spin] timeout (stall?), run dropped'); return None
    # 3. finish onto the line by hand
    print(f'[spin] finish onto the tape line: hold a (CCW) / d (CW) to nudge, c when aligned, q to drop the run')
    nudge = {'until': 0.0, 'w': 0.0}
    state = {'quit': False, 'done': False}

    def finish_cmd():
        for c in keys().lower():
            if c in 'ad':
                nudge['w'] = NUDGE if c == 'a' else -NUDGE
                nudge['until'] = time.monotonic() + NUDGE_S
            elif c == 'c':
                state['done'] = True
            elif c == 'q':
                state['quit'] = True
        w = nudge['w'] if time.monotonic() < nudge['until'] else 0.0
        sys.stdout.write(f'\r[spin] IMU {lead():+8.1f} deg ({sign * target:+.0f} at the line) ')
        sys.stdout.flush()
        return w

    s.tick(finish_cmd, lambda: state['quit'] or (state['done'] and time.monotonic() >= nudge['until']))
    print()
    if state['quit']:
        print('[spin] run dropped'); return None
    # 4. still again: second offset, end of the run
    s.tick(0.0, lambda: s.stopped())
    t_end = stamp(node); end = time.monotonic() + STILL_S
    s.tick(0.0, lambda: time.monotonic() > end)
    t_tail = stamp(node)
    wheel_deg = math.degrees(s.yaw_w - yaw_w0)
    true_deg = sign * target
    wq = wheel_deg / true_deg
    print(f'[spin] {name}: true {true_deg:+.0f} deg, wheel odometry {wheel_deg:+.1f} deg -> wheel/true {wq:.4f}')
    rows = []
    for b in BOARDS:
        off1 = s.mean_gz(b, t_end, t_tail)
        if off0[b] is None or off1 is None:
            print(f'[spin]   {b}: no data'); continue
        offset = (off0[b] + off1) / 2  # the turn sits between the two still windows
        imu_deg = s.imu_yaw(b, t_start, offset, t_end)
        if abs(imu_deg) < 180:
            print(f'[spin]   {b}: IMU {imu_deg:+.1f} deg, not usable'); continue
        scale = true_deg / imu_deg
        print(f'[spin]   {b:>11}: IMU {imu_deg:+8.1f} deg -> gyro scale {scale:.4f} '
              f'(offset {off0[b]:+.5f} -> {off1:+.5f})')
        rows.append([datetime.now().isoformat(timespec='seconds'), 'gyro_spin', b, name.lower(), args.turns, 0.0,
                     round(true_deg, 2), round(imu_deg, 2), round(wheel_deg, 2), round(scale, 5), round(wq, 5),
                     round(offset, 6), args.note])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--turns', type=int, default=5)
    ap.add_argument('--runs', type=int, default=3, help='runs per direction')
    ap.add_argument('--dir', choices=['both', 'ccw', 'cw'], default='both')
    ap.add_argument('--omega', type=float, default=0.4,
                    help='teleop omega [rad/s] (control maps it with skid_steer_factor)')
    ap.add_argument('--note', default='')
    args = ap.parse_args()
    if not sys.stdin.isatty():
        sys.exit('[spin] needs a terminal for the keys')
    if args.turns < 1 or args.runs < 1 or not 0 < args.omega <= 1.0:
        sys.exit('turns >= 1, runs >= 1, 0 < omega <= 1.0')
    signs = {'both': [1, -1], 'ccw': [1], 'cw': [-1]}[args.dir] * args.runs

    rclpy.init()
    node = rclpy.create_node('gyro_spin')
    s = Spin(node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    old = termios.tcgetattr(sys.stdin)
    rows = []
    try:
        print('[spin] put the robot front mark on the tape line, clear 0.5 m. Enter to start, q to quit')
        for i, sign in enumerate(signs, 1):
            line = []  # wait for Enter / q while feeding control zeros

            def entered():
                if select.select([sys.stdin], [], [], 0)[0]:
                    line.append(sys.stdin.readline())
                return bool(line)
            s.tick(0.0, entered)
            if line[0].strip().lower() == 'q':
                break
            tty.setcbreak(sys.stdin.fileno())
            try:
                print(f'[spin] run {i}/{len(signs)}')
                row = run_once(s, node, sign, args)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            if row:
                rows += row
                new = not SCALE_CSV.exists()
                with open(SCALE_CSV, 'a', newline='') as f:
                    w = csv.writer(f)
                    if new:
                        w.writerow(SCALE_FIELDS)
                    w.writerows(row)
            if i < len(signs):
                print('[spin] Enter for the next run (robot is on the line), q to stop')
    except KeyboardInterrupt:
        print('\n[spin] Ctrl+C')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(5):
            s.pub.publish(Twist()); time.sleep(0.02)
        LOGS.mkdir(exist_ok=True)
        raw = LOGS / f'{datetime.now():%Y%m%d-%H%M%S}_gyro_spin.csv'
        with open(raw, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['topic', 't', 'gz_or_v', 'w', 'yaw_unwrapped'])
            for b in BOARDS:
                w.writerows([f'imu_{b}', t, g, '', ''] for t, g in s.imu[b])
            w.writerows(['odom', t, v, wz, y] for t, v, wz, y in s.odom)
        print(f'[spin] {len(rows)} rows -> {SCALE_CSV}; raw {raw}')
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
