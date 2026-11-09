#!/usr/bin/env python3
"""Pure Pursuit straight-line test: stand in for planning, drive a 5 m reference path, plot ref vs real.

Needs test/drive/pp_bringup.launch.py running (control_master + ekf_local). This node does planning's part
only for this test: /planning/hold=false heartbeat, /odometry/local -> /planning/odom relay, and one
NAVIGATE task at constant speed (control's longitudinal side just holds the speed; lateral is Pure Pursuit).
The reference path starts at the current EKF pose and runs DISTANCE along the current yaw, shifted
--offset to the left (to see convergence). "Real" path = EKF pose: it cannot see wheel slip.

  pp_straight.py [--distance 5.0] [--speed 0.1] [--offset 0.0]
Writes test/drive/pp_logs/<stamp>/: samples.csv, ref_path.csv, summary.txt, plot.png.
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
import numpy as np
import yaml
from craft_interfaces.msg import TaskCommand, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool

HERE = Path(__file__).resolve().parent
CONTROL_YAML = HERE.parents[1] / 'src/craft_control/config/control.yaml'
STATUS = {getattr(TaskResult, k): k for k in ('RUNNING', 'SUCCEEDED', 'FAILED', 'STOPPING', 'STOPPED', 'REJECTED')}
DONE = (TaskResult.SUCCEEDED, TaskResult.FAILED, TaskResult.STOPPED, TaskResult.REJECTED)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runner:
    def __init__(self, node, odom_topic):
        self.node = node
        self.odom = None
        self.held = None
        self.result = None
        self.task_id = 0
        self.samples = []  # t, x, y, yaw, v, omega while recording
        self.recording_since = None
        self.hold_pub = node.create_publisher(Bool, '/planning/hold', 10)
        self.odom_pub = node.create_publisher(Odometry, '/planning/odom', 10)
        self.task_pub = node.create_publisher(TaskCommand, '/planning/task_command', 10)
        node.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        node.create_subscription(Bool, '/control/planning_held', self.on_held, 10)
        node.create_subscription(TaskResult, '/control/task_result', self.on_result, 10)
        self.hold = False
        node.create_timer(0.05, lambda: self.hold_pub.publish(Bool(data=self.hold)))

    def on_odom(self, msg):
        self.odom = msg
        self.odom_pub.publish(msg)
        if self.recording_since is not None:
            p = msg.pose.pose.position
            self.samples.append([time.monotonic() - self.recording_since, p.x, p.y, yaw_of(msg.pose.pose.orientation),
                                 msg.twist.twist.linear.x, msg.twist.twist.angular.z])

    def on_held(self, msg):
        self.held = msg.data

    def on_result(self, msg):
        if msg.task_id == self.task_id:
            if self.result is None or msg.status != self.result.status:
                print(f'[pp] task {STATUS.get(msg.status, msg.status)}' + (f': {msg.reason}' if msg.reason else ''))
            self.result = msg

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return True

    def send(self, operation, path=None, speed=0.0):
        c = TaskCommand()
        c.header.stamp = self.node.get_clock().now().to_msg()
        c.task_id, c.revision, c.operation, c.kind = self.task_id, 1, operation, TaskCommand.NAVIGATE
        if path is not None:
            c.path = path
        c.speed = speed
        self.task_pub.publish(c)


def make_path(odom, distance, offset, spacing=0.05):
    p, yaw = odom.pose.pose.position, yaw_of(odom.pose.pose.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    x0, y0 = p.x - s * offset, p.y + c * offset  # +offset = left of the robot
    path = PathMsg()
    path.header.frame_id = odom.header.frame_id
    n = max(1, round(distance / spacing))
    for i in range(n + 1):
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose.position.x = x0 + c * distance * i / n
        pose.pose.position.y = y0 + s * distance * i / n
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        path.poses.append(pose)
    return path, (x0, y0, yaw)


def analyse(samples, ref, distance, out, title):
    x0, y0, yaw = ref
    c, s = math.cos(yaw), math.sin(yaw)
    rows = []
    for t, x, y, th, v, w in samples:
        along = c * (x - x0) + s * (y - y0)
        lat = -s * (x - x0) + c * (y - y0)  # + = left of the reference line
        head = math.remainder(th - yaw, 2 * math.pi)
        rows.append([t, x, y, th, v, w, along, lat, head])
    with open(out / 'samples.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw', 'v', 'omega', 'along', 'lateral', 'heading_err'])
        w.writerows([[round(v, 6) for v in r] for r in rows])
    inside = [r for r in rows if 0.0 <= r[6] <= distance]
    if not inside:
        print('[pp] no samples along the reference path: nothing to analyse')
        return
    rmse = math.sqrt(sum(r[7] ** 2 for r in inside) / len(inside))
    max_lat = max(abs(r[7]) for r in inside)
    head_rmse = math.sqrt(sum(r[8] ** 2 for r in inside) / len(inside))
    last = rows[-1]
    goal_err = math.hypot(last[1] - (x0 + c * distance), last[2] - (y0 + s * distance))
    summary = (f'lateral RMSE {rmse * 1000:.1f} mm, max {max_lat * 1000:.1f} mm, heading RMSE '
               f'{math.degrees(head_rmse):.2f} deg, final lateral {last[7] * 1000:+.1f} mm, '
               f'final heading {math.degrees(last[8]):+.2f} deg, goal error {goal_err * 1000:.1f} mm, '
               f'{last[0]:.1f} s, {len(inside)} samples')
    (out / 'summary.txt').write_text(title + '\n' + summary + '\n')
    print('[pp] ' + summary)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    along = [r[6] for r in rows]
    lat_mm = [r[7] * 1000 for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(10, 11), gridspec_kw={'height_ratios': [1.3, 1, 0.8]})
    fig.suptitle(title, fontsize=11)
    ax[0].plot([x0, x0 + c * distance], [y0, y0 + s * distance], '--', color='#1f77b4', lw=2, label='ref path')
    ax[0].plot([r[1] for r in rows], [r[2] for r in rows], color='#d62728', lw=1.5, label='real path (EKF)')
    ax[0].plot(rows[0][1], rows[0][2], 'o', color='#2ca02c', label='start')
    ax[0].set_aspect('equal', adjustable='datalim')
    ax[0].set_xlabel('x [m] (odom)'); ax[0].set_ylabel('y [m] (odom)')
    ax[0].set_title('Path, true scale'); ax[0].grid(alpha=0.3); ax[0].legend(loc='best')
    ax[1].plot(along, lat_mm, color='#d62728', lw=1.5, label='lateral error')
    ax[1].axhline(0, color='#1f77b4', ls='--', lw=1.5, label='ref path')
    for sgn in (1, -1):
        ax[1].axhline(sgn * rmse * 1000, color='#7f7f7f', ls=':', lw=1.2,
                      label=f'±RMSE {rmse * 1000:.1f} mm' if sgn > 0 else None)
    ax[1].set_xlabel('distance along ref [m]'); ax[1].set_ylabel('lateral error [mm] (+ left)')
    ax[1].set_title(f'Lateral error: RMSE {rmse * 1000:.1f} mm, max {max_lat * 1000:.1f} mm')
    ax[1].grid(alpha=0.3); ax[1].legend(loc='best')
    ax[2].plot(along, [math.degrees(r[8]) for r in rows], color='#9467bd', lw=1.5)
    ax[2].axhline(0, color='#1f77b4', ls='--', lw=1)
    ax[2].set_xlabel('distance along ref [m]'); ax[2].set_ylabel('heading error [deg]')
    ax[2].set_title(f'Heading error: RMSE {math.degrees(head_rmse):.2f} deg'); ax[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / 'plot.png', dpi=110)
    print(f'[pp] plot: {out / "plot.png"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--distance', type=float, default=5.0, help='reference path length [m]')
    ap.add_argument('--speed', type=float, default=0.1, help='constant speed [m/s] (low: EKF cannot see slip)')
    ap.add_argument('--offset', type=float, default=0.0, help='shift the reference left [m] to test convergence')
    ap.add_argument('--odom', default='/odometry/local', help='pose source (EKF local)')
    args = ap.parse_args()
    if not 0 < args.speed <= 0.26 or not args.distance > 0:
        sys.exit('speed must be in (0, 0.26] m/s and distance > 0')
    params = yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']
    lookahead = round(float(np.interp(args.speed, params['lookahead_speeds'], params['lookahead_distances'])), 3)

    input(f'[pp] {args.distance} m at {args.speed} m/s, offset {args.offset} m, lookahead {lookahead} m. '
          f'Arm folded, clear {args.distance + 0.5:g} m ahead. Enter to start (Ctrl+C stops) ')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C must still be able to send STOP
    node = rclpy.create_node('pp_straight_test')
    r = Runner(node, args.odom)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    out = HERE / 'pp_logs' / f'{stamp}_v{args.speed}_la{lookahead}_off{args.offset}'
    try:
        if not r.spin_until(lambda: r.odom is not None, 5.0):
            sys.exit(f'[pp] no {args.odom}: is pp_bringup.launch.py running (ekf_local)?')
        if not r.spin_until(lambda: r.held is False, 5.0):
            sys.exit(f'[pp] control still held (planning_held={r.held}): control_master up? board fault? '
                     'check its terminal')
        path, ref = make_path(r.odom, args.distance, args.offset)
        r.task_id = int(time.time() * 1000)  # control rejects IDs not above the last one it saw
        r.samples, r.recording_since = [], time.monotonic()
        r.send(TaskCommand.EXECUTE, path, args.speed)
        print(f'[pp] driving (task {r.task_id})')
        timeout = args.distance / args.speed + 30
        if not r.spin_until(lambda: r.result is not None and r.result.status in DONE, timeout):
            print('[pp] no task result in time: stopping')
            r.send(TaskCommand.STOP)
        r.spin_until(lambda: False, 0.5)  # a little more pose after the stop
    except KeyboardInterrupt:
        print('\n[pp] Ctrl+C: stopping')
        r.send(TaskCommand.STOP)
        r.hold = True
        r.spin_until(lambda: False, 0.5)
    finally:
        r.hold = True
        r.recording_since = None
        for _ in range(5):
            r.hold_pub.publish(Bool(data=True))
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()

    if not r.samples:
        sys.exit('[pp] no samples recorded')
    out.mkdir(parents=True, exist_ok=True)
    with open(out / 'ref_path.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y'])
        w.writerows([[round(p.pose.position.x, 6), round(p.pose.position.y, 6)] for p in path.poses])
    status = STATUS.get(r.result.status, '?') if r.result else 'no result'
    title = (f'Pure Pursuit straight {args.distance} m, v {args.speed} m/s, lookahead {lookahead} m, '
             f'offset {args.offset} m, {status} ({stamp})')
    analyse(r.samples, ref, args.distance, out, title)


if __name__ == '__main__':
    main()
