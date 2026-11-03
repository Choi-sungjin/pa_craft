#!/usr/bin/env python3
"""Square laps for the scan-to-map score baseline (test/localization/README.md step 5).

Stands in for planning like test/drive/pp_straight.py: /planning/hold=false heartbeat, odom relay to
/planning/odom, and one NAVIGATE task per side so control turns in place at every corner (heading error
above wheel_heading_tolerance) instead of cutting it. Corners come from the start pose: forward SIDE, then
SIDE to the right (clockwise; --left for counter-clockwise), back to the start. Needs control, perception and
infra (craft.launch.py group:=control / perception / infra) running.

  square_drive.py [--side 0.5] [--laps 3] [--speed 0.1] [--left]
Prints every localization state change; read the scores from the perception_master log afterwards.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from craft_interfaces.msg import TaskCommand, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool, String

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
        self.state = None
        self.hold_pub = node.create_publisher(Bool, '/planning/hold', 10)
        self.odom_pub = node.create_publisher(Odometry, '/planning/odom', 10)
        self.task_pub = node.create_publisher(TaskCommand, '/planning/task_command', 10)
        node.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        node.create_subscription(Bool, '/control/planning_held', self.on_held, 10)
        node.create_subscription(TaskResult, '/control/task_result', self.on_result, 10)
        node.create_subscription(String, '/perception/localization/state', self.on_state, 10)
        self.hold = False
        node.create_timer(0.05, lambda: self.hold_pub.publish(Bool(data=self.hold)))

    def on_odom(self, msg):
        self.odom = msg
        self.odom_pub.publish(msg)

    def on_held(self, msg):
        self.held = msg.data

    def on_result(self, msg):
        if msg.task_id == self.task_id:
            if self.result is None or msg.status != self.result.status:
                print(f'[sq] task {STATUS.get(msg.status, msg.status)}' + (f': {msg.reason}' if msg.reason else ''))
            self.result = msg

    def on_state(self, msg):
        if msg.data != self.state:
            print(f'[sq] localization: {self.state} -> {msg.data}')
            self.state = msg.data

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


def segment(frame, a, b, spacing=0.05):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    path = PathMsg()
    path.header.frame_id = frame
    n = max(1, round(math.dist(a, b) / spacing))
    for i in range(n + 1):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = a[0] + (b[0] - a[0]) * i / n
        pose.pose.position.y = a[1] + (b[1] - a[1]) * i / n
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        path.poses.append(pose)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', type=float, default=0.5, help='square side [m]')
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--speed', type=float, default=0.1, help='[m/s]')
    ap.add_argument('--left', action='store_true', help='counter-clockwise (first turn left)')
    ap.add_argument('--odom', default='/perception/state/odom', help='pose source (what planning relays)')
    args = ap.parse_args()
    if not 0 < args.speed <= 0.26 or not args.side > 0 or args.laps < 1:
        sys.exit('speed must be in (0, 0.26] m/s, side > 0, laps >= 1')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C must still be able to send STOP
    node = rclpy.create_node('square_drive_test')
    r = Runner(node, args.odom)
    try:
        if not r.spin_until(lambda: r.odom is not None, 5.0):
            sys.exit(f'[sq] no {args.odom}: perception and infra up?')
        if not r.spin_until(lambda: r.held is False, 5.0):
            sys.exit(f'[sq] control still held (planning_held={r.held}): control_master up? board fault?')
        p, yaw = r.odom.pose.pose.position, yaw_of(r.odom.pose.pose.orientation)
        c, s, side = math.cos(yaw), math.sin(yaw), (1 if args.left else -1)
        local = [(0, 0), (args.side, 0), (args.side, side * args.side), (0, side * args.side)]
        corners = [(p.x + c * f - s * l, p.y + s * f + c * l) for f, l in local]
        print(f'[sq] {args.laps} laps of {args.side} m at {args.speed} m/s, '
              f'{"counter-clockwise" if args.left else "clockwise"}, start ({p.x:.3f}, {p.y:.3f}) '
              f'yaw {math.degrees(yaw):.1f} deg, frame {r.odom.header.frame_id}')
        for lap in range(1, args.laps + 1):
            for k in range(4):
                a, b = corners[k], corners[(k + 1) % 4]
                r.task_id = int(time.time() * 1000)  # control rejects IDs not above the last one it saw
                r.result = None
                r.send(TaskCommand.EXECUTE, segment(r.odom.header.frame_id, a, b), args.speed)
                print(f'[sq] lap {lap} side {k + 1}: ({a[0]:.2f}, {a[1]:.2f}) -> ({b[0]:.2f}, {b[1]:.2f})')
                if not r.spin_until(lambda: r.result is not None and r.result.status in DONE,
                                    args.side / args.speed + 20):
                    print('[sq] no task result in time: stopping')
                    r.send(TaskCommand.STOP)
                    return
                if r.result.status != TaskResult.SUCCEEDED:
                    print('[sq] side did not succeed: stopping here')
                    return
        q = r.odom.pose.pose.position
        print(f'[sq] done. end ({q.x:.3f}, {q.y:.3f}), {math.hypot(q.x - p.x, q.y - p.y) * 1000:.0f} mm from start '
              f'(EKF pose), yaw {math.degrees(yaw_of(r.odom.pose.pose.orientation)):.1f} deg')
    except KeyboardInterrupt:
        print('\n[sq] Ctrl+C: stopping')
        r.send(TaskCommand.STOP)
        r.hold = True
        r.spin_until(lambda: False, 0.5)
    finally:
        r.hold = True
        for _ in range(5):
            r.hold_pub.publish(Bool(data=True))
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
