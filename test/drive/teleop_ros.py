#!/usr/bin/env python3
"""Keyboard driving (WASD) through control_master, for SLAM mapping runs.

teleop.py drives the wheel board directly, so nothing publishes odometry or IMU while it runs. This one
publishes geometry_msgs/Twist on /control/teleop/cmd_vel at 20 Hz instead; control_master turns it into
wheel commands (skid steer separation, max_v / max_wheel) while no task runs, keeps publishing wheel
odometry and the IMUs, and stops the wheels when the input goes stale (teleop_timeout_s). Same keys and
ramp as teleop.py. Shows the EKF odometry and the SLAM pose live.

  w/s   faster forward / backward (v step)      a/d   turn left / right (omega step)
  space stop (targets 0, ramps down)            x     zero omega only (drive straight)
  q     quit (Ctrl+C too; both send zero first)

  teleop_ros.py [--max-speed 0.15] [--max-omega 0.6] [--v-step 0.02] [--w-step 0.1] [--accel 0.1] [--alpha 1.0]
"""
import argparse
import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry

HZ = 20


def toward(x, target, step):
    return min(target, x + step) if target > x else max(target, x - step)


def keys():
    out = ''
    while select.select([sys.stdin], [], [], 0)[0]:
        out += sys.stdin.read(1)
    return out


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--max-speed', type=float, default=0.15, help='|v| cap [m/s] (control also caps at max_v)')
    ap.add_argument('--max-omega', type=float, default=0.6, help='|omega| cap [rad/s]')
    ap.add_argument('--v-step', type=float, default=0.02, help='v change per w/s press [m/s]')
    ap.add_argument('--w-step', type=float, default=0.1, help='omega change per a/d press [rad/s]')
    ap.add_argument('--accel', type=float, default=0.1, help='v ramp [m/s^2]')
    ap.add_argument('--alpha', type=float, default=1.0, help='omega ramp [rad/s^2]')
    args = ap.parse_args()
    if not sys.stdin.isatty():
        sys.exit('[tele] needs a terminal for the keys')

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/control/teleop/cmd_vel', 10)
    seen = {'odom': None, 'slam': None}
    node.create_subscription(Odometry, '/odometry/local', lambda m: seen.update(odom=m.pose.pose), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/perception/lidar/pose',
                             lambda m: seen.update(slam=m.pose.pose), 10)
    print(__doc__.split('\n\n')[2])

    def send(v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        pub.publish(msg)

    def pose(p):
        return '      ---      ' if p is None else f'{p.position.x:+5.2f},{p.position.y:+5.2f} {yaw(p.orientation):+4.0f}'

    tv = tw = v = w = 0.0
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            now = time.monotonic()
            k = keys().lower()
            if 'q' in k:
                break
            for c in k:
                tv += {'w': args.v_step, 's': -args.v_step}.get(c, 0.0)
                tw += {'a': args.w_step, 'd': -args.w_step}.get(c, 0.0)
                if c == ' ':
                    tv = tw = 0.0
                elif c == 'x':
                    tw = 0.0
            tv = round(max(-args.max_speed, min(args.max_speed, tv)), 4)
            tw = round(max(-args.max_omega, min(args.max_omega, tw)), 4)
            v = toward(v, tv, args.accel / HZ)
            w = toward(w, tw, args.alpha / HZ)
            send(v, w)
            rclpy.spin_once(node, timeout_sec=0)
            sys.stdout.write(f'\r[tele] target v {tv:+.2f} w {tw:+.2f} | cmd v {v:+.3f} w {w:+.2f} | '
                             f'ekf {pose(seen["odom"])} | slam {pose(seen["slam"])} ')
            sys.stdout.flush()
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(5):
            send(0.0, 0.0)
            time.sleep(0.02)
        print()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
