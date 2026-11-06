#!/usr/bin/env python3
"""Corner demo: wherever the robot is put in the saved map, find it and drive to the bottom-right corner.

Needs the stack up with the saved map (ros2 launch craft_bringup corner_demo.launch.py), the robot
standing still. Steps:
  1. global localization: ~1 s of /lidar/merged matched over the whole map (map_tools.ScanMatcher);
     an ambiguous fit (runner-up within 8 % of the best) stops here: move the robot a little and rerun.
  2. /initialpose to slam_toolbox (localization mode), wait until its pose agrees (< 10 cm, < 5 deg).
  3. goal: reachable cell nearest the bottom-right corner, 0.30 m from walls (map_tools.corner_goal).
  4. planning START, goal on /planning/target_base_pose; wait for Run.Complete (or Fault / timeout).
Ctrl+C sends STOP. For RViz (all latched, map frame): /corner/goal, /corner/path (planned, from
/control/tracking_path), /corner/trail (driven, map->base_link every 2 cm).
  run_corner.py [--map ~/maps/arena3_clean.yaml] [--dry-run] [--skip-localize] [--corner br|tr|bl|tl] [--near X Y]
"""
import argparse
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_tools import MapGrid, ScanMatcher, corner_goal  # noqa: E402

CORNERS = {'br': (1, -1), 'tr': (1, 1), 'bl': (-1, -1), 'tl': (-1, 1)}
BODY_CENTER_X, BODY_R = 0.01275, 0.27  # merged scan is in base_link; drop anything on the robot
INTERRUPTED = False  # set by the SIGINT handler in main()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Demo:
    def __init__(self, node):
        self.n = node
        self.scans, self.state, self.odom = [], None, None
        # slam_toolbox's pose topic only comes with processed scans (the robot standing still: none), but it
        # keeps publishing map->odom: read map->base_link from TF instead.
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, node)
        node.create_subscription(LaserScan, '/lidar/merged', lambda m: self.scans.append(m), qos_profile_sensor_data)
        node.create_subscription(String, '/planning/state', lambda m: setattr(self, 'state', m.data),
                                 rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Odometry, '/perception/state/odom', lambda m: setattr(self, 'odom', m), 10)
        self.initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.command = node.create_publisher(String, '/planning/command', 10)
        self.goal = node.create_publisher(PoseStamped, '/planning/target_base_pose', 10)
        # RViz: latched so a late RViz still gets them
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.viz_goal = node.create_publisher(PoseStamped, '/corner/goal', latched)
        self.viz_path = node.create_publisher(PathMsg, '/corner/path', latched)
        self.viz_trail = node.create_publisher(PathMsg, '/corner/trail', latched)
        node.create_subscription(PathMsg, '/control/tracking_path',
                                 lambda m: m.poses and self.viz_path.publish(m), 10)
        self.trail = PathMsg()
        self.trail.header.frame_id = 'map'
        self.trail_timer = None

    @property
    def slam(self):
        """slam_toolbox's current map->base_link (x, y, yaw), None until it publishes map->odom."""
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def spin(self, sec, until=lambda: False):
        end = time.time() + sec
        while time.time() < end and not until():
            if INTERRUPTED:
                raise KeyboardInterrupt
            rclpy.spin_once(self.n, timeout_sec=0.05)
        return until()

    def scan_points(self, sec=1.0, keep=240):
        self.scans.clear()
        self.spin(sec)
        pts = []
        for m in self.scans:
            r = np.array(m.ranges, float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > 0.05) & (r < 7.5)
            x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
            far = np.hypot(x - BODY_CENTER_X, y) > BODY_R
            pts.append(np.c_[x[far], y[far]])
        if not pts:
            return None
        pts = np.concatenate(pts)
        rng = np.random.default_rng(0)
        return pts[rng.choice(len(pts), min(keep, len(pts)), replace=False)] if len(pts) else None

    def send_initial(self, pose):
        x, y, yaw = pose
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.0025
        m.pose.covariance[35] = 0.003
        self.initial.publish(m)

    def agrees(self, pose, dist=0.10, ang=math.radians(5)):
        return self.slam is not None and math.hypot(self.slam[0] - pose[0], self.slam[1] - pose[1]) < dist and \
            abs(math.remainder(self.slam[2] - pose[2], 2 * math.pi)) < ang

    def cmd(self, text):
        self.command.publish(String(data=text))

    def start_trail(self, period=0.2, spacing=0.02):
        def add():
            p = self.slam
            if p is None:
                return
            last = self.trail.poses[-1].pose.position if self.trail.poses else None
            if last is not None and math.hypot(p[0] - last.x, p[1] - last.y) < spacing:
                return
            s = PoseStamped()
            s.header.frame_id = 'map'
            s.header.stamp = self.n.get_clock().now().to_msg()
            s.pose.position.x, s.pose.position.y = p[0], p[1]
            s.pose.orientation.z, s.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
            self.trail.poses.append(s)
            self.trail.header.stamp = s.header.stamp
            self.viz_trail.publish(self.trail)
        add()
        self.trail_timer = self.n.create_timer(period, add)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=str(Path.home() / 'maps/arena3_clean.yaml'))
    ap.add_argument('--corner', choices=sorted(CORNERS), default='br')
    ap.add_argument('--dry-run', action='store_true', help='localize and print the goal, do not drive')
    ap.add_argument('--skip-localize', action='store_true', help='trust slam_toolbox as it is')
    ap.add_argument('--timeout', type=float, default=240.0, help='[s] drive timeout')
    ap.add_argument('--near', type=float, nargs=2, metavar=('X', 'Y'),
                    help='robot placed roughly here [m, map]: only candidates within 0.5 m compete')
    args = ap.parse_args()

    grid = MapGrid(args.map)
    matcher = ScanMatcher(grid)
    # Ctrl+C only sets a flag that spin() turns into KeyboardInterrupt: raised inside rclpy's message take it came
    # out as a RuntimeError, skipping STOP while planning kept driving (2026-11-01).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: globals().update(INTERRUPTED=True))
    node = rclpy.create_node('corner_demo')
    d = Demo(node)
    try:
        print('[demo] waiting for /lidar/merged and slam_toolbox ...')
        if not d.spin(20, lambda: d.scans and d.slam is not None):
            print('[demo] no /lidar/merged or no map->base_link TF: is corner_demo.launch.py up?'); return 1
        if args.skip_localize:
            pose = d.slam
        else:
            print('[demo] global localization: keep the robot still ...')
            pts = d.scan_points()
            if pts is None or len(pts) < 60:
                print('[demo] too few scan points'); return 1
            t0 = time.time()
            res = matcher.localize(pts, near=args.near)
            pose = res['pose']
            if pose is None:
                print(f'[demo] nothing fits within 0.5 m of --near {args.near}: check the position'); return 1
            print(f'[demo] found ({pose[0]:+.3f}, {pose[1]:+.3f}, {math.degrees(pose[2]):+.1f} deg) score '
                  f'{res["score"]:.2f}, runner-up {res["second"]:.2f}, {time.time() - t0:.1f} s')
            if res['second_pose'] is not None:
                p2 = res['second_pose']
                print(f'[demo]   runner-up at ({p2[0]:+.3f}, {p2[1]:+.3f}, {math.degrees(p2[2]):+.1f} deg)')
            if args.near and res['score'] < 0.85:
                # Searching only near the hint always returns something: a true fit scores 0.94-1.00 (2026-11-01
                # runs), a robot placed elsewhere at most ~0.70 around the hint.
                print(f'[demo] best fit near --near {args.near} scores only {res["score"]:.2f}: '
                      'is the robot really there? rerun with the right --near or without it'); return 1
            if res['ambiguous']:
                print('[demo] AMBIGUOUS (two places fit almost equally): turn or move the robot a little and rerun')
                return 1
            ok = False
            for _ in range(3):
                d.send_initial(pose)
                if d.spin(5.0, lambda: d.agrees(pose)):
                    ok = True
                    break
            if not ok:
                print(f'[demo] slam_toolbox did not settle on it (its map->base_link {d.slam}); rerun'); return 1
            print(f'[demo] slam_toolbox agrees: ({d.slam[0]:+.3f}, {d.slam[1]:+.3f}, {math.degrees(d.slam[2]):+.1f} deg)')
        gx, gy = corner_goal(grid, pose[:2], CORNERS[args.corner])
        print(f'[demo] goal ({args.corner}): ({gx:+.3f}, {gy:+.3f}), {math.hypot(gx - pose[0], gy - pose[1]):.2f} m '
              'straight-line from here')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        heading = math.atan2(gy - pose[1], gx - pose[0])  # not required (goal_heading_required: false)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        d.viz_goal.publish(goal)
        if args.dry_run:
            return 0

        d.spin(1.0)
        if d.state == 'Fault':
            # Planning wants map-frame odom and obstacles within 10 s of launch, but the map frame only exists
            # once localized: a late run finds it latched in "initialization timeout". Now the map is there.
            print('[demo] planning is in Fault (start-up timeout before localization?): RESET')
            d.cmd('RESET')
            d.spin(3.0, lambda: d.state != 'Fault')
        if not d.spin(20, lambda: d.state in ('Ready', 'Run.AwaitRequest')):
            print(f'[demo] planning not ready (state {d.state}): see /planning/diagnostics (control, odom, LiDAR '
                  'and, unless launched with arm:=false, arm joints must be fresh)'); return 1
        if d.state == 'Ready':
            d.cmd('START')
            if not d.spin(5, lambda: d.state == 'Run.AwaitRequest'):
                print(f'[demo] planning did not start (state {d.state})'); return 1
        goal.header.stamp = node.get_clock().now().to_msg()
        d.goal.publish(goal)
        d.start_trail()
        print('[demo] goal sent, driving ...')
        start, last = time.time(), None
        while time.time() - start < args.timeout:
            d.spin(1.0)
            if d.state != last:
                print(f'[demo]   planning: {d.state}')
                last = d.state
            if d.odom is not None:
                p = d.odom.pose.pose.position
                print(f'[demo]   at ({p.x:+.2f}, {p.y:+.2f}), {math.hypot(gx - p.x, gy - p.y):.2f} m to go', end='\r')
            if d.state == 'Run.Complete':
                print('\n[demo] arrived'); return 0
            if d.state == 'Fault':
                print('\n[demo] planning fault (see planning_master log)'); return 1
        print('\n[demo] timeout'); d.cmd('STOP'); return 1
    except KeyboardInterrupt:
        print('\n[demo] Ctrl+C: STOP'); d.cmd('STOP')
        globals().update(INTERRUPTED=False)  # let the STOP go out
        d.spin(0.3)
        return 1
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
