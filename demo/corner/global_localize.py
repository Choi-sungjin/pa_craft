#!/usr/bin/env python3
"""Global localization at start-up (craft.launch.py with slam_map:=): run_corner.py's step 1-2 as a node.

The robot standing still anywhere in the saved map: ~1 s of /lidar/merged is matched over the whole map
(map_tools.ScanMatcher), the pose goes to slam_toolbox (localization mode) on /initialpose, and it is done once
slam_toolbox's map->base_link agrees (< 10 cm, < 5 deg). --near X Y (craft.launch.py: initial_x/initial_y, the start
zone 0.56 0.98): only within --radius of it, and a fit there must score >= 0.85 (run_corner.py's --near rule). An ambiguous fit (runner-up within 8 %), too few points
or no agreement: tried again every --retry_s (move or turn the robot a little meanwhile).

/perception/localization/global (std_msgs/String, latched):
  "searching"                      not localized yet (waiting for the scan / TF, or retrying)
  "ok x y yaw_deg score"           localized; mission_master waits for this before its first drive
  "retry: <why>"                   the last try failed, trying again
--relocalize: a message on /perception/localization/global_request (std_msgs/Empty) runs it again.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_tools import MapGrid, ScanMatcher  # noqa: E402

BODY_CENTER_X, BODY_R = 0.01275, 0.27  # as run_corner.py: the merged scan is in base_link, drop the robot itself


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GlobalLocalize:
    def __init__(self, node, matcher, retry_s, near=None, radius=0.5, yaw=None):
        self.n, self.matcher, self.retry_s = node, matcher, retry_s
        self.near, self.radius, self.yaw = near, radius, yaw
        self.scans = []
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, node)
        node.create_subscription(LaserScan, '/lidar/merged', self.scans.append, qos_profile_sensor_data)
        node.create_subscription(Empty, '/perception/localization/global_request', lambda _m: self.request(), 10)
        self.initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status = node.create_publisher(String, '/perception/localization/global',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.wanted = True

    def request(self):
        self.wanted = True
        self.say('searching')

    def say(self, text):
        self.status.publish(String(data=text))
        self.n.get_logger().info(text)

    def slam(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def spin(self, sec, until=lambda: False):
        end = time.time() + sec
        while time.time() < end and rclpy.ok() and not until():
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
        s = self.slam()
        return s is not None and math.hypot(s[0] - pose[0], s[1] - pose[1]) < dist and \
            abs(math.remainder(s[2] - pose[2], 2 * math.pi)) < ang

    def attempt(self):
        """One localization; None when done, else why not."""
        pts = self.scan_points()
        if pts is None or len(pts) < 60:
            return 'too few scan points'
        t0 = time.time()
        res = self.matcher.localize(pts, near=self.near, radius=self.radius, yaw=self.yaw)
        pose = res['pose']
        if pose is None:
            return f'no fit within {self.radius} m of {self.near}' if self.near else 'no fit'
        self.n.get_logger().info(
            f'fit ({pose[0]:+.3f}, {pose[1]:+.3f}, {math.degrees(pose[2]):+.1f} deg) score {res["score"]:.2f}, '
            f'runner-up {res["second"]:.2f}, {time.time() - t0:.1f} s')
        if self.near and res['score'] < 0.85:
            # Near the hint something always fits: a true fit scores 0.94-1.00, a robot elsewhere ~0.70 (run_corner.py).
            return f'best fit near {self.near} scores only {res["score"]:.2f}: is the robot in the start zone?'
        if res['ambiguous']:
            p2 = res['second_pose']
            return (f'ambiguous: runner-up {res["second"]:.2f} at ({p2[0]:+.2f}, {p2[1]:+.2f}) vs '
                    f'{res["score"]:.2f}; turn or move the robot a little')
        for _ in range(3):
            self.send_initial(pose)
            if self.spin(5.0, lambda: self.agrees(pose)):
                s = self.slam()
                self.say(f'ok {s[0]:.3f} {s[1]:.3f} {math.degrees(s[2]):.1f} {res["score"]:.2f}')
                return None
        return f'slam_toolbox did not settle on ({pose[0]:+.2f}, {pose[1]:+.2f}) (its pose {self.slam()})'

    def run(self):
        self.say('searching')
        while rclpy.ok():
            if not self.wanted:
                self.spin(0.5)
                continue
            if not self.spin(5.0, lambda: self.scans and self.slam() is not None):
                self.n.get_logger().info('waiting for /lidar/merged and map->base_link (slam_toolbox) ...')
                continue
            why = self.attempt()
            if why is None:
                self.wanted = False
                continue
            self.say('retry: ' + why)
            self.spin(self.retry_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map yaml the scan is matched to (the slam_map pose graph frame)')
    ap.add_argument('--retry_s', type=float, default=3.0)
    ap.add_argument('--near', type=float, nargs=2, metavar=('X', 'Y'), help='rough start position (map frame)')
    ap.add_argument('--radius', type=float, default=0.5, help='[m] search radius around --near')
    ap.add_argument('--yaw', type=float, help='[deg] start heading: only headings within 20 deg of it')
    args, _ros = ap.parse_known_args()
    grid = MapGrid(str(Path(args.map).expanduser()))
    rclpy.init()
    node = rclpy.create_node('global_localize')
    try:
        GlobalLocalize(node, ScanMatcher(grid), args.retry_s, tuple(args.near) if args.near else None, args.radius,
                       math.radians(args.yaw) if args.yaw is not None else None).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
