#!/usr/bin/env python3
"""LiDAR yaw against the chassis, from a straight wall.

Put the robot with one side parallel to a long straight wall, 0.3-0.8 m from it: the distance from the front wheel
and from the rear wheel of that side to the wall equal within 1 mm (1 mm over the 0.23 m wheelbase is 0.25 deg). Stack up
(cornerup arm:=false), robot standing still:

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/lidar_yaw/wall_yaw.py [--side left|right|both] [--topic /lidar/merged] [--seconds 3]

Prints the angle of the wall in the scan (base_link: x forward, y left). A wall parallel to the chassis must read 0 deg;
what it reads is how far the LiDARs' "forward" is turned from the chassis: + = the scan sees the wall turned
counter-clockwise, so the LiDAR frame is turned clockwise (to the right) of the chassis by that much.
Turn the robot round (the other side to the same wall) and measure again: the two readings agree when the offset is the
LiDAR's and differ when the robot was not parallel or the wall is not straight.
The two LiDARs separately: --topic /lidar/front/scan or /lidar/rear/scan give the wall in that LiDAR's own frame (the
front one is mounted at 180 deg: its reading is against its own axis, not base_link).
"""
import argparse
import math

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wall_angle(pts):
    """Direction of the best line through pts [deg, in (-90, 90]], its rms distance [m] and length [m]."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    d = vt[0] if vt[0][0] >= 0 else -vt[0]
    along, across = (pts - c) @ d, (pts - c) @ np.array([-d[1], d[0]])
    return math.degrees(math.atan2(d[1], d[0])), float(np.sqrt(np.mean(across ** 2))), float(along.max() - along.min())


def side_points(pts, side, near, far, half_length):
    """Points beside the robot on one side: the wall, cut to the stretch next to the chassis."""
    y = pts[:, 1] if side == 'left' else -pts[:, 1]
    p = pts[(y > near) & (y < far) & (np.abs(pts[:, 0]) < half_length)]
    if len(p) < 30:
        return p
    # Only the wall: within 3 cm of the line through the points nearest the median distance (boxes, legs out).
    for _ in range(3):
        a, _, _ = wall_angle(p)
        d = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        across = (p - np.median(p, axis=0)) @ np.array([-d[1], d[0]])
        p = p[np.abs(across - np.median(across)) < 0.03]
        if len(p) < 30:
            break
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=['left', 'right', 'both'], default='both')
    ap.add_argument('--topic', default='/lidar/merged')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--near', type=float, default=0.20, help='[m] wall at least this far to the side')
    ap.add_argument('--far', type=float, default=1.20, help='[m] and at most this far')
    ap.add_argument('--half-length', type=float, default=0.80, help='[m] wall used this far ahead of and behind the robot')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('wall_yaw')
    scans = []
    node.create_subscription(LaserScan, args.topic, scans.append, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(scans) < 5:
        print(f'{len(scans)} scans on {args.topic} in {args.seconds} s: is the stack up (cornerup), same ROS_DOMAIN_ID?')
        return 1
    print(f'{len(scans)} scans on {args.topic}, frame {scans[0].header.frame_id}')
    for side in (['left', 'right'] if args.side == 'both' else [args.side]):
        angles = []
        for m in scans:
            r = np.array(m.ranges, float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > m.range_min)
            p = side_points(np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], side, args.near, args.far, args.half_length)
            if len(p) >= 30:
                ang, rms, length = wall_angle(p)
                if length > 0.5 and rms < 0.015:
                    angles.append((ang, rms, length, len(p)))
        if len(angles) < 3:
            print(f'  {side}: no straight wall {args.near}-{args.far} m to that side over 0.5 m ({len(angles)} of {len(scans)} scans)')
            continue
        v = np.array(angles)
        print(f'  {side}: wall at {np.median(v[:, 0]):+.2f} deg in the scan (scan to scan +-{np.std(v[:, 0]):.2f}), '
              f'{np.median(v[:, 2]):.2f} m of wall, {int(np.median(v[:, 3]))} points, {1000 * np.median(v[:, 1]):.1f} mm rms, '
              f'{len(v)} of {len(scans)} scans')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def demo():
    """Self-check: /usr/bin/python3 -c "import wall_yaw as w; w.demo()" """
    rng = np.random.default_rng(0)
    for true in (0.0, 3.0, -2.5):
        t = math.radians(true)
        x = np.linspace(-0.8, 0.8, 200)
        wall = np.c_[x * math.cos(t) - 0.5 * math.sin(t), x * math.sin(t) + 0.5 * math.cos(t)] + rng.normal(0, 0.004, (200, 2))
        clutter = np.c_[rng.uniform(-0.3, 0.3, 40), rng.uniform(0.25, 0.4, 40)]  # a box between the robot and the wall
        p = side_points(np.vstack([wall, clutter]), 'left', 0.2, 1.2, 0.8)
        ang, rms, length = wall_angle(p)
        assert abs(ang - true) < 0.15 and rms < 0.01 and length > 1.0, (true, ang, rms, length)
    print('ok')


if __name__ == '__main__':
    raise SystemExit(main())
