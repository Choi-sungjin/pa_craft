#!/usr/bin/env python3
"""Each LiDAR on its own, against the walls around a robot standing still (2026-11-12).

Every straight wall near the robot, in base_link with the launch yaw (TF), per LiDAR: the wall ahead (should read
90 deg: square to the chassis), behind (90), and the side walls (0: parallel). What a wall reads off that is
(placement error of the robot) + (yaw error of that LiDAR). Placement cancels in:
  - one side wall seen by both LiDARs (front part / rear part): their difference = rear yaw error - front yaw error;
  - many placements: placement errors average out, the LiDAR error stays.
Correction: yaw_new = yaw_now - reading (a wall turned + counter-clockwise = the LiDAR is turned clockwise).
  /usr/bin/python3 lidar_walls.py [--seconds 3]     (lidar_processor or the stack up, robot still)
"""
import argparse
import math

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros


def line(p):
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c)
    d = vt[0]
    across = (p - c) @ np.array([-d[1], d[0]])
    return math.degrees(math.atan2(d[1], d[0])), float(np.sqrt(np.mean(across ** 2))), float(np.ptp((p - c) @ d))


def fit(p, n):
    for _ in range(4):  # keep the points on the main line (boxes, corners out)
        if len(p) < n:
            return None
        a, _, _ = line(p)
        d = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        across = (p - np.median(p, 0)) @ np.array([-d[1], d[0]])
        p = p[np.abs(across - np.median(across)) < 0.02]
    return (*line(p), len(p)) if len(p) >= n else None


REGIONS = {  # base_link boxes [x0, x1, y0, y1] and the angle a square wall reads there
    'ahead': ((0.25, 3.0, -0.5, 0.5), 90.0),
    'behind': ((-3.0, -0.25, -0.5, 0.5), 90.0),
    'left': ((-0.8, 0.8, 0.18, 1.5), 0.0),
    'right': ((-0.8, 0.8, -1.5, -0.18), 0.0),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=3.0)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('lidar_walls')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    scans = {'front': [], 'rear': []}
    for k in scans:
        node.create_subscription(LaserScan, f'/lidar/{k}/scan', scans[k].append, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}
    for k, ms in scans.items():
        if len(ms) < 5:
            print(f'{k}: only {len(ms)} scans')
            continue
        t = buf.lookup_transform('base_link', ms[0].header.frame_id, rclpy.time.Time())
        q, tr = t.transform.rotation, t.transform.translation
        R = np.array([[1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z)],
                      [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)]])
        pts = []
        for m in ms:
            r = np.array(m.ranges, float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > max(m.range_min, 0.1)) & (r < 4)
            pts.append(np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])] @ R.T + [tr.x, tr.y])
        P = np.vstack(pts)
        for name, ((x0, x1, y0, y1), square) in REGIONS.items():
            sel = P[(P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)]
            f = fit(sel, 3 * len(ms))
            if not f or f[2] < 0.3 or f[1] > 0.005:  # short or not one straight wall
                continue
            off = (f[0] - square + 90) % 180 - 90
            if abs(off) > 15:  # another wall (a corner, a side passage) in that box
                continue
            out[(k, name)] = off
            print(f'{k:5s} LiDAR  {name:6s} wall: {off:+.2f} deg off square '
                  f'({f[2]:.2f} m of wall, {1000 * f[1]:.1f} mm rms, {f[3]} pts)')
    for side in REGIONS:
        if ('front', side) in out and ('rear', side) in out:
            print(f'{side} wall seen by both: rear - front = {out[("rear", side)] - out[("front", side)]:+.2f} deg '
                  '(rear yaw error minus front yaw error; the placement cancels)')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
