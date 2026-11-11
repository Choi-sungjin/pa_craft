#!/usr/bin/env python3
"""LiDAR mount check: TF, scan rate, front/rear agreement, floor returns, and a top-down plot.

Run with lidar_processor.launch.py (or slaminfra) up and the robot standing still in an open spot.
Points go to base_link through the full 3D TF rotation (not the merger's 2D pose), so a wrong roll/yaw
or a wrong flip shows up as front and rear disagreeing about the same wall.
  1. TF base_link -> laser_front / laser_rear: x, y, z, roll, yaw, upside down or not.
  2. Scan rate of /lidar/front/scan, /lidar/rear/scan (and /lidar/merged if published).
  3. Agreement: for each 2 deg base_link direction both LiDARs see, the nearest return of each; the
     median difference should be about the range noise (1-2 cm). A mirrored or turned scan gives
     tens of cm. With the LiDARs low between the wheels the sides are hidden, so there may be none.
  4. Floor: 15 deg sectors around the robot, median nearest return per LiDAR; a ring of returns at
     similar ranges all around in an open spot is the floor.
  5. Left/right, one LiDAR at a time (the sides are hidden): the plain run also saves its points as the
     baseline (logs/baseline.npz), nothing within 0.8 m. Put one object ~0.6 m away diagonally front-right, run --object front;
     then back-right, --object rear. Points new since the baseline must land at -y (right); a wrong flip
     puts them at +y. Non-interactive, so each step can wait for an ok from the person placing it.
Plot: test/lidar/logs/<stamp>_lidar_check.png.

  lidar_check.py [--seconds 3]                 checks 1-4, saves the baseline
  lidar_check.py --object front|rear           check 5 for one LiDAR against the baseline
"""
import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HERE = Path(__file__).resolve().parent
BASE = 'base_link'
SIDES = ('front', 'rear')
BODY_R = 0.25  # [m] around the body center: robot itself (lidar_processor body_radius_m)
NEAR = 0.8     # [m] keep this clear around the robot; the check-5 object goes inside it


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Check:
    def __init__(self, node):
        self.node = node
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, node)
        self.scans = {s: [] for s in SIDES + ('merged',)}
        for s in SIDES:
            node.create_subscription(LaserScan, f'/lidar/{s}/scan', lambda m, s=s: self.scans[s].append(m),
                                     qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/lidar/merged', lambda m: self.scans['merged'].append(m),
                                 qos_profile_sensor_data)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tf(self, side):
        t = self.buf.lookup_transform(BASE, f'laser_{side}', rclpy.time.Time())
        tr = t.transform.translation
        return np.array([tr.x, tr.y, tr.z]), quat_to_matrix(t.transform.rotation)

    def points(self, side, scans, tf):
        """All returns of these scans in base_link (x, y), via the full 3D rotation."""
        trans, rot = tf
        out = []
        for m in scans:
            r = np.array(m.ranges, dtype=float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
            p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])
            out.append((rot @ p).T[:, :2] + trans[:2])
        return np.concatenate(out) if out else np.zeros((0, 2))


def nearest_by_direction(pts, center, step_deg):
    """Nearest return per direction bin seen from center (body center): {bin: range}."""
    d = pts - center
    rng = np.hypot(d[:, 0], d[:, 1])
    keep = rng > BODY_R
    ang = np.degrees(np.arctan2(d[keep, 1], d[keep, 0]))
    bins = np.floor((ang + 180) / step_deg).astype(int)
    out = {}
    for b, r in zip(bins, rng[keep]):
        out[b] = min(out.get(b, np.inf), r)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--object', choices=['front', 'rear'], help='check 5: object placed diagonally on the right')
    ap.add_argument('--body-center-x', type=float, default=0.01275)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('lidar_check')
    c = Check(node)
    c.spin(args.seconds + 1.0)

    print('== 1. TF (base_link -> laser_*)')
    tfs = {}
    for s in SIDES:
        try:
            tfs[s] = c.tf(s)
        except Exception as e:  # noqa: BLE001
            sys.exit(f'   laser_{s}: no TF ({e}); is lidar_processor.launch.py up?')
        t, R = tfs[s]
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        print(f'   laser_{s:5}: x {t[0]:+.4f} y {t[1]:+.4f} z {t[2]:+.4f} m  yaw {yaw:+7.2f}  roll {roll:+7.2f} deg  '
              f'{"UPSIDE DOWN" if R[2, 2] < 0 else "upright"}')

    print(f'== 2. scan rate over the last {args.seconds:g} s')
    for s in SIDES + ('merged',):
        m = c.scans[s]
        stamps = [x.header.stamp.sec + x.header.stamp.nanosec * 1e-9 for x in m]
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 and stamps[-1] > stamps[0] else 0.0
        print(f'   /lidar/{s:6}: {len(m):3d} scans, {hz:5.1f} Hz' + ('  (not published)' if not m else ''))
    if not c.scans['front'] or not c.scans['rear']:
        sys.exit('   need both raw scans')

    center = np.array([args.body_center_x, 0.0])
    pts = {s: c.points(s, c.scans[s][-int(args.seconds * 10):], tfs[s]) for s in SIDES}
    baseline = HERE / 'logs' / 'baseline.npz'
    obj = None
    if args.object:
        if not baseline.exists():
            sys.exit('   no baseline: run without --object first, with nothing placed')
        s, where = args.object, {'front': 'front-right', 'rear': 'back-right'}[args.object]
        before = np.load(baseline)[s]
        now = pts[s]
        d = now - center
        rng = np.hypot(d[:, 0], d[:, 1])
        close = now[(rng > BODY_R) & (rng < NEAR)]
        if len(close) and len(before):  # only points that were not there before
            gap = np.min(np.hypot(*(close[:, None, :] - before[None, :, :]).transpose(2, 0, 1)), axis=1)
            close = close[gap > 0.05]
        print(f'== 5. left/right, {s} LiDAR, object {where}')
        if len(close) >= 3:
            m = close.mean(axis=0)
            obj = [m]
            ok = m[1] < 0 and (m[0] > 0) == (s == 'front')
            print(f'   {len(close)} new points around ({m[0]:+.2f}, {m[1]:+.2f}) m -> '
                  + ('RIGHT (-y), on the expected side: OK' if ok else
                     'LEFT (+y): WRONG, the scan is mirrored' if m[1] > 0 else
                     'right but on the wrong end (front/back): check the yaw'))
        else:
            print(f'   no new points within {NEAR} m: the object is hidden by the wheels/motors, move it further '
                  'from the side (more toward the ' + ('front' if s == 'front' else 'back') + ')')
    if not args.object:
        (HERE / 'logs').mkdir(exist_ok=True)
        np.savez(baseline, **pts)
        print(f'   baseline saved: {baseline} (for --object)')
    print('== 3. front vs rear: nearest return per 2 deg direction both see (median over bins)')
    near = {s: nearest_by_direction(pts[s], center, 2.0) for s in SIDES}
    both = sorted(set(near['front']) & set(near['rear']))
    if both:
        diff = np.array([near['front'][b] - near['rear'][b] for b in both])
        good = np.abs(diff) < 0.3  # same surface; larger = one sees something the other cannot
        print(f'   {len(both)} shared directions, {good.sum()} on the same surface (|diff| < 30 cm): '
              f'median |diff| {np.median(np.abs(diff[good])) * 100 if good.any() else float("nan"):.1f} cm, '
              f'median diff (front - rear) {np.median(diff[good]) * 100 if good.any() else float("nan"):+.1f} cm')
    else:
        print('   no direction seen by both')

    print('== 4. floor check: median nearest return per 15 deg sector from the body center [m] (front / rear)')
    sec = {s: nearest_by_direction(pts[s], center, 15.0) for s in SIDES}
    labels = {0: 'back', 6: 'right', 12: 'front', 18: 'left'}
    row = []
    for b in range(24):
        f, r = sec['front'].get(b, np.nan), sec['rear'].get(b, np.nan)
        deg = b * 15 - 180 + 7.5
        row.append(f'{deg:+6.1f} {labels.get(b, ""):>5}: {f:5.2f} / {r:5.2f}')
    for i in range(0, 24, 2):
        print('   ' + row[i] + '    ' + row[i + 1])

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8), dpi=130)
    fig.patch.set_facecolor('#fcfcfb'); ax.set_facecolor('#fcfcfb')
    for s, col in (('front', '#eb6834'), ('rear', '#2a78d6')):
        ax.plot(pts[s][:, 0], pts[s][:, 1], '.', ms=1.5, color=col, label=f'{s} LiDAR')
        t = tfs[s][0]
        ax.plot(t[0], t[1], marker='o', ms=8, color=col, mec='#0b0b0b')
    ax.add_patch(plt.Circle(center, BODY_R, fill=False, ls='--', color='#52514e', label='body filter 0.25 m'))
    ax.arrow(0, 0, 0.2, 0, width=0.01, color='#0b0b0b')
    ax.text(0.22, 0.02, 'front (+x)', color='#0b0b0b', fontsize=9)
    ax.text(0.02, -0.28, 'right (-y)', color='#52514e', fontsize=9)
    for m in obj or []:
        ax.plot(*m, marker='x', ms=12, color='#d03b3b')
    lim = 3.0
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect('equal')
    ax.grid(color='#e4e3df', lw=0.8); ax.set_axisbelow(True)
    ax.set_xlabel('x [m] (base_link)'); ax.set_ylabel('y [m]')
    ax.set_title('LiDAR returns in base_link (3D TF), 3 m around the robot', fontsize=11, loc='left')
    ax.legend(frameon=False, fontsize=9, loc='upper right', markerscale=6)
    (HERE / 'logs').mkdir(exist_ok=True)
    out = HERE / 'logs' / f'{datetime.now():%Y%m%d-%H%M%S}_lidar_check.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'== plot {out}')
    node.destroy_node(); rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
