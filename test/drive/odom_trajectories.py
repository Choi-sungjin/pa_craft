#!/usr/bin/env python3
"""Wheel odometry vs LiDAR odometry vs EKF (vs SLAM) trajectories from one rosbag, overlaid.

Record a loop that ends where it started (tape mark, same heading), lidar_odom:=true:
  ros2 bag record -o <bag> --topics /control/wheel/odom /perception/lidar/odom /odometry/local \\
      /perception/lidar/pose
Each source starts at the origin heading +x (its own frame rotated/shifted onto the first pose), so the
shapes compare directly. Back on the mark the true loop error is 0: each source's end-to-start distance and
heading error is its drift over the loop. Missing topics are skipped. Plot: <bag>_trajectories.png.

  odom_trajectories.py <bag dir>
"""
import argparse
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

warnings.filterwarnings('ignore', message='Use read_next_ext', category=DeprecationWarning)  # stamps unused
SOURCES = [  # topic, label, color (categorical slots 1-4), dashed
    ('/control/wheel/odom', 'wheel odometry', '#eb6834', True),
    ('/perception/lidar/odom', 'LiDAR odometry', '#1baf7a', True),
    ('/odometry/local', 'EKF (odom)', '#2a78d6', False),
    ('/perception/lidar/pose', 'SLAM (map)', '#0b0b0b', False),
]


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {t: [] for t, *_ in SOURCES if t in types}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in out:
            m = deserialize_message(data, types[topic])
            p, q = m.pose.pose.position, m.pose.pose.orientation  # Odometry and PoseWithCovarianceStamped alike
            out[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y,
                               2 * math.atan2(q.z, q.w)))
    return {t: np.array(sorted(v)) for t, v in out.items() if len(v) > 1}


def normalized(track):
    """x, y, yaw with the first pose at the origin heading +x."""
    t, x, y, yaw = track.T
    c, s = math.cos(-yaw[0]), math.sin(-yaw[0])
    dx, dy = x - x[0], y - y[0]
    return t, c * dx - s * dy, s * dx + c * dy, np.unwrap(yaw) - yaw[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', type=Path)
    args = ap.parse_args()
    tracks = read(args.bag)
    if not tracks:
        sys.exit(f'{args.bag}: none of {[t for t, *_ in SOURCES]}')
    print(f'{"source":>15} {"samples":>7} {"path m":>7} {"end-start m":>11} {"heading deg":>11}')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8), dpi=130)
    fig.patch.set_facecolor('#fcfcfb'); ax.set_facecolor('#fcfcfb')
    for topic, label, color, dashed in SOURCES:
        if topic not in tracks:
            print(f'{label:>15}  (not in the bag)')
            continue
        _, x, y, yaw = normalized(tracks[topic])
        path = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
        turns = round(yaw[-1] / (2 * math.pi))  # a loop may end after whole turns
        heading = math.degrees(yaw[-1] - turns * 2 * math.pi)
        print(f'{label:>15} {len(x):7d} {path:7.2f} {math.hypot(x[-1], y[-1]):11.3f} {heading:+11.1f}')
        ax.plot(x, y, color=color, lw=2, ls='--' if dashed else '-', label=label)
        ax.plot(x[-1], y[-1], 'o', color=color, ms=7)
    ax.plot(0, 0, marker='s', color='#52514e', ms=8, ls='none', label='start (= true end)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m] (start heading)', color='#0b0b0b'); ax.set_ylabel('y [m]', color='#0b0b0b')
    ax.set_title(f'Odometry trajectories ({args.bag.name}); dots = end', color='#0b0b0b', fontsize=11, loc='left')
    ax.grid(color='#e4e3df', lw=0.8); ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors='#52514e')
    ax.legend(frameon=False, fontsize=9, loc='best')
    out = args.bag.parent / f'{args.bag.name}_trajectories.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'[traj] plot {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
