"""Shared bits for the LiDAR floor tools: bag reading, TF, LiDAR mounts, map wall distance.

LiDAR mounts as lidar_processor.launch.py (2026-10-31, both upside down): front at base_link x 0.155 (laser x -> -x,
y -> y), rear at x -0.1295 (laser x -> x, y -> -y). Each laser's 0 deg points inward (at the other LiDAR), +-180
outward. The body filter is centred on the LiDAR midpoint, x 0.01275 (perception.yaml body_center_x).
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'demo/corner'))
from map_tools import MapGrid  # noqa: E402

FRONT, REAR = '/lidar/front/scan', '/lidar/rear/scan'
XF, XR, BC = 0.155, -0.1295, 0.01275
# topic -> (x0, sx, sy): base_x = x0 + sx * laser_x, base_y = sy * laser_y
MOUNT = {FRONT: (XF, -1, 1), REAR: (XR, 1, -1)}
LIDAR_H = {FRONT: 0.012, REAR: 0.011}   # floor_cal_0034: LiDAR beam height above a flat floor [m]


def bag_path(name):
    """'fov180_2359' -> ~/bags/fov180_2359; a path is used as it is."""
    p = os.path.expanduser(name)
    return p if os.path.exists(p) else os.path.expanduser('~/bags/' + name)


def read_bag(uri, topics=None):
    """Yield (topic, msg, t [s]) for the wanted topics (all when None)."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_path(uri), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {}
    while r.has_next():
        topic, data, t = r.read_next()
        if topics is not None and topic not in topics:
            continue
        if topic not in cls:
            cls[topic] = get_message(types[topic])
        yield topic, deserialize_message(data, cls[topic]), t / 1e9


def T2(tr):
    q = tr.transform.rotation
    return (tr.transform.translation.x, tr.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def comp(a, b):
    return (a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1],
            a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1], a[2] + b[2])


def scan_xy(m, rmax=None):
    """LaserScan -> (ranges, angles) of the valid beams (laser frame)."""
    rg = np.array(m.ranges)
    th = m.angle_min + np.arange(len(rg)) * m.angle_increment
    ok = np.isfinite(rg) & (rg > m.range_min) & (rg < (m.range_max if rmax is None else min(rmax, m.range_max)))
    return rg[ok], th[ok]


def to_base(topic, rg, th):
    x0, sx, sy = MOUNT[topic]
    return x0 + sx * rg * np.cos(th), sy * rg * np.sin(th)


class WallDist:
    """Distance from a map-frame point to the nearest occupied/unknown cell of the map (0 outside the map)."""

    def __init__(self, yaml_path):
        self.g = MapGrid(os.path.expanduser(yaml_path))

    def __call__(self, x, y):
        g = self.g
        x, y = np.asarray(x, float), np.asarray(y, float)
        c = ((x - g.ox) / g.res).astype(int)
        r = (g.h - (y - g.oy) / g.res).astype(int)
        ok = (r >= 0) & (r < g.h) & (c >= 0) & (c < g.w)
        out = np.zeros(x.shape)
        out[ok] = g.dist[r[ok], c[ok]]
        return out
