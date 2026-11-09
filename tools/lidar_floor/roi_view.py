#!/usr/bin/python3
"""Live floor-ROI viewer: raw LiDAR scans -> the same cut as scan_merger.cpp -> what stays / what the floor ROI removes.

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/lidar_floor/roi_view.py          # with lidar_processor.launch.py (its merger has no ROI)
  RViz: tools/lidar_floor/roi_view.rviz

Publishes (base_link, 10 Hz, front + rear combined):
  /lidar/roi/kept           PointCloud2  points scan_merger keeps (FOV, body filter, floor ROI)
  /lidar/roi/floor_removed  PointCloud2  points dropped only by the floor ROI (in the FOV, outside the body)
  /lidar/roi/region         Marker       outline of the kept area per LiDAR side (to 3 m)
Settings come from src/craft_perception/config/perception.yaml lidar (front/rear_fov_deg, fov_near_m, body_radius_m,
body_center_x, range_max, floor_roi), read at start; restart after editing it. LiDAR mounts as floor_common.py.
"""
import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from floor_common import FRONT, MOUNT, REAR, REPO

SIDES = {FRONT: ('front_left', 'front_right'), REAR: ('rear_left', 'rear_right')}


def lidar_config():
    cfg = yaml.safe_load(open(REPO / 'src/craft_perception/config/perception.yaml'))

    def find(d):
        if isinstance(d, dict):
            if 'floor_roi' in d and 'body_center_x' in d:
                return d
            for v in d.values():
                r = find(v)
                if r is not None:
                    return r
        return None
    return find(cfg)


class RoiView(Node):
    def __init__(self):
        super().__init__('lidar_roi_view')
        c = lidar_config()
        self.bc, self.body_r, self.fov_near = float(c['body_center_x']), float(c['body_radius_m']), float(c['fov_near_m'])
        self.range_max = float(c['range_max'])
        self.half_fov = {FRONT: math.radians(float(c['front_fov_deg'])) / 2, REAR: math.radians(float(c['rear_fov_deg'])) / 2}
        roi = c['floor_roi']
        self.roi_on, self.step = bool(roi['enabled']), float(roi['step_deg'])
        self.side = {n: (float(roi[f'{n}_clear_deg']), [float(v) for v in roi[f'{n}_max_range_m']])
                     for t in SIDES for n in SIDES[t]}
        self.get_logger().info(f'floor_roi enabled={self.roi_on}: ' + ', '.join(f'{n} {v[0]:.0f} deg' for n, v in self.side.items()))
        self.latest = {}
        for t in (FRONT, REAR):
            self.create_subscription(LaserScan, t, lambda m, t=t: self.on_scan(t, m), qos_profile_sensor_data)
        self.pub_kept = self.create_publisher(PointCloud2, '/lidar/roi/kept', 5)
        self.pub_floor = self.create_publisher(PointCloud2, '/lidar/roi/floor_removed', 5)
        self.pub_region = self.create_publisher(Marker, '/lidar/roi/region', 1)
        self.create_timer(0.1, self.publish)
        self.create_timer(1.0, self.publish_region)

    def cut(self, topic, m):
        """-> (kept xy, floor-removed xy) in base_link, same tests and order as ScanMerger::add_scan."""
        x0, sx, sy = MOUNT[topic]
        rg = np.array(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg >= m.range_min) & (rg <= m.range_max)
        rg, a = rg[ok], a[ok]
        x, y = x0 + sx * rg * np.cos(a), sy * rg * np.sin(a)
        d, body_d = np.hypot(x, y), np.hypot(x - self.bc, y)
        keep = (d <= self.range_max) & (body_d >= self.body_r)
        outward = math.atan2(0.0, x0 - self.bc)                              # +x for the front, -x for the rear
        dev = np.abs(np.remainder(np.arctan2(y, x - x0) - outward + math.pi, 2 * math.pi) - math.pi)
        keep &= ~((dev > self.half_fov[topic]) & (body_d < self.fov_near))
        floor = np.zeros(len(x), bool)
        if self.roi_on:
            deg = np.degrees(dev)
            for name, s in zip(SIDES[topic], (y >= 0, y < 0)):
                clear, ranges = self.side[name]
                inb = s & (deg > clear) & (deg <= 90.0)
                b = np.clip(((deg - clear) / self.step).astype(int), 0, len(ranges))   # < clear: unused (inb)
                lim = np.array(ranges + [-1.0])[b]
                floor |= keep & inb & (rg > lim)
        keep &= ~floor
        return np.c_[x[keep], y[keep]], np.c_[x[floor], y[floor]]

    def on_scan(self, topic, m):
        self.latest[topic] = (self.get_clock().now(), *self.cut(topic, m))

    def cloud(self, pts):
        h = Header(frame_id='base_link', stamp=self.get_clock().now().to_msg())
        return point_cloud2.create_cloud_xyz32(h, [(float(px), float(py), 0.0) for px, py in pts])

    def publish(self):
        now = self.get_clock().now()
        fresh = [v for v in self.latest.values() if (now - v[0]).nanoseconds < 5e8]
        if not fresh:
            return
        self.pub_kept.publish(self.cloud(np.vstack([v[1] for v in fresh])))
        self.pub_floor.publish(self.cloud(np.vstack([v[2] for v in fresh])))

    def publish_region(self, show=3.0):
        mk = Marker(header=Header(frame_id='base_link', stamp=self.get_clock().now().to_msg()), ns='roi', id=0,
                    type=Marker.LINE_LIST, action=Marker.ADD)
        mk.scale.x = 0.01
        mk.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.8)
        for topic in (FRONT, REAR):
            x0, sx, _ = MOUNT[topic]
            for name, sd in zip(SIDES[topic], (1.0, -1.0)):
                clear, ranges = self.side[name] if self.roi_on else (90.0, [])
                segs, lo = [(0.0, clear, show)], clear
                for r in ranges:
                    if lo >= 90 - 1e-9:
                        break
                    segs.append((lo, min(lo + self.step, 90.0), min(r, show)))
                    lo += self.step
                pts = [(x0, 0.0)]
                for a0, a1, r in segs:
                    for ad in np.linspace(a0, a1, max(2, int(a1 - a0) + 1)):
                        t = math.radians(ad)
                        pts.append((x0 + sx * r * math.cos(t), sd * r * math.sin(t)))
                pts.append((x0, 0.0))
                for p, q in zip(pts[:-1], pts[1:]):
                    mk.points += [Point(x=p[0], y=p[1]), Point(x=q[0], y=q[1])]
        self.pub_region.publish(mk)


def main():
    rclpy.init()
    try:
        rclpy.spin(RoiView())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
