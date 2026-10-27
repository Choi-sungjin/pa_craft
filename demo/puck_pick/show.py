#!/usr/bin/env python3
"""Throwaway demo: loop plan.yaml frames on /joint_states and draw the puck / front LiDAR in RViz.

Visualization only: publishes /joint_states and /demo/markers, never talks to control or hardware.
Delete with: rm -rf ~/craft/demo
"""
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
EXTRA = ['carousel_joint', 'idler_joint']  # not part of the plan; keep them at 0


class Show(Node):
    def __init__(self):
        super().__init__('puck_pick_demo')
        plan = yaml.safe_load(open(HERE / 'plan.yaml'))
        self.frames, self.a = plan['frames'], plan['assume']
        if not self.frames:
            raise SystemExit('plan.yaml has no frames (run plan.py; some waypoint failed?)')
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.mk = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.i = 0
        self.create_timer(0.05, self.tick)  # 20 Hz, same as plan.py

    def marker(self, mid, kind, xyz, scale, rgba, text=''):
        m = Marker()
        m.header.frame_id = 'base_link'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'puck_pick', mid, kind, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        m.text = text
        return m

    def tick(self):
        f = self.frames[self.i]; self.i = (self.i + 1) % len(self.frames)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(f['joints']) + EXTRA
        msg.position = [float(v) for v in f['joints'].values()] + [0.0] * len(EXTRA)
        self.js.publish(msg)
        a = self.a; h = a['puck_height']; fz = a['floor_z']
        lz0, lz1 = a['lidar_z']
        lidar = (a['lidar_center_x'], a['lidar_center_y'], (lz0 + lz1) / 2)
        ahead = (a['lidar_center_x'], a['lidar_front_y'] - a['puck_ahead'], fz + 0.001)
        cx, cy = a['lidar_center_x'], a['lidar_center_y'] + 0.15
        sh = a['shelf_height']; sx, sy = a['shelf_size']
        self.mk.publish(MarkerArray(markers=[
            self.marker(5, Marker.CUBE, (ahead[0], ahead[1], fz + sh / 2), (sx, sy, sh), (0.55, 0.35, 0.15, 0.6)),
            self.marker(0, Marker.CUBE, f['puck'], a['puck_size'], (0.9, 0.1, 0.1, 0.9)),
            self.marker(1, Marker.CYLINDER, lidar, (0.04, 0.04, lz1 - lz0), (0.1, 0.1, 0.1, 0.35)),
            self.marker(2, Marker.CUBE, (cx, cy, fz - 0.001), (0.6, 0.8, 0.002), (0.6, 0.6, 0.6, 0.35)),
            self.marker(3, Marker.TEXT_VIEW_FACING, (lidar[0], lidar[1], lidar[2] + 0.05),
                        (0, 0, 0.03), (0, 0, 0, 1), 'front LiDAR (CAD)'),
            self.marker(4, Marker.TEXT_VIEW_FACING, (ahead[0], ahead[1], fz + sh + 0.10),
                        (0, 0, 0.03), (0.6, 0, 0, 1), f"puck +{a['puck_ahead']*100:.0f} cm"),
        ]))


def main():
    rclpy.init()
    rclpy.spin(Show())


if __name__ == '__main__':
    main()
