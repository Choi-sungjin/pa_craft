#!/usr/bin/env python3
"""Loop plan.yaml frames on /joint_states and draw the shelf, puck and front LiDAR in RViz (V186 URDF).

Visualization only: publishes /joint_states and /demo/markers, never talks to control or hardware.
"""
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
EXTRA = ['carousel', 'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']  # held at 0


class Show(Node):
    def __init__(self):
        super().__init__('v186_pick_demo')
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
        m.ns, m.id, m.type, m.action = 'v186_pick', mid, kind, Marker.ADD
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
        a = self.a; fz = a['floor_z']; sh = a['shelf_height']; sx, sy = a['shelf_size']
        px = a['lidar_face_x'] + a['puck_ahead']; d = a['puck_diameter']
        lx, ly, lz = a['lidar_center']
        self.mk.publish(MarkerArray(markers=[
            self.marker(0, Marker.CUBE, (px, a['puck_y'], fz + sh / 2), (sx, sy, sh), (0.55, 0.35, 0.15, 0.6)),
            self.marker(1, Marker.CYLINDER, f['puck'], (d, d, a['puck_height']), (0.9, 0.1, 0.1, 0.9)),
            self.marker(2, Marker.CUBE, (0.2, 0.0, fz - 0.001), (0.9, 0.7, 0.002), (0.6, 0.6, 0.6, 0.35)),
            self.marker(3, Marker.TEXT_VIEW_FACING, (lx, ly, lz + 0.05), (0, 0, 0.025), (0, 0, 0, 1), 'front LiDAR'),
            self.marker(4, Marker.TEXT_VIEW_FACING, (px, a['puck_y'], fz + sh + a['puck_height'] + 0.08),
                        (0, 0, 0.025), (0.6, 0, 0, 1), f"puck +{a['puck_ahead'] * 100:.0f} cm"),
            self.marker(5, Marker.CUBE, (a['lidar_face_x'], ly, lz), (0.001, 0.04, 0.03), (0.1, 0.4, 1.0, 0.8)),
        ]))


def main():
    rclpy.init()
    rclpy.spin(Show())


if __name__ == '__main__':
    main()
