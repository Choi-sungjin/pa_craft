#!/usr/bin/python3
"""Box detections -> RViz squares (/perception/box/markers, visualization_msgs/MarkerArray, map frame).

  grey   : the box planning asked for (/planning/box_observation_request: expected_box, box_side)
  green  : the last valid LiDAR detection (/perception/box/observation, source lidar)
  blue   : the last valid upper-cam detection (/perception/uppercam/observation)
  dots   : every valid detection centre since the last request (green LiDAR, blue camera): a wall taken for the
           box shows up as dots away from the box
  text   : box id, source and valid / why not, above the expected box
  names  : every box of planning.yaml box_approach (S1..S6, L1..L3) at its place, once a second
An invalid detection removes that source's square. Works on a bag replay too:
  ros2 run craft_bringup box_marker.py   (with ros2 bag play --clock: --ros-args -p use_sim_time:=true)
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from craft_interfaces.msg import BoxObservation, BoxObservationRequest
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

GREY = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
COLOR = {'lidar': ColorRGBA(r=0.1, g=0.85, b=0.2, a=1.0), 'uppercam': ColorRGBA(r=0.15, g=0.45, b=1.0, a=1.0)}
ID = {'expected': 0, 'lidar': 1, 'uppercam': 2, 'text': 3, 'lidar_dots': 4, 'uppercam_dots': 5}


def box_places():
    """{name: (x, y)} from planning.yaml box_approach (names + [x, y, yaw] per name)."""
    path = os.path.join(get_package_share_directory('craft_planning'), 'config', 'planning.yaml')
    found = {}

    def walk(d):
        if isinstance(d, dict):
            if 'names' in d:
                found.update({n: d[n][:2] for n in d['names'] if isinstance(d.get(n), list)})
            for v in d.values():
                walk(v)
    with open(path) as f:
        walk(yaml.safe_load(f))
    return found


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


def square(x, y, yaw, side, z=0.02):
    c, s, h = math.cos(yaw), math.sin(yaw), side / 2
    corners = [(h, h), (-h, h), (-h, -h), (h, -h), (h, h)]
    return [Point(x=x + c * a - s * b, y=y + s * a + c * b, z=z) for a, b in corners]


class BoxMarker(Node):
    def __init__(self):
        super().__init__('box_marker')
        self.side = 0.205  # until a request says (planning.yaml box_approach side)
        self.box_id = ''
        self.status = {}
        self.dots = {'lidar': [], 'uppercam': []}
        self.pub = self.create_publisher(MarkerArray, '/perception/box/markers', 10)
        self.create_subscription(BoxObservationRequest, '/planning/box_observation_request', self.on_request, 10)
        self.create_subscription(BoxObservation, '/perception/box/observation', lambda m: self.on_obs(m, 'lidar'), 10)
        self.create_subscription(BoxObservation, '/perception/uppercam/observation',
                                 lambda m: self.on_obs(m, 'uppercam'), 10)
        self.names = []
        for i, (name, (x, y)) in enumerate(sorted(box_places().items())):
            m = Marker(ns='box_names', id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, text=name)
            m.header.frame_id = 'map'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.25
            m.pose.orientation.w = 1.0
            m.scale.z = 0.12
            m.color = ColorRGBA(r=1.0, g=0.85, b=0.2, a=1.0)
            self.names.append(m)
        self.create_timer(1.0, lambda: self.pub.publish(MarkerArray(markers=self.names)))  # RViz may join late

    def marker(self, name, header, kind=Marker.LINE_STRIP):
        m = Marker(ns='box', id=ID[name], type=kind, action=Marker.ADD)
        m.header.frame_id = header.frame_id or 'map'
        m.header.stamp = header.stamp
        m.pose.orientation.w = 1.0
        return m

    def on_request(self, req):
        if req.source == 'off':
            return  # the end of a box step: keep what was seen last
        if req.box_side > 0:
            self.side = req.box_side
        if req.box_id != self.box_id:
            self.dots = {'lidar': [], 'uppercam': []}
            self.status = {}
        self.box_id = req.box_id
        p = req.expected_box.position
        m = self.marker('expected', req.header)
        m.scale.x = 0.01
        m.color = GREY
        m.points = square(p.x, p.y, yaw_of(req.expected_box.orientation), self.side, z=0.01)
        self.expected = (p.x, p.y)
        self.pub.publish(MarkerArray(markers=[m, self.text(req.header)]))

    def text(self, header):
        m = self.marker('text', header, Marker.TEXT_VIEW_FACING)
        x, y = getattr(self, 'expected', (0.0, 0.0))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.35
        m.scale.z = 0.08
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = '\n'.join([self.box_id] + [f'{k}: {v}' for k, v in sorted(self.status.items())])
        return m

    def on_obs(self, obs, source):
        out = []
        box = self.marker(source, obs.header)
        p = obs.box_pose.position
        valid = obs.valid and all(math.isfinite(v) for v in (p.x, p.y)) and (p.x, p.y) != (0.0, 0.0)
        if valid:
            box.scale.x = 0.015
            box.color = COLOR[source]
            box.points = square(p.x, p.y, yaw_of(obs.box_pose.orientation), self.side, z=0.03)
            dots = self.dots[source]
            dots.append(Point(x=p.x, y=p.y, z=0.03))
            del dots[:-300]
            self.status[source] = f'ok ({p.x:.3f}, {p.y:.3f}) {math.degrees(yaw_of(obs.box_pose.orientation)):.0f} deg'
        else:
            box.action = Marker.DELETE
            self.status[source] = f'invalid: {obs.reason}'
        d = self.marker(f'{source}_dots', obs.header, Marker.POINTS)
        d.scale.x = d.scale.y = 0.02
        d.color = COLOR[source]
        d.points = list(self.dots[source])
        if not d.points:
            d.action = Marker.DELETE
        out += [box, d, self.text(obs.header)]
        self.pub.publish(MarkerArray(markers=out))


def main():
    rclpy.init()
    node = BoxMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
