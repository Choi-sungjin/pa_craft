#!/usr/bin/python3
"""Detected pucks -> flat, map-fixed RViz footprints and explicit attribute labels."""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from craft_perception.msg import PuckObservationArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray


def attributes(label):
    color = next((v for v in ('red', 'green') if v in label.split('_')), 'unknown')
    shape = 'cylinder' if 'cylinder' in label else 'prism' if ('prism' in label or 'cuboid' in label) else 'unknown'
    return color, shape


def rotate(q, xyz):
    x, y, z = xyz
    tx, ty, tz = 2 * (q.y*z-q.z*y), 2 * (q.z*x-q.x*z), 2 * (q.x*y-q.y*x)
    return (x+q.w*tx+q.y*tz-q.z*ty, y+q.w*ty+q.z*tx-q.x*tz, z+q.w*tz+q.x*ty-q.y*tx)


def outline(shape, posture):
    # Nominal dimensions: diameter/width 30 mm, length/height 60 mm.
    if shape == 'unknown' or posture == 'unknown':
        return [(0.02, 0), (0, .02), (-.02, 0), (0, -.02), (.02, 0)]
    if shape == 'cylinder':
        if posture == 'standing':
            return [(.015*math.cos(i*math.tau/40), .015*math.sin(i*math.tau/40)) for i in range(41)]
        pts = []
        for cx, start in ((.015, -math.pi/2), (-.015, math.pi/2)):
            pts += [(cx+.015*math.cos(start+i*math.pi/20), .015*math.sin(start+i*math.pi/20)) for i in range(21)]
        return pts + [pts[0]]
    half = .03 if posture == 'lying' else .015
    return [(half,.015),(-half,.015),(-half,-.015),(half,-.015),(half,.015)]


def markers_for(source, index, frame, obs, center, axis, z, lifetime, color_label=None):
    color, shape = attributes(obs.detection.label)
    if color == 'unknown' and color_label:
        color = attributes(color_label)[0]
    posture = 'unknown' if 'posture_unknown' in obs.detection.label else 'standing' if obs.standing else 'lying'
    angle_known = math.hypot(axis[0], axis[1]) > .1
    yaw = math.atan2(axis[1], axis[0]) if angle_known else 0.
    c, s = math.cos(yaw), math.sin(yaw)
    rgb = {'red': (1., .12, .10), 'green': (.1, .9, .25), 'unknown': (.65, .65, .65)}[color]
    ring = outline(shape, posture)
    points = [Point(x=center[0]+c*x-s*y, y=center[1]+s*x+c*y, z=z) for x,y in ring]
    fill = Marker(ns=source+'/footprint', id=index, type=Marker.TRIANGLE_LIST, action=Marker.ADD)
    fill.header.frame_id = frame; fill.pose.orientation.w = 1.
    fill.scale.x = fill.scale.y = fill.scale.z = 1.
    fill.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=.85)
    for a,b in zip(points,points[1:]):
        fill.points += [Point(x=center[0],y=center[1],z=z),a,b]
    edge = Marker(ns=source+'/outline', id=index, type=Marker.LINE_STRIP, action=Marker.ADD)
    edge.header.frame_id = frame; edge.pose.orientation.w = 1.; edge.scale.x = .002
    edge.color = ColorRGBA(r=1.,g=1.,b=1.,a=1.); edge.points = points
    label = Marker(ns=source+'/label', id=index, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
    label.header.frame_id = frame; label.pose.orientation.w = 1.
    label.pose.position = Point(x=center[0], y=center[1]+.045, z=z+.01)
    label.scale.z = .022; label.color = ColorRGBA(r=1.,g=1.,b=1.,a=1.)
    identity = f'#{obs.puck_id}' if obs.tag_valid and obs.puck_id >= 0 else f'@{index}'
    label.text = f'{identity} {posture} | {color} {shape}\n{source} {obs.detection.confidence:.0%}'
    if posture == 'lying' and not angle_known:
        label.text += ' | yaw unknown'
    for m in (fill,edge,label):
        m.lifetime = Duration(seconds=lifetime).to_msg()
    return [fill,edge,label]


class PuckMarker(Node):
    def __init__(self):
        super().__init__('puck_marker')
        self.frame = self.declare_parameter('fixed_frame', 'map').value
        self.z = float(self.declare_parameter('plane_z', .04).value)
        self.live_ttl = float(self.declare_parameter('live_timeout_s', 3.).value)
        self.shot_ttl = float(self.declare_parameter('snapshot_timeout_s', 30.).value)
        if min(self.live_ttl, self.shot_ttl) <= 0:
            raise ValueError('marker timeouts must be positive')
        self.tf = Buffer(); self.listener = TransformListener(self.tf,self)
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/perception/pucks/markers', qos)
        self.pending, self.views = {}, {}
        sources = {
            'uppercam': '/perception/vision/uppercam/puck_survey_observations',
            'gripcam': '/perception/vision/d435/puck_survey_observations',
            'picker_upper': '/perception/pucks/picker_upper',
            'picker_grip': '/perception/pucks/picker_grip',
        }
        for source, topic in sources.items():
            self.create_subscription(PuckObservationArray,topic,lambda m,s=source:self.receive(s,m),10)
        self.create_timer(.1,self.refresh)

    def receive(self, source, msg):
        self.pending.pop(source,None)
        if msg.observations:
            self.pending[source] = (msg,self.get_clock().now().nanoseconds)
        self.views.pop(source,None)  # a new empty/invalid observation must clear old detections

    def refresh(self):
        now = self.get_clock().now().nanoseconds
        for source,(msg,received) in list(self.pending.items()):
            ttl = self.shot_ttl if source.startswith('picker_') else self.live_ttl
            if (now-received)*1e-9 > ttl or now < received:
                self.pending.pop(source); self.views.pop(source,None); continue
            frame = msg.header.frame_id
            if frame == 'camera_color_optical_frame':
                frame = 'camera_wrist_color_optical_frame'  # project D435 is the gripper camera
            try:
                transform = None if frame == self.frame else self.tf.lookup_transform(self.frame,frame,Time.from_msg(msg.header.stamp)).transform
            except TransformException as e:
                self.get_logger().warning(f'puck display waiting for {frame} -> {self.frame}: {e}', throttle_duration_sec=5.)
                continue  # retry at the acquisition stamp; never place an untransformed point at the origin
            items = []
            for i,o in enumerate(msg.observations):
                if not o.axis_valid:
                    continue  # these messages have no measured 3D centre
                center,axis = (o.center.x,o.center.y,o.center.z),(o.axis.x,o.axis.y,o.axis.z)
                if not all(math.isfinite(v) for v in (*center,*axis)):
                    continue
                if transform:
                    center = rotate(transform.rotation,center)
                    t = transform.translation
                    center = (center[0]+t.x,center[1]+t.y,center[2]+t.z)
                    axis = rotate(transform.rotation,axis)
                items.append((i,o,center,axis))
            self.views[source] = (items,received,ttl)
            self.pending.pop(source)
        out = [Marker(action=Marker.DELETEALL)]
        upper_view = self.views.get('picker_upper',([],0,0))
        upper = upper_view[0] if 0 <= (now-upper_view[1])*1e-9 <= upper_view[2] else []
        detailed = self.views.get('picker_grip',([],0,0))
        detailed_items = detailed[0] if 0 <= (now-detailed[1])*1e-9 <= detailed[2] else []
        for source,(items,received,ttl) in list(self.views.items()):
            if (now-received)*1e-9 > ttl or now < received:
                self.views.pop(source); continue
            for i,o,center,axis in items:
                if source == 'picker_upper' and any(
                    attributes(d[1].detection.label)[1] == attributes(o.detection.label)[1] and
                    math.hypot(d[2][0]-center[0],d[2][1]-center[1]) < .06 for d in detailed_items):
                    continue  # the confirmed gripper view replaces this coarse footprint
                match = None
                if source == 'picker_grip' and upper:
                    candidate = min(upper,key=lambda v: math.hypot(v[2][0]-center[0],v[2][1]-center[1]))
                    if attributes(candidate[1].detection.label)[1] == attributes(o.detection.label)[1] and math.hypot(candidate[2][0]-center[0],candidate[2][1]-center[1]) < .06:
                        match = candidate[1].detection.label
                out += markers_for(source,i,self.frame,o,center,axis,self.z,ttl,match)
        self.pub.publish(MarkerArray(markers=out))


def main():
    rclpy.init(); node = PuckMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
