#!/usr/bin/python3
"""Odometry -> persistent point trail for RViz (RViz's Odometry display only draws arrows/axes and drops them).

Every pose at least min_spacing from the last kept one is added; the whole trail goes out as one
visualization_msgs/Marker POINTS in the odometry's frame (latched, so a late or re-toggled RViz gets it all).
In a map-fixed RViz the trail moves with map->odom: it shows odometry, drift included.
  ros2 run craft_bringup odom_trail.py --ros-args -p odom_topic:=/odometry/local -p trail_topic:=/odometry/local/trail
"""
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class OdomTrail(Node):
    def __init__(self):
        super().__init__('odom_trail')
        odom_topic = self.declare_parameter('odom_topic', '/odometry/local').value
        trail_topic = self.declare_parameter('trail_topic', '/odometry/local/trail').value
        self.spacing = self.declare_parameter('min_spacing', 0.02).value  # [m]
        self.max_points = self.declare_parameter('max_points', 100000).value  # oldest dropped beyond this
        period = 1.0 / self.declare_parameter('publish_hz', 5.0).value
        self.marker = Marker(ns='odom_trail', id=0, type=Marker.POINTS, action=Marker.ADD)
        self.marker.scale.x = self.marker.scale.y = self.declare_parameter('point_size', 0.03).value
        self.marker.color = ColorRGBA(r=42 / 255, g=120 / 255, b=214 / 255, a=1.0)
        self.marker.pose.orientation.w = 1.0
        self.changed = False
        self.pub = self.create_publisher(Marker, trail_topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # EKF publishes RELIABLE; a best-effort subscription matches either.
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(period, self.publish)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        if self.marker.header.frame_id and msg.header.frame_id != self.marker.header.frame_id:
            self.marker.points.clear()  # a different odometry frame: start over
        self.marker.header.frame_id = msg.header.frame_id
        last = self.marker.points[-1] if self.marker.points else None
        if last is not None and math.hypot(p.x - last.x, p.y - last.y) < self.spacing:
            return
        self.marker.points.append(Point(x=p.x, y=p.y, z=p.z))
        if len(self.marker.points) > self.max_points:
            del self.marker.points[:len(self.marker.points) - self.max_points]
        self.changed = True

    def publish(self):
        if not self.changed:
            return
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)
        self.changed = False


def main():
    rclpy.init()
    node = OdomTrail()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # a SIGTERM shutdown can surface as an RCL error from the wait set
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
