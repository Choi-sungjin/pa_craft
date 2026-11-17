#!/usr/bin/env python3
"""Observe the real mission stack; only camera fixtures and startup are simulated."""
import json
import math
import os
import re
import time
from pathlib import Path as FSPath

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray
from craft_interfaces.msg import ControlStatus, TaskCommand, TaskResult
from craft_perception.msg import TrafficLight

ROOT = FSPath('/home/craft/craft')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

def as_text(value):
    return value.decode('utf-8', errors='replace') if isinstance(value, bytes) else str(value)

def as_uint(value):
    return int.from_bytes(value, 'little') if isinstance(value, bytes) else int(value)

class Monitor(Node):
    def __init__(self):
        super().__init__('mission_sim_monitor')
        for k, v in {'planning_yaml': str(ROOT/'src/craft_planning/config/planning.yaml'),
                     'report_file': str(ROOT/'demo/mission_sim/runs/report.json'), 'auto_start': True}.items():
            self.declare_parameter(k, v)
        self.report_file = as_text(self.get_parameter('report_file').value)
        self.auto_start = self.get_parameter('auto_start').value
        with open(as_text(self.get_parameter('planning_yaml').value), encoding='utf-8') as f:
            cfg = yaml.safe_load(f)['planning_master']['ros__parameters']['box_approach']
        self.boxes = {n: cfg[n] for n in ['S1','S2','S3','S4','S5','S6','L1','L2','L3']}
        self.started = time.monotonic(); self.ready_since = None; self.sent_start = False
        self.global_status = ''; self.control = None; self.mission = ''; self.planner = ''
        self.current_box = ''; self.visits = []; self.completed = []; self.box_status = []
        self.errors = []; self.diagnostics = {}; self.task_commands = 0; self.revisions = 0
        self.last_task_key = None; self.tracking_count = 0; self.box_pose_count = 0
        self.truth = None; self.estimate = None; self.error_samples = 0; self.error_sum = 0.; self.error_max = 0.
        self.truth_trail = Path(); self.truth_trail.header.frame_id = 'map'
        self.estimated_trail = Path(); self.estimated_trail.header.frame_id = 'map'
        self.wheel = [0., 0.]; self.wheel_time = time.monotonic(); self.last_report = 0.
        self.command_pub = self.create_publisher(String, '/mission/command', 10)
        self.signal_pub = self.create_publisher(TrafficLight, '/perception/vision/uppercam/traffic_light', LATCHED)
        self.barcode_pub = self.create_publisher(String, '/perception/vision/uppercam/barcode', LATCHED)
        self.markers_pub = self.create_publisher(MarkerArray, '/mission_sim/markers', 10)
        self.path_pub = self.create_publisher(Path, '/mission_sim/planned_path', LATCHED)
        self.estimated_pub = self.create_publisher(Path, '/mission_sim/estimated_trail', 10)
        self.truth_pub = self.create_publisher(Path, '/mission_sim/truth_trail', 10)
        self.joints_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.summary_pub = self.create_publisher(String, '/mission_sim/summary', 10)
        sub = self.create_subscription
        sub(String, '/perception/localization/global', self.on_global, LATCHED)
        sub(String, '/mission/state', self.on_mission, LATCHED)
        sub(String, '/planning/state', self.on_planner, 10)
        sub(String, '/planning/box_status', self.on_box, 10)
        sub(ControlStatus, '/control/status', self.on_control, 10)
        sub(TaskResult, '/control/task_result', self.on_result, 10)
        sub(Odometry, '/perception/state/odom', self.on_estimate, 10)
        sub(Odometry, '/mission_sim/truth', self.on_truth, 10)
        sub(TaskCommand, '/planning/task_command', self.on_task, 10)
        sub(Path, '/control/tracking_path', self.on_tracking, 10)
        sub(DiagnosticArray, '/planning/diagnostics', self.on_diagnostics, 10)
        sub(PoseStamped, '/perception/box/pose', self.on_box_pose, 10)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Camera-excluded fixtures: traffic light and barcode only; no simulated planner/control results.')

    def on_global(self, m): self.global_status = as_text(m.data)
    def on_control(self, m):
        self.control = m
        if m.fault: self.add_error('control fault: ' + as_text(m.reason))
    def on_mission(self, m):
        data = as_text(m.data)
        if data != self.mission:
            self.mission = data
            match = re.search(r':\s+\w+\s+([SL][1-6])\b', data)
            if match:
                box = match.group(1)
                if box != self.current_box:
                    self.current_box = box; self.visits.append(box)
            if data.startswith('Fault'): self.add_error('mission: ' + data)
    def on_planner(self, m):
        self.planner = as_text(m.data)
        if 'fault' in self.planner.lower() or 'failed' in self.planner.lower(): self.add_error('planner: ' + self.planner)
    def on_box(self, m):
        data = as_text(m.data)
        self.box_status.append(data); self.box_status = self.box_status[-100:]
        # A successful dock is observable, but is not asserted to be completed mission work.
        match = re.search(r'\[box ([SL][1-6])\] END (\w+)', data)
        if match and match.group(2).lower() in ('ok', 'success', 'succeeded', 'done', 'aligned'):
            self.completed.append(match.group(1))
    def on_result(self, m):
        status = as_uint(m.status)
        if status in (as_uint(m.FAILED), as_uint(m.REJECTED)): self.add_error(f'task {m.task_id} status {status}: {as_text(m.reason)}')
    def on_task(self, m):
        if as_uint(m.kind) != as_uint(m.NAVIGATE): return
        self.task_commands += 1
        key = (m.task_id, m.revision, as_uint(m.operation))
        if key != self.last_task_key:
            if as_uint(m.operation) == as_uint(m.UPDATE_PATH): self.revisions += 1
            self.last_task_key = key
        if m.path.poses: self.path_pub.publish(m.path)
    def on_tracking(self, m): self.tracking_count += 1
    def on_box_pose(self, m): self.box_pose_count += 1
    def on_diagnostics(self, m):
        self.diagnostics = {as_text(s.name): {'level': as_uint(s.level), 'message': as_text(s.message),
                            'values': {as_text(v.key): as_text(v.value) for v in s.values}} for s in m.status}
    def on_truth(self, m): self.truth = m
    def on_estimate(self, m): self.estimate = m
    def add_error(self, s):
        if s not in self.errors: self.errors.append(s)
    def add_trail(self, trail, odom):
        if odom is None or as_text(odom.header.frame_id) != 'map': return
        p = odom.pose.pose.position
        if trail.poses:
            q = trail.poses[-1].pose.position
            if math.hypot(p.x-q.x, p.y-q.y) < .03: return
        item = PoseStamped(); item.header = odom.header; item.pose = odom.pose.pose
        trail.poses.append(item)
        if len(trail.poses) > 3000: del trail.poses[:100]
    def marker(self, ident, text, x, y, z, scale, color):
        m = Marker(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mission'; m.id = ident; m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = float(x); m.pose.position.y = float(y); m.pose.position.z = float(z)
        m.pose.orientation.w = 1.; m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = color; m.text = text
        return m
    def tick(self):
        now = time.monotonic(); dt = min(now-self.wheel_time, .25); self.wheel_time = now
        ready = self.global_status.startswith('ok') and self.control is not None and self.control.ready and not self.control.fault and self.control.stopped
        if ready:
            if self.ready_since is None: self.ready_since = now
            if self.auto_start and not self.sent_start and now-self.ready_since >= 3.:
                self.command_pub.publish(String(data='START')); self.sent_start = True
        else: self.ready_since = None
        if self.mission.startswith('WatchSignal'):
            light = TrafficLight(); light.header.stamp = self.get_clock().now().to_msg()
            light.red = light.yellow = light.green = 4; light.sequence = [0,1,2]*4
            light.complete = light.valid = True; self.signal_pub.publish(light)
        if self.mission.startswith('ReadBarcode'): self.barcode_pub.publish(String(data='P2-01'))
        if self.truth:
            v = self.truth.twist.twist.linear.x; w = self.truth.twist.twist.angular.z
            self.wheel[0] += (v - w*.289/2)*dt/.033; self.wheel[1] += (v + w*.289/2)*dt/.033
        j = JointState(); j.header.stamp = self.get_clock().now().to_msg()
        j.name = ['lm_slide','shoulder','elbow','wrist_pitch','tool_roll','jaw_upper','jaw_lower','carousel','front_left_wheel','rear_left_wheel','front_right_wheel','rear_right_wheel']
        j.position = [0.]*8 + [self.wheel[0],self.wheel[0],self.wheel[1],self.wheel[1]]; self.joints_pub.publish(j)
        self.add_trail(self.estimated_trail, self.estimate); self.add_trail(self.truth_trail, self.truth)
        stamp = self.get_clock().now().to_msg(); self.estimated_trail.header.stamp = stamp; self.truth_trail.header.stamp = stamp
        self.estimated_pub.publish(self.estimated_trail); self.truth_pub.publish(self.truth_trail)
        if self.estimate and self.truth and as_text(self.estimate.header.frame_id) == as_text(self.truth.header.frame_id) == 'map':
            a = self.estimate.pose.pose.position; b = self.truth.pose.pose.position
            e = math.hypot(a.x-b.x, a.y-b.y); self.error_samples += 1; self.error_sum += e; self.error_max = max(self.error_max, e)
        ma = MarkerArray()
        for i, (name, (x,y,*_)) in enumerate(self.boxes.items()):
            ma.markers.append(self.marker(i, name, x, y, .65, .20, (1.,.8,.1,1.) if name == self.current_box else (.9,.9,.9,1.)))
        hud = ('MISSION:' + (self.mission or 'waiting') + '\nPLANNER:' + (self.planner or 'waiting') +
               '\nTARGET:' + (self.current_box or 'none') + '\nROUTE:' + '->'.join(self.visits) +
               '\nCAMERA_EXCLUDED_FIXTURES:traffic_light_barcode\nraw_LiDAR->perception->SLAM/EKF->planning->control->PTY').replace(' ', '_')
        ma.markers.append(self.marker(100, hud, 2.2, 6.8, 1.2, .14, (1.,1.,1.,1.))); self.markers_pub.publish(ma)
        if now-self.last_report >= 1.: self.write_report(); self.last_report = now
    def report(self):
        return {'mission_label': self.mission, 'planner_label': self.planner, 'current_box': self.current_box,
                'box_visits': self.visits, 'completed_docks_from_box_status': self.completed, 'box_status': self.box_status,
                'path_commands': self.task_commands, 'path_revisions': self.revisions, 'tracking_path_messages': self.tracking_count,
                'box_pose_messages': self.box_pose_count, 'localization_status': self.global_status,
                'localization_error_m': {'samples': self.error_samples, 'mean': self.error_sum/self.error_samples if self.error_samples else None,
                                         'max': self.error_max if self.error_samples else None, 'reference': 'private simulation truth, map frame'},
                'diagnostics': self.diagnostics, 'errors': self.errors, 'start_sent': self.sent_start,
                'camera_excluded_fixtures': ['traffic_light_4_4_4', 'barcode_P2-01']}
    def write_report(self):
        report = self.report()
        if rclpy.ok(): self.summary_pub.publish(String(data=json.dumps(report)))
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.report_file)), exist_ok=True)
            tmp = self.report_file + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f: json.dump(report, f, indent=2)
            os.replace(tmp, self.report_file)
        except OSError as e: self.get_logger().error('report write failed: ' + str(e))

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node = Monitor()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.write_report(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
