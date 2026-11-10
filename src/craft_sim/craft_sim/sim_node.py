#!/usr/bin/env python3
"""craft 2D 시뮬레이터 노드: 가짜 바퀴 OpenCR 보드(PTY) + 참값 센서.

실제 control_master 는 이 노드가 만든 PTY 를 clock_wheel_port 로 열고, 실물 보드와 같은 CSV 를 주고받는다.
  host → 보드: wl, wr [rad/s], clock [motor rad]
  보드 → host (20 ms): wl, wr, clock, wheel_state, clock_state, imu_ok, gx, gy, gz [rad/s], ax, ay, az [g]
명령한 바퀴 속도로 차체를 이상적으로 움직이고, 플래닝이 기대하는 /perception/* 입력을 참값으로 발행한다.
"""
from __future__ import annotations

import argparse
import math
import os
import pty
import select
import signal
import sys
import threading
import time
import tty
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from craft_interfaces.msg import BoxObservation, BoxObservationRequest, TaskCommand

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from craft_sim.world import (DiffDrive, GridMap, ObstacleFilter, box_geometry, boxes_from_planning,  # noqa: E402
                             lidar_box_fit, load_params, obstacles_from_scenario, wrap)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def quat(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return q


class FakeWheelBoard:
    """PTY 한 쪽을 쥐고 있는 가짜 바퀴+clock OpenCR."""

    def __init__(self, command_timeout_s=0.25):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        self.pending = b''
        self.wl = self.wr = 0.0
        self.clock = None
        self.last_cmd = None
        self.had_cmd = False
        self.timeout = command_timeout_s
        self.lines_in = 0
        self.dropped = 0

    def poll(self):
        while select.select([self.master], [], [], 0)[0]:
            try:
                chunk = os.read(self.master, 4096)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break
            self.pending += chunk
        while b'\n' in self.pending:
            line, self.pending = self.pending.split(b'\n', 1)
            line = line.strip()
            if not line or line.startswith((b'IGAIN,', b'TOL,', b'TIME,')):
                continue
            try:
                v = [float(x) for x in line.split(b',')]
            except ValueError:
                continue
            if len(v) >= 3 and all(math.isfinite(x) for x in v[:3]):
                self.wl, self.wr, self.clock = v[0], v[1], v[2]
                self.last_cmd = time.monotonic()
                self.had_cmd = True
                self.lines_in += 1

    def state(self, now):
        """(wheel_state, clock_state): 0 정지, 1 이동, 2 명령 만료. 펌웨어처럼 만료되면 바퀴 0, 두 축 모두 2.
        바퀴 정지 허용 0.05 rad/s (firmware/wheel_clock/config.h)."""
        if self.had_cmd and now - self.last_cmd > self.timeout:
            self.wl = self.wr = 0.0
            return 2, 2
        return (1 if abs(self.wl) > 0.05 or abs(self.wr) > 0.05 else 0), 0

    def send(self, values):
        line = ','.join(f'{x:.6f}' if isinstance(x, float) else str(x) for x in values) + '\n'
        try:
            os.write(self.master, line.encode())
            return True
        except (BlockingIOError, OSError):
            self.dropped += 1
            return False

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


class Sim(Node):
    def __init__(self, scenario: dict, craft_src: Path, map_yaml: Path):
        super().__init__('craft_sim')
        self.sc = scenario
        opt = scenario.get('sim', {}) or {}
        self.planning, self.control = load_params(craft_src)
        sep = float(self.control['skid_steer_factor']) * float(self.control['wheel_track'])
        x0, y0, yaw0 = scenario['start']
        self.robot = DiffDrive(float(self.control['wheel_radius']), sep, (float(x0), float(y0), math.radians(float(yaw0))))
        self.map = GridMap(map_yaml)
        self.boxes = boxes_from_planning(self.planning, {k: (v[0], v[1], v[2]) for k, v in
                                                         (scenario.get('box_offsets') or {}).items()})
        self.obstacles = obstacles_from_scenario(scenario.get('obstacles'))
        self.front_face_x = float(self.planning['box_approach']['front_face_x'])
        self.cam_max = float(self.planning['box_approach'].get('camera_operational_gap_max', 0.2))
        self.scan_beams = int(opt.get('scan_beams', 360))
        self.scan_max = float(opt.get('lidar_max_range', 6.0))
        self.cam_hz = float(opt.get('camera_hz', 15.0))
        self.lidar_unc = float(opt.get('lidar_gap_uncertainty', 0.010))
        self.cam_unc = float(opt.get('camera_gap_uncertainty', 0.003))
        # 실주행 재현용: 가까워질수록 LiDAR 간격이 이동량보다 덜 줄어듦. [기준 간격 m, 기울기] (예 [0.53, 0.024] = 0.43 m 이동에 10 mm)
        self.lidar_gap_drift = opt.get('lidar_gap_drift')
        # 실측 카메라 재현(2026-11-09 bag): 촬영→도착 지연, 2프레임 무효 묶음(nonfinite camera geometry)
        self.cam_latency = float(opt.get('camera_latency_s', 0.0))
        self.cam_burst_rate = float(opt.get('camera_burst_rate_hz', 0.0))
        self.cam_burst_frames = int(opt.get('camera_burst_frames', 2))
        self.cam_rng = __import__('random').Random(int(opt.get('camera_seed', 1)))
        self.cam_burst_left = 0
        self.cam_queue = __import__('collections').deque()
        # 실측: 이동 중 100 mm 이상에서는 상면 검출이 대부분 실패(18:07 lidar_dock 208프레임 중 유효 21). 0 = 끔
        self.cam_valid_max_gap = float(opt.get('camera_valid_max_gap', 0.0))
        # base_link box of the chassis from the URDF base_link mesh (2026-11-10): x -0.141..0.225, y +-0.177
        self.footprint = opt.get('footprint', [-0.141, 0.225, -0.177, 0.177])
        # 실제 LiDAR 장애물 필터(perception.yaml lidar_obstacle)와 같은 기준
        self.obst_range = float(opt.get('obstacle_max_range', 3.0))
        self.wall_margin = float(opt.get('wall_margin', 0.06))
        self.boxes_as_obstacles = bool(opt.get('boxes_as_obstacles', True))
        ba = self.planning['box_approach']
        self.front_lidar_x = float(ba.get('front_lidar_x', 0.155))
        self.rear_lidar_x = float(opt.get('rear_lidar_x', -0.1295))   # lidar_processor.launch.py 기본값
        self.box_lidar_range = tuple(opt.get('box_lidar_range', (0.10, 1.0)))  # uppercam_sensor_policy.json
        self.filter = ObstacleFilter()
        self.pose_lock = threading.Lock()
        self.event_lock = threading.Lock()
        self.last_task = None
        self.map.render_dynamic(self.obstacles, self.boxes.values() if self.boxes_as_obstacles else ())
        self.map.exclude_boxes_from_collision(list(self.boxes.values()) + list(boxes_from_planning(self.planning).values()))
        self.board = FakeWheelBoard()
        self.lock = threading.Lock()
        self.request: BoxObservationRequest | None = None
        self.t_goal = None  # 첫 목표(이동 요청)를 받은 monotonic 시각: 장애물 시간 기준
        self.trail = PathMsg()
        self.trail.header.frame_id = 'map'
        self.collisions = 0

        # 발행
        self.pub_odom = self.create_publisher(Odometry, '/perception/state/odom', 10)
        self.pub_truth = self.create_publisher(Odometry, '/sim/truth', 10)
        self.pub_lodom = self.create_publisher(Odometry, '/perception/lidar/odom', 10)
        self.pub_imu = self.create_publisher(Imu, '/perception/imu/data', qos_profile_sensor_data)
        self.pub_loc = self.create_publisher(String, '/perception/localization/state', 10)
        self.pub_obst = self.create_publisher(PoseArray, '/perception/lidar/obstacle', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/lidar/merged', qos_profile_sensor_data)
        self.pub_box = self.create_publisher(BoxObservation, '/perception/box/observation', 10)
        self.pub_cam = self.create_publisher(BoxObservation, '/perception/uppercam/observation', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/planning/map', LATCHED)
        self.pub_markers = self.create_publisher(MarkerArray, '/sim/markers', 10)
        self.pub_trail = self.create_publisher(PathMsg, '/sim/trail', 10)
        self.pub_event = self.create_publisher(String, '/sim/events', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)

        # 구독
        self.create_subscription(BoxObservationRequest, '/planning/box_observation_request', self.on_request, LATCHED)
        for topic, typ in (('/planning/target_base_pose', PoseStamped), ('/planning/approach_box', String),
                           ('/planning/approach_box_near', String)):
            self.create_subscription(typ, topic, self.on_goal, 10)
        self.create_subscription(TaskCommand, '/planning/task_command', self.on_task, 10)

        self.publish_static()
        self.publish_map()
        self.create_timer(0.2, self.publish_markers)
        self.create_timer(0.5, self.publish_trail)
        self.create_timer(1.0, lambda: self.pub_loc.publish(String(data='ok')))
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'가짜 바퀴 보드 PTY: {self.board.port}')

    # ------------------------------------------------------------------ 입력
    def on_request(self, msg: BoxObservationRequest):
        with self.lock:
            self.request = msg
        self.event(f'관측 요청 #{msg.request_id} source={msg.source} box={msg.box_id}', type='request',
                   source=msg.source, request_id=int(msg.request_id))

    def on_task(self, msg: TaskCommand):
        # 실제 필터처럼 새 Navigate task 의 EXECUTE 가 장애물 기억을 지운다.
        if msg.operation == TaskCommand.EXECUTE and msg.kind == TaskCommand.NAVIGATE and msg.task_id != self.last_task:
            self.last_task = msg.task_id
            with self.lock:
                self.filter.clear()

    def on_goal(self, _msg):
        if self.t_goal is None:
            self.t_goal = time.monotonic()
            self.event('첫 이동 요청 수신: 장애물 시간 기준 시작', type='t0')

    def event(self, text, **data):
        """사람이 읽는 text + 기계가 읽는 data 를 JSON 한 줄로 /sim/events 에 낸다."""
        import json
        self.get_logger().info(text)
        with self.event_lock:
            self.pub_event.publish(String(data=json.dumps({'t': time.time(), 'text': text, **data}, ensure_ascii=False)))

    # ------------------------------------------------------------------ 주기 루프 (50 Hz)
    def loop(self):
        try:
            self._loop()
        except Exception as e:  # 루프가 조용히 죽으면 보드가 멈춘 것처럼 보이므로 크게 남긴다
            import traceback
            self.get_logger().error(f'시뮬레이터 루프 예외: {e!r}\n{traceback.format_exc()}')
            self.event(f'시뮬레이터 루프 예외: {e!r}', type='error')

    def _loop(self):
        period, tick = 0.02, 0
        last = time.monotonic()
        next_t = last + period
        next_cam = last
        while self.running and rclpy.ok():
            now = time.monotonic()
            dt = now - last
            last = now
            self.board.poll()
            wstate, cstate = self.board.state(now)
            wl, wr = self.board.wl, self.board.wr
            with self.pose_lock:
                self.robot.step(wl, wr, dt)
                w = self.robot.w
            clock = self.board.clock if self.board.clock is not None else float(self.control.get('clock_zero_angle', 0.0))
            if not self.board.send([wl, wr, clock, wstate, cstate, 1, 0.0, 0.0, w, 0.0, 0.0, 1.0]) and \
                    self.board.dropped in (1, 10, 100):
                self.event(f'PTY 피드백 쓰기 실패 {self.board.dropped}회', type='warn')
            stamp = self.get_clock().now().to_msg()
            self.publish_pose(stamp)
            self.update_obstacles()
            if tick % 5 == 0:
                self.publish_scan_and_lidar(stamp)
            if now >= next_cam:
                next_cam = now + 1.0 / self.cam_hz
                self.publish_camera(stamp)
            self.flush_camera()
            if tick % 25 == 0:
                self.check_collision()
            tick += 1
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            next_t += period
            if next_t < time.monotonic() - period:
                next_t = time.monotonic() + period

    def publish_pose(self, stamp):
        with self.pose_lock:
            x, y, th = self.robot.pose
            v, w = self.robot.v, self.robot.w
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'map', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = quat(th)
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        for i in (0, 7, 35):
            o.pose.covariance[i] = 1e-6
            o.twist.covariance[i] = 1e-6
        self.pub_odom.publish(o)
        self.pub_truth.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = quat(th)
        self.tf.sendTransform(t)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        imu.orientation = quat(th)
        imu.angular_velocity.z = w
        imu.linear_acceleration.z = 9.80665
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = imu.angular_velocity_covariance[8] = 1e-6
        imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] = imu.linear_acceleration_covariance[8] = 1e-4
        self.pub_imu.publish(imu)

    def publish_static(self):
        ts = []
        for parent, child, x in (('map', 'odom', 0.0), ('base_link', 'laser_merged', 0.0)):  # 병합 스캔은 회전 중심 기준
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            ts.append(t)
        self.tf_static.sendTransform(ts)

    def publish_map(self):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info = MapMetaData(resolution=self.map.res, width=self.map.w, height=self.map.h)
        g.info.origin.position.x, g.info.origin.position.y = self.map.origin
        g.info.origin.orientation.w = 1.0
        g.data = self.map.occupancy_msg_data()
        self.pub_map.publish(g)

    # ------------------------------------------------------------------ 장애물·스캔
    def update_obstacles(self):
        t_run = None if self.t_goal is None else time.monotonic() - self.t_goal
        x, y, _ = self.robot.pose
        changed = False
        for ob in self.obstacles:
            if ob.update(t_run, (x, y)):
                changed = True
                self.event(f'장애물 {ob.name} {"나타남" if ob.active else "사라짐"} (t_run={t_run if t_run is not None else 0:.1f}s)',
                           type='obstacle', name=ob.name, active=ob.active)
        if changed:
            self.map.render_dynamic(self.obstacles, self.boxes.values() if self.boxes_as_obstacles else ())

    def raycast_lidar(self, pose, mount_x, facing):
        """한 라이다(base_link x=mount_x, 바깥 방향 facing=0 앞/π 뒤)의 180° 스캔: 원점, 광선 각(world), 거리, 동적 히트."""
        x, y, th = pose
        ox, oy = x + mount_x * math.cos(th), y + mount_x * math.sin(th)
        rel = np.radians(np.arange(-90, 91, 360.0 / self.scan_beams))
        ang = th + facing + rel
        rng, dyn = self.map.raycast(ox, oy, ang, self.scan_max)
        return ox, oy, ang, rng, dyn

    def publish_scan_and_lidar(self, stamp):
        with self.pose_lock:
            pose = self.robot.pose
            v, w = self.robot.v, self.robot.w
        x, y, th = pose
        scans = [self.raycast_lidar(pose, self.front_lidar_x, 0.0), self.raycast_lidar(pose, self.rear_lidar_x, math.pi)]
        # RViz 용 병합 스캔: base_link 원점 기준 각도 칸(실제 병합도 회전 중심 기준)
        n = self.scan_beams
        merged = np.full(n, np.inf)
        cand = []
        for ox, oy, ang, rng, dyn in scans:
            hit = np.isfinite(rng)
            hx, hy = ox + rng[hit] * np.cos(ang[hit]), oy + rng[hit] * np.sin(ang[hit])
            bx, by = np.cos(th) * (hx - x) + np.sin(th) * (hy - y), -np.sin(th) * (hx - x) + np.cos(th) * (hy - y)
            k = np.round((np.arctan2(by, bx) + math.pi) / (2 * math.pi / n)).astype(int) % n
            np.minimum.at(merged, k, np.hypot(bx, by))
            sel = dyn & (rng <= self.obst_range)
            if sel.any():
                cand.append(np.column_stack((ox + rng[sel] * np.cos(ang[sel]), oy + rng[sel] * np.sin(ang[sel]))))
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser_merged'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / n
        scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
        scan.range_min, scan.range_max = 0.05, self.scan_max
        scan.scan_time, scan.time_increment = 0.1, 0.0
        scan.ranges = [float(r) if np.isfinite(r) else float('inf') for r in merged]
        self.pub_scan.publish(scan)

        # 장애물 점: 맵에 없는 물체(장애물·박스)에 맞은 점 → 맵 벽 wall_margin 이내 제거 → 실제 필터 단계
        hits = np.vstack(cand) if cand else np.zeros((0, 2))
        if len(hits):
            hits = hits[self.map.wall_distance(hits[:, 0], hits[:, 1]) >= self.wall_margin]
        rays = [(ox, oy, ang, np.where(np.isfinite(rng), rng, self.scan_max)) for ox, oy, ang, rng, _ in scans]
        with self.lock:
            allp = self.filter.step(hits, rays)
        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = stamp, 'map'
        for px, py in allp:
            p = Pose()
            p.position.x, p.position.y = float(px), float(py)
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_obst.publish(pa)

        lo = Odometry()
        lo.header.stamp, lo.header.frame_id, lo.child_frame_id = stamp, 'map', 'base_link'
        lo.pose.pose.position.x, lo.pose.pose.position.y = x, y
        lo.pose.pose.orientation = quat(th)
        lo.twist.twist.linear.x, lo.twist.twist.angular.z = v, w
        for i in (0, 7, 35):
            lo.pose.covariance[i] = 1e-4
            lo.twist.covariance[i] = 1e-4
        self.pub_lodom.publish(lo)
        self.publish_lidar_box(stamp, pose)

    # ------------------------------------------------------------------ 박스 관측
    def target_box(self, req):
        if req.box_id in self.boxes:
            return self.boxes[req.box_id]
        p = req.expected_box.position
        return min(self.boxes.values(), key=lambda b: math.hypot(b.x - p.x, b.y - p.y))

    def publish_lidar_box(self, stamp, pose):
        """요청이 lidar 일 때, 실제 브리지 fit_box_face 와 같은 계약으로 참값 관측(접선 중심 = 요청 prior)."""
        with self.lock:
            req = self.request
        if req is None or req.source != 'lidar':
            return
        box = self.target_box(req)
        q = req.expected_box.orientation
        eyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # As the bridge: the expected yaw, then turned 40 deg either way (a box turned past the 20 deg slope limit).
        for turn in (0.0, math.radians(40), -math.radians(40)):
            fit, why = lidar_box_fit(pose, box, (req.expected_box.position.x, req.expected_box.position.y, eyaw + turn),
                                     req.box_side, req.front_face_x, self.front_lidar_x, *self.box_lidar_range)
            if fit is not None:
                break
        m = BoxObservation()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        m.request_id, m.source = req.request_id, 'lidar'
        m.extrapolated = False
        m.gap_uncertainty_m = self.lidar_unc
        if fit is None:
            m.valid, m.reason = False, why
            m.gap_uncertainty_m = 1.0
        else:
            m.valid, m.reason = True, 'lidar face fit (sim truth)'
            m.box_pose.position.x, m.box_pose.position.y = fit['box_center_map']
            m.box_pose.orientation = quat(fit['box_yaw'])
            if self.lidar_gap_drift:
                ref, slope = float(self.lidar_gap_drift[0]), float(self.lidar_gap_drift[1])
                d = slope * max(0.0, ref - fit['gap_m'])
                cx, cy = fit['box_center_map']
                ux, uy = cx - pose[0], cy - pose[1]
                n = math.hypot(ux, uy) or 1.0
                m.box_pose.position.x, m.box_pose.position.y = cx + d * ux / n, cy + d * uy / n
                fit = dict(fit, gap_m=fit['gap_m'] + d)
            m.gap_m, m.lateral_m, m.yaw_rad = fit['gap_m'], fit['lateral_m'], fit['yaw_rad']
            m.face_centre_observed, m.face_centre_uncertainty_m = False, 0.0
        self.pub_box.publish(m)

    def publish_camera(self, stamp):
        with self.lock:
            req = self.request
        if req is None or req.source != 'uppercam':
            return
        with self.pose_lock:
            pose = self.robot.pose
        box = self.target_box(req)
        g = box_geometry(pose, box, self.front_face_x)
        m = BoxObservation()
        m.header.stamp, m.header.frame_id = stamp, 'base_link'
        m.request_id, m.source = 0, 'uppercam'
        ok = 0.0 <= g['gap_m'] <= self.cam_max   # 실제 정책: 간격 0~200 mm (uppercam_bridge_utils)
        m.valid = bool(ok)
        m.reason = 'sim truth' if ok else 'box outside camera range'
        m.gap_m, m.lateral_m, m.yaw_rad = g['gap_m'], g['lateral_m'], g['yaw_rad']
        # 센서 브리지와 같이 base_link 기준 박스 중심
        cx = self.front_face_x + g['gap_m'] + box.side * 0.5 * math.cos(g['yaw_rad'])
        cy = g['lateral_m'] + box.side * 0.5 * math.sin(g['yaw_rad'])
        m.box_pose.position.x, m.box_pose.position.y = cx, cy
        m.box_pose.orientation = quat(g['yaw_rad'])
        m.extrapolated = False
        m.gap_uncertainty_m = self.cam_unc
        m.face_centre_observed, m.face_centre_uncertainty_m = True, 0.003
        if self.cam_valid_max_gap > 0 and m.valid and g['gap_m'] > self.cam_valid_max_gap:
            m.valid, m.reason, m.gap_m, m.lateral_m, m.yaw_rad = False, 'nonfinite camera geometry', 0.0, 0.0, 0.0
            m.gap_uncertainty_m = 1.0
        if self.cam_burst_left == 0 and self.cam_burst_rate > 0 and self.cam_rng.random() < self.cam_burst_rate / self.cam_hz:
            self.cam_burst_left = self.cam_burst_frames
        if self.cam_burst_left > 0:
            self.cam_burst_left -= 1
            m.valid, m.reason, m.gap_m, m.lateral_m, m.yaw_rad = False, 'nonfinite camera geometry', 0.0, 0.0, 0.0
            m.gap_uncertainty_m = 1.0
        if self.cam_latency > 0:
            self.cam_queue.append((time.monotonic() + self.cam_latency, m))
        else:
            self.pub_cam.publish(m)

    def flush_camera(self):
        now = time.monotonic()
        while self.cam_queue and self.cam_queue[0][0] <= now:
            self.pub_cam.publish(self.cam_queue.popleft()[1])

    # ------------------------------------------------------------------ 충돌·시각화
    def check_collision(self):
        with self.pose_lock:
            pose = self.robot.pose
        hit_map, hit_dyn = self.map.footprint_hits(pose, self.footprint)
        if hit_map or hit_dyn:
            self.collisions += 1
            if self.collisions == 1 or self.collisions % 10 == 0:
                self.event(f'충돌: 차체 사각형이 {"맵 벽" if hit_map else ""}{"·" if hit_map and hit_dyn else ""}'
                           f'{"장애물/박스" if hit_dyn else ""} 칸과 겹침 ({self.robot.x:.3f}, {self.robot.y:.3f})',
                           type='collision', map=hit_map, obstacle=hit_dyn, x=self.robot.x, y=self.robot.y)

    def publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def mk(ns, mid, typ, color, a=0.8):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
            m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(a))
            m.pose.orientation.w = 1.0
            return m

        for i, b in enumerate(self.boxes.values()):
            m = mk('boxes', i, Marker.CUBE, (0.85, 0.6, 0.2))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = b.x, b.y, 0.1
            m.pose.orientation = quat(b.yaw)
            m.scale.x = m.scale.y = b.side
            m.scale.z = 0.2
            ma.markers.append(m)
            t = mk('box_names', i, Marker.TEXT_VIEW_FACING, (1, 1, 1), 1.0)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = b.x, b.y, 0.3
            t.scale.z, t.text = 0.12, b.name
            ma.markers.append(t)
        for i, ob in enumerate(self.obstacles):
            typ = Marker.CYLINDER if ob.shape == 'circle' else Marker.CUBE
            m = mk('obstacles', i, typ, (0.9, 0.1, 0.1) if ob.active else (0.5, 0.5, 0.5), 0.8 if ob.active else 0.25)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = ob.center[0], ob.center[1], 0.1
            m.pose.orientation = quat(ob.yaw)
            if ob.shape == 'circle':
                m.scale.x = m.scale.y = 2 * ob.radius
            else:
                m.scale.x, m.scale.y = ob.size
            m.scale.z = 0.2
            ma.markers.append(m)
        with self.pose_lock:
            x, y, th = self.robot.pose
        fx0, fx1, fy0, fy1 = self.footprint
        body = mk('robot', 0, Marker.LINE_STRIP, (0.2, 0.8, 1.0), 1.0)
        body.scale.x = 0.01
        c, s = math.cos(th), math.sin(th)
        for a, b in ((fx0, fy0), (fx1, fy0), (fx1, fy1), (fx0, fy1), (fx0, fy0)):
            body.points.append(Point(x=x + c * a - s * b, y=y + s * a + c * b, z=0.02))
        ma.markers.append(body)
        arrow = mk('robot', 1, Marker.ARROW, (0.2, 0.8, 1.0), 1.0)
        arrow.pose.position.x, arrow.pose.position.y = x, y
        arrow.pose.orientation = quat(th)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.25, 0.03, 0.03
        ma.markers.append(arrow)
        # The scenario's name and description above the arena (2026-11-10: which test is this?).
        title = mk('scenario', 0, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 0.3), 1.0)
        title.pose.position.x, title.pose.position.y, title.pose.position.z = 2.2, 6.75, 0.5
        title.scale.z = 0.22
        title.text = str(self.sc.get('name', ''))
        ma.markers.append(title)
        desc = str(self.sc.get('description', '')).strip()
        if desc:
            words, lines, line = desc.split(), [], ''
            for w in words:
                if len(line) + len(w) + 1 > 46:
                    lines.append(line); line = w
                else:
                    line = (line + ' ' + w).strip()
            lines.append(line)
            text = mk('scenario', 1, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 1.0)
            text.pose.position.x, text.pose.position.y, text.pose.position.z = 2.2, 6.75 - 0.18 - 0.06 * len(lines), 0.5
            text.scale.z = 0.1
            text.text = '\n'.join(lines[:6])
            ma.markers.append(text)
        self.pub_markers.publish(ma)

    def publish_trail(self):
        with self.pose_lock:
            x, y, th = self.robot.pose
        poses = self.trail.poses
        if not poses or math.hypot(poses[-1].pose.position.x - x, poses[-1].pose.position.y - y) > 0.01:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation = quat(th)
            poses.append(p)
        self.trail.header.stamp = self.get_clock().now().to_msg()
        self.pub_trail.publish(self.trail)

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.board.close()


def wrap_arr(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--craft-src', required=True, help='로봇 코드의 src 폴더 (planning.yaml, control.yaml 위치)')
    ap.add_argument('--map', required=True, help='맵 yaml')
    ap.add_argument('--pty-out', required=True, help='PTY 경로를 쓸 파일')
    a = ap.parse_args()
    scenario = yaml.safe_load(Path(a.scenario).read_text())
    rclpy.init()
    node = Sim(scenario, Path(a.craft_src), Path(a.map))
    Path(a.pty_out).write_text(node.board.port)

    def stop(*_):
        node.running = False
        rclpy.try_shutdown()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
