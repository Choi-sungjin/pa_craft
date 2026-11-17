#!/usr/bin/env python3
"""PTY OpenCR and raw, upside-down LiDAR simulator. No production state is published."""
import json
import math
import os
import pty
import select
import signal
import tempfile
import threading
import time
import tty
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
EXTRINSICS = {'front': (.203, 0., .021, math.radians(175.7)),
              'rear': (-.0815, 0., .021, math.radians(-4.5))}
N = 720


def segments_from_grid(occupied, resolution, origin):
    """Merge cell edges on the occupied/free boundary into long axis-aligned segments."""
    a = np.pad(occupied, 1, constant_values=False)
    h, w = occupied.shape
    segments = []
    def append_runs(row, fixed, horizontal):
        d = np.diff(np.r_[False, row, False].astype(np.int8))
        for start, end in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)):
            if horizontal: segments.append((start, fixed, end, fixed))
            else: segments.append((fixed, start, fixed, end))
    horizontal = a[:-1, 1:-1] != a[1:, 1:-1]
    vertical = a[1:-1, :-1] != a[1:-1, 1:]
    for y in range(h + 1): append_runs(horizontal[y], y, True)
    for x in range(w + 1): append_runs(vertical[:, x], x, False)
    v = np.array(segments, dtype=float).reshape(-1, 4) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    for i in (0, 2):
        x, y = v[:, i].copy(), v[:, i + 1].copy()
        v[:, i] = origin[0] + c*x - s*y
        v[:, i + 1] = origin[1] + s*x + c*y
    return v


def box_segments(boxes, side=.205):
    edges = []
    for x, y, yaw in boxes:
        a = math.radians(yaw); c, s = math.cos(a), math.sin(a)
        corners = [(-side/2, -side/2), (side/2, -side/2), (side/2, side/2), (-side/2, side/2)]
        p = [(x+c*u-s*v, y+s*u+c*v) for u,v in corners]
        edges.extend((*p[i], *p[(i+1)%4]) for i in range(4))
    return np.array(edges, dtype=float).reshape(-1, 4)


def load_world(map_file, planning_file):
    with open(map_file) as f: m = yaml.safe_load(f)
    im = np.asarray(Image.open(Path(map_file).parent / m['image']).convert('L'), dtype=float)
    p = im/255. if m.get('negate', 0) else (255.-im)/255.
    occupied = np.flipud(p > m['occupied_thresh'])
    with open(planning_file) as f: plan = yaml.safe_load(f)['planning_master']['ros__parameters']['box_approach']
    names = plan['names']
    boxes = [plan[name] for name in names]
    return World(occupied, m['resolution'], m['origin'], box_segments(boxes)), dict(zip(names, boxes))


class World:
    def __init__(self, occupied, resolution, origin, boxes):
        self.grid = occupied; self.res = float(resolution); self.origin = origin
        self.boxes = boxes

    def cast(self, ox, oy, dx, dy):
        # Half-cell samples detect every normal wall, and stay within one centimetre
        # of its raster boundary. Boxes use exact intersections below.
        d = raycast(self.boxes, ox, oy, dx, dy)
        active = np.arange(len(ox)); h, w = self.grid.shape
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        for distance in np.arange(.1, 8.001, min(.01, self.res / 2)):
            if not len(active): break
            x = ox[active] + distance*dx[active] - self.origin[0]
            y = oy[active] + distance*dy[active] - self.origin[1]
            ix = np.floor((c*x+s*y)/self.res).astype(int)
            iy = np.floor((-s*x+c*y)/self.res).astype(int)
            valid = (ix>=0) & (iy>=0) & (ix<w) & (iy<h)
            hit = np.zeros(len(active), dtype=bool)
            hit[valid] = self.grid[iy[valid], ix[valid]]
            d[active[hit]] = np.minimum(d[active[hit]], distance)
            active = active[~hit & (d[active] > distance)]
        return d


def raycast(segments, ox, oy, dx, dy, max_range=8.):
    """Exact ray/segment intersections, batched to keep memory bounded."""
    ox, oy, dx, dy = np.broadcast_arrays(ox, oy, dx, dy)
    out = np.full(ox.size, np.inf)
    for start in range(0, len(segments), 256):
        seg = segments[start:start+256]
        sx, sy = seg[:, 0], seg[:, 1]
        vx, vy = seg[:, 2]-sx, seg[:, 3]-sy
        px, py = sx[None,:]-ox[:,None], sy[None,:]-oy[:,None]
        cross = dx[:,None]*vy - dy[:,None]*vx
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (px*vy-py*vx)/cross
            u = (px*dy[:,None]-py*dx[:,None])/cross
        good = (np.abs(cross)>1e-12) & (t >= .1) & (t <= max_range) & (u >= 0) & (u <= 1)
        out = np.minimum(out, np.min(np.where(good, t, np.inf), axis=1, initial=np.inf))
    return out


def scan_geometry(poses, ext):
    poses = np.asarray(poses)
    x, y, yaw = poses[:,0], poses[:,1], poses[:,2]
    ex, ey, _, eyaw = ext
    ox = x + np.cos(yaw)*ex - np.sin(yaw)*ey
    oy = y + np.sin(yaw)*ex + np.cos(yaw)*ey
    # Both sensors have roll=pi: positive raw angle rotates clockwise in base_link.
    angle = yaw + eyaw - np.arange(len(poses))*2*math.pi/N
    return ox, oy, np.cos(angle), np.sin(angle)


class Board:
    def __init__(self, control, start, seed=16):
        self.radius = float(control['wheel_radius']); track = float(control['wheel_track'])
        factor = float(control['skid_steer_factor'])
        self.separation = track * (factor if factor > 0 else 1+(float(control['wheelbase'])/track)**2)
        self.max_wheel = float(control['max_wheel'])
        self.pose = np.array(start, dtype=float); self.vel = np.zeros(3); self.cmd = np.zeros(3)
        self.last_cmd = 0.; self.clock = 0.; self.lock = threading.Lock()
        self.history = deque(maxlen=100); self.rng = np.random.default_rng(seed)
        self.master, self.slave = pty.openpty(); tty.setraw(self.slave)
        os.set_blocking(self.master, False); self.port = os.ttyname(self.slave)
        self.running = True; self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _command(self, line, now):
        try:
            vals = [float(x) for x in line.decode('ascii').strip().split(',')]
            if len(vals) != 3 or not np.all(np.isfinite(vals)): return
            if abs(vals[0]) > self.max_wheel or abs(vals[1]) > self.max_wheel: return
            with self.lock: self.cmd[:] = vals; self.last_cmd = now
        except (ValueError, UnicodeError): pass  # configuration and malformed lines are ignored

    def _run(self):
        buffer = b''; last = time.monotonic(); deadline = last
        while self.running:
            now = time.monotonic(); dt = min(now-last, .1); last = now
            try:
                while select.select([self.master], [], [], 0)[0]:
                    data = os.read(self.master, 4096)
                    if not data: break
                    buffer += data
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1); self._command(line, now)
                    if len(buffer) > 8192: buffer = b''
            except (BlockingIOError, OSError): pass
            with self.lock:
                wl, wr = self.cmd[:2] if now-self.last_cmd <= .5 else (0., 0.)
                v = self.radius*(wl+wr)/2; w = self.radius*(wr-wl)/self.separation
                yaw = self.pose[2]; mid = yaw + w*dt/2
                self.pose += [v*math.cos(mid)*dt, v*math.sin(mid)*dt, w*dt]
                self.clock = self.cmd[2] if now-self.last_cmd <= .5 else self.clock
                self.vel[:] = [v, 0., w]; self.history.append((now, *self.pose))
                noise = self.rng.normal(0, .00003, 5)
                fields = [wl, wr, self.clock, 0, 0, 1, noise[0], noise[1], w+noise[2], noise[3], noise[4], 1.]
            try: os.write(self.master, (','.join(f'{v:.7f}' for v in fields)+'\n').encode())
            except (BlockingIOError, OSError): pass
            deadline += .02
            if deadline < time.monotonic()-.02: deadline = time.monotonic()
            time.sleep(max(0., deadline-time.monotonic()))

    def samples(self, end):
        with self.lock:
            h = np.array(self.history); pose = self.pose.copy(); vel = self.vel.copy(); age = time.monotonic()-self.last_cmd if self.last_cmd else float('inf')
        if not len(h): return np.tile(pose, (N,1)), pose, vel, age
        # Driver timestamp is the end of the scan; index zero is the newest beam.
        ts = end - np.arange(N)*(.1/N)
        angles = np.unwrap(h[:,3]); result = np.column_stack([np.interp(ts,h[:,0],h[:,1]), np.interp(ts,h[:,0],h[:,2]), np.interp(ts,h[:,0],angles)])
        return result, pose, vel, age

    def close(self):
        self.running = False; self.thread.join(timeout=1)
        os.close(self.master); os.close(self.slave)


def write_ports(path, port):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix='.ports_')
    try:
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump({'control_master': {'ros__parameters': {'clock_wheel_port': port, 'arm_enabled': False}}}, f)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    # Background shells can inherit SIG_IGN for SIGINT; install explicit handlers
    # for both launcher cleanup and ordinary terminal shutdown.
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    class Sim(Node):
        def __init__(self):
            super().__init__('mission_hardware_sim')
            defaults = {'world_map': '/home/craft/maps/noboxes_legend_opt_arena.yaml', 'planning_yaml': str(ROOT/'src/craft_planning/config/planning.yaml'), 'control_yaml': str(ROOT/'src/craft_control/config/control.yaml'), 'ports_file': '/tmp/mission_sim/ports.yaml', 'start': [0.648, 1., math.pi], 'seed': 16}
            for name, value in defaults.items(): self.declare_parameter(name, value)
            get = lambda name: self.get_parameter(name).value
            self.world, self.boxes = load_world(get('world_map'), get('planning_yaml'))
            with open(get('control_yaml')) as f: control = yaml.safe_load(f)['control_master']['ros__parameters']
            self.board = Board(control, get('start'), get('seed')); self.rng = np.random.default_rng(get('seed'))
            self.ports_file = get('ports_file'); write_ports(self.ports_file, self.board.port)
            self.scan_pubs = {side: self.create_publisher(LaserScan, f'/lidar/{side}/scan', qos_profile_sensor_data) for side in EXTRINSICS}
            self.truth_pub = self.create_publisher(Odometry, '/mission_sim/truth', 10)
            self.state_pub = self.create_publisher(String, '/mission_sim/state', 10)
            self.broadcaster = StaticTransformBroadcaster(self); tf = []
            for side, (x,y,z,yaw) in EXTRINSICS.items():
                t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id='base_link'; t.child_frame_id='laser_'+side
                t.transform.translation.x=x; t.transform.translation.y=y; t.transform.translation.z=z
                t.transform.rotation.x=math.cos(yaw/2); t.transform.rotation.y=math.sin(yaw/2)
                t.transform.rotation.z=0.; t.transform.rotation.w=0.
                tf.append(t)
            self.broadcaster.sendTransform(tf); self.scan_count=0; self.started=time.monotonic(); self.scan_duration=0.
            self.create_timer(.02, self.truth); self.create_timer(1., self.state)
            self.stop_scans = threading.Event()
            self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True); self.scan_thread.start()

        def truth(self):
            _, pose, vel, _ = self.board.samples(time.monotonic())
            msg=Odometry(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='map'; msg.child_frame_id='mission_sim_truth'
            msg.pose.pose.position.x=float(pose[0]); msg.pose.pose.position.y=float(pose[1]); msg.pose.pose.orientation.z=math.sin(pose[2]/2); msg.pose.pose.orientation.w=math.cos(pose[2]/2)
            msg.twist.twist.linear.x=float(vel[0]); msg.twist.twist.angular.z=float(vel[2]); self.truth_pub.publish(msg)

        def scan_loop(self):
            deadline=time.monotonic()
            while not self.stop_scans.is_set():
                self.scans(); deadline += .1
                if deadline < time.monotonic()-.1: deadline = time.monotonic()
                self.stop_scans.wait(max(0., deadline-time.monotonic()))

        def scans(self):
            begin=time.monotonic(); poses, _, _, _=self.board.samples(begin); stamp=self.get_clock().now().to_msg()
            for side, ext in EXTRINSICS.items():
                ox,oy,dx,dy=scan_geometry(poses,ext); ranges=self.world.cast(ox,oy,dx,dy)
                good=np.isfinite(ranges); ranges[good]=np.clip(ranges[good]+self.rng.normal(0,.003,good.sum()),.1,8.)
                ranges[self.rng.random(N)<.01]=np.inf
                msg=LaserScan(); msg.header.stamp=stamp; msg.header.frame_id='laser_'+side
                msg.angle_min=0.; msg.angle_increment=2*math.pi/N; msg.angle_max=2*math.pi
                msg.scan_time=.1; msg.time_increment=.1/N; msg.range_min=.1; msg.range_max=100.; msg.ranges=ranges.tolist()
                self.scan_pubs[side].publish(msg)
            self.scan_count+=1; self.scan_duration=time.monotonic()-begin

        def state(self):
            _, pose, vel, age=self.board.samples(time.monotonic()); elapsed=time.monotonic()-self.started
            data={'pose': pose.tolist(), 'velocity': vel.tolist(), 'serial_command_age_s': age if math.isfinite(age) else None, 'scan_pairs': self.scan_count, 'raw_scan_hz': self.scan_count/elapsed, 'scan_pair_duration_s': self.scan_duration, 'laser_extrinsics': EXTRINSICS, 'clock_wheel_port': self.board.port, 'boxes':self.boxes}
            msg=String(); msg.data=json.dumps(data); self.state_pub.publish(msg)
            report=Path(self.ports_file).parent/'hw_report.json'; report.write_text(json.dumps(data, indent=2))

    node=None
    try:
        node=Sim(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node is not None:
            node.stop_scans.set(); node.scan_thread.join(); node.board.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
