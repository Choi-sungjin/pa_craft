#!/usr/bin/python3
"""D435 color + aligned depth -> a session dir that puck_axis_offline and the CenterPose adapter both read.

The stack's realsense_processor keeps depth only in /dev/shm/craft_vision, so a ros2 bag never has it.
  --source shm  (default) stack running: read the shm slot named by /vision/d435/frame_meta
  --source rs   stack stopped: open the D435 directly with realsense_processor's settings
                (640x360 color, 424x240 depth aligned to color, 30 fps, depth unit 1 mm)

Session: <out>/<name>_<YYMMDD_HHMMSS>/{meta.json, frames.csv, frames/NNNNNN_color.png, NNNNNN_depth.png}
         shm mode also writes NNNNNN_scan.json (latest /lidar/front/scan) for the shelf <-> LiDAR match.
Keep the puck still during one session (stability is a ground-truth-free metric); one pose per session.

pyrealsense2 lives in ~/craft/venv-craft (not /usr/bin/python3); rclpy works there too.
Usage: source ~/craft/install/setup.bash
       ~/craft/venv-craft/bin/python3 record.py --name stand_a --label standing --frames 100
       ~/craft/venv-craft/bin/python3 record.py --name lie_30 --label lying --yaw-deg 30 --note "tag up, 35 cm"
"""
import argparse
import fcntl
import json
import mmap
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

W, H, FPS, DEPTH_W, DEPTH_H = 640, 360, 30, 424, 240


class Session:
    def __init__(self, args):
        self.dir = Path(args.out).expanduser() / f'{args.name}_{datetime.now():%y%m%d_%H%M%S}'
        (self.dir / 'frames').mkdir(parents=True)
        self.args = args
        self.csv = open(self.dir / 'frames.csv', 'w')
        self.csv.write('frame,stamp_ns,sequence\n')
        self.count = 0
        self.last = 0.0

    def write_meta(self, fx, fy, cx, cy, dist, w, h, depth_unit_m):
        a = self.args
        meta = dict(fx=fx, fy=fy, cx=cx, cy=cy, dist=list(dist), width=w, height=h, depth_unit_m=depth_unit_m,
                    source=a.source, label=a.label, yaw_deg=a.yaw_deg, note=a.note,
                    created=datetime.now().isoformat(timespec='seconds'))
        meta = {k: v for k, v in meta.items() if v is not None}  # OpenCV's JSON parser rejects null
        (self.dir / 'meta.json').write_text(json.dumps(meta, indent=1))

    def due(self):
        return time.monotonic() - self.last >= 1.0 / self.args.hz

    def save(self, bgr, depth16, stamp_ns, sequence, scan=None):
        self.last = time.monotonic()
        name = f'{self.count:06d}'
        cv2.imwrite(str(self.dir / 'frames' / f'{name}_color.png'), bgr)
        cv2.imwrite(str(self.dir / 'frames' / f'{name}_depth.png'), depth16)
        if scan is not None:  # latest front LiDAR scan, its own frame_id (no TF applied)
            (self.dir / 'frames' / f'{name}_scan.json').write_text(json.dumps(dict(
                frame_id=scan.header.frame_id,
                stamp_ns=scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec,
                angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                ranges=[round(r, 4) if np.isfinite(r) else None for r in scan.ranges])))
        self.csv.write(f'{name},{stamp_ns},{sequence}\n')
        self.csv.flush()
        self.count += 1
        if self.count % 10 == 0:
            print(f'{self.count} frames')
        return self.count >= self.args.frames


def record_shm(session):
    import rclpy
    from sensor_msgs.msg import CameraInfo, LaserScan
    from std_msgs.msg import UInt64MultiArray

    rclpy.init()
    node = rclpy.create_node('puck_compare_record')
    fd = open('/dev/shm/craft_vision', 'rb')
    shm = mmap.mmap(fd.fileno(), 0, prot=mmap.PROT_READ)
    state = {'info': None, 'done': False, 'scan': None}

    def on_info(msg):
        if state['info'] is None:
            state['info'] = msg

    def on_meta(msg):
        offset, w, h, sequence, stamp_ns, unit_nm = msg.data
        if state['info'] is None or state['done'] or not session.due():
            return
        if session.count == 0:
            k = state['info'].k
            session.write_meta(k[0], k[4], k[2], k[5], state['info'].d, w, h, unit_nm * 1e-9)
        color_bytes, depth_bytes = w * h * 3, w * h * 2
        fcntl.flock(fd, fcntl.LOCK_SH)
        try:
            slot = shm[offset:offset + color_bytes + depth_bytes + 8]
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
        if int.from_bytes(slot[-8:], 'little') != sequence:
            return  # slot already overwritten by a newer frame
        bgr = np.frombuffer(slot, np.uint8, color_bytes).reshape(h, w, 3)
        depth = np.frombuffer(slot, np.uint16, w * h, color_bytes).reshape(h, w)
        state['done'] = session.save(bgr, depth, stamp_ns, sequence, state['scan'])

    node.create_subscription(CameraInfo, '/vision/d435/camera_info', on_info, 10)
    node.create_subscription(LaserScan, '/lidar/front/scan', lambda m: state.update(scan=m), 10)
    node.create_subscription(UInt64MultiArray, '/vision/d435/frame_meta', on_meta, 1)
    print('waiting for /vision/d435/frame_meta + camera_info (realsense_processor running?)')
    try:
        while rclpy.ok() and not state['done']:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def record_rs(session):
    import pyrealsense2 as rs

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
    profile = pipe.start(cfg)
    sensor = profile.get_device().first_depth_sensor()
    if sensor.supports(rs.option.depth_units):
        sensor.set_option(rs.option.depth_units, 0.001)
    i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    session.write_meta(i.fx, i.fy, i.ppx, i.ppy, i.coeffs, W, H, sensor.get_depth_scale())
    align = rs.align(rs.stream.color)
    try:
        for _ in range(FPS):  # auto exposure settles
            pipe.wait_for_frames()
        while True:
            frames = align.process(pipe.wait_for_frames())
            if not session.due():
                continue
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if session.save(np.asanyarray(color.get_data()), np.asanyarray(depth.get_data()),
                            time.time_ns(), color.get_frame_number()):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True)
    ap.add_argument('--label', choices=['standing', 'lying', 'mixed'], required=True)
    ap.add_argument('--yaw-deg', type=float, default=None, help='lying: puck yaw you set, for the notes')
    ap.add_argument('--note', default='')
    ap.add_argument('--source', choices=['shm', 'rs'], default='shm')
    ap.add_argument('--frames', type=int, default=100)
    ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--out', default='~/bags/puck_compare')
    args = ap.parse_args()
    if args.source == 'shm' and not Path('/dev/shm/craft_vision').exists():
        raise SystemExit('/dev/shm/craft_vision 없음: realsense_processor가 안 돌고 있음.\n'
                         '  스택 켜기: ros2 launch craft_bringup craft.launch.py  (LiDAR 스캔도 같이 저장)\n'
                         '  또는 D435 직접: --source rs  (LiDAR 스캔 없음)')
    session = Session(args)
    (record_shm if args.source == 'shm' else record_rs)(session)
    print(f'{session.count} frames -> {session.dir}')


if __name__ == '__main__':
    main()
