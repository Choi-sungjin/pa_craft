#!/usr/bin/env python3
"""Gripper-cam (그립캠, D435 261622073363) bags of a standing puck at known yaws, for the yaw algorithm (2026-11-15).

Arm first at the pre-descend pose over cell 8 (far-centre) and left there:
  source ~/craft/install/setup.bash && python3 ~/craft/demo/puck_pick/execute.py --stand --cell 8
Then (venv python3: needs pyrealsense2; close the live gripper window, it holds the camera):
  python3 grip_yaw_bag.py [--yaws 0,22.5,45,67.5,90] [--seconds 3] [--out ~/bags/gripyaw_MMDD_HHMM] [--tag cylinder]

Per yaw: turn the puck to that yaw by hand, Enter -> records --seconds of colour 1280x720 + depth 640x360 @30
(librealsense 2.59 native = rosbag2 sqlite, raw, not aligned) to <out>/yaw_<deg*10>.db3 plus one aligned preview png (colour) and
depth npy (mm, aligned to colour). s + Enter = skip, r + Enter = record that yaw again, q + Enter = stop.
meta.json: serial, intrinsics, depth scale, depth->colour extrinsic, the yaws recorded and their files.
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs

SERIAL = '261622073363'


def intr(p):
    return dict(zip(('width', 'height', 'fx', 'fy', 'ppx', 'ppy', 'model', 'coeffs'),
                    (p.width, p.height, p.fx, p.fy, p.ppx, p.ppy, str(p.model), list(p.coeffs))))


def record(path, seconds, w, h, prefix, dw, dh):
    cfg = rs.config()
    cfg.enable_device(SERIAL)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, 30)
    cfg.enable_record_to_file(path)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    n, t0, fs = 0, time.time(), None
    try:
        while time.time() - t0 < seconds:
            fs = pipe.wait_for_frames(2000); n += 1
        a = align.process(fs)
        dev = prof.get_device()
        scale = dev.first_depth_sensor().get_depth_scale()
        cv2.imwrite(prefix + '.png', np.asanyarray(a.get_color_frame().get_data()))
        np.save(prefix + '_depth_mm.npy', (np.asanyarray(a.get_depth_frame().get_data()) * scale * 1000).astype(np.uint16))
        cs = prof.get_stream(rs.stream.color).as_video_stream_profile()
        ds = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        ex = ds.get_extrinsics_to(cs)
        cam = dict(serial=dev.get_info(rs.camera_info.serial_number), firmware=dev.get_info(rs.camera_info.firmware_version),
                   librealsense=rs.__version__, depth_scale=scale,
                   color_intrinsics=intr(cs.get_intrinsics()), depth_intrinsics=intr(ds.get_intrinsics()),
                   depth_to_color=dict(rotation=list(ex.rotation), translation=list(ex.translation)))
    finally:
        pipe.stop()
    return n, cam


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaws', default='0,22.5,45,67.5,90')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--size', default='1280x720', help='colour')
    # depth 640x360 (2026-11-15): at 1280x720 the D435 sees nothing nearer than 26 cm and the prism top is ~22 cm away
    # (848x480: 17 cm, 640x360: 13 cm, 424x240: 8.6 cm min, measured on the gripper cam)
    ap.add_argument('--depth-size', default='640x360')
    ap.add_argument('--cell', type=int, default=8, help='grid cell the arm waits over (execute.py --stand --cell N)')
    ap.add_argument('--tag', default='', help='puck kind etc., kept in meta.json')
    ap.add_argument('--out', default=os.path.expanduser(time.strftime('~/bags/gripyaw_%m%d_%H%M')))
    a = ap.parse_args()
    w, h = map(int, a.size.split('x'))
    dw, dh = map(int, a.depth_size.split('x'))
    yaws = [float(y) for y in a.yaws.split(',')]
    os.makedirs(a.out, exist_ok=True)
    meta = dict(recorded=time.strftime('%Y-%m-%d %H:%M:%S'), pose=f'execute.py --stand --cell {a.cell} (pre-descend, tool_roll 90)', cell=a.cell,
                tag=a.tag, seconds=a.seconds, size=a.size, depth_size=a.depth_size, shots=[])
    k = 0
    while k < len(yaws):
        y = yaws[k]
        ans = input(f'\n[{k + 1}/{len(yaws)}] 퍽을 yaw {y:g} deg로 놓고 Enter (s 건너뛰기, r 이전 다시, q 끝): ').strip().lower()
        if ans == 'q':
            break
        if ans == 's':
            k += 1; continue
        if ans == 'r' and k > 0:
            k -= 1; continue
        name = f'yaw_{int(round(y * 10)):04d}'
        path = os.path.join(a.out, name + '.db3')
        if os.path.exists(path):
            os.remove(path)
        print(f'  REC {path} ({a.seconds:g} s)', flush=True)
        n, cam = record(path, a.seconds, w, h, os.path.join(a.out, name), dw, dh)
        meta['camera'] = cam
        meta['shots'] = [s for s in meta['shots'] if s['yaw_deg'] != y] + [
            dict(yaw_deg=y, bag=name + '.db3', png=name + '.png', depth=name + '_depth_mm.npy', framesets=n, t=time.time())]
        with open(os.path.join(a.out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        print(f'  {n} framesets, {os.path.getsize(path) / 1e6:.0f} MB', flush=True)
        k += 1
    print(f'\n{len(meta["shots"])} yaws -> {a.out}')


if __name__ == '__main__':
    main()
