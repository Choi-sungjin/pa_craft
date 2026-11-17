#!/usr/bin/env python3
"""One RealSense bag of the upper cam (어퍼캠): colour + depth, one file.

  python3 rs_bag.py [--seconds N] [--serial 261322075477] [--out ~/bags/boxgap_MMDD_HHMM]
  (venv python3: needs pyrealsense2; /usr/bin/python3 has none)

Records until Ctrl+C (or --seconds N). Writes into --out: boxgap.db3 (librealsense 2.59 native = rosbag2 sqlite: colour 640x360 BGR8 + depth 424x240 Z16
@30, raw, not aligned; intrinsics/extrinsics inside) and meta.json (serial, firmware, intrinsics, depth scale,
depth->colour extrinsic). A tick (bell) every 3 s: elapsed time, so the box can be moved on the beat.
"""
import argparse
import json
import os
import time

import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = until Ctrl+C')
    ap.add_argument('--serial', default='261322075477', help='어퍼캠 D435 (box_range.py default)')
    ap.add_argument('--tick', type=float, default=3.0, help='print + bell every n seconds')
    ap.add_argument('--out', default=os.path.expanduser(time.strftime('~/bags/boxgap_%m%d_%H%M')))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    bag = os.path.join(a.out, 'boxgap.db3')

    cfg = rs.config()
    cfg.enable_device(a.serial)
    cfg.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    cfg.enable_record_to_file(bag)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    dev = prof.get_device()
    n, t0, next_tick = 0, time.time(), a.tick
    print(f'REC {bag}  ' + (f'{a.seconds:.0f} s' if a.seconds > 0 else 'Ctrl+C로 정지'))
    try:
        while a.seconds <= 0 or time.time() - t0 < a.seconds:
            pipe.wait_for_frames(2000); n += 1
            el = time.time() - t0
            if el >= next_tick:
                print(f'\a  {el:4.1f} s', flush=True); next_tick += a.tick
    except KeyboardInterrupt:
        print('\n정지: 저장 중')
    finally:
        cs = prof.get_stream(rs.stream.color).as_video_stream_profile()
        ds = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        intr = lambda p: dict(zip(('width', 'height', 'fx', 'fy', 'ppx', 'ppy', 'model', 'coeffs'),
                                  (p.width, p.height, p.fx, p.fy, p.ppx, p.ppy, str(p.model), list(p.coeffs))))
        ex = ds.get_extrinsics_to(cs)
        meta = dict(serial=dev.get_info(rs.camera_info.serial_number),
                    firmware=dev.get_info(rs.camera_info.firmware_version),
                    librealsense=rs.__version__, recorded=time.strftime('%Y-%m-%d %H:%M:%S'),
                    start_epoch=t0,   # wall clock at pipeline start = db3 timestamp 0 (sync with ros2 bags)
                    seconds=round(time.time() - t0, 2), framesets=n,
                    depth_scale=dev.first_depth_sensor().get_depth_scale(),
                    color_intrinsics=intr(cs.get_intrinsics()), depth_intrinsics=intr(ds.get_intrinsics()),
                    depth_to_color=dict(rotation=list(ex.rotation), translation=list(ex.translation)))
        pipe.stop()
        with open(os.path.join(a.out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        print(f'{bag}  {os.path.getsize(bag) / 1e6:.0f} MB, {n} framesets, {meta["seconds"]} s')


if __name__ == '__main__':
    main()
