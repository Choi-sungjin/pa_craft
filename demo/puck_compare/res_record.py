#!/usr/bin/env python3
"""Robot side, TODO A1 (docs/todo/puck_perception.md): upper-cam colour at one resolution -> PNG session.

Saves full-resolution PNGs (no JPEG loss: the puck tags are a few px) plus meta.json (intrinsics), and
optionally streams a small JPEG preview to stdout for live_view.py, so the pucks can be placed by eye.

  # on the robot (upper-cam bridge must be off: it holds the camera)
  PYTHONPATH=~/.cache/craft_rs_only /usr/bin/python3 res_record.py --size 1280x720 --name four_a
  # from the PC, with preview
  ssh craft@<ip> 'cd ~/craft/demo/puck_compare && PYTHONPATH=$HOME/.cache/craft_rs_only /usr/bin/python3 \
      res_record.py --size 1920x1080 --name four_a --preview' | python live_view.py --model <puck .pt>
Output: ~/bags/puck_res/<name>_<WxH>/{frames/000000.png, frames.csv, meta.json}
"""
import argparse
import json
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--size', default='1280x720', help='colour WxH: 640x360 | 1280x720 | 1920x1080')
ap.add_argument('--name', required=True, help='scene name, e.g. four_a (same name for every resolution)')
ap.add_argument('--seconds', type=float, default=20.0)
ap.add_argument('--hz', type=float, default=5.0)
ap.add_argument('--serial', default='261322075477', help='upper cam')
ap.add_argument('--out', default='~/bags/puck_res')
ap.add_argument('--preview', action='store_true', help='length-prefixed 640-wide JPEG frames on stdout')
ap.add_argument('--depth', default='', help='also save depth aligned to colour (16-bit PNG <n>_depth.png), depth '
                'stream WxH e.g. 848x480 (best D435 depth mode); empty = colour only')
a = ap.parse_args()
w, h = (int(v) for v in a.size.lower().split('x'))
session = Path(a.out).expanduser() / f'{a.name}_{w}x{h}'
(session / 'frames').mkdir(parents=True, exist_ok=False)
log = sys.stderr

pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_device(a.serial)
cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
if a.depth:
    dw, dh = (int(v) for v in a.depth.lower().split('x'))
    cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, 30)
profile = pipe.start(cfg)
depth_unit = 0.0
align = None
if a.depth:
    sensor = profile.get_device().first_depth_sensor()
    if sensor.supports(rs.option.depth_units):
        sensor.set_option(rs.option.depth_units, 0.001)
    depth_unit = sensor.get_depth_scale()
    align = rs.align(rs.stream.color)
i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
json.dump(dict(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy, coeffs=list(i.coeffs), width=w, height=h, serial=a.serial,
               name=a.name, depth=a.depth, depth_unit_m=depth_unit, started=time.strftime('%Y-%m-%d %H:%M:%S')),
          open(session / 'meta.json', 'w'), indent=1)
print(f'[res_record] {a.serial} {w}x{h} -> {session}', file=log, flush=True)
out = sys.stdout.buffer
index, period, last, start, fps_t, fps_n = 0, 1.0 / a.hz, 0.0, None, time.monotonic(), 0
try:
    with open(session / 'frames.csv', 'w') as csv:
        csv.write('frame,stamp_ns\n')
        for _ in range(30):  # auto exposure settles
            pipe.wait_for_frames()
        while True:
            frames = pipe.wait_for_frames(5000)
            frame = frames.get_color_frame()
            now = time.monotonic()
            fps_n += 1
            start = start or now
            if now - start > a.seconds:
                break
            if now - last < period:
                continue
            last = now
            if align is not None:  # depth on the colour pixel grid: bgr(v,u) and depth(v,u) are one point
                aligned = align.process(frames)
                frame = aligned.get_color_frame()
                cv2.imwrite(str(session / 'frames' / f'{index:06d}_depth.png'), np.asanyarray(aligned.get_depth_frame().get_data()))
            img = np.asanyarray(frame.get_data())
            cv2.imwrite(str(session / 'frames' / f'{index:06d}.png'), img)
            csv.write(f'{index:06d},{time.time_ns()}\n')
            index += 1
            if a.preview:
                small = cv2.resize(img, (640, int(640 * h / w)), interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    out.write(struct.pack('<I', len(jpg)) + jpg.tobytes())
                    out.flush()
except (KeyboardInterrupt, BrokenPipeError):
    pass
finally:
    pipe.stop()
fps = fps_n / max(time.monotonic() - fps_t, 1e-3)
meta = json.load(open(session / 'meta.json'))
meta.update(frames=index, camera_fps=round(fps, 1))
json.dump(meta, open(session / 'meta.json', 'w'), indent=1)
print(f'[res_record] {index} frames saved, camera delivered {fps:.1f} fps at {w}x{h}', file=log, flush=True)
