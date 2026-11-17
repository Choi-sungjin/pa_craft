#!/usr/bin/env python3
"""Live upper-cam window with the puck detections and their base_link position (puck_locate.py pipeline).

  PYTHONPATH=~/.cache/craft_rs_only /usr/bin/python3 puck_live.py [--gripper]   (DISPLAY on the desktop; q / Esc ends)
  --gripper: the gripper D435 (261622073363), detections only (its pose moves with the arm: no base_link position)

Colour 640x360 + depth aligned to it, 30 fps. YOLO (cv2.dnn, CPU, ~4 Hz) runs in its own thread on the newest frame;
the window shows every camera frame with the latest detections, so the boxes lag the picture a little.
"""
import os
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import puck_locate as pl  # noqa: E402
import box_range as br  # noqa: E402


GRIPPER_SERIAL = '261622073363'


def main():
    gripper = '--gripper' in sys.argv
    title = 'gripper cam pucks' if gripper else 'uppercam pucks'
    cfg = rs.config()
    cfg.enable_device(GRIPPER_SERIAL if gripper else pl.UPPER_SERIAL)
    cfg.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    R, t, z_floor, laser = pl.camera_pose()
    sis1 = np.array([laser[0] + br.LIDAR_FACE_TO_CENTRE + pl.SIS1_AHEAD_OF_FACE, laser[1] + pl.SIS1_ALONG_LM])
    shared = {'frame': None, 'dets': [], 'hz': 0.0, 'run': True}
    lock = threading.Lock()

    def detector():  # newest frame -> detections with base_link centres
        while shared['run']:
            with lock:
                item = shared['frame']; shared['frame'] = None
            if item is None:
                time.sleep(0.01); continue
            bgr, d = item
            t0 = time.time(); out = []
            for (x, y, w, h), c, p in pl.detect(bgr):
                patch = d[max(int(y + 0.3 * h), 0):int(y + 0.7 * h), max(int(x + 0.3 * w), 0):int(x + 0.7 * w)]
                z = np.median(patch[patch > 0.1]) if np.any(patch > 0.1) else np.nan
                ray = np.linalg.inv(K) @ np.array([x + w / 2, y + h / 2, 1.0])
                surf = (ray * z) @ R.T + t
                hz = surf - t; hz[2] = 0; hz /= np.linalg.norm(hz) + 1e-9
                out.append(((x, y, w, h), c, p, None if gripper else surf + hz * pl.PUCK_RADIUS, z))
            with lock:
                shared['dets'] = out; shared['hz'] = 1 / max(time.time() - t0, 1e-3)

    th = threading.Thread(target=detector, daemon=True); th.start()
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 640, 360)
    cv2.moveWindow(title, 660 if gripper else 0, 40)  # side by side
    last = time.time()
    try:
        while True:
            f = align.process(pipe.wait_for_frames())
            bgr = np.asanyarray(f.get_color_frame().get_data()).copy()
            d = np.asanyarray(f.get_depth_frame().get_data()) * scale
            with lock:
                shared['frame'] = (bgr.copy(), d); dets = list(shared['dets']); det_hz = shared['hz']
            for (x, y, w, h), c, p, cen, z in dets:
                cv2.rectangle(bgr, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 1)
                cv2.putText(bgr, f'{pl.CLASSES[c]} {p:.2f}', (int(x), int(y) - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                if cen is None:
                    txt = f'depth {z:.3f} m'
                else:
                    dx, dy = (cen[:2] - sis1) * 1000
                    txt = f'x {cen[0] * 1000:.0f} y {cen[1] * 1000:+.0f} (sis1 {dx:+.0f},{dy:+.0f})'
                cv2.putText(bgr, txt, (int(x), int(y) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            now = time.time()
            cv2.putText(bgr, f'view {1 / max(now - last, 1e-3):.0f} fps  detect {det_hz:.1f} Hz' + ('' if gripper else f'  sis1 x {sis1[0] * 1000:.0f} y {sis1[1] * 1000:+.0f}'),
                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            last = now
            cv2.imshow(title, bgr)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        shared['run'] = False
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
