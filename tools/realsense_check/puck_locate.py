#!/usr/bin/env python3
"""Upper-cam puck position in base_link (V307 URDF, REP-103: x ahead, y left, z up), one aligned RealSense frame.

  puck_locate.py [prefix ...]      analyse saved rs_snap captures
  puck_locate.py --capture         grab one frame from the upper D435 (261322075477) first

Pucks: YOLOv8n pucks_v8n_v2 (src/craft_perception/models, ONNX through cv2.dnn). Depth: median of the inner 40 % of
the box. Camera pose: box_range.py URDF mode (V307 pose, yaw CAM_YAW_DEG, height CAM_HEIGHT), so the same
calibration as the box distance. The visible surface point is pushed back by the puck radius along the horizontal
view ray to estimate the centre. Compared with the sis1 puck spot (demo/puck_pick/plan.py ASSUME: 130 mm ahead of the
front LiDAR face, LiDAR face 19 mm ahead of its centre, 10 mm along the LM axis).
"""
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import box_range as br  # noqa: E402

MODEL = os.path.normpath(os.path.join(HERE, '..', '..', 'src', 'craft_perception', 'models', 'pucks_v8n_v2', 'model.onnx'))
CLASSES = ['red_cylinder', 'red_prism', 'green_cylinder', 'green_prism']
UPPER_SERIAL = '261322075477'
LENS_Y = 0.0322         # colour lens y in base_link [m] (box_live.py --lens-y default)
PUCK_RADIUS = 0.015      # half the 30 mm puck width (plan.yaml puck_size)
SIS1_AHEAD_OF_FACE = 0.130   # plan.py: puck centre y -0.096 vs LiDAR front y 0.034 (front = -Y there)
SIS1_ALONG_LM = -0.010   # plan.py puck_x_offset; sign in base_link not checked (LM axis = base y)


_NET = None


def detect(bgr, conf=0.25, iou=0.45):
    global _NET
    if _NET is None:  # load once (re-reading the ONNX each frame cost most of the time)
        _NET = cv2.dnn.readNetFromONNX(MODEL)
    net = _NET
    h, w = bgr.shape[:2]
    s = 640 / max(h, w)
    img = np.full((640, 640, 3), 114, np.uint8)
    img[:int(h * s), :int(w * s)] = cv2.resize(bgr, (int(w * s), int(h * s)))
    net.setInput(cv2.dnn.blobFromImage(img, 1 / 255.0, (640, 640), swapRB=True))
    out = net.forward()[0].T                       # (8400, 4 + classes)
    scores = out[:, 4:]
    cls = scores.argmax(1); sc = scores.max(1)
    keep = sc >= conf
    boxes = []
    for (cx, cy, bw, bh), c, p in zip(out[keep, :4], cls[keep], sc[keep]):
        boxes.append(([(cx - bw / 2) / s, (cy - bh / 2) / s, bw / s, bh / s], int(c), float(p)))
    idx = cv2.dnn.NMSBoxes([b for b, _, _ in boxes], [p for _, _, p in boxes], conf, iou)
    return [boxes[i] for i in np.array(idx).flatten()] if len(boxes) else []


def camera_pose():
    R, t, z_floor, laser = br.urdf_frames(br.URDF_DEFAULT)
    t = t.copy(); t[2] = z_floor + br.CAM_HEIGHT
    t[1] = LENS_Y  # V307 has the colour lens mirrored (-0.0325); box_live.py uses the datasheet +0.0322
    c, s = np.cos(np.radians(br.CAM_YAW_DEG)), np.sin(np.radians(br.CAM_YAW_DEG))
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R
    return R, t, z_floor, laser


def analyse(prefix):
    d, K, ser = br.load(prefix)
    bgr = cv2.imread(prefix + '_color.ppm')
    R, t, z_floor, laser = camera_pose()
    face_x = laser[0] + br.LIDAR_FACE_TO_CENTRE
    sis1 = np.array([face_x + SIS1_AHEAD_OF_FACE, laser[1] + SIS1_ALONG_LM])
    print(f'{prefix} (D435 {ser}), camera {np.round(t * 1000)} mm in base_link, floor z {z_floor * 1000:.0f} mm')
    print(f'  sis1 puck spot (plan.py): x {sis1[0] * 1000:.0f} y {sis1[1] * 1000:+.0f} mm (front_laser x {laser[0] * 1000:.0f})')
    found = detect(bgr)
    if not found:
        print('  no puck detected'); return
    vis = bgr.copy()
    for (x, y, w, h), c, p in sorted(found, key=lambda f: -f[2]):
        x0, y0, x1, y1 = int(x + 0.3 * w), int(y + 0.3 * h), int(x + 0.7 * w), int(y + 0.7 * h)
        patch = d[max(y0, 0):y1, max(x0, 0):x1]
        z = np.median(patch[patch > 0.1]) if np.any(patch > 0.1) else np.nan
        u, v = x + w / 2, y + h / 2
        ray = np.linalg.inv(K) @ np.array([u, v, 1.0])
        surf = (ray * z) @ R.T + t                     # visible surface point in base_link
        hz = (surf - t); hz[2] = 0; hz /= np.linalg.norm(hz) + 1e-9
        cen = surf + hz * PUCK_RADIUS
        dx, dy = (cen[:2] - sis1) * 1000
        print(f'  {CLASSES[c]:15s} {p:.2f}  px ({u:.0f},{v:.0f}) depth {z:.3f} m  surface z {(surf[2] - z_floor) * 1000:.0f} mm over floor'
              f'  centre x {cen[0] * 1000:.0f} y {cen[1] * 1000:+.0f} mm  -> vs sis1 spot dx {dx:+.0f} dy {dy:+.0f} mm')
        cv2.rectangle(vis, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
        cv2.putText(vis, f'{CLASSES[c]} {p:.2f}', (int(x), int(y) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imwrite(prefix + '_pucks.png', vis)
    print(f'  boxes drawn: {prefix}_pucks.png')


def main():
    prefixes = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--capture' in sys.argv:
        p = os.path.join(HERE, 'out', time.strftime('puck_%H%M%S'))
        subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), p, '1280', '720', UPPER_SERIAL], check=True)
        prefixes.append(p)
    for p in prefixes:
        analyse(p)


if __name__ == '__main__':
    main()
