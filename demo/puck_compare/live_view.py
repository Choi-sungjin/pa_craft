#!/usr/bin/env python3
"""PC side: length-prefixed JPEG frames on stdin (res_record.py --preview over ssh) -> puck YOLO -> window.

  ssh craft@<ip> '...res_record.py ... --preview' | ~/craft/custom_yolo26/.venv/bin/python live_view.py --model M
Keys: q = quit, s = save the current frame (with boxes) next to this script.
"""
import argparse
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

COLORS = {'red_cylinder': (0, 0, 255), 'red_prism': (80, 80, 255), 'green_cylinder': (0, 200, 0),
          'green_prism': (120, 255, 120)}

ap = argparse.ArgumentParser()
ap.add_argument('--model', default=str(Path('~/dataset/export_pucks_v8n_v2/pucks_v8n_v2_ep24.pt').expanduser()))
ap.add_argument('--conf', type=float, default=0.4)
ap.add_argument('--no-model', action='store_true', help='just show the frames (already drawn, survey_stream.py)')
a = ap.parse_args()
model = None if a.no_model else YOLO(a.model)
src = sys.stdin.buffer
window, fps, last, saved = 'puck preview (q quit, s save)', 0.0, time.monotonic(), 0
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 1280, 720)
while True:
    head = src.read(4)
    if len(head) < 4:
        break
    image = cv2.imdecode(np.frombuffer(src.read(struct.unpack('<I', head)[0]), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        continue
    boxes = [] if model is None else model.predict(image, imgsz=640, conf=a.conf, verbose=False)[0].boxes
    for (x1, y1, x2, y2), c, s in ([] if model is None else zip(boxes.xyxy.int().tolist(), boxes.cls.int().tolist(),
                                      boxes.conf.tolist())):
        name = model.names[c]
        color = COLORS.get(name, (200, 200, 200))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f'{name} {s:.2f}', (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    now = time.monotonic()
    fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
    last = now
    cv2.putText(image, f'{fps:4.1f} fps' + ('' if model is None else f'  pucks={len(boxes)}'), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2)
    cv2.imshow(window, image)
    key = cv2.waitKey(1) & 0xFF
    try:
        closed = cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        closed = True
    if key == ord('q') or closed:
        break
    if key == ord('s'):
        path = Path(__file__).with_name(f'live_{saved:03d}.jpg')
        cv2.imwrite(str(path), image)
        saved += 1
        print('saved', path, flush=True)
