#!/usr/bin/python3
"""Viewer for a res_record.py session: YOLO boxes, AprilTags inside them, the red-prism rule and the tag axis.

Inputs (made by res_analyze.py and puck_axis_offline --box yolotag):
  <session>/res_boxes.json          YOLO boxes (v8n_v2) at full resolution
  <tagaxis dir>/results/yolotag_iou1.csv (else yolotag.csv) PuckTagAxis per YOLO box (axis, center, standing, tags)
Draws per puck box: class + score, tags (green quads), red boxes: "PRISM by tags" (rule fired) or "yolo only",
and the tag axis as a 60 mm line through the puck center (S = standing, L = lying, fit error, tag count).
Keys: space pause/play, d / a next / previous frame (paused), q quit. Loops.

  /usr/bin/python3 axis_view.py ~/craft/bags_robot/puck_res/pucks_1112_0944_1920x1080 <tagaxis dir>
Same OpenCV as the C++ build (system 4.6), so the tags drawn are the tags production would see.
"""
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

session, tagaxis = Path(sys.argv[1]).expanduser(), Path(sys.argv[2]).expanduser()
meta = json.loads((session / 'meta.json').read_text())
K = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1]])
boxes = json.loads((session / 'res_boxes.json').read_text())['v8n_v2']
axes = {}
result = next(p for p in (tagaxis / 'results' / 'yolotag_iou1.csv', tagaxis / 'results' / 'yolotag.csv') if p.exists())
for r in csv.DictReader(open(result)):
    axes.setdefault(r['frame'], []).append(r)
COLORS = {'red_cylinder': (0, 0, 255), 'red_prism': (60, 60, 255), 'green_cylinder': (0, 190, 0),
          'green_prism': (90, 255, 90)}
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
par = cv2.aruco.DetectorParameters_create()
par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
SIDE_RATIO, SKEW_DEG, ASPECT, GROW = 0.85, 10.0, 1.6, 1.4


def face_on(c):
    sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    if min(sides) / max(sides) < SIDE_RATIO:
        return False
    for i in range(4):
        a, b = c[i - 1] - c[i], c[(i + 1) % 4] - c[i]
        if abs(np.degrees(np.arccos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))) - 90) > SKEW_DEG:
            return False
    return True


def project(p):
    q = K @ p
    return int(q[0] / q[2]), int(q[1] / q[2])


def draw(name):
    img = cv2.imread(str(session / 'frames' / name))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    frame = name[:-4]
    tags = []
    for x1, y1, x2, y2, cls, sc in boxes.get(name, []):
        if sc < 0.4:
            continue
        cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * GROW, (y2 - y1) * GROW
        a, b = int(max(0, cx - w / 2)), int(max(0, cy - h / 2))
        corners, ids, _ = cv2.aruco.detectMarkers(gray[b:int(min(H, cy + h / 2 + 1)), a:int(min(W, cx + w / 2 + 1))],
                                                  dic, parameters=par)
        for i, c in zip([] if ids is None else ids.ravel(), corners or []):
            c = c.reshape(4, 2) + [a, b]
            if not any(i == j and np.linalg.norm(c.mean(0) - q.mean(0)) < 3 for j, q in tags):
                tags.append((int(i), c))
    for i, c in tags:
        cv2.polylines(img, [c.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(img, str(i), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    by_class = {r['id']: r for r in axes.get(frame, [])}
    for x1, y1, x2, y2, cls, sc in boxes.get(name, []):
        if sc < 0.4:
            continue
        color = COLORS.get(cls, (200, 200, 200))
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, color, 2)
        inside = [c for i, c in tags if i < 12 and x1 <= c[:, 0].mean() <= x2 and y1 <= c[:, 1].mean() <= y2]
        label = f'{cls} {sc:.2f} tags={len(inside)}'
        if cls.startswith('red'):
            long_box = max(x2 - x1, y2 - y1) / max(1e-6, min(x2 - x1, y2 - y1)) >= ASPECT
            fired = len(inside) >= 2 or (len(inside) == 1 and long_box and face_on(inside[0]))
            label += '  PRISM by tags' if fired else '  yolo only'
        cv2.putText(img, label, (int(x1), max(18, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        k = str(['red_cylinder', 'red_prism', 'green_cylinder', 'green_prism'].index(cls)) if cls in COLORS else None
        r = by_class.get(k)
        if r and r['axis_valid'] == '1':
            c = np.array([float(r['cx']), float(r['cy']), float(r['cz'])])
            ax = np.array([float(r['ax']), float(r['ay']), float(r['az'])])
            cv2.line(img, project(c - ax * 0.03), project(c + ax * 0.03), (255, 0, 255), 3)
            cv2.circle(img, project(c), 5, (255, 0, 255), -1)
            iou = f" IoU {float(r['box_iou']):.2f}" if r.get('box_iou') not in (None, '', '-1.000') else ''
            txt = f"axis {'S' if r['standing'] == '1' else 'L'} fit {float(r['fit_error_deg']):.1f}deg tags {r['tags']}{iou}"
            cv2.putText(img, txt, (int(x1), int(y2) + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        elif r:
            cv2.putText(img, 'axis: no answer', (int(x1), int(y2) + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 2)
    cv2.putText(img, f'{session.name}  frame {frame}', (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return img


names = sorted(boxes)
window = 'puck axis / red prism rule (space, a/d, q)'
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 1600, 900)
i, paused = 0, False
while True:
    cv2.imshow(window, draw(names[i]))
    key = cv2.waitKey(0 if paused else 150) & 0xFF
    if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
        break
    if key == ord(' '):
        paused = not paused
    elif key == ord('a'):
        i = (i - 1) % len(names)
    elif key == ord('d') or not paused:
        i = (i + 1) % len(names)
