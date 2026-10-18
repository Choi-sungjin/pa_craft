#!/usr/bin/env python3
"""Knob lever direction from a webcam (Logitech C310) + YOLO knob bbox.

1. YOLO (~/git/yolo26_kant) finds the knob bbox.
2. Inside the bbox: red pixels (HSV), then Otsu on brightness splits the bright lever from the
   darker red face. Largest blob = lever.
3. Lever axis = main PCA axis of the blob; it points toward the end farther from the knob center
   (bbox center), drawn as an arrow.

angle: clock style, 0 deg = 12 o'clock, clockwise (90 = 3 o'clock).

usage (venv-craft):
  python3 knob_lever.py [--source auto|/dev/videoN|N|video.mp4] [--model .../v4.pt] [--headless]
  --source auto: first /dev/v4l/by-id/*046d_081b*-video-index0 (C310 USB id). q or ESC to quit.
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np

YOLO_DIR = os.path.expanduser('~/git/yolo26_kant')


def lever_direction(crop, min_frac=0.02, max_frac=0.5):
    """Return (unit direction (dx, dy) in image coords, lever mask) or (None, mask) if not found.

    ponytail: color + brightness heuristic, bbox center as pivot. Side views (pivot off bbox
    center) and a symmetric bar (180 deg ambiguity) are its limits; a keypoint model
    (pivot + lever tip) is the upgrade if it isn't good enough.
    """
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    red = ((h <= 10) | (h >= 165)) & (s >= 90)
    if red.sum() < 30:
        return None, red
    t, _ = cv2.threshold(v[red].reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx((red & (v > t)).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None, mask > 0
    k = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    lever = lab == k
    frac = stats[k, cv2.CC_STAT_AREA] / lever.size
    if not min_frac <= frac <= max_frac:  # too small = noise, too big = red background
        return None, lever
    ys, xs = np.nonzero(lever)
    pts = np.column_stack((xs, ys)).astype(np.float32)
    _, _, vt = np.linalg.svd(pts - pts.mean(0), full_matrices=False)
    d = vt[0]
    proj = (pts - np.array([crop.shape[1] / 2, crop.shape[0] / 2])) @ d
    if proj.max() < -proj.min():  # point to the lever end farther from the knob center
        d = -d
    return d, lever


def clock_angle(d):
    """Image direction -> degrees from 12 o'clock, clockwise."""
    return float(np.degrees(np.arctan2(d[0], -d[1])) % 360)


def open_source(src):
    if src == 'auto':
        found = sorted(glob.glob('/dev/v4l/by-id/*046d_081b*-video-index0'))  # Logitech C310 USB id
        if not found:
            raise SystemExit('C310 not found in /dev/v4l/by-id (plugged in?). Use --source to pick one.')
        src = found[0]
    if src.isdigit():
        src = int(src)
    is_cam = isinstance(src, int) or src.startswith('/dev/')
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if is_cam else cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {src}')
    return cap, src


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='auto')
    ap.add_argument('--model', default=os.path.join(YOLO_DIR, 'weights', 'v4.pt'))
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--headless', action='store_true', help='terminal output only, no window')
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    knob = next(i for i, n in model.names.items() if n == 'knob')
    cap, src = open_source(args.source)
    print(f'source={src} model={args.model}  (q/ESC to quit)')

    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print('end of stream / frame read failed')
            break
        r = model.predict(frame, conf=args.conf, classes=[knob], verbose=False)[0]
        found = []  # (box, conf, direction or None)
        for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.cpu().numpy().astype(int), r.boxes.conf.cpu().numpy()):
            x1, y1 = max(0, x1), max(0, y1)
            d, _ = lever_direction(frame[y1:y2, x1:x2]) if x2 > x1 and y2 > y1 else (None, None)
            found.append(((x1, y1, x2, y2), c, d))

        now = time.time()
        if now - last_print > 0.5:
            last_print = now
            print(', '.join(
                f'knob conf={c:.2f} center=({(b[0] + b[2]) // 2},{(b[1] + b[3]) // 2}) '
                + (f'lever={clock_angle(d):.0f}deg ({clock_angle(d) / 30:.1f} o\'clock)' if d is not None
                   else 'lever=not found')
                for b, c, d in found) or 'no knob', flush=True)
        if args.headless:
            continue

        for (x1, y1, x2, y2), c, d in found:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'knob {c:.2f} '
            if d is not None:
                length = 0.8 * max(x2 - x1, y2 - y1)
                tip = (int(cx + d[0] * length), int(cy + d[1] * length))
                cv2.arrowedLine(frame, (cx, cy), tip, (255, 0, 255), 3, tipLength=0.25)
                label += f'{clock_angle(d):.0f}deg'
            else:
                label += 'lever ?'
            cv2.putText(frame, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow('knob_lever', frame)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
