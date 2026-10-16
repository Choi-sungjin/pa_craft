#!/usr/bin/env python3
"""knob bbox -> knob depth + normal of the panel around the knob.

YOLO (~/git/yolo26_kant) + D435 depth aligned to color (camera.py).
Same camera profile as camera.launch.py: color 640x360, depth 424x240 aligned to color, 30fps.

panel normal:
  1. square donut = bbox_large (bbox scaled by --grow) - bbox
  2. keep donut pixels whose depth is within --tol mm of the knob depth -> panel depth map
  3. deproject to 3D (color intrinsics) and fit a plane (SVD) -> unit normal, pointing at the camera

Needs ultralytics + pyrealsense2 (venv-craft):
  source ~/craft_ws/venv-craft/bin/activate
  python3 knob_depth.py [--model .../weights/v4.pt] [--conf 0.5] [--grow 2.0] [--tol 30] [--headless]
q or ESC to quit.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_heatmap import depth_to_heatmap  # noqa: E402

YOLO_DIR = os.path.expanduser('~/git/yolo26_kant')


def aligned_frames(serial=None):
    """Yield (BGR image, Z16 depth aligned to color, depth unit [m], color intrinsics (fx, fy, ppx, ppy))."""
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    if serial:
        config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    profile = pipeline.start(config)
    i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intr = (i.fx, i.fy, i.ppx, i.ppy)
    try:
        align = rs.align(rs.stream.color)
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue
            yield (np.asanyarray(color.get_data()).copy(),
                   np.asanyarray(depth.get_data()).copy(), depth.get_units(), intr)
    finally:
        pipeline.stop()


def bbox_depth(depth, box, inner=0.5):
    """Median of valid depth (raw units) in the central `inner` part of box (x1, y1, x2, y2). None if no depth."""
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    hw, hh = (x2 - x1) * inner / 2, (y2 - y1) * inner / 2
    roi = depth[max(0, int(cy - hh)):int(cy + hh) + 1, max(0, int(cx - hw)):int(cx + hw) + 1]
    valid = roi[roi > 0]
    return float(np.median(valid)) if valid.size else None


def plane_mask(depth, box, knob_d, tol, grow=2.0):
    """Square donut (bbox_large - bbox) & |depth - knob_d| < tol -> panel pixels around the knob.

    knob_d and tol in raw depth units. bbox_large = bbox scaled by `grow` around the same center.
    """
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    hw, hh = (x2 - x1) * grow / 2, (y2 - y1) * grow / 2
    ring = np.zeros(depth.shape, bool)
    ring[max(0, int(cy - hh)):int(cy + hh) + 1, max(0, int(cx - hw)):int(cx + hw) + 1] = True
    ring[max(0, int(y1)):int(y2) + 1, max(0, int(x1)):int(x2) + 1] = False
    return ring & (depth > 0) & (np.abs(depth.astype(np.float32) - knob_d) < tol)


def plane_normal(depth, mask, unit, intr, min_points=50):
    """Fit a plane to the masked pixels in the camera frame [m] (x right, y down, z forward).

    Returns (unit normal pointing at the camera, centroid, rms [m]) or None if too few points.
    ponytail: plain SVD least squares, the depth mask is the only outlier filter.
    Add RANSAC if walls/edges inside the tol band skew the normal. Lens distortion ignored.
    """
    v, u = np.nonzero(mask)
    if v.size < min_points:
        return None
    fx, fy, ppx, ppy = intr
    z = depth[v, u] * unit
    pts = np.column_stack(((u - ppx) / fx * z, (v - ppy) / fy * z, z))
    c = pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[2]
    if n @ c > 0:  # flip to face the camera
        n = -n
    return n, c, s[2] / np.sqrt(len(pts))


def project(p, intr):
    fx, fy, ppx, ppy = intr
    return int(fx * p[0] / p[2] + ppx), int(fy * p[1] / p[2] + ppy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.join(YOLO_DIR, 'weights', 'v4.pt'))
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--serial', help='device serial when several cameras are plugged in')
    ap.add_argument('--grow', type=float, default=2.0, help='bbox_large = bbox x grow (same center)')
    ap.add_argument('--tol', type=float, default=30,
                    help='panel depth band around knob depth [mm]; must exceed the knob protrusion')
    ap.add_argument('--near', type=int, default=200, help='heatmap red at this depth [mm]')
    ap.add_argument('--far', type=int, default=300, help='heatmap blue at this depth [mm]')
    ap.add_argument('--headless', action='store_true', help='terminal output only, no window')
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    knob = next(i for i, n in model.names.items() if n == 'knob')
    print(f'model={args.model} knob class={knob} grow={args.grow} tol={args.tol}mm  (q/ESC to quit)')

    frames = aligned_frames(args.serial)
    last_print = 0.0
    try:
        for color, depth, unit, intr in frames:
            r = model.predict(color, conf=args.conf, classes=[knob], verbose=False)[0]
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            tol = args.tol / (unit * 1000)  # mm -> raw units

            panel = np.zeros(depth.shape, bool)
            found = []  # (box, conf, knob depth raw, plane fit)
            for b, c in zip(boxes, confs):
                d = bbox_depth(depth, b)
                fit = None
                if d is not None:
                    m = plane_mask(depth, b, d, tol, args.grow)
                    panel |= m
                    fit = plane_normal(depth, m, unit, intr)
                found.append((b, c, d, fit))

            now = time.time()
            if now - last_print > 0.5:
                last_print = now
                lines = []
                for b, c, d, fit in found:
                    s = f'knob conf={c:.2f} center=({(b[0] + b[2]) / 2:.0f},{(b[1] + b[3]) / 2:.0f}) '
                    s += f'depth={d * unit:.3f}m' if d is not None else 'depth=none'
                    if fit:
                        n, _, rms = fit
                        tilt = np.degrees(np.arccos(np.clip(-n[2], -1, 1)))
                        s += (f' normal=({n[0]:+.3f},{n[1]:+.3f},{n[2]:+.3f}) tilt={tilt:.1f}deg '
                              f'rms={rms * 1000:.1f}mm')
                    elif d is not None:
                        s += ' normal=none (too few panel points)'
                    lines.append(s)
                print('\n'.join(lines) or 'no knob', flush=True)
            if args.headless:
                continue

            # right: panel depth map (donut & depth band) as heatmap, black elsewhere
            heat = depth_to_heatmap(np.where(panel, depth * (unit * 1000), 0), args.near, args.far)
            view = color.copy()
            for b, c, d, fit in found:
                x1, y1, x2, y2 = b.astype(int)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                hw, hh = int((x2 - x1) * args.grow / 2), int((y2 - y1) * args.grow / 2)
                label = f'knob {c:.2f} ' + (f'{d * unit * 1000:.0f}mm' if d is not None else 'no depth')
                for img in (view, heat):
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.rectangle(img, (cx - hw, cy - hh), (cx + hw, cy + hh), (255, 255, 0), 1)
                    cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 2)
                    cv2.putText(img, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 0), 2)
                    if fit:  # normal as a 5 cm arrow from the panel centroid
                        n, p, _ = fit
                        cv2.arrowedLine(img, project(p, intr), project(p + 0.05 * n, intr),
                                        (255, 0, 255), 2, tipLength=0.25)
            cv2.imshow('knob_depth: color | panel depth (donut & depth band)', np.hstack((view, heat)))
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        frames.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
