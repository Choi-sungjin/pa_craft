#!/usr/bin/python3
"""Live: colour image with the box top square and the distance to the box (box_range.py on every frame).

  /usr/bin/python3 box_live.py [--serial 261322075477] [--cam-height 0.627] [--cam-pitch 43.6] [--lidar-ahead 0.2495]
  q / Esc closes. Green: the 205 mm square (completed from the far edge when the near edge is hidden),
  yellow: what was seen. Big text: the robot (LiDAR face) to the box near edge; below: box centre ahead / right.
"""
import argparse
import os
import struct
import subprocess
import time

import cv2
import numpy as np

import box_range as br

HERE = os.path.dirname(os.path.abspath(__file__))
# Top-down (orthographic onto the floor XY plane) panel: ahead is up, the robot's right is right.
TD_AHEAD = (-0.10, 0.95)      # [m] range shown, from the camera foot point
TD_SIDE = (-0.45, 0.45)


def topdown(d, K, up, cam_h, size_px, lidar, S=None, info=None):
    """Render the orthographic floor-plane view: depth points by height, LiDAR, box square, distances."""
    H = size_px; ppm = H / (TD_AHEAD[1] - TD_AHEAD[0]); W = int(ppm * (TD_SIDE[1] - TD_SIDE[0]))
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd); right = -np.cross(up, fwd)
    to_px = lambda a, r: (np.int32((r - TD_SIDE[0]) * ppm), np.int32((TD_AHEAD[1] - a) * ppm))
    img = np.full((H, W, 3), 18, np.uint8)
    h, w = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h:2, 0:w:2]; z = d[::2, ::2]; ok = z > 0
    P = np.dstack([(uu - cx) * z / fx, (vv - cy) * z / fy, z])[ok]
    a, r, hz = P @ fwd, P @ right, cam_h + P @ up
    keep = (a > TD_AHEAD[0]) & (a < TD_AHEAD[1]) & (r > TD_SIDE[0]) & (r < TD_SIDE[1]) & (hz < 0.4)
    order = np.argsort(hz[keep])                                   # higher points drawn last (on top)
    X, Y = to_px(a[keep][order], r[keep][order])
    col = cv2.applyColorMap(np.uint8(np.clip(hz[keep][order] / 0.12, 0, 1) * 255)[:, None], cv2.COLORMAP_VIRIDIS)[:, 0]
    X, Y = np.clip(X, 0, W - 1), np.clip(Y, 0, H - 1)
    img[Y, X] = col; img[np.clip(Y + 1, 0, H - 1), X] = col
    for m in np.arange(0, TD_AHEAD[1], 0.1):                         # 100 mm grid, camera foot = 0
        y = int((TD_AHEAD[1] - m) * ppm); cv2.line(img, (0, y), (W, y), (60, 60, 60), 1)
        cv2.putText(img, f'{m * 1000:.0f}', (4, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
    lx, ly = to_px(np.array([lidar[0]]), np.array([lidar[1]])); lc = (int(lx[0]), int(ly[0]))
    fx_, fy_ = to_px(np.array([lidar[0] + 0.019] * 2), np.array([lidar[1] - 0.15, lidar[1] + 0.15]))
    cv2.line(img, (int(fx_[0]), int(fy_[0])), (int(fx_[1]), int(fy_[1])), (0, 0, 255), 2)      # LiDAR face line
    cv2.circle(img, lc, max(2, int(0.022 * ppm)), (0, 0, 255), -1)                             # LiDAR centre
    cv2.putText(img, 'LiDAR', (lc[0] + 10, lc[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    if S is not None:
        Sa, Sr = S @ fwd, S @ right; bx, by = to_px(Sa, Sr)
        cv2.polylines(img, [np.column_stack([bx, by])], True, (0, 255, 0), 2, cv2.LINE_AA)
        k = np.argsort(Sa)[:2]                                              # near edge, its centre = (x1, y1)
        cv2.line(img, (int(bx[k[0]]), int(by[k[0]])), (int(bx[k[1]]), int(by[k[1]])), (255, 0, 255), 3, cv2.LINE_AA)
        bc = (int(bx[k].mean()), int(by[k].mean()))
        f0 = to_px(np.array([lidar[2]]), np.array([lidar[1]])); lc = (int(f0[0][0]), int(f0[1][0]))   # (x0, y0): face
        cv2.line(img, lc, bc, (255, 255, 255), 2, cv2.LINE_AA)                                   # centre to centre
        corner = (lc[0], bc[1])
        cv2.line(img, lc, corner, (0, 200, 255), 1, cv2.LINE_AA); cv2.line(img, corner, bc, (0, 200, 255), 1, cv2.LINE_AA)
        if info is not None:
            cv2.putText(img, f'{info[0] * 1000:.0f} mm', ((lc[0] + bc[0]) // 2 + 8, (lc[1] + bc[1]) // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, f'x {info[1] * 1000:+.0f}', (corner[0] + 6, (corner[1] + lc[1]) // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.putText(img, f'y {info[2] * 1000:+.0f}', ((corner[0] + bc[0]) // 2 - 20, corner[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
    cv2.putText(img, 'top-down XY (orthographic), ahead = up', (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serial', default='261322075477')
    ap.add_argument('--cam-height', type=float, default=br.CAM_HEIGHT)  # 0.6229 m, triangulation (box_range.py)
    ap.add_argument('--cam-pitch', type=float, default=43.6)    # floor_pose.py 2026-11-07, roll 0.0
    ap.add_argument('--lidar-ahead', type=float, default=0.2525)  # box_range.py: provisional, re-take on the carpet
    ap.add_argument('--lidar-right', type=float, default=0.0322)  # LiDAR centre right of the colour axis: box_range.py
    ap.add_argument('--lens-y', type=float, default=0.0322)       # colour lens y in base_link (datasheet; V307 says -0.0325)
    ap.add_argument('--cam-yaw', type=float, default=br.CAM_YAW_DEG)
    ap.add_argument('--x-bias', type=float, default=br.X_BIAS)          # added to x1-x0 [m]
    ap.add_argument('--scale', type=float, default=2.0, help='window size factor')
    a = ap.parse_args()
    src = subprocess.Popen([os.path.join(HERE, 'build', 'rs_stream'), a.serial], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    hdr = src.stdout.read(4 + 8 + 20)
    assert hdr[:4] == b'RSF1', 'rs_stream did not start'
    w, h = struct.unpack('<2i', hdr[4:12]); fx, fy, cx, cy, sc = struct.unpack('<5f', hdr[12:32])
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    # Camera pose from V307 (src/craft_description) with the colour lens at y +32.2 mm (datasheet) and yaw +1.5 deg.
    Ru, tu, z_floor, laser = br.urdf_frames(br.URDF_DEFAULT)
    tu = tu.copy(); tu[1] = a.lens_y; tu[2] = z_floor + a.cam_height  # measured height (triangulation)
    cy_, sy_ = np.cos(np.radians(a.cam_yaw)), np.sin(np.radians(a.cam_yaw))
    Ru = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]]) @ Ru
    up = Ru.T @ np.array([0, 0, 1.0])
    rng = np.random.default_rng(0)
    cv2.namedWindow('box_range', cv2.WINDOW_AUTOSIZE)
    hist, fps, last, logged = [], 0.0, time.time(), 0.0
    ema, ema_S, shown = None, None, None
    ALPHA = 0.25                                                     # EMA weight of a new frame
    try:
        while True:
            src.stdin.write(b'n'); src.stdin.flush()
            color = np.frombuffer(src.stdout.read(w * h * 3), np.uint8).reshape(h, w, 3).copy()
            d = np.frombuffer(src.stdout.read(w * h * 2), '<u2').reshape(h, w) * sc
            view = cv2.resize(color, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_LINEAR)
            view[: int(view.shape[0] * 0.5)] //= 3                     # upper half not used: dimmed
            proj = lambda P3: np.column_stack([fx * P3[:, 0] / P3[:, 2] + cx, fy * P3[:, 1] / P3[:, 2] + cy]) * a.scale
            lines = []
            try:
                top, cen, n, area, rms = br.box_top(d, K, up, a.cam_height, np.random.default_rng(0), 0.5,
                                                    a.lidar_ahead + br.MAX_GAP + br.SIDE)
                C3, px = br.rectangle(top, K, cen, n)
                _, cut, width, depth_seen, g, _ = br.geometry(C3, px, K, up, a.cam_height)
                seen = br.far_corner_visibility(d, K, cen, n, C3, up)
                S, flags = br.complete_square_far(C3, up, seen)
                c = g['square (depth)']; gap = c['ahead'] - br.SIDE / 2 - a.lidar_ahead
                Sb = S @ Ru.T + tu                                         # base_link
                near_b = Sb[np.argsort(Sb[:, 0])[:2]].mean(0)
                d01 = near_b[:2] - laser[:2]; d01[0] += a.x_bias          # persistent -7 mm: add 7 mm (user)
                hist = (hist + [(np.hypot(*d01), d01[0], d01[1])])[-7:]
                m = np.median(np.array(hist), 0)                         # drop single-frame outliers
                ema = m if ema is None else (1 - ALPHA) * ema + ALPHA * m
                ema_S = S if ema_S is None else (1 - ALPHA) * ema_S + ALPHA * S
                if shown is None or np.any(np.abs(ema - shown) >= 0.001):  # 1 mm deadband: no flicker
                    shown = np.round(ema, 3)
                cv2.polylines(view, [np.int32(proj(ema_S))], True, (0, 255, 0), 3, cv2.LINE_AA)
                fwd_ = np.array([0, 0, 1.0]) - up[2] * up; fwd_ /= np.linalg.norm(fwd_)
                near2 = ema_S[np.argsort(ema_S @ fwd_)[:2]]
                nc = proj(near2.mean(0)[None])[0]
                cv2.line(view, tuple(np.int32(proj(near2[:1])[0])), tuple(np.int32(proj(near2[1:])[0])), (255, 0, 255), 4, cv2.LINE_AA)
                cv2.drawMarker(view, (int(nc[0]), int(nc[1])), (255, 0, 255), cv2.MARKER_CROSS, 26, 3)
                lines = [f'front_laser -> box near edge (XY)  {shown[0] * 1000:.0f} mm',
                         f'(x1-x0) {shown[1] * 1000:+.0f} ahead, (y1-y0) {shown[2] * 1000:+.0f} left  '
                         f'[magenta: near edge, + its centre (x1,y1)]',
                         'far corners seen ' + ' / '.join('yes' if v else 'NO' for v in seen.values())
                         + (f'  {"; ".join(flags)}' if flags else '')]
            except RuntimeError as e:
                hist, ema, ema_S, shown = [], None, None, None
                lines = ['no box: ' + str(e)]
            now = time.time(); fps = 0.8 * fps + 0.2 / max(now - last, 1e-3); last = now
            lines.append(f'{fps:.1f} fps (median 7 + EMA, 1 mm deadband)')
            for i, s in enumerate(lines):
                big = i == 0 and s.startswith('front_laser')
                size, thick, y = (1.6, 4, 60) if big else (0.75, 2, 60 + 44 + 30 * (i - 1) if i else 40)
                cv2.putText(view, s, (14, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), thick + 4, cv2.LINE_AA)
                cv2.putText(view, s, (14, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 255, 0) if big else (255, 255, 255),
                            thick, cv2.LINE_AA)
            if now - logged >= 1.0:                                     # status line once a second (log)
                print(' | '.join(lines), flush=True); logged = now
            td = topdown(d, K, up, a.cam_height, view.shape[0], (a.lidar_ahead - br.LIDAR_FACE_TO_CENTRE, a.lidar_right, a.lidar_ahead),
                         ema_S if (lines and lines[0].startswith('front_laser')) else None,
                         shown if (lines and lines[0].startswith('front_laser')) else None)
            view = np.hstack([view, td])
            cv2.imshow('box_range', view)
            if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                break
    finally:
        src.stdin.close(); src.terminate(); cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
