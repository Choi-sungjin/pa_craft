#!/usr/bin/env python3
"""rs_bag.py recording (librealsense 2.59 rosbag2 .db3) -> rs_snap-style captures, no ROS / librealsense needed.

  python3 read_bag.py <bag_dir> [--step 3] [--out <bag_dir>/frames]     (numpy, opencv-python)

<bag_dir>/boxgap.db3 is a plain rosbag2 sqlite file: colour = sensor_msgs/CompressedImage PNG bgr8 640x360,
depth = CompressedImage '16UC1; compressedDepth png' 424x240 (12-byte header, then PNG), both at 30 fps.
Each colour frame is paired with the nearest depth frame (bag receive time, within 20 ms); depth is aligned to the
colour image like rs2::align(COLOR) (deproject with the depth intrinsics, depth->colour extrinsic, project, nearest
wins). Intrinsics / extrinsic: <bag_dir>/meta.json (no distortion: all coefficients are 0).
Writes <out>/t<ms from the first colour frame>_{color.ppm, depth.u16 (little-endian u16, row-major, 1 mm units),
meta.txt ("w h fx fy ppx ppy depth_scale serial")} = the format box_range.load() reads.
"""
import argparse
import json
import os
import sqlite3
import struct

import cv2
import numpy as np


def cdr_compressed_image(b):
    """sensor_msgs/CompressedImage CDR -> (format, data bytes)."""
    o = 4 + 8                                                           # encapsulation header, stamp
    n = struct.unpack_from('<I', b, o)[0]; o += 4 + n                   # frame_id
    o = (o + 3) & ~3; n = struct.unpack_from('<I', b, o)[0]; o += 4
    fmt = b[o:o + n - 1].decode(); o += n
    o = (o + 3) & ~3; n = struct.unpack_from('<I', b, o)[0]; o += 4
    return fmt, b[o:o + n]


def align_depth_to_color(dep_m, meta):
    """Raw depth [m] (depth image) -> depth [m] on the colour image grid, the rs2::align(COLOR) rule: each depth
    pixel's corners (u -+ 0.5, v -+ 0.5) are projected into the colour image, the covered colour pixels take its z,
    the nearest z wins where footprints overlap, footprints touching the border are dropped."""
    di, ci = meta['depth_intrinsics'], meta['color_intrinsics']
    R = np.array(meta['depth_to_color']['rotation']).reshape(3, 3).T   # librealsense stores it column-major
    t = np.array(meta['depth_to_color']['translation'])
    W, H = ci['width'], ci['height']
    v, u = np.nonzero(dep_m > 0); z = dep_m[v, u]

    def corner(o):
        P = np.stack([(u + o - di['ppx']) / di['fx'] * z, (v + o - di['ppy']) / di['fy'] * z, z], 1) @ R.T + t
        x = P[:, 0] / P[:, 2] * ci['fx'] + ci['ppx']; y = P[:, 1] / P[:, 2] * ci['fy'] + ci['ppy']
        return np.trunc(x + 0.5).astype(int), np.trunc(y + 0.5).astype(int)
    x0, y0 = corner(-0.5); x1, y1 = corner(0.5)
    ok = (x0 >= 0) & (y0 >= 0) & (x1 < W) & (y1 < H)
    x0, y0, x1, y1, z = x0[ok], y0[ok], x1[ok], y1[ok], z[ok]
    out = np.full(H * W, np.inf)
    for dy in range(int((y1 - y0).max(initial=0)) + 1):
        for dx in range(int((x1 - x0).max(initial=0)) + 1):
            m = (x0 + dx <= x1) & (y0 + dy <= y1)
            np.minimum.at(out, (y0[m] + dy) * W + x0[m] + dx, z[m])
    out[np.isinf(out)] = 0
    return out.reshape(H, W)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--step', type=int, default=3)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.bag_dir, 'meta.json')))
    out = a.out or os.path.join(a.bag_dir, 'frames'); os.makedirs(out, exist_ok=True)
    db = sqlite3.connect(os.path.join(a.bag_dir, 'boxgap.db3'))
    tid = {name: i for i, name in db.execute('select id, name from topics')}
    q = lambda name: db.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                                (tid[name],)).fetchall()
    col = q('/device_0/sensor_1/Color_0/image/data/compressed')
    dep = q('/device_0/sensor_0/Depth_0/image/data/compressedDepth')
    dts = np.array([t for t, _ in dep])
    ci = meta['color_intrinsics']; t_first = col[0][0]; n = 0
    for k, (tc, bc) in enumerate(col):
        if k % a.step:
            continue
        j = int(np.argmin(np.abs(dts - tc)))
        if abs(dts[j] - tc) > 20e6:
            continue
        img = cv2.imdecode(np.frombuffer(cdr_compressed_image(bc)[1], np.uint8), cv2.IMREAD_COLOR)
        d16 = cv2.imdecode(np.frombuffer(cdr_compressed_image(dep[j][1])[1][12:], np.uint8), cv2.IMREAD_UNCHANGED)
        al = align_depth_to_color(d16 * meta['depth_scale'], meta)
        p = os.path.join(out, f't{round((tc - t_first) / 1e6):06d}')
        cv2.imwrite(p + '_color.ppm', img)
        np.round(al / meta['depth_scale']).astype('<u2').tofile(p + '_depth.u16')
        with open(p + '_meta.txt', 'w') as f:
            f.write(f"{ci['width']} {ci['height']} {ci['fx']} {ci['fy']} {ci['ppx']} {ci['ppy']} "
                    f"{meta['depth_scale']} {meta['serial']}\n")
        n += 1
    print(f'{n} frames -> {out}')


if __name__ == '__main__':
    main()
