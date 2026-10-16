#!/usr/bin/env python3
"""RealSense depth map -> heatmap (near = red, far = blue, no depth = black).

usage: python3 depth_heatmap.py [--device /dev/realsense_depth] [--near 200] [--far 300]
  --device: D435 depth (Z16) node; /dev/realsense_depth is the udev symlink from
            99-realsense-d435-streams.rules.
  --near/--far: color range in mm (D435 Z16 = 1 mm per unit). q or ESC to quit.
"""
import argparse
import fcntl
import itertools
import mmap
import os
import select
import struct
import time

import cv2
import numpy as np

# OpenCV 4.6 / GStreamer can't open the D435 Z16 node, so talk to V4L2 directly (x86_64 struct layouts).
Z16 = struct.unpack('<I', b'Z16 ')[0]
BUF_TYPE_CAPTURE, MEMORY_MMAP = 1, 1
VIDIOC_S_FMT, VIDIOC_REQBUFS = 0xC0D05605, 0xC0145608
VIDIOC_QUERYBUF, VIDIOC_QBUF, VIDIOC_DQBUF = 0xC0585609, 0xC058560F, 0xC0585611
VIDIOC_STREAMON, VIDIOC_STREAMOFF = 0x40045612, 0x40045613
FMT = 'I4xIIIIIIII168x'  # v4l2_format: type, pad, v4l2_pix_format(8 u32), rest of 200-byte union
BUF = 'IIIII4x16x16xIII4xII4x4x'  # v4l2_buffer: index,type,bytesused,flags,field,sequence,memory,m.offset,length,reserved2


def depth_frames(dev, w, h, nbufs=4):
    """Yield uint16 (h, w) depth frames in mm from device path dev. Raises OSError if not a Z16 node."""
    fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
    try:
        fmt = bytearray(struct.pack(FMT, BUF_TYPE_CAPTURE, w, h, Z16, 0, w * 2, 0, 0, 0))
        fcntl.ioctl(fd, VIDIOC_S_FMT, fmt)
        _, gw, gh, gfmt = struct.unpack(FMT, fmt)[:4]
        if gfmt != Z16 or (gw, gh) != (w, h):
            raise OSError(f'{dev}: no Z16 {w}x{h}')
        fcntl.ioctl(fd, VIDIOC_REQBUFS, bytearray(struct.pack('IIII4x', nbufs, BUF_TYPE_CAPTURE, MEMORY_MMAP, 0)))
        maps = []
        for i in range(nbufs):
            buf = bytearray(struct.pack(BUF, i, BUF_TYPE_CAPTURE, 0, 0, 0, 0, MEMORY_MMAP, 0, 0, 0))
            fcntl.ioctl(fd, VIDIOC_QUERYBUF, buf)
            offset, length = struct.unpack(BUF, buf)[7:9]
            maps.append(mmap.mmap(fd, length, mmap.MAP_SHARED, mmap.PROT_READ, offset=offset))
            fcntl.ioctl(fd, VIDIOC_QBUF, buf)
        fcntl.ioctl(fd, VIDIOC_STREAMON, struct.pack('I', BUF_TYPE_CAPTURE))
        try:
            while True:
                if not select.select([fd], [], [], 2.0)[0]:
                    raise OSError(f'{dev}: frame timeout')
                buf = bytearray(struct.pack(BUF, 0, BUF_TYPE_CAPTURE, 0, 0, 0, 0, MEMORY_MMAP, 0, 0, 0))
                fcntl.ioctl(fd, VIDIOC_DQBUF, buf)
                idx = struct.unpack(BUF, buf)[0]
                frame = np.frombuffer(maps[idx], np.uint16, w * h).reshape(h, w).copy()
                fcntl.ioctl(fd, VIDIOC_QBUF, buf)  # hand buffer back to the driver
                yield frame
        finally:
            fcntl.ioctl(fd, VIDIOC_STREAMOFF, struct.pack('I', BUF_TYPE_CAPTURE))
            for m in maps:
                m.close()
    finally:
        os.close(fd)


def depth_to_heatmap(depth_mm, near, far):
    """uint16 depth (mm) -> BGR heatmap. 0 (no depth) stays black."""
    d = np.clip(depth_mm.astype(np.float32), near, far)
    scaled = ((far - d) * (255.0 / (far - near))).astype(np.uint8)  # near -> 255 -> red
    heat = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    heat[depth_mm == 0] = 0
    return heat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/realsense_depth')
    ap.add_argument('--near', type=int, default=200)
    ap.add_argument('--far', type=int, default=300)
    ap.add_argument('--size', default='640x480', help='depth resolution WxH')
    args, _ = ap.parse_known_args()

    w, h = (int(v) for v in args.size.split('x'))
    try:
        frames = depth_frames(args.device, w, h)
        next_frame = next(frames)
    except OSError as e:
        raise SystemExit(f'{e} (RealSense plugged in? udev rule installed?)')
    print(f'depth device={args.device} -> {os.path.realpath(args.device)} {w}x{h} range={args.near}-{args.far}mm  (q/ESC to quit)')

    last_print = 0.0
    for depth in itertools.chain([next_frame], frames):

        now = time.time()
        if now - last_print > 0.5:
            last_print = now
            valid = depth[depth > 0]
            center = depth[h // 2, w // 2]
            print(f'center={center}mm  valid={valid.size * 100 // depth.size}%  '
                  f'min={valid.min() if valid.size else 0}mm max={valid.max() if valid.size else 0}mm')

        heat = depth_to_heatmap(depth, args.near, args.far)
        cv2.drawMarker(heat, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(heat, f'{depth[h // 2, w // 2]} mm', (w // 2 + 12, h // 2 - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow('depth_heatmap', heat)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
