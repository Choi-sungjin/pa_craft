#!/usr/bin/env python3
"""ArUco marker reader.

ROS mode (rclpy found, no --camera): frames come from realsense_processor through shared
memory /dev/shm/craft_vision; /vision/image_offset carries only [offset, width, height].
Publishes /vision/aruco (std_msgs/Int32MultiArray, ids left -> right) for mission_fsm.
Standalone (no rclpy, or --camera DEV): opens the camera itself.

usage: python3 aruco_reader.py [--camera /dev/realsense_color] [--dict DICT_4X4_50]   q or ESC to quit.
"""
import argparse
import mmap
import os
import time

import cv2
import numpy as np


def make_detector(dict_name):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return detector.detectMarkers
    params = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6 (apt)
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)


def detect(detect_fn, gray):
    """Return [(id, corners(4x2))] sorted by marker center x (left -> right)."""
    corners, ids, _ = detect_fn(gray)
    if ids is None:
        return []
    found = [(int(i), c.reshape(4, 2)) for c, i in zip(corners, ids.flatten())]
    return sorted(found, key=lambda m: m[1][:, 0].mean())


def draw(frame, markers):
    for order, (i, c) in enumerate(markers, 1):
        pts = c.astype(int)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        cv2.circle(frame, tuple(pts[0]), 5, (0, 0, 255), -1)  # top-left corner
        cx, cy = pts.mean(axis=0).astype(int)
        cv2.putText(frame, f'#{order} id:{i}', (cx - 40, cy), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 0, 0), 2)
    cv2.putText(frame, f'ids (left->right): {[i for i, _ in markers]}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)


def camera_frames(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'camera {device} open failed')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('frame read failed')
                return
            yield frame
    finally:
        cap.release()


def shm_frames(node, rclpy):
    """Yield zero-copy views into realsense_processor's shared memory."""
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import UInt64MultiArray
    latest = []
    node.create_subscription(UInt64MultiArray, '/vision/image_offset',
                             lambda m: latest.append(m.data), qos_profile_sensor_data)
    mm = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if not latest:
            cv2.waitKey(1)  # keep window responsive while waiting
            continue
        off, w, h = (int(v) for v in latest[-1])
        latest.clear()
        if mm is None:
            fd = os.open('/dev/shm/craft_vision', os.O_RDONLY)
            mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
            os.close(fd)
        yield np.ndarray((h, w, 3), np.uint8, buffer=mm, offset=off)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default=None,
                    help='open camera directly: /dev/realsense_color (udev symlink) or an index like 0')
    ap.add_argument('--dict', default='DICT_4X4_50')
    args, _ = ap.parse_known_args()  # ignore --ros-args from ros2 launch/run

    pub = rclpy = None
    if args.camera is None:
        try:
            import rclpy
            from std_msgs.msg import Int32MultiArray
        except ImportError:
            rclpy = None
    if rclpy:
        rclpy.init()
        node = rclpy.create_node('aruco_reader')
        pub = node.create_publisher(Int32MultiArray, '/vision/aruco', 10)
        frames = shm_frames(node, rclpy)
        print('ROS: /vision/image_offset (shm) -> /vision/aruco')
    else:
        cam = args.camera or '/dev/realsense_color'
        cam = int(cam) if cam.isdigit() else cam
        frames = camera_frames(cam)
        print(f'standalone: camera {cam}')
    detect_fn = make_detector(args.dict)
    print(f'dict={args.dict}  (q/ESC to quit)')

    last_print = 0.0
    for frame in frames:
        markers = detect(detect_fn, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if pub:
            pub.publish(Int32MultiArray(data=[i for i, _ in markers]))

        now = time.time()
        if now - last_print > 0.5:  # 30fps -> print twice a second
            last_print = now
            if markers:
                print(f'{len(markers)} marker(s): ' + ', '.join(
                    f'id={i} center=({c[:, 0].mean():.0f},{c[:, 1].mean():.0f}) '
                    f'size={np.linalg.norm(c[0] - c[1]):.0f}px' for i, c in markers))
            else:
                print('no marker')

        view = frame.copy()  # shm is read-only and shared; draw on a copy for display only
        draw(view, markers)
        cv2.imshow('aruco_reader', view)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    cv2.destroyAllWindows()
    if pub:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
