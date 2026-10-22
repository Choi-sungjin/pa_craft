#!/usr/bin/env python3
"""Synthetic cameras + real NCNN inference; no physical devices or motor commands.

Run after sourcing ROS and the built workspace. Requires python3-opencv/rclpy.
Uses unique shm files and a separate ROS domain; never opens a real camera.
"""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

# Tasks that read the D435 frame besides d435_yolo/apriltag (puck_axis runs its own model).
D435_CONSUMERS = ('knob_depth', 'object_detector', 'obstacle_detector', 'puck_axis')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--puck-model', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'models' / 'puck_blue')
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config' / 'perception.yaml')
    parser.add_argument('--low-copy', action='store_true',
                        help='use 2Hz vision modules with 30Hz cameras and check copy reduction')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(30 + os.getpid() % 150)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import cv2
    import numpy as np
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import CameraInfo, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32MultiArray, UInt64MultiArray

    names = [f'/craft_test_{os.getpid()}_rs', f'/craft_test_{os.getpid()}_c310']
    files, child = [], None
    rclpy.init()
    node = rclpy.create_node('perception_synthetic_input')
    detections, statuses = [], {}
    odometry_results, obstacle_results = [], []
    node.create_subscription(Int32MultiArray, '/perception/vision/d435/apriltag',
                             lambda msg: detections.append(list(msg.data)), 10)

    def on_diagnostic(message):
        for status in message.status:
            statuses[status.name] = {item.key: float(item.value) for item in status.values}

    node.create_subscription(DiagnosticArray, '/perception/diagnostics', on_diagnostic, 10)
    from geometry_msgs.msg import PoseArray
    node.create_subscription(Odometry, '/perception/state/odom', odometry_results.append, 10)
    node.create_subscription(PoseArray, '/perception/lidar/obstacle', obstacle_results.append, 10)
    odometry_pub = node.create_publisher(Odometry, '/wheel/odom', 1)
    scan_pub = node.create_publisher(LaserScan, '/lidar/merged', 1)
    publishers = [node.create_publisher(UInt64MultiArray, topic, 1)
                  for topic in ('/vision/d435/frame_meta', '/vision/c310/frame_meta')]
    info_pub = node.create_publisher(CameraInfo, '/vision/d435/camera_info', 10)
    image = np.full((360, 640, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, 7, 100)
    image[100:200, 100:200] = marker[:, :, None]
    depth = np.full((360, 640), 250, dtype=np.uint16)
    payloads = [image.tobytes() + depth.tobytes(), image.tobytes()]
    sizes = [len(payload) + 8 for payload in payloads]
    log = tempfile.TemporaryFile(mode='w+')
    overrides = tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml')
    detector_hz, apriltag_hz = (2.0, 2.0) if args.low_copy else (100.0, 7.0)
    overrides.write('perception_master:\n  ros__parameters:\n'
                    '    lidar.input_mode: merged\n'
                    f'    schedule.d435_yolo.hz: {detector_hz}\n'
                    f'    schedule.knob_lever.hz: {detector_hz}\n'
                    f'    schedule.apriltag.hz: {apriltag_hz}\n')
    if args.low_copy:
        overrides.write(''.join(f'    schedule.{name}.hz: 2.0\n' for name in D435_CONSUMERS)
                        + '    max_input_age_s: 1.5\n')
    overrides.flush()
    try:
        for name, size in zip(names, sizes):
            fd = os.open('/dev/shm' + name, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
            files.append(fd)
            os.ftruncate(fd, 4 * size)
        child = subprocess.Popen([
            args.executable, '--ros-args', '--params-file', str(args.config.resolve()),
            '--params-file', overrides.name,
            '-p', f'd435.yolo.model_dir:={Path(args.model).resolve()}',
            '-p', f'c310.yolo.model_dir:={Path(args.model).resolve()}',
            '-p', f'd435.puck_yolo.model_dir:={args.puck_model.resolve()}',
            '-p', f'd435.shm:={names[0]}', '-p', f'c310.shm:={names[1]}',
        ], stdout=log, stderr=log)
        deadline, next_frame, sequence = time.monotonic() + 8, 0, 0
        next_odometry, next_scan = 0, 0
        while time.monotonic() < deadline:
            if child.poll() is not None:
                log.seek(0)
                raise AssertionError('master exited: ' + log.read())
            if time.monotonic() >= next_odometry:
                next_odometry = time.monotonic() + 0.01
                odom = Odometry()
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.pose.pose.orientation.w = 1.0
                odometry_pub.publish(odom)
            if time.monotonic() >= next_scan:
                next_scan = time.monotonic() + 0.05
                scan = LaserScan()
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.header.frame_id = 'laser'
                scan.angle_increment = 0.1
                scan.range_min, scan.range_max = 0.1, 5.0
                scan.ranges = [1.0, 2.0]
                scan_pub.publish(scan)
            if time.monotonic() >= next_frame:
                next_frame = time.monotonic() + 1 / 30
                sequence += 1
                stamp = node.get_clock().now()
                info = CameraInfo()
                info.header.stamp = stamp.to_msg()
                info.header.frame_id = 'camera_color_optical_frame'
                info.width, info.height = 640, 360
                info.k = [600., 0., 320., 0., 600., 180., 0., 0., 1.]
                info_pub.publish(info)
                for camera, fd in enumerate(files):
                    offset = (sequence % 4) * sizes[camera]
                    fcntl.flock(fd, fcntl.LOCK_EX)
                    os.pwrite(fd, payloads[camera] + struct.pack('Q', sequence), offset)
                    fcntl.flock(fd, fcntl.LOCK_UN)
                    meta = [offset, 640, 360, sequence, stamp.nanoseconds]
                    if camera == 0:
                        meta.append(1000000)
                    publishers[camera].publish(UInt64MultiArray(data=meta))
            rclpy.spin_once(node, timeout_sec=0.002)
        assert len(detections) >= (10 if args.low_copy else 40), f'AprilTag starved: {len(detections)}'
        assert all(ids == [7] for ids in detections), detections[:5]
        assert len(odometry_results) >= 500, f'odometry starved: {len(odometry_results)}'
        assert len(obstacle_results) >= 100, f'scan processing starved: {len(obstacle_results)}'
        for name in ('d435_yolo', 'apriltag', 'knob_lever', 'lidar_obstacle', 'state_estimator',
                     *D435_CONSUMERS):
            stats = statuses['perception/' + name]
            assert stats['processed'] > 0, (name, stats)
            assert stats['errors'] == 0, (name, stats)
        for name, expected in (('d435_yolo', detector_hz), ('knob_lever', detector_hz), ('apriltag', apriltag_hz)):
            assert statuses['perception/' + name]['configured_hz'] == expected
        for camera, key in enumerate(('realsense', 'c310')):
            stats = statuses['perception/shared_memory/' + key]
            assert stats['snapshots'] > 0
            assert stats['bytes_copied'] == stats['snapshots'] * len(payloads[camera])
            assert stats['snapshots'] <= stats['input_frames']
            if args.low_copy:
                assert stats['snapshots'] < stats['input_frames'] / 2, stats
            print(key, 'shared memory:', stats)
        # Stop input and ensure the master does not fabricate fresh results.
        until = time.monotonic() + 0.8
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        count = len(detections)
        until = time.monotonic() + 0.4
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert len(detections) == count, 'repeated AprilTag output without a new frame'
        print('Master integration passed:', len(detections), 'AprilTag results under two-camera NCNN load')
        print('Odometry results:', len(odometry_results), 'obstacle results:', len(obstacle_results))
        for name in ('d435_yolo', 'knob_lever', 'puck_axis', 'apriltag'):
            print(name, statuses['perception/' + name])
    finally:
        if child:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for fd in files:
            os.close(fd)
        for name in names:
            Path('/dev/shm' + name).unlink(missing_ok=True)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        overrides.close()


if __name__ == '__main__':
    main()
