#!/usr/bin/env python3
"""Run after sourcing the workspace: python3 test_lidar_integration.py.

Exercises the master's LiDAR module with synthetic scans/static TF;
no serial drivers, cameras or motors. Uses an isolated ROS domain.
"""
import math
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-mode', choices=('raw', 'merged'), default='raw')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(30 + os.getpid() % 150)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import PoseArray, TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('lidar_contract_test')
    outputs = []
    merged_stamps = set()
    node.create_subscription(LaserScan, '/lidar/merged',
                             lambda scan: merged_stamps.add((scan.header.stamp.sec,
                                                            scan.header.stamp.nanosec)),
                             qos_profile_sensor_data)
    node.create_subscription(PoseArray, '/perception/lidar/obstacle', outputs.append, 10)
    publishers = [node.create_publisher(LaserScan, f'/lidar/{side}/scan',
                                       qos_profile_sensor_data) for side in ('front', 'rear')]
    tf_pub = node.create_publisher(TFMessage, '/tf_static',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transforms = []
    for side, x, yaw in [('front', 0.15, 0.0), ('rear', -0.15, math.pi)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', f'laser_{side}'
        tf.transform.translation.x = x
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        transforms.append(tf)
    tf_pub.publish(TFMessage(transforms=transforms))

    processes = []
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            config = Path(get_package_share_directory('craft_perception')) / 'config/perception.yaml'
            master = [str(Path(get_package_prefix('craft_perception')) / 'lib/craft_perception/perception_master'),
                      '--ros-args', '--params-file', str(config), '-p', f'lidar.input_mode:={args.input_mode}']
            # Keep only the LiDAR task; camera configuration is still validated.
            for task in ('d435_yolo', 'apriltag', 'knob_depth', 'object_detector',
                         'obstacle_detector', 'puck_axis', 'knob_lever', 'state_estimator'):
                master += ['-p', f'schedule.{task}.enabled:=false']
            commands = [master]
            if args.input_mode == 'merged':
                commands.insert(0, [str(Path(get_package_prefix('lidar_processor')) /
                                       'lib/lidar_processor/lidar_processor')])
            for command in commands:
                processes.append(subprocess.Popen(command, stdout=log, stderr=log))

            def publish(ranges):
                stamp = node.get_clock().now().to_msg()
                # Rear first so its matching sample is available to the front callback.
                for index in (1, 0):
                    scan = LaserScan()
                    scan.header.stamp = stamp
                    scan.header.frame_id = ('laser_front', 'laser_rear')[index]
                    scan.angle_min, scan.angle_increment = 0.0, 0.1
                    scan.angle_max = 0.3
                    scan.range_min, scan.range_max = 0.1, 8.0
                    scan.ranges = ranges
                    publishers[index].publish(scan)

            def wait_for(ranges, predicate):
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    assert all(p.poll() is None for p in processes), 'node exited'
                    publish(ranges)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if any(predicate(result) for result in outputs):
                        return
                raise AssertionError('no matching LiDAR output')

            def projected(result):
                if len(result.poses) != 2:
                    return False
                assert result.header.frame_id == 'base_link'
                if (result.header.stamp.sec, result.header.stamp.nanosec) not in merged_stamps:
                    return False
                xs = sorted(p.position.x for p in result.poses)
                assert abs(xs[0] + 1.15) < 0.02 and abs(xs[1] - 1.15) < 0.02, xs
                for pose in result.poses:
                    assert abs(pose.position.y) < 0.02 and pose.position.z == 0
                    assert pose.orientation.w == 1
                return True

            # Far points survive merging but must be removed by the master's 3 m limit.
            wait_for([1.0, math.nan, math.inf, 4.0], projected)
            outputs.clear()
            wait_for([math.nan] * 4, lambda result: not result.poses)
            print(f'PASS ({args.input_mode}): best-effort raw scans -> reliable merged base_link scan -> '
                  'master PoseArray; front/rear positions, range filtering and empty output')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            for process in processes:
                process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
