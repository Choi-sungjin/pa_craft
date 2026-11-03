"""Pure Pursuit straight-line test bringup: control_master + ekf_local + IMU TF only (no cameras/LiDAR/SLAM).

Same nodes and configs as craft.launch.py group:=control plus the infra EKF/TF, with two overrides:
task_timeout_s (control.yaml 30 s) so a slow 5 m run is not cut off, and require_arm_board=false so an
arm/LM fault does not hold the wheels. control.yaml itself is unchanged.
Run under both board locks (_craft_locked all); then test/drive/pp_straight.py in another terminal.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_yaml = os.path.join(get_package_share_directory('craft_control'), 'config', 'control.yaml')
    ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('task_timeout_s', default_value='120.0',
                              description='control task timeout for this test [s]'),
        Node(package='craft_control', executable='control_master', output='screen',
             parameters=[control_yaml, {'task_timeout_s': ParameterValue(
                 LaunchConfiguration('task_timeout_s'), value_type=float),
                 # Drive-only: the arm board port must exist, but its fault does not hold the wheels.
                 'require_arm_board': False}]),
        # OpenCR on-board IMU faces up: no rotation (craft.launch.py, offsets 0).
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
        Node(package='robot_localization', executable='ekf_node', name='ekf_local', output='screen',
             parameters=[ekf_yaml], remappings=[('odometry/filtered', '/odometry/local')]),
    ])
