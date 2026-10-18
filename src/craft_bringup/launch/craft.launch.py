import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

NODES = {
    'craft_sensors': ['lidar_processor', 'realsense_processor', 'c310_processor', 'imu_processor',
                      'wheel_encoder_processor', 'arm_encoder_processor'],
    'craft_perception': ['state_estimator', 'obstacle_detector', 'object_pose_estimator',
                         'object_detector', 'aruco_reader'],
    'craft_mission': ['mission_fsm'],
    'craft_control': ['control_master'],
}


def generate_launch_description():
    slam_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam.yaml')
    camera_yaml = os.path.join(get_package_share_directory('craft_sensors'), 'config', 'camera.yaml')
    control_yaml = os.path.join(get_package_share_directory('craft_control'), 'config', 'control.yaml')
    params = {'craft_sensors': [camera_yaml], 'craft_control': [control_yaml]}
    map_file = os.path.join(get_package_share_directory('craft_planning'), 'maps', 'map.pgm')

    nodes = [Node(package=pkg, executable=exe, output='screen',
                  parameters=params.get(pkg, []))
             for pkg, exes in NODES.items() for exe in exes]
    nodes.append(Node(package='craft_planning', executable='planning_master', output='screen',
                      parameters=[{'map_file': map_file}]))
    nodes.append(IncludeLaunchDescription(  # 2D SLAM
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
        launch_arguments={'slam_params_file': slam_yaml}.items()))
    return LaunchDescription(nodes)
