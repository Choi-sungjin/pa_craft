import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = {
    'craft_sensors': ['realsense_processor', 'c310_processor'],
    'craft_perception': ['perception_master'],
    'craft_control': ['control_master'],
}


def generate_launch_description():
    slam_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam.yaml')
    camera_yaml = os.path.join(get_package_share_directory('craft_sensors'), 'config', 'camera.yaml')
    control_yaml = os.path.join(get_package_share_directory('craft_control'), 'config', 'control.yaml')
    planning_yaml = os.path.join(get_package_share_directory('craft_planning'), 'config', 'planning.yaml')
    perception_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'perception.yaml')
    params = {'craft_sensors': [camera_yaml], 'craft_control': [control_yaml],
              'craft_perception': [perception_yaml, {
                  'lidar.input_mode': LaunchConfiguration('lidar_input_mode'),
                  'd435.yolo.model_dir': LaunchConfiguration('d435_model_dir'),
                  'c310.yolo.model_dir': LaunchConfiguration('c310_model_dir')}]}

    nodes = [Node(package=pkg, executable=exe, output='screen',
                  parameters=params.get(pkg, []))
             for pkg, exes in NODES.items() for exe in exes]
    nodes.append(IncludeLaunchDescription(
        os.path.join(get_package_share_directory('lidar_processor'), 'launch',
                     'lidar_processor.launch.py'),
        launch_arguments={'start_processor': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar'))))
    nodes.append(Node(package='craft_planning', executable='planning_master', output='screen',
                      parameters=[planning_yaml, {'map_file': LaunchConfiguration('map_file')}]))
    nodes.append(IncludeLaunchDescription(  # 2D SLAM
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
        launch_arguments={'slam_params_file': slam_yaml}.items()))
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start dual LiDAR drivers and static TF; '
                                          'set false when already running on the Pi'),
        DeclareLaunchArgument('lidar_input_mode', default_value='raw',
                              description='raw: master merges scans; merged: use external /lidar/merged '
                                          '(also set start_lidar:=false)'),
        DeclareLaunchArgument('map_file', default_value='maps/maze.stl',
                              description='Gazebo binary STL, ROS map YAML or block text '
                                          '(relative to craft_planning share)'),
        DeclareLaunchArgument('d435_model_dir', default_value='models/knob_sign',
                              description='D435 YOLO NCNN model; relative to craft_perception share'),
        DeclareLaunchArgument('c310_model_dir', default_value='models/knob_sign',
                              description='C310 YOLO NCNN model (knob_lever); relative to craft_perception share'),
        *nodes,
    ])
