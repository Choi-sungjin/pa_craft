"""Corner demo stack: control, LiDAR + EKF + slam_toolbox localizing in a saved map, perception, planning
on the same map. Then, in another terminal (robot standing still anywhere in the map):

  ros2 launch craft_bringup corner_demo.launch.py [map:=/home/craft/maps/arena5_legend_opt_arena1cm]
  /usr/bin/python3 ~/craft/demo/corner/run_corner.py [--dry-run]

map is the slam_toolbox save base: <map>.posegraph/.data for localization, <map>.yaml/.pgm for planning.
/perception/lidar/map is slam_toolbox's grid rendered from the pose graph; the planning map (<map>.yaml, e.g. 1 cm
with the boxes completed) goes out on /planning/map (RViz "planning map").
Unlike bro, no board check runs first: checkwheel / checkarm by hand if something does not move.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def _setup(context):
    base = str(Path(LaunchConfiguration('map').perform(context)).expanduser())
    for ext in ('.posegraph', '.data', '.yaml'):
        if not Path(base + ext).is_file():
            raise RuntimeError(f'{base + ext} not found (map:= is the save base without extension)')
    craft = str(Path(get_package_share_directory('craft_bringup')) / 'launch' / 'craft.launch.py')
    common = {'localization': 'slam', 'start_cameras': 'false', 'slam_map': base, 'map_file': base + '.yaml',
              'lidar_odom': LaunchConfiguration('lidar_odom').perform(context),
              'wheel_free': LaunchConfiguration('wheel_free').perform(context),
              'arm': LaunchConfiguration('arm').perform(context),
              'lidar_fov_deg': LaunchConfiguration('lidar_fov_deg').perform(context)}
    planning_map = LifecycleNode(package='nav2_map_server', executable='map_server', name='planning_map_server',
                                 namespace='', output='log', autostart=True,
                                 parameters=[{'yaml_filename': base + '.yaml', 'topic_name': '/planning/map'}])
    return [GroupAction([IncludeLaunchDescription(craft, launch_arguments={**common, 'group': g}.items())])
            for g in ('control', 'infra', 'perception', 'planning')] + [planning_map]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(Path.home() / 'maps' / 'arena5_legend_opt_arena1cm'),
                              description='saved map base path, no extension'),
        DeclareLaunchArgument('wheel_free', default_value='true', choices=['true', 'false'],
                              description='Exclude wheel odometry from EKFs; requires lidar_odom:=true'),
        DeclareLaunchArgument('lidar_odom', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='false: drive with the wheel/clock board only'),
        DeclareLaunchArgument('lidar_fov_deg', default_value='180.0',
                              description='front and rear LiDAR field of view kept in /lidar/merged [deg]'),
        OpaqueFunction(function=_setup),
    ])
