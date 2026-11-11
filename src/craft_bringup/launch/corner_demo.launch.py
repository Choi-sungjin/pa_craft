"""Corner demo stack: control, LiDAR + EKF + slam_toolbox localizing in a saved map, perception, planning
on the same map. Then, in another terminal (robot standing still anywhere in the map):

  ros2 launch craft_bringup corner_demo.launch.py [map:=/home/craft/maps/arena3_clean]
  /usr/bin/python3 ~/craft/demo/corner/run_corner.py [--dry-run]

map is the slam_toolbox save base: <map>.posegraph/.data for localization, <map>.yaml/.pgm for planning.
Unlike bro, no board check runs first: checkwheel / checkarm by hand if something does not move.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context):
    base = str(Path(LaunchConfiguration('map').perform(context)).expanduser())
    for ext in ('.posegraph', '.data', '.yaml'):
        if not Path(base + ext).is_file():
            raise RuntimeError(f'{base + ext} not found (map:= is the save base without extension)')
    craft = str(Path(get_package_share_directory('craft_bringup')) / 'launch' / 'craft.launch.py')
    common = {'localization': 'slam', 'start_cameras': 'false', 'slam_map': base, 'map_file': base + '.yaml',
              'lidar_odom': LaunchConfiguration('lidar_odom').perform(context),
              'arm': LaunchConfiguration('arm').perform(context),
              'lidar_fov_deg': LaunchConfiguration('lidar_fov_deg').perform(context)}
    return [GroupAction([IncludeLaunchDescription(craft, launch_arguments={**common, 'group': g}.items())])
            for g in ('control', 'infra', 'perception', 'planning')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(Path.home() / 'maps' / 'arena3_clean'),
                              description='saved map base path, no extension'),
        DeclareLaunchArgument('lidar_odom', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='false: drive with the wheel/clock board only'),
        DeclareLaunchArgument('lidar_fov_deg', default_value='180.0',
                              description='front and rear LiDAR field of view kept in /lidar/merged [deg]'),
        OpaqueFunction(function=_setup),
    ])
