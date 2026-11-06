"""Mapping run in one launch: control_master, LiDAR + EKF + slam_toolbox, perception (scan merge, IMU,
LiDAR odometry) and a rosbag of the mapping topics. Same as bro + slaminfra + slamper + the bag command.
Teleop stays in its own terminal (it reads the keyboard): slamwasd.

  ros2 launch craft_bringup slam.launch.py [lidar_odom:=true] [record:=true] [bag:=loop_7]
  save BEFORE stopping:  ros2 run craft_bringup save_map.sh arena2

The bag goes to <ws>/test/drive/logs/<bag>; without bag:= it is the next free loop_<n>. Unlike bro, no
board check runs first: checkwheel / checkarm by hand if the wheels do not move.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Analysis inputs: test/drive/odom_trajectories.py and slam_graph.py read these; the map also lets a
# missed save be recovered from the bag.
BAG_TOPICS = ['/control/wheel/odom', '/perception/lidar/odom', '/odometry/local', '/perception/lidar/pose',
              '/perception/lidar/map', '/perception/lidar/slam_toolbox/pose_graph',
              '/perception/lidar/slam_toolbox/loop_closure_event',
              '/perception/lidar/slam_toolbox/new_node_event']


def _setup(context):
    share = get_package_share_directory('craft_bringup')
    craft = str(Path(share) / 'launch' / 'craft.launch.py')
    common = {'localization': 'slam', 'start_cameras': 'false',
              'lidar_odom': LaunchConfiguration('lidar_odom').perform(context),
              'slam_resolution': LaunchConfiguration('slam_resolution').perform(context),
              'arm': LaunchConfiguration('arm').perform(context)}
    # One scoped include per group, so each keeps its own group:= (craft.launch.py runs one group or all).
    actions = [GroupAction([IncludeLaunchDescription(craft, launch_arguments={**common, 'group': g}.items())])
               for g in ('control', 'infra', 'perception')]
    if LaunchConfiguration('record').perform(context) == 'true':
        logs = Path(share).parents[3] / 'test' / 'drive' / 'logs'  # <ws>/install/craft_bringup/share/craft_bringup
        logs.mkdir(parents=True, exist_ok=True)
        name = LaunchConfiguration('bag').perform(context)
        if not name:
            n = 1
            while (logs / f'loop_{n}').exists():
                n += 1
            name = f'loop_{n}'
        if (logs / name).exists():
            raise RuntimeError(f'{logs / name} exists: pick another bag:=')
        print(f'[slam.launch] recording {logs / name}')
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(logs / name), '--topics', *BAG_TOPICS],
                                      output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_odom', default_value='true', choices=['true', 'false'],
                              description='LiDAR odometry as the EKF x/y source'),
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='false: control without the arm board (arm battery out)'),
        DeclareLaunchArgument('slam_resolution', default_value='0.02', description='map cell [m] (slaminfra uses 0.02)'),
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false'],
                              description='record the mapping topics to <ws>/test/drive/logs'),
        DeclareLaunchArgument('bag', default_value='', description='bag name; empty = next free loop_<n>'),
        OpaqueFunction(function=_setup),
    ])
