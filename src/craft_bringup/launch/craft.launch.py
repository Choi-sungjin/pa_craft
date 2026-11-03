import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

# group:= selects what this launch starts; the dashboard runs one group per pane.
GROUPS = ('all', 'perception', 'planning', 'control', 'infra')

# slam_toolbox publishers, remapped to /perception/lidar/<name>.
SLAM_TOPICS = ['pose', 'map', 'map_metadata', 'slam_toolbox/graph_visualization',
               'slam_toolbox/scan_visualization', 'slam_toolbox/pose_graph',
               'slam_toolbox/loop_closure_event', 'slam_toolbox/new_node_event']


def localization_is(mode):
    return PythonExpression(["'", LaunchConfiguration('localization'), f"' == '{mode}'"])


def in_group(name):
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration('group'), f"' in ('all', '{name}')"]))


def generate_launch_description():
    ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf.yaml')
    slam_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam.yaml')
    slam_ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam_ekf.yaml')
    slam_wide_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam_wide.yaml')
    ekf_lidar_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf_lidar.yaml')
    slam_localization_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config',
                                          'slam_localization.yaml')
    localize_in_map = ["'", LaunchConfiguration('slam_map'), "' != ''"]  # slam_map:= set: localization mode
    # lidar_odom:=true: ekf.yaml + ekf_lidar.yaml (LiDAR odometry x/y); repeating ekf.yaml is a no-op.
    ekf_params = [ekf_yaml, PythonExpression(["'", ekf_lidar_yaml, "' if '", LaunchConfiguration('lidar_odom'),
                                              "' == 'true' else '", ekf_yaml, "'"])]
    camera_yaml = os.path.join(get_package_share_directory('craft_sensors'), 'config', 'camera.yaml')
    control_yaml = os.path.join(get_package_share_directory('craft_control'), 'config', 'control.yaml')
    planning_yaml = os.path.join(get_package_share_directory('craft_planning'), 'config', 'planning.yaml')
    # arm:=false drives with the wheel/clock board alone: control skips the arm board, planning is Ready without arm joints.
    arm_enabled = ParameterValue(LaunchConfiguration('arm'), value_type=bool)
    perception_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'perception.yaml')
    perception_params = [perception_yaml, {
        'lidar.input_mode': LaunchConfiguration('lidar_input_mode'),
        # Each LiDAR's kept field of view around its outward axis [deg] (perception.yaml default 180 = +-90).
        'lidar.front_fov_deg': ParameterValue(LaunchConfiguration('lidar_fov_deg'), value_type=float),
        'lidar.rear_fov_deg': ParameterValue(LaunchConfiguration('lidar_fov_deg'), value_type=float),
        'odometry_source_topic': PythonExpression(
            ["'/odometry/global' if '", LaunchConfiguration('localization'), "' == 'ekf' else '/odometry/local'"]),
        'localization.enabled': localization_is('ekf'),
        'lidar_odometry.enabled': ParameterValue(LaunchConfiguration('lidar_odom'), value_type=bool),
        'd435.yolo.model_dir': LaunchConfiguration('d435_model_dir'),
        'c310.yolo.model_dir': LaunchConfiguration('c310_model_dir')}]

    masters = [
        Node(package='craft_perception', executable='perception_master', output='screen',
             parameters=perception_params, condition=in_group('perception')),
        Node(package='craft_planning', executable='planning_master', output='screen',
             parameters=[planning_yaml, {'map_file': LaunchConfiguration('map_file'), 'require_arm_joints': arm_enabled}],
             condition=in_group('planning')),
        Node(package='craft_control', executable='control_master', output='screen',
             parameters=[control_yaml, {'arm_enabled': arm_enabled}], condition=in_group('control')),
    ]

    # Everything that is not a master: camera drivers, LiDAR, EKF, SLAM.
    nodes = [Node(package='craft_sensors', executable=exe, output='screen', parameters=[camera_yaml],
                  condition=IfCondition(LaunchConfiguration('start_cameras')))
             for exe in ('realsense_processor', 'c310_processor')]
    nodes.append(IncludeLaunchDescription(
        os.path.join(get_package_share_directory('lidar_processor'), 'launch',
                     'lidar_processor.launch.py'),
        launch_arguments={'start_processor': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar'))))
    # ekf_local owns odom->base_link (wheel + IMU; + LiDAR odometry x/y with lidar_odom:=true). map->odom: slam_toolbox (localization:=slam, slam.yaml)
    # or ekf_global fusing slam_toolbox's pose (localization:=ekf, ekf.yaml + slam_ekf.yaml).
    nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_local', output='screen',
                      parameters=ekf_params, remappings=[('odometry/filtered', '/odometry/local')]))
    nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_global', output='screen',
                      parameters=ekf_params, remappings=[('odometry/filtered', '/odometry/global')],
                      condition=IfCondition(localization_is('ekf'))))
    # RViz: EKF odometry as a persistent point trail (/odometry/local/trail, latched Marker POINTS).
    nodes.append(Node(package='craft_bringup', executable='odom_trail.py', name='odom_trail', output='log'))
    # 2D SLAM, started directly (online_async_launch.py takes no remaps and defaults to sim time).
    # LiDAR-derived outputs live under /perception/lidar/; input stays /lidar/merged (slam.yaml).
    # slam_map:=<base> localizes in a saved pose graph instead (slam_localization.yaml, /initialpose).
    slam = LifecycleNode(package='slam_toolbox', executable=PythonExpression(
                             ["'localization_slam_toolbox_node' if ", *localize_in_map,
                              " else 'async_slam_toolbox_node'"]),
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[slam_yaml, PythonExpression(  # ekf: overlay slam_ekf.yaml
                             ["'", slam_ekf_yaml, "' if '", LaunchConfiguration('localization'),
                              "' == 'ekf' else '", slam_yaml, "'"]), PythonExpression(  # wide: slam_wide.yaml
                             ["'", slam_wide_yaml, "' if '", LaunchConfiguration('slam_search'),
                              "' == 'wide' else '", slam_yaml, "'"]), PythonExpression(  # saved map
                             ["'", slam_localization_yaml, "' if ", *localize_in_map, " else '", slam_yaml, "'"]),
                             {'use_sim_time': False, 'map_file_name': LaunchConfiguration('slam_map'),
                              'resolution': ParameterValue(LaunchConfiguration('slam_resolution'),
                                                           value_type=float)}],
                         remappings=[(t, f'/perception/lidar/{t}') for t in SLAM_TOPICS])
    nodes += [slam,
              EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                          transition_id=Transition.TRANSITION_CONFIGURE)),
              RegisterEventHandler(OnStateTransition(
                  target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
                  entities=[EmitEvent(event=ChangeState(
                      lifecycle_node_matcher=matches_action(slam),
                      transition_id=Transition.TRANSITION_ACTIVATE))]))]
    return LaunchDescription([
        DeclareLaunchArgument('lidar_fov_deg', default_value='180.0',
                              description='front and rear LiDAR field of view kept in /lidar/merged [deg]'),
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='false: wheel/clock only (arm board absent or faulted)'),
        DeclareLaunchArgument('group', default_value='all', choices=GROUPS,
                              description='all, one master (perception/planning/control) '
                                          'or infra (sensors, LiDAR, EKF, SLAM)'),
        DeclareLaunchArgument('localization', default_value='ekf', choices=['slam', 'ekf'],
                              description='map->odom from slam_toolbox (slam) or from ekf_global fusing the '
                                          'SLAM pose (ekf); give the same value to the infra and perception groups'),
        DeclareLaunchArgument('lidar_odom', default_value='false', choices=['true', 'false'],
                              description='LiDAR odometry (perception, /perception/lidar/odom) as the EKF x/y '
                                          'source; give the same value to the infra and perception groups'),
        DeclareLaunchArgument('slam_map', default_value='',
                              description='saved pose graph to localize in, absolute path without extension '
                                          '(e.g. /home/craft/maps/arena3_clean); empty = build a new map'),
        DeclareLaunchArgument('slam_search', default_value='wide', choices=['narrow', 'wide'],
                              description='scan-match position search: narrow +-5 cm (slam.yaml) or wide '
                                          '+-25 cm (slam_wide.yaml, temporary while wheel slip is large)'),
        DeclareLaunchArgument('slam_resolution', default_value='0.05',
                              description='slam_toolbox map cell size [m]; finer costs scan-matching CPU'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start dual LiDAR drivers and static TF; '
                                          'set false when already running on the Pi'),
        DeclareLaunchArgument('start_cameras', default_value='true',
                              description='Start the D435 / C310 drivers; false for LiDAR-only runs (mapping)'),
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
        *masters,
        GroupAction(nodes, scoped=False, condition=in_group('infra')),
    ])
