import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler, OpaqueFunction, SetLaunchConfiguration)
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

# util-linux: SIGTERM to the node when its parent (the launch) exits, so a killed launch leaves no orphans.
DIE_WITH_LAUNCH = 'setpriv --pdeathsig TERM'

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


def validate_localization_inputs(context):
    if (LaunchConfiguration('wheel_free').perform(context) == 'true' and
            LaunchConfiguration('lidar_odom').perform(context) != 'true'):
        raise RuntimeError('wheel_free:=true requires lidar_odom:=true in every launch group')
    return []


def repo_root():
    """The craft checkout: this file through the --symlink-install link, else CRAFT_REPO, else ~/craft."""
    here = os.path.realpath(__file__)
    for parent in [os.path.dirname(here)] + [here.rsplit(os.sep, k)[0] for k in range(2, 6)]:
        if os.path.isfile(os.path.join(parent, 'demo', 'corner', 'global_localize.py')):
            return parent
    return os.environ.get('CRAFT_REPO', os.path.expanduser('~/craft'))


def generate_launch_description():
    ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf.yaml')
    slam_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam.yaml')
    slam_ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam_ekf.yaml')
    slam_wide_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam_wide.yaml')
    # An overlay that is off: no parameters (slam.yaml again would undo the overlays before it).
    slam_none_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'slam_none.yaml')
    ekf_lidar_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf_lidar.yaml')
    slam_localization_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config',
                                          'slam_localization.yaml')
    localize_in_map = ["'", LaunchConfiguration('slam_map'), "' != ''"]  # slam_map:= set: localization mode
    amcl_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'amcl.yaml')
    # global_localize.py runs (and mission_master waits for it) in a saved map: slam_map:= or localization:=amcl.
    find_start = ["(", *localize_in_map, " or '", LaunchConfiguration('localization'), "' == 'amcl') and '",
                  LaunchConfiguration('global_localize'), "' == 'true'"]
    # lidar_odom:=true: ekf.yaml + ekf_lidar.yaml (LiDAR odometry x/y); repeating ekf.yaml is a no-op.
    ekf_params = [ekf_yaml, PythonExpression(["'", ekf_lidar_yaml, "' if '", LaunchConfiguration('lidar_odom'),
                                              "' == 'true' else '", ekf_yaml, "'"])]
    wheel_free_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf_wheel_free.yaml')
    ekf_params.append(PythonExpression(["'", wheel_free_yaml, "' if '", LaunchConfiguration('wheel_free'),
                                        "' == 'true' else ('", ekf_lidar_yaml, "' if '", LaunchConfiguration('lidar_odom'),
                                        "' == 'true' else '", ekf_yaml, "')"]))
    camera_yaml = os.path.join(get_package_share_directory('craft_sensors'), 'config', 'camera.yaml')
    # Loop rates and timeouts of the three masters, after each master's own yaml.
    rates_yaml = os.path.join(get_package_share_directory('craft_bringup'), 'config', 'rates.yaml')
    control_yaml = os.path.join(get_package_share_directory('craft_control'), 'config', 'control.yaml')
    planning_yaml = os.path.join(get_package_share_directory('craft_planning'), 'config', 'planning.yaml')
    mission_yaml = os.path.join(get_package_share_directory('craft_planning'), 'config', 'mission.yaml')
    # arm:=false drives with the wheel/clock board alone: control skips the arm board, planning is Ready without arm joints.
    arm_enabled = ParameterValue(LaunchConfiguration('arm'), value_type=bool)
    perception_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'perception.yaml')
    perception_params = [perception_yaml, rates_yaml, {
        'lidar.input_mode': LaunchConfiguration('lidar_input_mode'),
        # Each LiDAR's kept field of view around its outward axis [deg] (perception.yaml default 180 = +-90).
        'lidar.front_fov_deg': ParameterValue(LaunchConfiguration('lidar_fov_deg'), value_type=float),
        'lidar.rear_fov_deg': ParameterValue(LaunchConfiguration('lidar_fov_deg'), value_type=float),
        'odometry_source_topic': PythonExpression(
            ["'/odometry/global' if '", LaunchConfiguration('localization'), "' == 'ekf' else '/odometry/local'"]),
        'localization.enabled': localization_is('ekf'),
        'lidar_odometry.enabled': ParameterValue(LaunchConfiguration('lidar_odom'), value_type=bool),
        # Mapping only (slamper): no scan to SLAM on slopes over this [deg]; 0 = off (localization keeps every scan).
        'localization_scan.max_slope_deg': ParameterValue(LaunchConfiguration('mapping_tilt_deg'), value_type=float),
        'd435.yolo.model_dir': LaunchConfiguration('d435_model_dir'),
        'uppercam.yolo.model_dir': LaunchConfiguration('uppercam_model_dir'),
        'uppercam.puck_survey.yolo.model_dir': LaunchConfiguration('uppercam_puck_model_dir')}]

    masters = [
        Node(package='craft_perception', executable='perception_master', output='screen',
             parameters=perception_params, condition=in_group('perception')),
        Node(package='craft_planning', executable='planning_master', output='screen',
             parameters=[planning_yaml, rates_yaml, {'map_file': LaunchConfiguration('map_file'), 'require_arm_joints': arm_enabled,
                                                    'require_lidar_localization': ParameterValue(
                                                        LaunchConfiguration('wheel_free'), value_type=bool)}],
             condition=in_group('planning')),
        Node(package='craft_control', executable='control_master', output='screen',
             parameters=[control_yaml, rates_yaml, {'arm_enabled': arm_enabled}], condition=in_group('control')),
        # The planning map (map_file:=, a .yaml such as brief_arena1cm) on /planning/map for RViz "planning map" and
        # craft_tui; planning_master loads the file itself and publishes only /planning/padding.
        LifecycleNode(package='nav2_map_server', executable='map_server', name='planning_map_server', namespace='',
                      output='log', autostart=True,
                      parameters=[{'yaml_filename': LaunchConfiguration('map_file'), 'topic_name': '/planning/map'}],
                      condition=IfCondition(PythonExpression(
                          ["'", LaunchConfiguration('group'), "' in ('all', 'planning') and '",
                           LaunchConfiguration('map_file'), "'.endswith('.yaml')"]))),
        # Dice memory mission on top of planning_master (mission:=true); it waits for /mission/command START.
        Node(package='craft_planning', executable='mission_master', output='screen', parameters=[mission_yaml, {
                 # its first drive waits for global_localize's "ok" whenever that runs (slam_map:= or amcl)
                 'wait_global_localization': ParameterValue(PythonExpression(find_start), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ]

    # Everything that is not a master: camera drivers, LiDAR, EKF, SLAM.
    # Gripper D435 only (C310 removed 2026-11-11); the upper-cam D435 is opened by demo/corner/run_uppercam_bridge.sh.
    nodes = [Node(package='craft_sensors', executable='realsense_processor', output='screen', parameters=[camera_yaml],
                  prefix=f'{DIE_WITH_LAUNCH} nice -n 10',  # camera yields CPU to control/EKF when the Jetson is full
                  condition=IfCondition(LaunchConfiguration('start_cameras')))]
    nodes.append(IncludeLaunchDescription(
        os.path.join(get_package_share_directory('lidar_processor'), 'launch',
                     'lidar_processor.launch.py'),
        launch_arguments={'start_processor': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar'))))
    # ekf_local owns odom->base_link (IMU and optional wheel/LiDAR velocity). map->odom: slam_toolbox (localization:=slam, slam.yaml)
    # or ekf_global fusing slam_toolbox's pose (localization:=ekf, ekf.yaml + slam_ekf.yaml).
    nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_local', output='screen',
                      parameters=ekf_params, remappings=[('odometry/filtered', '/odometry/local')]))
    nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_global', output='screen',
                      parameters=ekf_params, remappings=[('odometry/filtered', '/odometry/global')],
                      condition=IfCondition(localization_is('ekf'))))
    # RViz: EKF odometry as a persistent point trail (/odometry/local/trail, latched Marker POINTS).
    nodes.append(Node(package='craft_bringup', executable='odom_trail.py', name='odom_trail', output='log'))
    # RViz: where the box was asked for and where it was detected (/perception/box/markers, box_marker.py).
    nodes.append(Node(package='craft_bringup', executable='box_marker.py', name='box_marker', output='log'))
    # 2D SLAM, started directly (online_async_launch.py takes no remaps and defaults to sim time).
    # LiDAR-derived outputs live under /perception/lidar/; input stays /lidar/merged (slam.yaml).
    # slam_map:=<base> localizes in a saved pose graph instead (slam_localization.yaml, /initialpose).
    slam = LifecycleNode(package='slam_toolbox', executable=PythonExpression(
                             ["'localization_slam_toolbox_node' if ", *localize_in_map,
                              " else 'async_slam_toolbox_node'"]),
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[slam_yaml, PythonExpression(  # ekf: overlay slam_ekf.yaml
                             ["'", slam_ekf_yaml, "' if '", LaunchConfiguration('localization'),
                              "' == 'ekf' else '", slam_none_yaml, "'"]), PythonExpression(  # wide: slam_wide.yaml
                             ["'", slam_wide_yaml, "' if '", LaunchConfiguration('slam_search'),
                              "' == 'wide' else '", slam_none_yaml, "'"]), PythonExpression(  # saved map
                             ["'", slam_localization_yaml, "' if ", *localize_in_map, " else '", slam_none_yaml, "'"]),
                             {'use_sim_time': False, 'map_file_name': LaunchConfiguration('slam_map'),
                              'resolution': ParameterValue(LaunchConfiguration('slam_resolution'),
                                                           value_type=float)}],
                         remappings=[(t, f'/perception/lidar/{t}') for t in SLAM_TOPICS])
    # slam_map:= (localization mode): where the robot stands, matched over the whole map (run_corner.py's global
    # localization) -> /initialpose, status on /perception/localization/global. The map: the pose graph's own yaml.
    # localization:=amcl: the scan is matched to the planning map (map_file:=) itself, no slam_toolbox.
    nodes.append(ExecuteProcess(
        cmd=['/usr/bin/python3', os.path.join(repo_root(), 'demo', 'corner', 'global_localize.py'),
             '--map', PythonExpression(["'", LaunchConfiguration('map_file'), "' if '", LaunchConfiguration('localization'),
                                        "' == 'amcl' else '", LaunchConfiguration('slam_map'), ".yaml'"]),
             '--near', LaunchConfiguration('initial_x'), LaunchConfiguration('initial_y'),
             '--yaw', LaunchConfiguration('initial_yaw')],
        name='global_localize', output='screen',
        condition=IfCondition(PythonExpression(find_start))))
    nodes.append(GroupAction([slam,
              EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                          transition_id=Transition.TRANSITION_CONFIGURE)),
              RegisterEventHandler(OnStateTransition(
                  target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
                  entities=[EmitEvent(event=ChangeState(
                      lifecycle_node_matcher=matches_action(slam),
                      transition_id=Transition.TRANSITION_ACTIVATE))]))],
        scoped=False, condition=UnlessCondition(localization_is('amcl'))))
    # localization:=amcl (user 2026-11-13): map->odom from nav2 AMCL on the brief drawing (map_file:=), which also
    # goes out on /perception/lidar/map in place of slam_toolbox's grid (perception's LiDAR obstacle filter, RViz).
    nodes.append(GroupAction([
        LifecycleNode(package='nav2_map_server', executable='map_server', name='localization_map_server',
                      namespace='', output='log', autostart=True,
                      parameters=[{'yaml_filename': LaunchConfiguration('map_file'),
                                   'topic_name': '/perception/lidar/map'}]),
        LifecycleNode(package='nav2_amcl', executable='amcl', name='amcl', namespace='', output='screen',
                      autostart=True,
                      parameters=[amcl_yaml, {
                          'initial_pose.x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                          'initial_pose.y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float)}])],
        scoped=False, condition=IfCondition(localization_is('amcl'))))
    return LaunchDescription([
        # Every process launched here (and in included launch files) dies with this launch, even when the launch
        # itself is killed or its ssh drops: no orphan nodes left running (2026-11-12). A node with its own
        # prefix= must start with DIE_WITH_LAUNCH too.
        SetLaunchConfiguration('launch-prefix', DIE_WITH_LAUNCH),
        DeclareLaunchArgument('wheel_free', default_value='false', choices=['true', 'false'],
                              description='Exclude wheel odometry from EKFs; requires lidar_odom:=true'),
        DeclareLaunchArgument('lidar_fov_deg', default_value='180.0',
                              description='front and rear LiDAR field of view kept in /lidar/merged [deg]'),
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='false: wheel/clock only (arm board absent or faulted)'),
        DeclareLaunchArgument('mission', default_value='false', choices=['true', 'false'],
                              description='true: start mission_master (config/mission.yaml of craft_planning)'),
        DeclareLaunchArgument('group', default_value='all', choices=GROUPS,
                              description='all, one master (perception/planning/control) '
                                          'or infra (sensors, LiDAR, EKF, SLAM)'),
        DeclareLaunchArgument('localization', default_value='slam', choices=['slam', 'ekf', 'amcl'],  # slam: user 2026-11-12 (ekf rejected the /initialpose jump: sigma 3.5 m, dead_reckoning)
                              description='map->odom from slam_toolbox (slam), from ekf_global fusing the '
                                          'SLAM pose (ekf), or from AMCL on map_file (amcl, config/amcl.yaml); '
                                          'give the same value to the infra and perception groups'),
        DeclareLaunchArgument('mapping_tilt_deg', default_value='0.0',
                              description='mapping (slamper): no scan to SLAM while the chassis stands tilted over this [deg]; 0 = off'),
        DeclareLaunchArgument('lidar_odom', default_value='false', choices=['true', 'false'],
                              description='LiDAR odometry (perception, /perception/lidar/odom) as the EKF x/y '
                                          'source; give the same value to the infra and perception groups'),
        DeclareLaunchArgument('slam_map', default_value='',
                              description='saved pose graph to localize in, absolute path without extension '
                                          '(e.g. /home/craft/maps/arena3_clean); empty = build a new map'),
        DeclareLaunchArgument('global_localize', default_value='true', choices=['true', 'false'],
                              description='with slam_map:=, find the robot over the whole map at start-up '
                                          '(demo/corner/global_localize.py -> /initialpose)'),
        DeclareLaunchArgument('initial_x', default_value='0.600',  # user 2026-11-14: start (0.6, 1.0), facing -x
                              description='rough start position x [m, map]: global_localize searches within 0.5 m'),
        DeclareLaunchArgument('initial_yaw', default_value='180.0',
                              description='start heading [deg, map]: global_localize searches within 20 deg of it'),
        DeclareLaunchArgument('initial_y', default_value='1.000',
                              description='rough start position y [m, map] (start zone, user 2026-11-12)'),
        DeclareLaunchArgument('slam_search', default_value='wide', choices=['narrow', 'wide'],
                              description='scan-match position search: narrow +-5 cm (slam.yaml) or wide '
                                          '+-25 cm (slam_wide.yaml, temporary while wheel slip is large)'),
        DeclareLaunchArgument('slam_resolution', default_value='0.05',
                              description='slam_toolbox map cell size [m]; finer costs scan-matching CPU'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start dual LiDAR drivers and static TF; '
                                          'set false when already running on the Pi'),
        DeclareLaunchArgument('start_cameras', default_value='true',
                              description='Start the gripper D435 driver; false for LiDAR-only runs (mapping)'),
        DeclareLaunchArgument('lidar_input_mode', default_value='raw',
                              description='raw: master merges scans; merged: use external /lidar/merged '
                                          '(also set start_lidar:=false)'),
        DeclareLaunchArgument('map_file', default_value=os.path.join(  # absolute: map_server needs it too
                              get_package_share_directory('craft_planning'), 'maps', 'brief_arena1cm.yaml'),
                              description='Gazebo binary STL, ROS map YAML or block text '
                                          '(relative to craft_planning share)'),
        DeclareLaunchArgument('d435_model_dir', default_value='models/knob_sign_v4',
                              description='Gripper D435 YOLO NCNN model; relative to craft_perception share'),
        DeclareLaunchArgument('uppercam_model_dir', default_value='models/knob_sign_v4',
                              description='Upper-cam YOLO NCNN model (traffic_light); relative to craft_perception share'),
        DeclareLaunchArgument('uppercam_puck_model_dir', default_value='models/pucks_v8n_v2',
                              description='Upper-cam puck YOLO NCNN model (uppercam_puck, after the light); relative to craft_perception share'),
        OpaqueFunction(function=validate_localization_inputs),
        *masters,
        GroupAction(nodes, scoped=False, condition=in_group('infra')),
    ])
