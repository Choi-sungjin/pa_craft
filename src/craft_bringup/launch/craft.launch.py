import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

NODES = {
    'craft_sensors': ['realsense_processor', 'c310_processor'],
    'craft_perception': ['perception_master'],
    'craft_control': ['control_master'],
}

# slam_toolbox publishers, remapped to /perception/lidar/<name>.
SLAM_TOPICS = ['pose', 'map', 'map_metadata', 'slam_toolbox/graph_visualization',
               'slam_toolbox/scan_visualization', 'slam_toolbox/pose_graph',
               'slam_toolbox/loop_closure_event', 'slam_toolbox/new_node_event']


def generate_launch_description():
    ekf_yaml = os.path.join(get_package_share_directory('craft_perception'), 'config', 'ekf.yaml')
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
    # OpenCR on-board IMU is mounted upside down: roll = pi. Offsets are launch args (measure them).
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
                      arguments=['--x', LaunchConfiguration('imu_x'), '--y', LaunchConfiguration('imu_y'),
                                 '--z', LaunchConfiguration('imu_z'), '--roll', str(math.pi),
                                 '--yaw', LaunchConfiguration('imu_yaw'),
                                 '--frame-id', 'base_link', '--child-frame-id', 'imu_link']))
    # Dual EKF: local owns odom->base_link, global owns map->odom (see craft_perception/config/ekf.yaml).
    for name, output in (('ekf_local', '/odometry/local'), ('ekf_global', '/odometry/global')):
        nodes.append(Node(package='robot_localization', executable='ekf_node', name=name, output='screen',
                          parameters=[ekf_yaml], remappings=[('odometry/filtered', output)]))
    # 2D SLAM, started directly (online_async_launch.py takes no remaps and defaults to sim time).
    # LiDAR-derived outputs live under /perception/lidar/; input stays /lidar/merged (slam.yaml).
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[slam_yaml, {'use_sim_time': False}],
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
        DeclareLaunchArgument('imu_x', default_value='0.0', description='IMU offset from base_link [m]'),
        DeclareLaunchArgument('imu_y', default_value='0.0', description='IMU offset from base_link [m]'),
        DeclareLaunchArgument('imu_z', default_value='0.0', description='IMU offset from base_link [m]'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0',
                              description='IMU yaw misalignment after the upside-down flip [rad]'),
        *nodes,
    ])
