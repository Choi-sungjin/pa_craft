"""IK 결과와 바퀴 이동을 RViz에서 보기.

world(바닥) 에 노브 받침대가 고정되고, 로봇은 바퀴로 world 위를 앞뒤로 움직인다.
  robot_state_publisher (로봇)      : /robot_description, /joint_states
  robot_state_publisher (station)   : /station/robot_description, /station/joint_states
  static TF world -> station_base
  ik_rviz_node                      : IK + world -> base_link TF + 관절값 발행
  rviz2

목표 보내기 (다른 터미널):
  ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 0      # 노브 수직 하향 잡기 (world 기준)
  ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 -90    # 잡은 채 노브 +90 deg
  ros2 run v75_arm_ik ik_goal 150 300 250 0 0             # 팔만 이동 (base_link 기준)
"""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = Path(get_package_share_directory('v75_robot_description'))
    ik_share = Path(get_package_share_directory('v75_arm_ik'))
    robot_urdf = str(desc_share / 'urdf/v75_robot.urdf')
    station_urdf = str(desc_share / 'urdf/knob_station.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('move_time', default_value='2.0',
                              description='한 번 움직이는 시간 [s] (바퀴는 앞 40% 구간에서 이동)'),
        DeclareLaunchArgument('y_ref', default_value='-0.110',
                              description='수직 하향 잡기 기준 위치 (base_link y, m)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 xacro.process_file(robot_urdf).toxml(), value_type=str)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='station', name='station_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 xacro.process_file(station_urdf).toxml(), value_type=str)}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'station_base']),
        Node(package='v75_arm_ik', executable='ik_rviz_node', output='screen',
             parameters=[{'urdf': robot_urdf, 'station_urdf': station_urdf,
                          'move_time': ParameterValue(LaunchConfiguration('move_time'), value_type=float),
                          'y_ref': ParameterValue(LaunchConfiguration('y_ref'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(ik_share / 'rviz/ik_demo.rviz')]),
    ])
