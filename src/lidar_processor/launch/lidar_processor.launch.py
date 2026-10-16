#!/usr/bin/env python3
"""
[Pi 에서 실행] 전/후방 LDS-03 → lidar_processor → /lidar/merged

  ros2 launch lidar_processor lidar_processor.launch.py
  ros2 launch lidar_processor lidar_processor.launch.py front_fov_deg:=190 rear_fov_deg:=190

토픽
  /lidar/front/scan, /lidar/rear/scan   coin_d4_driver 원본 (드라이버가 별도 프로세스라 내부 전달용으로 남음)
  /lidar/merged                         lidar_processor 출력 (로봇 중앙 기준 360도, frame base_link)
  /tf_static                            base_link → laser_front / laser_rear
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    a = context.launch_configurations
    actions = []

    # 1) LDS-03 두 대를 노드 하나로 구동
    lidar_params = {'lidar_list': ['lidar_front', 'lidar_rear']}
    for side in ('front', 'rear'):
        lidar_params[f'lidar_{side}'] = {
            'port': a[f'{side}_port'],
            'frame_id': f'laser_{side}',
            'baudrate': 230400,
            'version': 4,
            'topic_name': f'/lidar/{side}/scan',
            'reverse': False,
            'warmup_time': 5,
        }
    actions.append(Node(package='coin_d4_driver', executable='multi_coin_d4_node',
                        name='dual_lidar_node', parameters=[lidar_params], output='screen'))

    # 2) base_link(로봇 중앙) → laser_front / laser_rear
    for side in ('front', 'rear'):
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_laser_{side}',
            arguments=['--x', a[f'{side}_x'], '--y', a[f'{side}_y'], '--z', a['laser_z'],
                       '--yaw', str(math.radians(float(a[f'{side}_yaw_deg']))),
                       '--frame-id', 'base_link', '--child-frame-id', f'laser_{side}'],
        ))

    # 3) FOV 필터 + 로봇 중앙 기준 병합
    actions.append(Node(
        package='lidar_processor', executable='lidar_processor', name='lidar_processor',
        output='screen',
        parameters=[{'front_fov_deg': float(a['front_fov_deg']),
                     'rear_fov_deg': float(a['rear_fov_deg']),
                     'fov_near_m': float(a['fov_near_m']),
                     'body_radius_m': float(a['body_radius_m'])}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('front_port', default_value='/dev/lidar_front'),
        DeclareLaunchArgument('rear_port', default_value='/dev/lidar_rear'),
        # ---- base_link(로봇 중앙) 기준 [m]. 퍽 보정(lidar_calibrator) 결과 2026-10-16 ----
        # ---- lidar_calibrator(퍽 보정) 결과를 여기에 반영 ----
        DeclareLaunchArgument('front_x', default_value='0.1437'),
        DeclareLaunchArgument('front_y', default_value='-0.0113'),
        DeclareLaunchArgument('rear_x', default_value='-0.1437'),
        DeclareLaunchArgument('rear_y', default_value='0.0113'),
        # 두 LiDAR 모두 180도 돌려 장착됨 → 전방 180도, 후방 0도
        DeclareLaunchArgument('front_yaw_deg', default_value='180.0'),
        DeclareLaunchArgument('rear_yaw_deg', default_value='0.97'),
        DeclareLaunchArgument('laser_z', default_value='0.122',
                              description='스캔 높이 (미실측, 2D 결과에는 영향 없음)'),
        # 각 LiDAR 의 로봇 바깥 방향 기준 전체 FOV [deg] (기둥이 안 걸리게 조정)
        DeclareLaunchArgument('front_fov_deg', default_value='200.0'),
        DeclareLaunchArgument('rear_fov_deg', default_value='200.0'),
        # FOV 밖이라도 로봇 중앙에서 이 거리 이상이면 남김 [m] (기둥만 지우고 옆쪽은 보이게)
        DeclareLaunchArgument('fov_near_m', default_value='0.30'),
        # 로봇 중앙에서 이 거리 안쪽 점은 무조건 버림 [m] (몸체·기둥)
        DeclareLaunchArgument('body_radius_m', default_value='0.25'),
        OpaqueFunction(function=_setup),
    ])
