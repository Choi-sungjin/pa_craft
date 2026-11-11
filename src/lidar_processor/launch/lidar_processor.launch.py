#!/usr/bin/env python3
"""
[Pi 에서 실행] 전/후방 LDS-03 → lidar_processor → /lidar/merged

  ros2 launch lidar_processor lidar_processor.launch.py
  ros2 launch lidar_processor lidar_processor.launch.py front_fov_deg:=190 rear_fov_deg:=190

토픽
  /lidar/front/scan, /lidar/rear/scan   coin_d4_driver 원본 (드라이버가 별도 프로세스라 내부 전달용으로 남음)
  /lidar/merged                         lidar_processor 출력 (회전 중심 기준 360도, frame base_link)
  /tf_static                            base_link → laser_front / laser_rear
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    # 2) base_link(실제 회전 중심 = 앞뒤 바퀴 축 가운데) → laser_front / laser_rear
    for side in ('front', 'rear'):
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_laser_{side}',
            arguments=['--x', a[f'{side}_x'], '--y', a[f'{side}_y'], '--z', a['laser_z'],
                       '--yaw', str(math.radians(float(a[f'{side}_yaw_deg']))),
                       '--roll', str(math.radians(float(a[f'{side}_roll_deg']))),
                       '--frame-id', 'base_link', '--child-frame-id', f'laser_{side}'],
        ))

    # 3) FOV 필터(몸체 중앙 기준) + 회전 중심 기준 병합
    actions.append(Node(
        package='lidar_processor', executable='lidar_processor', name='lidar_processor',
        condition=IfCondition(LaunchConfiguration('start_processor')),
        output='screen',
        parameters=[{'front_fov_deg': float(a['front_fov_deg']),
                     'rear_fov_deg': float(a['rear_fov_deg']),
                     'fov_near_m': float(a['fov_near_m']),
                     'body_radius_m': float(a['body_radius_m']),
                     'body_center_x': float(a['body_center_x'])}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_processor', default_value='true',
                              description='false: drivers and TF only; perception_master merges scans'),
        DeclareLaunchArgument('front_port', default_value='/dev/lidar_front'),
        DeclareLaunchArgument('rear_port', default_value='/dev/lidar_rear'),
        # ---- base_link(실제 회전 중심) 기준 [m]. 퍽 보정(lidar_calibrator) 결과 2026-10-16 ----
        # ---- lidar_calibrator(퍽 보정, -p body_center_x 같은 값) 결과를 여기에 반영 ----
        # 2026-10-30: base_link 를 두 LiDAR 가운데에서 앞뒤 바퀴 축 가운데로 옮김(4WD 스키드 스티어의
        # 회전 중심). 두 LiDAR 가운데가 축 가운데보다 26 mm 앞(실측) → x 에 +0.026 (예전 ±0.1437)
        # 2026-10-31 (축 간격 0.16 m 차체): 회전축 중심이 전방 LiDAR 중심에서 15.5 cm 뒤(실측) -> 가운데 +0.01275
        DeclareLaunchArgument('body_center_x', default_value='0.01275',
                              description='몸체 중앙(두 LiDAR 가운데)의 base_link x [m], 몸체 필터 중심'),
        # 2026-10-31 뒤집어 재장착 후 실측: 1층 판 32 cm 양끝 안쪽에 LiDAR, 몸체 내측 24.9 cm -> 한 개 두께
        # 3.55 cm, 몸체 중심 간 28.45 cm. 전방 LiDAR 중심이 회전축 중심에서 15.5 cm 앞 -> 후방 -0.1295.
        # 두 LiDAR 모두 로봇 좌우 가운데 선 위(확인) -> y 0.
        DeclareLaunchArgument('front_x', default_value='0.155'),
        DeclareLaunchArgument('front_y', default_value='0.0'),
        DeclareLaunchArgument('rear_x', default_value='-0.1295'),
        DeclareLaunchArgument('rear_y', default_value='0.0'),
        # 2026-10-31: 두 LiDAR 를 뒤집어(업사이드다운) 다시 장착 -> roll 180도. 병합·보정 코드는 roll 180도(z 축
        # 아래)를 보고 스캔 각도 부호를 뒤집는다. 각 LiDAR 의 스캔 0도는 로봇 안쪽(반대편 LiDAR)을 향함 (실측:
        # 모터 4개가 raw 0도 기준 좌우 대칭, 전방 LiDAR 를 손으로 덮으면 raw 90~240도가 막힘) -> 전방 yaw 180도,
        # 후방 0도 (뒤집기 전과 같은 yaw). 처음 넣은 0 / 180 은 180도 틀려 로봇 앞뒤가 뒤집혀 보였다.
        DeclareLaunchArgument('front_yaw_deg', default_value='180.0'),
        DeclareLaunchArgument('rear_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('front_roll_deg', default_value='180.0',
                              description='180: 뒤집어 장착(업사이드다운), 0: 똑바로'),
        DeclareLaunchArgument('rear_roll_deg', default_value='180.0',
                              description='180: 뒤집어 장착(업사이드다운), 0: 똑바로'),
        DeclareLaunchArgument('laser_z', default_value='0.021',
                              description='스캔 높이: 지면 ~ LiDAR 원통 높이 중앙 (2026-10-31 실측, 뒤집은 뒤; 2D 결과에는 영향 없음)'),
        # 각 LiDAR 의 로봇 바깥 방향(전방 +x, 후방 -x) 기준 전체 FOV [deg]. 2026-10-31: 180 (+-90도).
        # 업사이드다운으로 낮게 달린 LiDAR 는 차체 아래로 반대편까지 보고, 바깥쪽을 6도 들어 안쪽은 ~20 cm
        # 에서 바닥을 찍으므로 안쪽 반원은 쓰지 않는다.
        DeclareLaunchArgument('front_fov_deg', default_value='180.0'),
        DeclareLaunchArgument('rear_fov_deg', default_value='180.0'),
        # FOV 밖이라도 몸체 중앙에서 이 거리 이상이면 남김 [m]. 10 > range_max(8): FOV 밖은 모두 버림
        # (예전 0.30: 기둥만 지우고 옆쪽 먼 점은 유지)
        DeclareLaunchArgument('fov_near_m', default_value='10.0'),
        # 몸체 중앙에서 이 거리 안쪽 점은 무조건 버림 [m] (몸체·기둥)
        DeclareLaunchArgument('body_radius_m', default_value='0.25'),
        OpaqueFunction(function=_setup),
    ])
