from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    share = Path(get_package_share_directory('craft_description'))
    urdf = LaunchConfiguration('urdf').perform(context)
    if '/' not in urdf:  # 이름만 주면 share/urdf/<이름>[.urdf|.xacro]
        cands = [share / 'urdf' / n for n in (urdf, urdf + '.urdf', urdf + '.urdf.xacro', urdf + '.xacro')]
        found = [c for c in cands if c.is_file()]
        if not found:
            names = sorted(x.name for x in (share / 'urdf').iterdir() if x.is_file())
            raise RuntimeError(f"URDF '{urdf}' 없음. 고를 수 있는 이름: {names}")
        urdf = str(found[0])
    if urdf.endswith('.xacro'):
        import xacro  # .xacro일 때만 필요 (sudo apt install ros-humble-xacro)
        description = xacro.process_file(urdf).toxml()
    else:
        description = Path(urdf).read_text()
    robot_description = ParameterValue(description, value_type=str)
    # 구간별 링크·브라켓 경계 (make_robotis_arm_urdf.py가 씀). 없으면 링크 길이 슬라이더 하나씩
    seg = Path(urdf).with_suffix('.segments.json')
    segments = seg.read_text() if seg.is_file() else ''

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # 관절 이름, 앞면 위치, 바닥 높이 등은 V307 기본값. 다른 로봇이면 box_reach.py 파라미터로 바꿈
        Node(package='craft_description', executable='box_reach.py',
             parameters=[{'robot_description': robot_description,
                          'segments': ParameterValue(segments, value_type=str),
                          'box_distance': float(LaunchConfiguration('box_distance').perform(context)) / 1000,
                          'tcp_tol_deg': float(LaunchConfiguration('tol').perform(context)),
                          'stroke': float(LaunchConfiguration('stroke').perform(context)) / 1000,
                          # 빈 리스트는 타입 추론이 안 되므로 ['']로 넘기고 노드에서 걸러냄
                          'limit_overrides': LaunchConfiguration('limits').perform(context).split()
                          or ['']}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/box_reach.rviz')]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('craft_description'))
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value='v307_full_robot',
                              description='URDF 이름(urdf/ 폴더, 확장자 생략 가능: v307_full_robot, '
                                          'v307_robotis_arm) 또는 .urdf/.xacro 경로'),
        DeclareLaunchArgument('stroke', default_value='0',
                              description='시작 LM 스트로크 [mm]. 0이면 URDF 값 (V307: 129)'),
        DeclareLaunchArgument('box_distance', default_value='20',
                              description='시작 로봇 앞면-박스 거리 [mm]'),
        DeclareLaunchArgument('tol', default_value='10',
                              description='시작 TCP 각도 허용 (수직 아래 기준) [deg]'),
        DeclareLaunchArgument('limits', default_value='',
                              description="관절 한계 바꾸기 '관절:하한:상한' 공백 구분 (deg/mm). "
                                          "예: 'shoulder:0:45'. URDF 파일은 그대로"),
        OpaqueFunction(function=launch_setup),
    ])
