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
    if urdf.endswith('.xacro'):
        import xacro  # .xacro일 때만 필요 (sudo apt install ros-humble-xacro)
        description = xacro.process_file(urdf).toxml()
    else:
        description = Path(urdf).read_text()
    robot_description = ParameterValue(description, value_type=str)

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # 관절 이름, 앞면 위치, 바닥 높이 등은 V307 기본값. 다른 로봇이면 box_reach.py 파라미터로 바꿈
        Node(package='craft_description', executable='box_reach.py',
             parameters=[{'robot_description': robot_description,
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
        DeclareLaunchArgument('urdf', default_value=str(share / 'urdf/v307_full_robot.urdf'),
                              description='표시할 URDF(.urdf/.xacro) 경로'),
        DeclareLaunchArgument('stroke', default_value='0',
                              description='시작 LM 스트로크 [mm]. 0이면 URDF 값 (V307: 129)'),
        DeclareLaunchArgument('limits', default_value='',
                              description="관절 한계 바꾸기 '관절:하한:상한' 공백 구분 (deg/mm). "
                                          "예: 'shoulder:0:45'. URDF 파일은 그대로"),
        OpaqueFunction(function=launch_setup),
    ])
