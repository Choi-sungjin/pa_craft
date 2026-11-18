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
    joints = LaunchConfiguration('joints').perform(context).split()
    rviz = LaunchConfiguration('rviz').perform(context)
    robot_description = ParameterValue(description, value_type=str)

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='craft_description', executable='arm_sliders.py',
             parameters=[{'robot_description': robot_description,
                          # 빈 리스트는 타입 추론이 안 되므로 ['']로 넘기고 노드에서 걸러냄
                          'joints': joints or ['']}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz or str(share / 'rviz/box_reach.rviz')]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('craft_description'))
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value='v307_full_robot',
                              description='URDF 이름(urdf/ 폴더, 확장자 생략 가능: v307_full_robot, '
                                          'v307_robotis_arm) 또는 .urdf/.xacro 경로'),
        DeclareLaunchArgument('joints',
                              default_value='lm_slide shoulder elbow wrist_pitch tool_roll',
                              description="슬라이더로 띄울 관절(공백 구분). '' 이면 움직이는 관절 전부"),
        DeclareLaunchArgument('rviz', default_value='',
                              description='RViz 설정 파일. 비우면 rviz/box_reach.rviz'),
        OpaqueFunction(function=launch_setup),
    ])
