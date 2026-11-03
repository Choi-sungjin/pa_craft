"""ros2 launch craft_voice voice.launch.py [camera:=d435|c310|topic|none] [scene_period_s:=30]

Arguments left empty keep the value from params_file (config/voice.yaml).
Needs OPENAI_API_KEY in the environment of this terminal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

OVERRIDES = {  # launch argument -> type of the node parameter
    'camera': str, 'image_topic': str, 'image_transport': str, 'state_mode': str,
    'scene_period_s': float, 'model': str, 'tts_voice': str,
}


def node(context):
    overrides = {}
    for name, kind in OVERRIDES.items():
        value = context.launch_configurations.get(name, '')
        if value:
            overrides[name] = kind(value)
    actions = []
    if not os.environ.get('OPENAI_API_KEY'):
        actions.append(LogInfo(msg='OPENAI_API_KEY is not set: voice_node runs with fixed templates only'))
    actions.append(Node(package='craft_voice', executable='voice_node', name='voice_node', output='screen',
                        parameters=[context.launch_configurations['params_file'], overrides]))
    return actions


def generate_launch_description():
    params = os.path.join(get_package_share_directory('craft_voice'), 'config', 'voice.yaml')
    args = [DeclareLaunchArgument('params_file', default_value=params, description='voice_node parameter file')]
    args += [DeclareLaunchArgument(name, default_value='', description=f'override {name} ({kind.__name__})')
             for name, kind in OVERRIDES.items()]
    return LaunchDescription(args + [OpaqueFunction(function=node)])
