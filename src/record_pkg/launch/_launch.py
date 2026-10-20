"""Launch an idle recorder; GUI controls /data/record independently."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('record_pkg'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share / 'config' / 'recording.json')),
        DeclareLaunchArgument('output_root', default_value=str(Path.cwd() / 'record')),
        DeclareLaunchArgument('allow_incomplete', default_value='false'),
        Node(
            package='record_pkg', executable='record', name='record', output='screen',
            sigterm_timeout='20', sigkill_timeout='5',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'output_root': LaunchConfiguration('output_root'),
                'allow_incomplete': ParameterValue(LaunchConfiguration('allow_incomplete'), value_type=bool),
            }],
        ),
    ])
