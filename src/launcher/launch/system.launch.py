#!/usr/bin/env python3

"""Start the operator GUI, which owns the remaining system processes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    auto_start_components = LaunchConfiguration('auto_start_components')
    gui_launch = (
        Path(get_package_share_directory('gui_py_pkg'))
        / 'launch/_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start_components',
            default_value='false',
            description=(
                'Start components selected in the GUI configuration after '
                'the GUI opens.'
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gui_launch)),
            launch_arguments={
                'auto_start_components': auto_start_components,
            }.items(),
        ),
    ])
