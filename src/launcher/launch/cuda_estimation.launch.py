#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launcher_share = Path(get_package_share_directory('launcher'))
    estimation_share = Path(get_package_share_directory('estimation_pkg'))

    return LaunchDescription([
        DeclareLaunchArgument('device_type', default_value='D405$'),
        DeclareLaunchArgument('serial_no', default_value="''"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launcher_share / 'launch/cuda_realsense.launch.py')
            ),
            launch_arguments={
                'device_type': LaunchConfiguration('device_type'),
                'serial_no': LaunchConfiguration('serial_no'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(estimation_share / 'launch/_launch.py')
            )
        ),
    ])
