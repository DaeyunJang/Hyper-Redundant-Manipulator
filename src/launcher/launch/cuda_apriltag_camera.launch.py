#!/usr/bin/env python3

"""Launch an independent RGB-only CUDA RealSense camera for AprilTags."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launcher_share = Path(get_package_share_directory('launcher'))
    return LaunchDescription([GroupAction(scoped=True, actions=[
        DeclareLaunchArgument('device_type', default_value='D435i$'),
        DeclareLaunchArgument('serial_no', default_value="''"),
        DeclareLaunchArgument('camera_name', default_value='tag_camera'),
        DeclareLaunchArgument('camera_namespace', default_value='tag_camera'),
        DeclareLaunchArgument(
            'stream_config',
            default_value=str(launcher_share / 'config/realsense_apriltag.yaml'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launcher_share / 'launch/cuda_realsense.launch.py')
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in (
                    'device_type', 'serial_no', 'camera_name', 'camera_namespace', 'stream_config'
                )
            }.items(),
        ),
    ])])
