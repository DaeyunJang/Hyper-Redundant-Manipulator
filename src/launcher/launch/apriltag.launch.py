#!/usr/bin/env python3

"""AprilTag node formerly embedded in launcher/system.launch.py."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = (
        Path(get_package_share_directory('apriltag_ros'))
        / 'cfg/tags_36h11.yaml'
    )

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[str(parameter_file)],
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
        ),
    ])
