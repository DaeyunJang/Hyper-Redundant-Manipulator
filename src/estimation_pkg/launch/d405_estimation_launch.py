#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from estimation_pkg.runtime import configure_image_transport


def generate_launch_description():
    estimation_share = get_package_share_directory('estimation_pkg')
    profile = configure_image_transport()
    transport_actions = (
        [SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile)]
        if profile else []
    )
    realsense_share = get_package_share_directory('realsense2_camera')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'config_file': os.path.join(
                estimation_share, 'config', 'realsense_d405.yaml'
            ),
        }.items(),
    )
    estimator = Node(
        package='estimation_pkg',
        executable='segment_angle_estimator',
        name='segment_angle_estimator',
        output='screen',
    )

    return LaunchDescription([*transport_actions, camera, estimator])
