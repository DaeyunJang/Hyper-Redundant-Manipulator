#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    roi_path = Path(get_package_share_directory('estimation_pkg')) / 'config_ROI_ref.json'
    with roi_path.open(encoding='utf-8') as handle:
        roi = json.load(handle)
    fields = (('roi_x', 'x', 0), ('roi_y', 'y', 0),
              ('roi_width', 'w', 1), ('roi_height', 'h', 1))
    arguments = []
    for name, key, minimum in fields:
        if type(roi[key]) is not int or roi[key] < minimum:
            raise ValueError(f'{roi_path}: {key} must be an integer >= {minimum}.')
        arguments.append(DeclareLaunchArgument(
            name, default_value=str(roi[key]),
            description='Startup-only estimation crop in pixels; restart estimation to change.'))
    return LaunchDescription(
        arguments + [
            # ExecuteProcess(
            #     cmd=rqt_command,
            #     output='screen'
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
            #         launch_arguments={
            #         'rgb_camera.color_profile': '1280,720,30',
            #         'depth_module.depth_profile': '1280,720,30',
            #         'rgb_camera.enable_auto_exposure': 'true',
            #         # 'rgb_camera.profile': '640,480,30',
            #         # 'depth_module.profile': '640,480,30',
            #         }.items()
            # ),
            Node(
                package="estimation_pkg",
                executable="segment_angle_estimator",
                name="segment_angle_estimator",
                output="screen",
                parameters=[{
                    name: ParameterValue(LaunchConfiguration(name), value_type=int)
                    for name, _, _ in fields
                }],
                additional_env={
                    'OPENBLAS_NUM_THREADS': '1',
                    'OMP_NUM_THREADS': '1',
                    'MKL_NUM_THREADS': '1',
                },
                # prefix='taskset -c 2 3'
            ),
        ]
    )
