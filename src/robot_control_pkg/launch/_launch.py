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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    motor_output_enabled = LaunchConfiguration('motor_output_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_output_enabled',
            default_value='false',
            description=(
                'Publish actuator motor_command messages. Keep false for IK '
                'preview and software-only tests.'
            ),
        ),
        Node(
            package='robot_control_pkg',
            executable='robot_control',
            name='robot_control',
            output='screen',
            parameters=[{
                'motor_output_enabled': ParameterValue(
                    motor_output_enabled,
                    value_type=bool,
                ),
            }],
        ),
    ])
