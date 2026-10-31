"""Launch only diagnostic force/contact inference, never robot_control."""

import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    venv_python = Path.home() / 'HRM_env/bin/python3'
    python = os.environ.get('HRM_INFERENCE_PYTHON') or (
        str(venv_python) if venv_python.is_file() else sys.executable)
    arguments = [
        DeclareLaunchArgument('python_executable', default_value=python),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Selected trusted hrm_bundle.pt, or its parent folder'),
        DeclareLaunchArgument('id_localization_config', default_value=str(
            Path(get_package_share_directory('estimation_pkg')) / 'id_localization_config.json'),
            description='Startup-only inferred-force load ranges JSON; restart to reload'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('body_contact_enabled', default_value='false'),
        DeclareLaunchArgument('id_tol', default_value='2'),
        DeclareLaunchArgument('sample_hz', default_value='30.0'),
        DeclareLaunchArgument('input_timeout_sec', default_value='0.25'),
        DeclareLaunchArgument('max_input_skew_sec', default_value='0.1'),
        DeclareLaunchArgument('max_window_gap_sec', default_value='0.15'),
    ]
    types = dict(model_path=str, id_localization_config=str, device=str, body_contact_enabled=bool,
                 id_tol=int, sample_hz=float, input_timeout_sec=float,
                 max_input_skew_sec=float, max_window_gap_sec=float)
    return LaunchDescription(arguments + [Node(
        package='estimation_pkg', executable='force_id_estimator',
        name='force_id_estimator', output='screen',
        prefix=LaunchConfiguration('python_executable'),
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                     for name, kind in types.items()}],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                        'MKL_NUM_THREADS': '1'},
    )])
