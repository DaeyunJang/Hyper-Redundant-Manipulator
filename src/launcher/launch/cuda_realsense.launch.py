#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _find_workspace_root():
    launcher_prefix = Path(get_package_prefix('launcher')).resolve()
    candidates = [Path.cwd().resolve(), launcher_prefix, *launcher_prefix.parents]

    source_file = Path(__file__).resolve()
    candidates.extend([source_file.parent, *source_file.parents])

    for candidate in candidates:
        cuda_root = candidate / '.cuda_realsense'
        if (
            (cuda_root / 'install/librealsense/lib').is_dir()
            and (cuda_root / 'install/realsense-ros/share/realsense2_camera').is_dir()
        ):
            return candidate

    raise RuntimeError(
        'CUDA RealSense installation was not found. Run '
        './scripts/build_realsense_cuda.sh from the workspace root first.'
    )


def _prepend_path(path, current_value):
    values = [str(path)]
    if current_value:
        values.append(current_value)
    return os.pathsep.join(values)


def generate_launch_description():
    workspace_root = _find_workspace_root()
    cuda_root = workspace_root / '.cuda_realsense'
    librealsense_lib = cuda_root / 'install/librealsense/lib'
    realsense_ros_prefix = cuda_root / 'install/realsense-ros'
    realsense_ros_lib = realsense_ros_prefix / 'lib'
    realsense_launch = (
        realsense_ros_prefix
        / 'share/realsense2_camera/launch/rs_launch.py'
    )
    camera_config = Path(
        get_package_share_directory('estimation_pkg')
    ) / 'config/realsense_d405.yaml'

    ament_prefix_path = _prepend_path(
        realsense_ros_prefix, os.environ.get('AMENT_PREFIX_PATH', '')
    )
    library_path = _prepend_path(
        librealsense_lib,
        _prepend_path(realsense_ros_lib, os.environ.get('LD_LIBRARY_PATH', '')),
    )

    # Package/executable lookup occurs in the launch process as well as in its
    # children, so update both the process environment and launch environment.
    os.environ['AMENT_PREFIX_PATH'] = ament_prefix_path
    os.environ['LD_LIBRARY_PATH'] = library_path

    return LaunchDescription([
        SetEnvironmentVariable('AMENT_PREFIX_PATH', ament_prefix_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
        LogInfo(
            msg=f'Using CUDA librealsense: {librealsense_lib}'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(realsense_launch)),
            launch_arguments={
                'config_file': str(camera_config),
            }.items(),
        ),
    ])
