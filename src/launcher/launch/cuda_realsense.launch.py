#!/usr/bin/env python3

import os
from pathlib import Path
import re

import yaml

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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


def _validate_camera_selection(context, camera_config=None):
    # rs_launch.py applies its YAML AFTER launch parameters. Even an empty
    # identity entry would discard the explicit model/serial selection.
    if camera_config is None:
        camera_config = LaunchConfiguration('stream_config').perform(context)
    with Path(camera_config).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if config is None:
        config = {}
    if not isinstance(config, dict):
        raise RuntimeError('RealSense configuration must be a parameter mapping.')
    identity_keys = {'device_type', 'serial_no', 'usb_port_id'} & config.keys()
    if identity_keys:
        raise RuntimeError(
            'Remove camera identity keys from the stream configuration '
            f'{camera_config}: {", ".join(sorted(identity_keys))}. '
            'Select the camera with device_type and serial_no launch arguments.'
        )
    frame_keys = {
        key for key in config
        if key in {'camera_name', 'camera_namespace', 'tf_prefix', '__node', '__ns'}
        or (isinstance(key, str) and key.endswith('frame_id'))
    }
    if frame_keys:
        raise RuntimeError(
            'Remove camera name/namespace/frame keys from the stream configuration '
            f'{camera_config}: {", ".join(sorted(frame_keys))}. '
            'Use camera_name and camera_namespace launch arguments for unique TF and topics.'
        )

    model = LaunchConfiguration('device_type').perform(context).strip()
    if not re.fullmatch(r'D[0-9]{3}[A-Za-z]*\$?', model, flags=re.IGNORECASE):
        raise RuntimeError('device_type must name a RealSense model, e.g. D405 or D455.')
    # The driver uses regex_search, so anchor the suffix: D455 must not select
    # D455f. Accept the GUI's already-anchored model and ordinary CLI model names.
    model = model if model.endswith('$') else model + '$'

    serial = LaunchConfiguration('serial_no').perform(context).strip()
    if serial in ('', "''", '""'):
        serial = "''"
    else:
        if serial.startswith('_'):
            serial = serial[1:]
        if not re.fullmatch(r'[0-9]+', serial):
            raise RuntimeError('serial_no must be empty or an SDK serial containing ASCII digits.')
        # Vendor launch YAML-coerces substitutions. Its factory strips this
        # documented prefix, keeping the serial a string and its leading zeros.
        serial = '_' + serial

    return [
        SetLaunchConfiguration('device_type', model),
        SetLaunchConfiguration('serial_no', serial),
    ]


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
    estimation_config = Path(get_package_share_directory('estimation_pkg')) / 'config'
    camera_config = estimation_config / 'realsense_d405.yaml'
    # A 512 KiB default Fast DDS segment is smaller than one 848x480 image.
    # Both the GUI button and run_realsense_cuda.sh use this launch, so give
    # the camera writer enough SHM space regardless of how it is started.
    transport_environment = []
    if (os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') in
            ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp')
            and not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            and not os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE')):
        # This transport profile belongs to estimation_pkg, even when a second
        # camera supplies a stream_config YAML from another package/directory.
        image_profile = estimation_config / 'fastdds_images.xml'
        if not image_profile.is_file():
            raise RuntimeError('Rebuild estimation_pkg to install fastdds_images.xml.')
        transport_environment = [
            SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(image_profile)),
            LogInfo(msg=f'Using image-sized DDS shared memory: {image_profile}'),
        ]

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

    # Vendor declarations and our defaults must not leak into another camera
    # include. Explicit arguments from the including launch remain available.
    return LaunchDescription([GroupAction(scoped=True, actions=[
        DeclareLaunchArgument(
            'device_type', default_value='D405$',
            description='Exact RealSense model for HRM capture (default D405; no model fallback).',
        ),
        DeclareLaunchArgument(
            'serial_no', default_value="''",
            description='Optional exact SDK serial; an empty value selects only by model.',
        ),
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Unique camera node name and TF frame prefix.',
        ),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description='ROS namespace for this camera node.',
        ),
        DeclareLaunchArgument(
            'stream_config', default_value=str(camera_config),
            description='Stream parameter YAML; camera identity and frame overrides are forbidden.',
        ),
        OpaqueFunction(
            function=_validate_camera_selection,
        ),
        *transport_environment,
        SetEnvironmentVariable('AMENT_PREFIX_PATH', ament_prefix_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
        LogInfo(
            msg=f'Using CUDA librealsense: {librealsense_lib}'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(realsense_launch)),
            launch_arguments={
                'config_file': LaunchConfiguration('stream_config'),
                'device_type': LaunchConfiguration('device_type'),
                'serial_no': LaunchConfiguration('serial_no'),
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
            }.items(),
        ),
    ])])
