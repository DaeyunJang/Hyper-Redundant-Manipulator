#!/usr/bin/env python3

"""Rectify the dedicated tag camera, detect tags, and publish stamped poses."""

import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _topic(context, name):
    value = LaunchConfiguration(name).perform(context)
    if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', value):
        raise RuntimeError(f'{name} must be an absolute ROS topic name, got {value!r}.')
    return value


def _camera_info_remappings(image_topic, info_topic):
    # AprilTag resolves image_rect before image_transport derives its sibling
    # camera_info. Map that absolute sibling too, for nonstandard CLI topics.
    sibling = image_topic.rsplit('/', 1)[0] + '/camera_info'
    return [('camera_info', info_topic), (sibling, info_topic)]


def _local_image_proc_prefix():
    # Optional official debs installed without sudo by the workspace helper.
    # Resolve symlinks so this also works through a colcon symlink install.
    for root in Path(__file__).resolve().parents:
        prefix = root / '.ros_image_proc/root/opt/ros/humble'
        if ((prefix / 'share/ament_index/resource_index/packages/image_proc').is_file()
                and (prefix / 'lib/librectify.so').is_file()):
            return prefix
    return None


def _image_proc_environment(context):
    prefix = _local_image_proc_prefix()
    try:
        installed_share = Path(get_package_share_directory('image_proc')).resolve()
        if prefix is None or installed_share != (prefix / 'share/image_proc').resolve():
            return []  # Prefer an existing ROS installation; do not shadow it.
        # AMENT_PREFIX_PATH may already expose our local extraction while its
        # dependent libraries are still missing from LD_LIBRARY_PATH. Complete
        # both environments instead of treating this as a system installation.
    except PackageNotFoundError as exc:
        if prefix is None:
            raise RuntimeError(
                'D435i RGB requires rectification: install ros-humble-image-proc '
                'or run bash scripts/install_image_proc_local.sh. '
                'Use rectify:=false only with an already rectified image input.') from exc
    actions = []
    for name, directory in (('AMENT_PREFIX_PATH', prefix), ('LD_LIBRARY_PATH', prefix / 'lib')):
        # Preserve parent launch overrides, which need not be in os.environ.
        current = context.environment.get(name, '').split(os.pathsep)
        value = os.pathsep.join([str(directory)] + [
            entry for entry in current if entry and entry != str(directory)])
        # Both the launch process (component resource lookup) and child container
        # (plugin loading) need the prefix. This changes only this launch process.
        os.environ[name] = value
        actions.append(SetEnvironmentVariable(name, value))
    actions.append(LogInfo(msg=f'Using isolated official image_proc: {prefix}'))
    return actions


def _image_transport_environment(context):
    # The camera launch is a separate process. Its image-sized DDS profile is
    # not inherited by this rectifier, which publishes full RGB images too.
    environment = context.environment
    if (environment.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') not in
            ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp')
            or environment.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            or environment.get('FASTDDS_DEFAULT_PROFILES_FILE')):
        return []
    profile = (Path(get_package_share_directory('estimation_pkg'))
               / 'config/fastdds_images.xml')
    if not profile.is_file():
        raise RuntimeError('Rebuild estimation_pkg to install fastdds_images.xml.')
    return [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile)),
        LogInfo(msg=f'AprilTag rectifier uses image-sized DDS shared memory: {profile}'),
    ]


def _launch_pipeline(context):
    raw_image = _topic(context, 'image_topic')
    info = _topic(context, 'camera_info_topic')
    rectify = LaunchConfiguration('rectify').perform(context).lower()
    if rectify not in ('true', 'false'):
        raise RuntimeError('rectify must be true or false.')
    actions, detector_image = [], raw_image
    if rectify == 'true':
        detector_image = _topic(context, 'rectified_image_topic')
        if detector_image in (raw_image, info):
            raise RuntimeError('rectified_image_topic must differ from both input topics.')
        actions.extend(_image_transport_environment(context))
        actions.extend(_image_proc_environment(context))
        actions.append(ComposableNodeContainer(
            package='rclcpp_components', executable='component_container',
            name='container', namespace='/apriltag_preprocess', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='image_proc', plugin='image_proc::RectifyNode',
                name='rectify_color', namespace='/apriltag_preprocess',
                # Humble image_proc uses input publisher QoS (sensor_data fallback).
                # RGB + CameraInfo are matched by source timestamp; rectification
                # retains header/frame/size and uses CameraInfo K,D,R,P.
                remappings=[('image', raw_image), ('image_rect', detector_image)]
                + _camera_info_remappings(raw_image, info),
            )],
        ))
    actions.extend([
        Node(
            package='apriltag_ros', executable='apriltag_node',
            name='apriltag_node', namespace='/', output='screen',
            parameters=[LaunchConfiguration('params_file')],
            # AprilTag uses P (rectified intrinsics), not K+D distortion correction.
            # Original CameraInfo remains correct and keeps the source image stamp.
            remappings=[('image_rect', detector_image)]
            + _camera_info_remappings(detector_image, info),
        ),
        Node(
            package='record_pkg', executable='apriltag_pose',
            name='apriltag_pose', namespace='/', output='screen',
            parameters=[{
                'tag0_frame': LaunchConfiguration('tag0_frame'),
                'tag1_frame': LaunchConfiguration('tag1_frame'),
            }],
        ),
    ])
    return actions


def generate_launch_description():
    parameter_file = (
        Path(get_package_share_directory('launcher'))
        / 'config/apriltag.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(parameter_file)),
        DeclareLaunchArgument(
            'image_topic', default_value='/tag_camera/tag_camera/color/image_raw',
            description='Full raw RGB; with rectify:=false this must already be rectified.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/tag_camera/tag_camera/color/camera_info'),
        DeclareLaunchArgument(
            'rectify', default_value='true',
            description='Disable only for already rectified inputs such as D405 image_rect_raw.'),
        DeclareLaunchArgument(
            'rectified_image_topic', default_value='/tag_camera/tag_camera/color/image_rect'),
        DeclareLaunchArgument('tag0_frame', default_value='ID0',
                              description='Must match tag.frames[0] in params_file.'),
        DeclareLaunchArgument('tag1_frame', default_value='ID1',
                              description='Must match tag.frames[1] in params_file.'),
        OpaqueFunction(function=_launch_pipeline),
    ])
