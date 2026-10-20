#!/usr/bin/env python3
"""Replay synthetic moving RGB-D at 30 Hz in isolated ROS domain 73.

Run after sourcing install/setup.bash. This starts estimator, dry-run FK, and
the C++ pipeline probe, never a real camera or TCP/motor bridge. Logs and the
tested input configuration are printed/saved in a unique temporary directory.
This tests throughput and timestamp integrity, not real-camera angle accuracy.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def publish_synthetic(duration):
    if os.environ.get('ROS_DOMAIN_ID') != '73':
        raise RuntimeError('Synthetic camera requires isolated ROS_DOMAIN_ID=73.')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    cv2.setNumThreads(1)
    rclpy.init()
    node = Node('synthetic_camera_benchmark')
    color = node.create_publisher(
        Image, '/camera/camera/color/image_rect_raw', qos_profile_sensor_data)
    depth = node.create_publisher(
        Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile_sensor_data)
    info = node.create_publisher(
        CameraInfo, '/camera/camera/color/camera_info', qos_profile_sensor_data)
    depth_data = np.full((480, 848), 180, np.uint16).tobytes()
    start = time.monotonic()
    count = 0

    def tick():
        nonlocal count
        elapsed = time.monotonic() - start
        y = np.arange(40, 440)
        x = (390 + (70 + 30 * np.sin(elapsed)) * np.sin((440 - y) / 400 * 1.3)).astype(np.int32)
        image = np.zeros((480, 848, 3), np.uint8)
        cv2.polylines(image, [np.column_stack((x, y))], False, (255, 255, 255), 32)
        stamp = node.get_clock().now().to_msg()
        d = Image(height=480, width=848, encoding='16UC1', step=848 * 2, data=depth_data)
        d.header.stamp = stamp
        d.header.frame_id = 'camera_color_optical_frame'
        c = Image(height=480, width=848, encoding='rgb8', step=848 * 3, data=image.tobytes())
        c.header = d.header
        i = CameraInfo(height=480, width=848, k=[432., 0., 422., 0., 432., 241., 0., 0., 1.])
        i.header = d.header
        info.publish(i)
        depth.publish(d)
        color.publish(c)
        count += 1

    node.create_timer(1 / 30, tick)
    try:
        while rclpy.ok() and time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        print(f'Synthetic input: {count} frames in {time.monotonic() - start:.3f}s')
        node.destroy_node()
        rclpy.try_shutdown()


def benchmark(args):
    from ament_index_python.packages import get_package_share_directory
    from estimation_pkg.runtime import configure_image_transport

    # All benchmark processes share the same profile. Do not mutate the user's
    # default ROS domain or replace any running production node.
    profile = args.dds_profile or configure_image_transport()
    env = dict(os.environ, ROS_DOMAIN_ID='73', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    if profile:
        env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(profile).resolve())
    log_dir = Path(tempfile.mkdtemp(prefix='hrm-pipeline-'))
    roi = Path(get_package_share_directory('estimation_pkg')) / 'config_ROI_ref.json'
    print(f'Domain 73; profile={profile}; ROI={roi}; logs={log_dir}', flush=True)
    commands = [
        ('estimator', ['ros2', 'run', 'estimation_pkg', 'segment_angle_estimator',
                       '--ros-args', '-p', 'debug_rate_enabled:=true']),
        ('fk', ['ros2', 'launch', 'robot_control_pkg', '_launch.py',
                'motor_output_enabled:=false']),
        ('probe', ['ros2', 'run', 'image_rate_probe', 'pipeline_rate_probe']),
        ('source', [sys.executable, str(Path(__file__).resolve()), '--publish-only',
                    '--duration', str(args.duration)]),
    ]
    processes = []
    try:
        for name, command in commands:
            with (log_dir / f'{name}.log').open('w') as log:
                processes.append(subprocess.Popen(
                    command, env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True))
        if processes[-1].wait(timeout=args.duration + 15) != 0:
            raise RuntimeError(f'Synthetic source failed; see {log_dir}.')
        for process, (name, _) in zip(processes[:-1], commands[:-1]):
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited early; see {log_dir}.')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    print((log_dir / 'source.log').read_text())
    print('\n'.join((log_dir / 'probe.log').read_text().splitlines()[-19:]))
    print(f'Full per-window rates, latency and estimator/FK logs: {log_dir}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--dds-profile', help='Optional XML override for transport A/B tests')
    parser.add_argument('--publish-only', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    if args.publish_only:
        publish_synthetic(args.duration)
    else:
        benchmark(args)
