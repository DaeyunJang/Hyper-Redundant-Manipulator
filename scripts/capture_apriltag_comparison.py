#!/usr/bin/env python3
"""Read-only RGB/CameraInfo capture for compare_apriltag_backends.py.

Only exact source-stamp pairs are saved. Incomplete captures retain their valid
samples, but return a nonzero exit code and status in capture_metadata.json.
No camera settings, detector processes, or other hardware are touched.
"""

import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np


def configure_dds():
    for name in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        if os.environ.get(name):
            return os.environ[name]
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') not in (
            'rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return None
    profile = Path(__file__).resolve().parents[1] / 'src/estimation_pkg/config/fastdds_images.xml'
    if not profile.is_file():
        raise FileNotFoundError(profile)
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    return str(profile)


def validate_options(frames, sample_hz, timeout):
    if type(frames) is not int or not 1 <= frames <= 300:
        raise ValueError('frames must be an integer in 1..300')
    for name, value in (('sample-hz', sample_hz), ('timeout', timeout)):
        if (isinstance(value, bool) or not math.isfinite(value) or value <= 0
                or (name == 'sample-hz' and not math.isfinite(1e9 / value))):
            raise ValueError(f'{name} must be finite and positive')


class CameraSamples:
    """Bounded matching queues; conversion/copy occurs only for selected frames."""

    def __init__(self, frames, sample_hz):
        self.limit, self.period_ns = frames, max(1, round(1e9 / sample_hz))
        self.images, self.stamps, self.pending = [], [], ({}, {})
        self.info_json = self.calibration = None
        self.next_sample_ns = 0
        self.counts = dict(color=0, camera_info=0, matched=0, invalid_stamp=0, evicted=0)
        self.to_rgb = self.to_dict = None  # Injected ROS adapters; pure tests need no ROS.

    def push(self, stream, message):
        self.counts[('color', 'camera_info')[stream]] += 1
        stamp = message.header.stamp
        key = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if key <= 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self.counts['invalid_stamp'] += 1
            return
        if len(self.images) >= self.limit or (self.stamps and key <= self.stamps[-1]):
            return
        queue = self.pending[stream]
        queue[key] = message
        while len(queue) > 12:
            queue.pop(next(iter(queue)))
            self.counts['evicted'] += 1
        if key not in self.pending[1 - stream]:
            return
        image, info = (queue.pop(key) for queue in self.pending)
        self.counts['matched'] += 1
        if (min(info.width, info.height) <= 0
                or (image.width, image.height) != (info.width, info.height)):
            raise ValueError('Image and CameraInfo dimensions differ.')
        if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
            raise ValueError('Image and CameraInfo frames differ or are empty.')
        info_dict = self.to_dict(info)
        signature = dict(info_dict, header={'frame_id': info.header.frame_id})
        calibration = json.dumps(signature, sort_keys=True, allow_nan=False)
        if len(info.k) != 9 or info.k[0] <= 0 or info.k[4] <= 0:
            raise ValueError('CameraInfo must have calibrated positive focal lengths.')
        if self.calibration is not None and calibration != self.calibration:
            raise ValueError('Camera dimensions, frame, or calibration changed during capture.')
        if key < self.next_sample_ns:
            return
        pixels = self.to_rgb(image)
        if pixels.dtype != np.uint8 or pixels.shape != (info.height, info.width, 3):
            raise ValueError('Converted frame is not RGB8 with the CameraInfo dimensions.')
        if self.info_json is None:
            self.info_json = json.dumps(info_dict, allow_nan=False)
            self.calibration, self.next_sample_ns = calibration, key
        self.images.append(pixels.copy())
        self.stamps.append(key)
        # Preserve sampling phase despite camera timestamp jitter / dropped frames.
        self.next_sample_ns += ((key - self.next_sample_ns) // self.period_ns + 1) * self.period_ns


def capture_live(args, samples):
    """Only two subscriptions; caller controls bounded duration and output path."""
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import CameraInfo, Image

    configure_dds()  # Must precede ROS context / DDS participant creation.
    os.environ.setdefault('ROS_LOG_DIR', str(args.output / 'ros_log'))
    bridge = CvBridge()
    samples.to_rgb = lambda message: bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
    samples.to_dict = message_to_ordereddict
    rclpy.init(args=[])
    node = None
    try:
        node = Node('apriltag_comparison_capture', use_global_arguments=False)
        node.create_subscription(Image, args.image_topic,
                                 lambda msg: samples.push(0, msg), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, args.info_topic,
                                 lambda msg: samples.push(1, msg), qos_profile_sensor_data)
        deadline = time.monotonic() + args.timeout
        while len(samples.images) < args.frames and rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f'Captured only {len(samples.images)}/{args.frames} matching frames.')
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
        if len(samples.images) < args.frames:
            raise RuntimeError('ROS stopped before capture completed.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def save_capture(args, samples, status, error, elapsed):
    count, stamps = len(samples.images), samples.stamps
    metadata = dict(
        schema_version=1, status=status, error=error,
        topics={'image': args.image_topic, 'camera_info': args.info_topic},
        requested_frames=args.frames, saved_frames=count, requested_sample_hz=args.sample_hz,
        timeout_sec=args.timeout, elapsed_capture_sec=elapsed, counts=samples.counts,
        first_source_stamp_ns=stamps[0] if count else None,
        last_source_stamp_ns=stamps[-1] if count else None,
        actual_sample_rate_hz=(count - 1) * 1e9 / (stamps[-1] - stamps[0]) if count > 1 else None,
        image_shape=list(samples.images[0].shape) if count else None,
        camera_frame_id=json.loads(samples.info_json)['header']['frame_id'] if count else None,
    )
    if count:
        np.savez_compressed(args.output / 'capture.npz', images=np.stack(samples.images),
                            stamps_ns=np.asarray(stamps, dtype=np.int64),
                            camera_info_json=np.asarray(samples.info_json))
    if count and args.save_preview:
        try:
            import cv2
            if not cv2.imwrite(str(args.output / 'first_rgb.png'), samples.images[0][:, :, ::-1]):
                raise RuntimeError('PNG writer returned false.')
        except Exception as exc:
            metadata['preview_warning'] = str(exc)  # Optional preview cannot invalidate the NPZ.
    (args.output / 'capture_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='New directory; existing paths are refused.')
    parser.add_argument('--frames', type=int, default=90)
    parser.add_argument('--sample-hz', type=float, default=10.0)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--image-topic', default='/camera/camera/color/image_rect_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--save-preview', action='store_true',
                        help='Also write first_rgb.png if cv2 is available.')
    args = parser.parse_args(argv)
    try:
        validate_options(args.frames, args.sample_hz, args.timeout)
        args.output = args.output.expanduser().absolute()
        args.output.mkdir(parents=True, exist_ok=False)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    samples, started = CameraSamples(args.frames, args.sample_hz), time.monotonic()
    status, error, code = 'complete', None, 0
    try:
        capture_live(args, samples)
    except KeyboardInterrupt:
        status, error, code = 'interrupted', 'Capture interrupted.', 130
    except Exception as exc:
        status = 'timeout' if isinstance(exc, TimeoutError) else 'failed'
        error, code = str(exc), 2
    save_capture(args, samples, status, error, time.monotonic() - started)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
