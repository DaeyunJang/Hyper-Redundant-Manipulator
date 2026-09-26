#!/usr/bin/env python3
"""Compare AprilTag settings on identical saved RGB frames, without hardware.

Source ROS first. NPZ requires uint8 images[N,H,W,3] RGB, unique increasing
int64 stamps_ns[N], and scalar camera_info_json from message_to_ordereddict.
Example: python3 scripts/compare_apriltag_settings.py frames.npz --domain 92
Only an owned C++ apriltag_node is launched; all image/info/detection/TF topics
are private to an otherwise empty localhost ROS domain. No bag replay, camera,
motor commands, or live-node parameter changes occur.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np


PREFIX = '/apriltag_compare'
VARIANTS = ((2.0, 0), (1.0, 0), (1.0, 1))


def summary(values):
    return (dict(count=len(values), mean=float(np.mean(values)), std=float(np.std(values)),
                 minimum=float(np.min(values)), maximum=float(np.max(values)))
            if values else dict(count=0, mean=None, std=None, minimum=None, maximum=None))


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            return
        process.send_signal(sig)  # Direct child, never discover/kill by ROS node name.
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            continue


def report_variant(frames, decimate, hamming, elapsed):
    received = [frame for frame in frames if frame['status'] == 'response']
    result = dict(decimate=decimate, max_hamming=hamming, input_frames=len(frames),
                  response_frames=len(received), timeout_frames=len(frames) - len(received),
                  empty_response_frames=sum(not frame['detections'] for frame in received),
                  response_coverage=len(received) / len(frames), wall_seconds=elapsed,
                  sequential_replay_rate_hz=len(frames) / elapsed,
                  round_trip_ms=summary([frame['round_trip_ms'] for frame in received]),
                  tags={}, frames=frames)
    for tag_id in (0, 1):
        detections = [det for frame in received for det in frame['detections']
                      if det['id'] == tag_id]
        histogram = {}
        for det in detections:
            key = str(det['hamming'])
            histogram[key] = histogram.get(key, 0) + 1
        steps_m, steps_deg, previous = [], [], None
        for frame in frames:
            pose = frame['poses'].get(f'ID{tag_id}')
            if pose is not None and previous is not None:
                steps_m.append(float(np.linalg.norm(np.array(pose['xyz_m']) - previous['xyz_m'])))
                a, b = np.asarray(pose['xyzw']), np.asarray(previous['xyzw'])
                dot = abs(float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))))
                steps_deg.append(math.degrees(2 * math.acos(min(1.0, max(0.0, dot)))))
            previous = pose  # Missing TF breaks adjacency; do not compare across a gap.
        detected_frames = sum(any(det['id'] == tag_id for det in frame['detections'])
                              for frame in received)
        result['tags'][str(tag_id)] = dict(
            detected_frames=detected_frames, fraction_of_input=detected_frames / len(frames),
            fraction_of_responses=detected_frames / len(received) if received else None,
            hamming_histogram=histogram,
            decision_margin=summary([det['decision_margin'] for det in detections]),
            min_edge_pixels=summary([det['min_edge_pixels'] for det in detections]),
            consecutive_pose_translation_step_m=summary(steps_m),
            consecutive_pose_rotation_step_deg=summary(steps_deg))
    return result


def compare(args):
    with np.load(args.input_npz, allow_pickle=False) as archive:
        images, stamps = archive['images'], archive['stamps_ns']
        info_dict = json.loads(str(archive['camera_info_json'].item()))
    if (images.dtype != np.uint8 or images.ndim != 4 or images.shape[-1] != 3
            or min(images.shape[:3]) < 1 or stamps.dtype != np.int64
            or stamps.shape != (len(images),) or np.any(np.diff(stamps) <= 0)
            or np.any(stamps <= 0)):
        raise ValueError(
            'Expected nonempty RGB8 frames and unique increasing positive int64 stamps.')
    if (info_dict['height'], info_dict['width']) != images.shape[1:3]:
        raise ValueError('CameraInfo dimensions must match the full RGB images.')
    logs = Path(tempfile.mkdtemp(prefix='hrm-apriltag-compare-'))
    output = Path(args.output).expanduser().resolve() if args.output else logs / 'comparison.json'
    if output.exists():
        raise FileExistsError(f'Refusing to replace {output}')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs))
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    if not any(os.environ.get(key) for key in (
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')):
        profile = Path(get_package_share_directory('estimation_pkg')) / 'config/fastdds_images.xml'
        if profile.is_file():
            os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    import rclpy
    from apriltag_msgs.msg import AprilTagDetectionArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.set_message import set_message_fields
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node('apriltag_settings_probe', use_global_arguments=False)
    detector, rows, reports, responses, poses = None, [], [], {}, {}

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if detector is not None and detector.poll() is not None:
                raise RuntimeError(f'Detector exited; inspect {logs}.')
            rclpy.spin_once(node, timeout_sec=0.005)
        return predicate()

    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def receive_detection(message):
        responses[stamp_ns(message.header)] = (time.monotonic(), message)

    def receive_tf(message):
        for transform in message.transforms:
            if transform.child_frame_id in ('ID0', 'ID1'):
                p, q = transform.transform.translation, transform.transform.rotation
                poses.setdefault(stamp_ns(transform.header), {})[transform.child_frame_id] = dict(
                    parent=transform.header.frame_id,
                    xyz_m=[p.x, p.y, p.z], xyzw=[q.x, q.y, q.z, q.w])

    result = dict(input_npz=str(Path(args.input_npz).resolve()), domain=args.domain,
                  frame_count=len(images), image_shape=list(images.shape[1:]),
                  source_stamp_range_ns=[int(stamps[0]), int(stamps[-1])],
                  logs=str(logs), variants=reports,
                  note='Sequential identical-frame comparison, not live-camera FPS. round_trip_ms '
                       'includes Python publish, DDS, scheduling, detector and return callback; '
                       'it is NOT pure detector latency. Pose steps include real scene motion '
                       'and are not ground-truth pose error or guaranteed jitter measures.')
    try:
        spin_until(lambda: False, 1.5)
        if node.get_node_names_and_namespaces() != [('apriltag_settings_probe', '/')]:
            raise RuntimeError(f'ROS domain {args.domain} is occupied; refusing to publish inputs.')
        image_pub = node.create_publisher(Image, PREFIX + '/image', qos_profile_sensor_data)
        info_pub = node.create_publisher(
            CameraInfo, PREFIX + '/camera_info', qos_profile_sensor_data)
        node.create_subscription(
            AprilTagDetectionArray, PREFIX + '/detections', receive_detection, 10)
        node.create_subscription(TFMessage, PREFIX + '/tf', receive_tf, qos_profile_sensor_data)
        executable = Path(get_package_prefix('apriltag_ros')) / 'lib/apriltag_ros/apriltag_node'
        for decimate, hamming in VARIANTS:
            config = dict(family='36h11', size=0.017, max_hamming=hamming,
                          **{'detector.threads': 1, 'detector.decimate': decimate,
                             'detector.blur': 0.0, 'detector.refine': True,
                             'detector.sharpening': 0.25, 'detector.debug': False,
                             'tag.ids': [0, 1], 'tag.frames': ['ID0', 'ID1'],
                             'tag.sizes': [0.017, 0.017]})
            command = [str(executable), '--ros-args', '-r', '__node:=apriltag_compare_detector']
            for source, target in (('image_rect', '/image'), ('camera_info', '/camera_info'),
                                   ('detections', '/detections'), ('/tf', '/tf'),
                                   ('/tf_static', '/tf_static')):
                command += ['-r', f'{source}:={PREFIX}{target}']
            for key, value in config.items():
                command += ['-p', f'{key}:={json.dumps(value)}']
            with (logs / f'decimate_{decimate}_hamming_{hamming}.log').open('w') as log:
                detector = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                            stdin=subprocess.DEVNULL, start_new_session=True)
                try:
                    if not spin_until(lambda: image_pub.get_subscription_count() > 0
                                      and info_pub.get_subscription_count() > 0
                                      and node.count_publishers(PREFIX + '/detections') > 0, 8.0):
                        raise RuntimeError('Detector input/output discovery timed out.')
                    spin_until(lambda: False, 0.25)
                    responses.clear()
                    poses.clear()
                    rows, started, consecutive_timeouts = [], time.monotonic(), 0
                    for index, (pixels, stamp) in enumerate(zip(images, stamps)):
                        stamp = int(stamp)
                        info = CameraInfo()
                        set_message_fields(info, info_dict)
                        info.header.stamp.sec, info.header.stamp.nanosec = divmod(
                            stamp, 1_000_000_000)
                        image = Image(height=images.shape[1], width=images.shape[2],
                                      encoding='rgb8',
                                      step=images.shape[2] * 3, data=pixels.tobytes())
                        image.header = info.header
                        sent = time.monotonic()
                        info_pub.publish(info)
                        image_pub.publish(image)
                        received = spin_until(lambda: stamp in responses, args.timeout)
                        consecutive_timeouts = 0 if received else consecutive_timeouts + 1
                        if consecutive_timeouts >= 3:
                            raise RuntimeError(
                                'Three consecutive response timeouts: transport/detector '
                                'failure, NOT evidence of three missing-tag images.')
                        row = dict(index=index, source_stamp_ns=stamp,
                                   status='response' if received else 'timeout',
                                   round_trip_ms=None, detections=[], poses={})
                        if received:
                            arrived, message = responses[stamp]
                            row['round_trip_ms'] = (arrived - sent) * 1000
                            for det in message.detections:
                                corners = np.asarray([(point.x, point.y) for point in det.corners])
                                row['detections'].append(dict(
                                    id=det.id, hamming=det.hamming,
                                    decision_margin=det.decision_margin,
                                    min_edge_pixels=float(np.min(np.linalg.norm(
                                        corners - np.roll(corners, 1, axis=0), axis=1)))))
                            expected = {f'ID{det.id}' for det in message.detections
                                        if det.id in (0, 1)}
                            spin_until(lambda: expected <= poses.get(stamp, {}).keys(), 0.05)
                            row['poses'] = dict(poses.get(stamp, {}))
                        rows.append(row)
                        if (index + 1) % 30 == 0:
                            print(f'decimate={decimate}, hamming={hamming}: '
                                  f'{index + 1}/{len(images)} frames processed', flush=True)
                    report = report_variant(rows, decimate, hamming, time.monotonic() - started)
                    report.update(parameters=config, command=command)
                    reports.append(report)
                    print(f'decimate={decimate}, max_hamming={hamming}: '
                          f'ID1 {report["tags"]["1"]["detected_frames"]}/{len(images)}, '
                          f'timeouts={report["timeout_frames"]}', flush=True)
                finally:
                    stop_owned(detector)
                    detector = None
                if not spin_until(lambda: node.count_publishers(PREFIX + '/detections') == 0, 5.0):
                    raise RuntimeError(
                        'Previous detector graph remains; refusing overlapping runs.')
        result['status'] = 'complete'
    except Exception as error:
        result.update(status='error', error=str(error))
        raise
    finally:
        stop_owned(detector)
        node.destroy_node()
        rclpy.shutdown()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
        print(f'Results: {output}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_npz')
    parser.add_argument('--output')
    parser.add_argument('--domain', type=int, default=92)
    parser.add_argument('--timeout', type=float, default=1.0,
                        help='Per-frame response timeout seconds')
    arguments = parser.parse_args()
    if not 1 <= arguments.domain <= 101 or not 0 < arguments.timeout <= 5:
        parser.error('Use an isolated domain 1..101 and timeout > 0 and <= 5 seconds.')
    compare(arguments)
