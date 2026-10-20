#!/usr/bin/env python3
"""Replay identical saved RGB frames through CPU AprilTag and Isaac ROS 3.1.

Source the dedicated Isaac comparison environment first for --backends isaac.
NPZ: RGB8 images[N,H,W,3], increasing int64 stamps_ns[N], camera_info_json scalar.
No camera, motor, production topics, bag replay or system configuration changes.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

from compare_apriltag_settings import summary


PREFIX = '/apriltag_backend_compare'


def package_provenance(package):
    """Record the discovered package manifest, not a guessed release version."""
    result = dict(package=package)
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        path = Path(get_package_share_directory(package)) / 'package.xml'
        content = path.read_bytes()
        result.update(prefix=get_package_prefix(package), package_xml=str(path),
                      version=ET.fromstring(content).findtext('version'),
                      package_xml_sha256=hashlib.sha256(content).hexdigest())
    except Exception as error:
        result.update(version=None, unavailable=str(error))
    return result


def stats(values):
    values = [float(value) for value in values if value is not None and math.isfinite(value)]
    return dict(summary(values), p95=float(np.percentile(values, 95)) if values else None)


def normalize_pose(parent, position, quaternion):
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or
            not np.isfinite(q).all() or not math.isfinite(norm) or norm < 1e-12):
        return None
    q = q / norm
    if q[3] < 0:
        q = -q
    return dict(parent=parent, xyz_m=p.tolist(), xyzw=q.tolist())


def relative_pose(first, second):
    if not first or not second or first['parent'] != second['parent']:
        return None
    x, y, z, w = first['xyzw']
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    inverse_q = np.array([-x, -y, -z, w])
    other = np.asarray(second['xyzw'])
    vector = (inverse_q[3] * other[:3] + other[3] * inverse_q[:3] +
              np.cross(inverse_q[:3], other[:3]))
    quaternion = [*vector, inverse_q[3] * other[3] - np.dot(inverse_q[:3], other[:3])]
    return normalize_pose('ID0', rotation.T @ (np.array(second['xyz_m']) - first['xyz_m']),
                          quaternion)


def pose_steps(frames, key):
    translation, angles, previous = [], [], None
    for frame in frames:
        pose = frame['poses'].get(key)
        if pose is not None and previous is not None:
            translation.append(np.linalg.norm(np.array(pose['xyz_m']) - previous['xyz_m']))
            dot = abs(float(np.dot(pose['xyzw'], previous['xyzw'])))
            angles.append(math.degrees(2 * math.acos(min(1.0, dot))))
        previous = pose
    return dict(translation_step_m=stats(translation), rotation_step_deg=stats(angles))


def report_frames(frames, elapsed):
    responses = [row for row in frames if row['status'] == 'response']
    miss_run, best = [], []
    for row in frames:
        if row['status'] == 'response' and 1 not in row['ids']:
            miss_run.append(row['source_stamp_ns'])
            if len(miss_run) > len(best):
                best = list(miss_run)
        else:  # Transport timeouts are unknown, not detector misses.
            miss_run = []
    return dict(
        processed_frames=len(frames), response_frames=len(responses),
        response_timeouts=len(frames)-len(responses),
        empty_response_frames=sum(not row['ids'] for row in responses),
        paired_id0_id1_frames=sum({0, 1} <= set(row['ids']) for row in responses),
        wall_seconds=elapsed, sequential_replay_hz=len(frames)/elapsed if elapsed else None,
        round_trip_ms=stats([row['round_trip_ms'] for row in responses]),
        longest_id1_miss_run=dict(
            frames=len(best), first_stamp_ns=best[0] if best else None,
            last_stamp_ns=best[-1] if best else None,
            source_span_sec=(best[-1]-best[0])/1e9 if best else 0.0),
        tags={str(tag): dict(
            detected_frames=sum(tag in row['ids'] for row in responses),
            min_edge_pixels=stats([det['min_edge_pixels'] for row in responses
                                   for det in row['detections'] if det['id'] == tag]),
            hamming=stats([det['hamming'] for row in responses
                           for det in row['detections'] if det['id'] == tag]),
            decision_margin=stats([det['decision_margin'] for row in responses
                                   for det in row['detections'] if det['id'] == tag]),
            consecutive_pose_steps=pose_steps(frames, f'ID{tag}')) for tag in (0, 1)},
        relative_pose_steps=pose_steps(frames, 'ID1_in_ID0'))


def detection_record(detection, parent, isaac=False):
    corners = [[float(point.x), float(point.y)] for point in detection.corners]
    points = np.asarray(corners)
    pose = None
    if isaac:
        value = detection.pose.pose.pose
        p, q = value.position, value.orientation
        pose = normalize_pose(detection.pose.header.frame_id or parent,
                              [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def finite(value):
        return float(value) if math.isfinite(value) else None
    edge = np.linalg.norm(points-np.roll(points, 1, axis=0), axis=1).min()
    return dict(id=int(detection.id), family=detection.family,
                corners_xy_pixels=[[finite(value) for value in point] for point in corners],
                hamming=None if isaac else int(detection.hamming),
                decision_margin=None if isaac else finite(detection.decision_margin), pose=pose,
                corners_valid=bool(np.isfinite(points).all()),
                min_edge_pixels=finite(edge))


def stop_owned(process):
    """Signal only the process/session created by this harness, including children."""
    if process is None:
        return
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, 0)
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        # The parent may have exited before its component/container children.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)


def compare(args):
    with np.load(args.input_npz, allow_pickle=False) as data:
        images, stamps = data['images'], data['stamps_ns']
        info_dict = json.loads(str(data['camera_info_json'].item()))
    if (images.dtype != np.uint8 or images.ndim != 4 or images.shape[-1] != 3 or
            min(images.shape[:3]) < 1 or stamps.dtype != np.int64 or
            stamps.shape != (len(images),) or np.any(stamps <= 0) or np.any(np.diff(stamps) <= 0)):
        raise ValueError('Expected RGB8 frames and increasing positive int64 source stamps.')
    if (info_dict['height'], info_dict['width']) != images.shape[1:3]:
        raise ValueError('CameraInfo dimensions differ from full RGB frame dimensions.')
    images, stamps = images[:args.limit], stamps[:args.limit]
    logs = Path(tempfile.mkdtemp(prefix='hrm-apriltag-backends-'))
    output = Path(args.output).expanduser().resolve() if args.output else logs/'comparison.json'
    if output.exists():
        raise FileExistsError(f'Refusing to replace {output}')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(logs))
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.set_message import set_message_fields
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = Node('apriltag_backend_probe', use_global_arguments=False)
    process, responses, poses, variants = None, {}, {}, []
    result = dict(status='running', input_npz=str(Path(args.input_npz).resolve()), logs=str(logs),
                  frame_count=len(images), image_shape=list(images.shape[1:]), domain=args.domain,
                  source_stamp_range_ns=[int(stamps[0]), int(stamps[-1])], camera_info=info_dict,
                  variants=variants, note='Sequential identical saved frames, not live-camera FPS. '
                  'Roundtrip includes Python publish/DDS/scheduling/detection/return, not pure '
                  'detector latency. Pose steps include real motion, not guaranteed noise/error. '
                  'ID1 miss source span is first-to-last missing response stamp (single miss=0); '
                  'timeouts break miss runs. Tag labels/quaternions normalized, no undocumented '
                  'pose-axis correction between backends. Isaac 3.1 has no hamming/margin output.')

    def stamp(header):
        return header.stamp.sec * 10**9 + header.stamp.nanosec

    def spin_until(predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'Detector exited; inspect logs {logs}')
            rclpy.spin_once(node, timeout_sec=0.005)
        return predicate()

    def on_tf(message):
        for transform in message.transforms:
            child = transform.child_frame_id
            tag = next((n for n in (0, 1) if child in (f'ID{n}', f'tag36h11:{n}')), None)
            if tag is not None:
                p, q = transform.transform.translation, transform.transform.rotation
                pose = normalize_pose(transform.header.frame_id,
                                      [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                if pose:
                    poses.setdefault(stamp(transform.header), {})[f'ID{tag}'] = pose

    try:
        spin_until(lambda: False, 1.5)
        if node.get_node_names_and_namespaces() != [('apriltag_backend_probe', '/')]:
            raise RuntimeError(f'Domain {args.domain} is occupied; refusing to publish any images.')
        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_pub = node.create_publisher(Image, PREFIX+'/image', reliable)
        info_pub = node.create_publisher(CameraInfo, PREFIX+'/camera_info', reliable)
        node.create_subscription(TFMessage, PREFIX+'/tf', on_tf, best_effort)
        choices = ([('cpu_h0', False, 0), ('cpu_h1', False, 1)] if 'cpu' in args.backends else [])
        choices += [('isaac_gpu', True, None)] if 'isaac' in args.backends else []
        for name, isaac, hamming in choices:
            parameters = ({'size': 0.017, 'max_tags': 64, 'tile_size': 4} if isaac else dict(
                family='36h11', size=0.017, max_hamming=hamming,
                **{'detector.threads': 1, 'detector.decimate': 1.0, 'detector.blur': 0.0,
                   'detector.refine': True, 'detector.sharpening': 0.25, 'detector.debug': False,
                   'tag.ids': [0, 1], 'tag.frames': ['ID0', 'ID1'], 'tag.sizes': [0.017, 0.017]}))
            if isaac:
                command = ['ros2', 'launch', str(Path(__file__).with_name(
                    'apriltag_backend_probe.launch.py').resolve())]
                message_type = 'isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray'
            else:
                executable = (Path(get_package_prefix('apriltag_ros')) /
                              'lib/apriltag_ros/apriltag_node')
                command = [str(executable), '--ros-args', '-r', '__node:=detector',
                           '-r', '__ns:='+PREFIX+'/cpu']
                for source, target in [('image_rect', 'image'), ('camera_info', 'camera_info'),
                                       ('detections', 'detections'), ('/tf', 'tf'),
                                       ('/tf_static', 'tf_static')]:
                    command += ['-r', f'{source}:={PREFIX}/{target}']
                for key, value in parameters.items():
                    command += ['-p', f'{key}:={json.dumps(value)}']
                message_type = 'apriltag_msgs/msg/AprilTagDetectionArray'
            subscription = node.create_subscription(
                get_message(message_type), PREFIX+'/detections',
                lambda msg: responses.__setitem__(stamp(msg.header), (time.monotonic(), msg)),
                best_effort)
            rows, warmup, started = [], [], time.monotonic()
            variant = dict(name=name, parameters=parameters, command=command, frames=rows,
                           warmup=warmup, expected_frames=len(images), status='running',
                           backend_package=package_provenance(
                               'isaac_ros_apriltag' if isaac else 'apriltag_ros'))
            variants.append(variant)
            try:
                with (logs/f'{name}.log').open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               stdin=subprocess.DEVNULL, start_new_session=True)
                if not spin_until(lambda: image_pub.get_subscription_count() > 0 and
                                  info_pub.get_subscription_count() > 0 and
                                  node.count_publishers(PREFIX+'/detections') > 0,
                                  args.startup_timeout):
                    raise RuntimeError('Detector input/output discovery timed out.')
                responses.clear()
                poses.clear()
                consecutive_timeouts = 0
                samples = [(None, images[0], int(stamps[0])-args.warmup+i)
                           for i in range(args.warmup)]
                samples += [(i, pixels, int(t))
                            for i, (pixels, t) in enumerate(zip(images, stamps))]
                for index, pixels, source_stamp in samples:
                    info = CameraInfo()
                    set_message_fields(info, info_dict)
                    info.header.stamp.sec, info.header.stamp.nanosec = divmod(source_stamp, 10**9)
                    image = Image(height=images.shape[1], width=images.shape[2], encoding='rgb8',
                                  step=images.shape[2]*3, data=pixels.tobytes(), header=info.header)
                    sent = time.monotonic()
                    info_pub.publish(info)
                    image_pub.publish(image)
                    arrived = spin_until(lambda: source_stamp in responses,
                                         max(5.0, args.timeout) if index is None else args.timeout)
                    consecutive_timeouts = 0 if arrived else consecutive_timeouts + 1
                    row = dict(index=index, source_stamp_ns=source_stamp,
                               status='response' if arrived else 'timeout', round_trip_ms=None,
                               ids=[], detections=[], poses={})
                    if arrived:
                        received, message = responses.pop(source_stamp)
                        row['round_trip_ms'] = (received-sent)*1000
                        row['detections'] = [detection_record(det, message.header.frame_id, isaac)
                                             for det in message.detections]
                        row['ids'] = [d['id'] for d in row['detections']]
                        expected = {f'ID{n}' for n in (0, 1) if row['ids'].count(n) == 1}
                        if not isaac:
                            spin_until(lambda: expected <= poses.get(source_stamp, {}).keys(), 0.05)
                        row['poses'] = ({f'ID{d["id"]}': d['pose'] for d in row['detections']
                                         if d['pose'] and f'ID{d["id"]}' in expected} if isaac else
                                        {k: v for k, v in poses.get(source_stamp, {}).items()
                                         if k in expected})
                        relative = relative_pose(row['poses'].get('ID0'), row['poses'].get('ID1'))
                        if relative:
                            row['poses']['ID1_in_ID0'] = relative
                    (warmup if index is None else rows).append(row)
                    if index == 0:
                        started = sent
                    if consecutive_timeouts >= 3:
                        raise RuntimeError('Three consecutive response timeouts. Missing response '
                                           'does not prove a detector miss; inspect backend logs.')
                    if index is not None and (index+1) % 30 == 0:
                        print(f'{name}: {index+1}/{len(images)}', flush=True)
                variant['status'] = 'complete'
            except Exception as error:
                variant.update(status='error', error=str(error))
                raise
            finally:
                variant.update(report_frames(rows, time.monotonic()-started))
                stop_owned(process)
                process = None
                node.destroy_subscription(subscription)
            if not spin_until(lambda: node.count_publishers(PREFIX+'/detections') == 0, 5.0):
                raise RuntimeError('Previous detector graph remains; refusing overlapping runs.')
            print(f'{name}: ID1={variant["tags"]["1"]["detected_frames"]}/{len(images)}, '
                  f'timeouts={variant["response_timeouts"]}', flush=True)
        result['status'] = 'complete'
    except Exception as error:
        result.update(status='error', error=str(error))
        raise
    finally:
        stop_owned(process)
        node.destroy_node()
        rclpy.shutdown()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
        print(f'Results: {output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_npz')
    parser.add_argument('--output')
    parser.add_argument('--domain', type=int, default=95)
    parser.add_argument('--backends', default='cpu,isaac', help='cpu, isaac, or cpu,isaac')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--timeout', type=float, default=1.0)
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    parser.add_argument('--warmup', type=int, default=3)
    args = parser.parse_args()
    args.backends = args.backends.split(',')
    if (not 1 <= args.domain <= 101 or not 0 < args.timeout <= 10 or
            not 0 < args.startup_timeout <= 60 or not 0 <= args.warmup <= 10 or
            (args.limit is not None and args.limit < 1) or
            not set(args.backends) <= {'cpu', 'isaac'} or not args.backends):
        parser.error('Use cpu/isaac, isolated domain 1..101, positive timeout/limit, warmup 0..10.')
    compare(args)


if __name__ == '__main__':
    main()
