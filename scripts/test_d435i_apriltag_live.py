#!/usr/bin/env python3
"""Bounded, isolated D435i RGB/rectification/AprilTag live validation.

Source the workspace first. This DOES open the exact requested D435i, but never
starts HRM estimation, motors, recording, or the D405. It refuses an occupied
localhost ROS domain and only terminates the launch processes that it creates.
Example: python3 scripts/test_d435i_apriltag_live.py --serial-no 949122070024 \
    --domain-id 96 --duration 15 --output-dir /tmp/d435i-apriltag-test-new
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


TOPICS = {
    'raw': '/tag_camera/tag_camera/color/image_raw',
    'rect': '/tag_camera/tag_camera/color/image_rect',
    'info': '/tag_camera/tag_camera/color/camera_info',
    'detections': '/detections',
    'tag0': '/apriltag/tag0/pose',
    'tag1': '/apriltag/tag1/pose',
    'relative': '/apriltag/tag1_in_tag0/pose',
}
REQUIRED_TRANSPORT = ('raw', 'rect', 'info', 'detections')
PARAMETERS = (
    'device_type', 'serial_no', 'camera_name', 'enable_color', 'enable_depth',
    'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_gyro', 'enable_accel',
    'rgb_camera.color_profile', 'align_depth.enable', 'enable_sync', 'pointcloud.enable',
)
IDENTITY_PARAMETERS = ('device_type', 'serial_no')


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class TopicStats:
    def __init__(self):
        self.stamps = []
        self.received = []
        self.frames = Counter()

    def observe(self, message, now=None):
        self.stamps.append(stamp_ns(message.header))
        self.received.append(time.monotonic() if now is None else now)
        self.frames[message.header.frame_id] += 1

    def report(self):
        stamps = sorted(set(self.stamps))
        span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
        receive_span = self.received[-1] - self.received[0] if len(self.received) > 1 else 0
        return dict(
            count=len(self.stamps), unique_source_stamps=len(stamps),
            frame_ids=dict(self.frames), source_stamp_ns=stamps,
            source_rate_hz=(len(stamps) - 1) * 1e9 / span if span > 0 else None,
            receive_rate_hz=(len(self.received) - 1) / receive_span if receive_span > 0 else None,
            max_receive_gap_ms=max(
                ((b - a) * 1000 for a, b in zip(self.received, self.received[1:])), default=None),
        )


def parameter_value(value):
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
    return getattr(value, fields[value.type]) if value.type in fields else None


def accept_parameter_response(name, values, received, issues):
    """One undeclared optional parameter must not invalidate identity readback."""
    if len(values) != 1 or parameter_value(values[0]) is None:
        issues[name] = f'Expected one declared scalar value; received {len(values)} value(s).'
        return False
    received[name] = parameter_value(values[0])
    issues.pop(name, None)
    return True


def configure_probe_transport(configure):
    # Child launches must demonstrate their own DDS setup, as in independent
    # GUI processes. Only an explicitly preexisting operator profile is shared.
    launch_environment = os.environ.copy()
    profile = configure()
    return launch_environment, profile


def summarize(stats, detection_rows, require_tag1=False):
    counts = Counter(str(tag['id']) for row in detection_rows for tag in row['tags'])
    missing = [name for name in REQUIRED_TRANSPORT if not stats[name].stamps]
    matches = {}
    for downstream, upstream in (
            ('rect', 'raw'), ('info', 'raw'), ('detections', 'rect'),
            ('tag0', 'detections'), ('tag1', 'detections'), ('relative', 'tag0')):
        child, parent = set(stats[downstream].stamps), set(stats[upstream].stamps)
        matches[f'{downstream}_in_{upstream}'] = dict(
            matched=len(child & parent), observed=len(child), unmatched=len(child - parent),
            fraction=len(child & parent) / len(child) if child else None)
    simultaneous = sum({0, 1} <= {tag['id'] for tag in row['tags']} for row in detection_rows)
    missing_poses = [name for name, tag_id in (('tag0', '0'), ('tag1', '1'))
                     if counts[tag_id] and not stats[name].stamps]
    if simultaneous and not stats['relative'].stamps:
        missing_poses.append('relative')
    return dict(
        topics={name: values.report() for name, values in stats.items()},
        missing_transport=missing, transport_ok=not missing,
        tag_detection_counts=dict(counts), tag1_detected=counts['1'] > 0,
        tag1_requirement_ok=not require_tag1 or counts['1'] > 0,
        empty_detection_arrays=sum(not row['tags'] for row in detection_rows),
        simultaneous_tag_frames=simultaneous, missing_detected_tag_poses=missing_poses,
        pose_pipeline_ok=not missing_poses,
        image_stamp_match=matches, detection_rows=detection_rows,
    )


def stop_owned(process):
    """Graceful launch shutdown, then only its own new-session process group."""
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            pass
    for sig, grace in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        os.killpg(process.pid, sig)
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
    try:
        process.wait(timeout=0.1)
    except subprocess.TimeoutExpired:
        pass


def save_snapshot(message, path):
    import cv2
    from cv_bridge import CvBridge
    image = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f'Could not save {path}')
    return dict(path=str(path), source_stamp_ns=stamp_ns(message.header),
                frame_id=message.header.frame_id, width=message.width, height=message.height,
                encoding=message.encoding)


def camera_library_evidence(camera_process, log_path, proc_root=Path('/proc')):
    """Inspect only driver PIDs announced by our launch and in its own process group."""
    content = log_path.read_text(errors='replace')
    pids = sorted(set(int(value) for value in re.findall(
        r'realsense2_camera_node[^\n]*process started with pid \[(\d+)\]', content)))
    records = []
    for pid in pids:
        try:
            if os.getpgid(pid) != camera_process.pid:
                records.append(dict(pid=pid, skipped='Not in the owned launch process group.'))
                continue
            lines = (proc_root / str(pid) / 'maps').read_text().splitlines()
            libraries = sorted(set(line.split()[-1] for line in lines
                                   if 'librealsense2' in line or 'libcudart' in line))
            records.append(dict(pid=pid, libraries=libraries))
        except (OSError, PermissionError) as error:
            records.append(dict(pid=pid, error=str(error)))
    return dict(
        driver_processes=records,
        cuda_sdk_path_loaded=any(
            '.cuda_realsense/install/librealsense/lib/' in path and 'librealsense2' in path
            for item in records for path in item.get('libraries', [])),
        device_log_lines=[line for line in content.splitlines() if any(
            phrase in line for phrase in ('Device Name:', 'Device Serial No:',
                                          'Device USB type:', 'Device FW version:'))],
        note='Loaded CUDA SDK is not proof of GPU workload; this camera uses RGB only.',
    )


def run(args):
    output = Path(args.output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)  # Never replace a previous test.
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(output / 'ros_logs'))
    result = dict(status='error', serial_no=args.serial_no, device_type='D435i$',
                  domain_id=args.domain_id, requested_duration_sec=args.duration,
                  notes=[
                      'Source rates and callback receive rates are different measurements.',
                      'Missing ID1 is not a transport failure when detection arrays arrive.',
                      'PNG is the latest observed image, not necessarily a tag-detected image.',
                      'CUDA SDK loading is checked only in our owned camera process group.',
                      'Source-stamp mismatches may include boundary/discovery/observer losses.',
                  ])
    processes, logs, node, context, executor, rclpy = [], [], None, None, None, None
    stats = {name: TopicStats() for name in TOPICS}
    latest, detection_rows, last_poses = {}, [], {}
    try:
        from estimation_pkg.runtime import configure_image_transport
        launch_environment, result['dds_profile'] = configure_probe_transport(
            configure_image_transport)
        result['child_transport_environment'] = {
            name: launch_environment.get(name) for name in (
                'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE')}
        import rclpy
        from apriltag_msgs.msg import AprilTagDetectionArray
        from geometry_msgs.msg import PoseStamped
        from rcl_interfaces.srv import GetParameters
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import CameraInfo, Image

        context = Context()
        rclpy.init(context=context)
        node = Node('d435i_apriltag_live_probe', context=context, use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def spin_once():
            for name, child in processes:
                if child.poll() is not None:
                    raise RuntimeError(f'{name} launch exited ({child.returncode}); see its log.')
            executor.spin_once(timeout_sec=0.05)

        discovery_until = time.monotonic() + 2.0
        while time.monotonic() < discovery_until:
            spin_once()
        graph = node.get_node_names_and_namespaces()
        if set(graph) != {('d435i_apriltag_live_probe', '/')} or len(graph) != 1:
            raise RuntimeError(
                f'Domain {args.domain_id} is occupied: {graph}; no launches started.')

        def receive(name, message):
            stats[name].observe(message)
            if name in ('raw', 'rect', 'info'):
                latest[name] = message
            elif name == 'detections':
                detection_rows.append(dict(
                    source_stamp_ns=stamp_ns(message.header), frame_id=message.header.frame_id,
                    tags=[dict(id=int(tag.id), hamming=int(tag.hamming),
                               decision_margin=float(tag.decision_margin))
                          for tag in message.detections]))
            else:
                last_poses[name] = message_to_ordereddict(message)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, topic in TOPICS.items():
            message_type = (Image if name in ('raw', 'rect') else
                            CameraInfo if name == 'info' else
                            AprilTagDetectionArray if name == 'detections' else PoseStamped)
            node.create_subscription(message_type, topic,
                                     lambda message, key=name: receive(key, message), qos)
        commands = {
            'camera': ['ros2', 'launch', 'launcher', 'cuda_apriltag_camera.launch.py',
                       'device_type:=D435i$', f'serial_no:=_{args.serial_no}'],
            'apriltag': ['ros2', 'launch', 'launcher', 'apriltag.launch.py', 'rectify:=true'],
        }
        result['commands'] = commands
        for name, command in commands.items():
            log = (output / f'{name}.log').open('x')
            logs.append(log)
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     env=launch_environment)
            processes.append((name, child))

        client = node.create_client(GetParameters, '/tag_camera/tag_camera/get_parameters')
        parameter_futures, parameter_issues, params, attempts, last_attempt = {}, {}, {}, {}, {}
        startup_started = time.monotonic()
        startup_deadline = startup_started + args.startup_timeout
        while time.monotonic() < startup_deadline:
            spin_once()
            transport_ready = all(stats[key].stamps for key in REQUIRED_TRANSPORT)
            if client.service_is_ready():
                for name in PARAMETERS:
                    if name in params or name in parameter_futures:
                        continue
                    if (name not in IDENTITY_PARAMETERS
                            and (not transport_ready or name in attempts)):
                        continue
                    if (attempts.get(name, 0) >= 3
                            or time.monotonic() - last_attempt.get(name, 0) < 0.5):
                        continue
                    parameter_futures[name] = client.call_async(
                        GetParameters.Request(names=[name]))
                    attempts[name] = attempts.get(name, 0) + 1
                    last_attempt[name] = time.monotonic()
            for name, future in list(parameter_futures.items()):
                if future.done():
                    del parameter_futures[name]
                    try:
                        accept_parameter_response(name, future.result().values,
                                                  params, parameter_issues)
                    except Exception as error:
                        parameter_issues[name] = str(error)
            if (transport_ready and all(name in params for name in IDENTITY_PARAMETERS)
                    and len(attempts) == len(PARAMETERS) and not parameter_futures):
                break
        result['camera_parameters'] = params
        result['parameter_readback_issues'] = parameter_issues
        result['parameter_readback_attempts'] = attempts
        result['startup_seconds'] = time.monotonic() - startup_started
        result['startup_counts'] = {name: len(item.stamps) for name, item in stats.items()}
        if params.get('device_type') != 'D435i$' or str(params.get('serial_no', '')).lstrip('_') \
                != args.serial_no:
            raise RuntimeError('Exact D435i device/serial parameter readback failed.')
        if not all(stats[key].stamps for key in REQUIRED_TRANSPORT):
            raise RuntimeError('Startup transport timeout (raw/rect/info/detections required).')

        # Measure a separate steady observation window, excluding driver startup.
        stats = {name: TopicStats() for name in TOPICS}
        detection_rows.clear()
        last_poses.clear()
        started, next_progress = time.monotonic(), time.monotonic() + 5.0
        while time.monotonic() - started < args.duration:
            spin_once()
            if time.monotonic() >= next_progress:
                print('Live counts: ' + ', '.join(
                    f'{name}={len(item.stamps)}' for name, item in stats.items()), flush=True)
                next_progress += 5.0
        result['observed_seconds'] = time.monotonic() - started
        result['camera_library_evidence'] = camera_library_evidence(
            dict(processes)['camera'], output / 'camera.log')
        result.update(summarize(stats, detection_rows, args.require_tag1))
        result['last_poses'] = last_poses
        if 'info' in latest:
            result['camera_info'] = message_to_ordereddict(latest['info'])
        info = latest['info']
        expected_frame = 'tag_camera_color_optical_frame'
        result['geometry_checks'] = dict(
            camera_info_calibrated=all((info.k[0] > 0, info.k[4] > 0,
                                        info.p[0] > 0, info.p[5] > 0)),
            nonzero_raw_distortion=any(abs(value) > 1e-12 for value in info.d),
            expected_camera_frame=all(
                set(stats[key].frames) == {expected_frame} for key in REQUIRED_TRANSPORT),
            image_sizes_match=all(
                (latest[key].width, latest[key].height) == (info.width, info.height)
                for key in ('raw', 'rect')),
            source_stamp_matches_present=all(
                result['image_stamp_match'][key]['matched'] > 0
                for key in ('rect_in_raw', 'info_in_raw', 'detections_in_rect')),
        )
        result['geometry_ok'] = all(value for key, value in result['geometry_checks'].items()
                                    if key != 'nonzero_raw_distortion')
        result['snapshots'] = {}
        for name in ('raw', 'rect'):
            if name in latest:
                try:
                    result['snapshots'][name] = save_snapshot(latest[name], output / f'{name}.png')
                except Exception as error:
                    result.setdefault('snapshot_errors', []).append(str(error))
        result['status'] = ('complete' if result['transport_ok'] and result['tag1_requirement_ok']
                            and result['pose_pipeline_ok'] and result['geometry_ok']
                            else 'failed')
    except (Exception, KeyboardInterrupt) as error:
        result.update(status='error', error=f'{type(error).__name__}: {error}')
        result.update(summarize(stats, detection_rows, args.require_tag1))
    finally:
        for name, child in reversed(processes):
            try:
                stop_owned(child)
            except Exception as error:
                result.setdefault('cleanup_errors', []).append(f'{name}: {error}')
                result['status'] = 'error'
        for log in logs:
            log.close()
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            rclpy.shutdown(context=context)
        result['child_exit_codes'] = {name: child.poll() for name, child in processes}
        with (output / 'report.json').open('x') as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
        print(f'Result: {result["status"]}; {output / "report.json"}', flush=True)
    return 0 if result['status'] == 'complete' else 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-no', required=True, help='Exact D435i SDK serial, ASCII digits.')
    parser.add_argument('--domain-id', type=int, default=96)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--startup-timeout', type=float, default=20.0)
    parser.add_argument('--output-dir', required=True,
                        help='New output directory (must not exist).')
    parser.add_argument('--require-tag1', action='store_true')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[0-9]+', args.serial_no):
        parser.error('--serial-no must contain only ASCII digits; no model fallback is permitted.')
    if not 1 <= args.domain_id <= 101:
        parser.error('--domain-id must be an isolated domain in 1..101, never domain 0.')
    if not 0 < args.duration <= 30 or not 0 < args.startup_timeout <= 20:
        parser.error('--duration must be in (0,30], --startup-timeout in (0,20] seconds.')
    return args


if __name__ == '__main__':
    raise SystemExit(run(parse_args()))
