#!/usr/bin/env python3
"""Bounded passive capture of an already running HRM; never command hardware.

Reuses record_pkg's bag/export pipeline in a separate owned session. Does not
start/restart cameras, estimators, controllers or serial/TCP nodes. Records all
selected topics without requiring unused diagnostics; missing data are reported.
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output-root', type=Path, default=Path('record/tip_validation'))
    parser.add_argument('--telemetry-only', action='store_true',
                        help='Skip Image/PointCloud2 payloads; retain markers, TF and all sensors.')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be in 1..60')

    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/hrm-tip-validation-ros-log')
    if (os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp').startswith('rmw_fastrtps')
            and not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            and not os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE')):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
            root / 'src/estimation_pkg/config/fastdds_images.xml')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
    from record_pkg.capture import CaptureSession, save_json, selected_topics
    from record_pkg.finalize_session import finalize_session
    from record_pkg.metadata import ParameterSnapshot, reference_file

    config_path = root / 'src/record_pkg/config/recording.json'
    config = json.loads(config_path.read_text())
    # Tag rectified images aid blur/detection diagnosis even with raw disabled.
    config['extra_topics'] = list(dict.fromkeys(
        config.get('extra_topics', []) + ['/tag_camera/tag_camera/color/image_rect']))
    config.setdefault('notes', {})['passive_validation'] = (
        'User requested a stationary no-command test. Original required list is retained '
        'for audit, but this bounded diagnostic capture permits incomplete input. '
        'No automatic hardware launch, parameter mutation, command or rosbag replay.')
    session = CaptureSession(config, args.output_root)
    rclpy.init(args=[])
    node = Node('hrm_passive_tip_capture', use_global_arguments=False)
    counts, samples, tf_counts = Counter(), {}, Counter()
    last_status = {}
    subscriptions = []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)

    def status_callback(message):
        nonlocal last_status
        last_status = json.loads(message.data)

    subscriptions.append(node.create_subscription(
        String, '/data/record_status', status_callback,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def receive(topic, message):
        counts[topic] += 1
        if topic == '/tf':
            for transform in message.transforms:
                if transform.child_frame_id in ('hrm_fk_tip', 'estimated_segment_center_19'):
                    tf_counts[transform.child_frame_id] += 1
            return
        # Small messages only; no duplicate image/cloud ingestion in Python.
        samples[topic] = message_to_ordereddict(message)

    def spin_for(seconds, snapshot=None):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0, deadline - time.monotonic())))
            if snapshot:
                snapshot.poll()

    started = False
    try:
        spin_for(3)
        if any(name.startswith('rosbag2_recorder') for name in node.get_node_names()):
            raise RuntimeError('Another recorder is running; refusing duplicate capture.')
        if last_status.get('state') not in ('idle', 'stopped', 'failed'):
            raise RuntimeError(f'Cannot verify GUI recorder is idle: {last_status}')
        graph = dict(node.get_topic_names_and_types())
        for topic in selected_topics(config):
            names = graph.get(topic, [])
            if len(names) != 1 or names[0] in (
                    'sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2',
                    'visualization_msgs/msg/MarkerArray', 'rcl_interfaces/msg/ParameterEvent'):
                continue
            subscriptions.append(node.create_subscription(
                get_message(names[0]), topic,
                lambda msg, name=topic: receive(name, msg), qos))
        parameters = ParameterSnapshot(node, config.get('metadata_nodes', []), timeout_sec=4)
        spin_for(4.5, parameters)
        published = {topic: names for topic, names in node.get_topic_names_and_types()
                     if topic in session.topics and node.count_publishers(topic)}
        if args.telemetry_only:
            excluded = [topic for topic in session.topics
                        if 'image' in topic or topic.endswith('centerline_points')]
            original_required = config['required_topics'].copy()
            config['topic_groups'] = {'passive_telemetry': [
                topic for topic in session.topics if topic not in excluded]}
            config['enabled_groups'] = ['passive_telemetry']
            config['extra_topics'] = []
            config['required_topics'] = [topic for topic in original_required
                                         if topic not in excluded]
            config['notes']['telemetry_only'] = dict(
                excluded_topics=excluded, original_required_topics=original_required,
                reason='Bounded position/angle comparison without image disk backpressure.')
            session = CaptureSession(config, args.output_root)
            published = {topic: names for topic, names in published.items()
                         if topic in session.topics}
        print(json.dumps(dict(preflight_counts=dict(counts), tf_counts=dict(tf_counts),
                              recorder_status=last_status), ensure_ascii=False), flush=True)
        required_comparison = ('/apriltag/tag1_in_tag0/pose', '/estimated_tip_position')
        if any(not counts[topic] for topic in required_comparison):
            raise RuntimeError(
                'No live comparison samples; leave hardware unchanged and inspect preflight.')
        snapshot = dict(
            published_topics=published, runtime_parameters=parameters.report,
            reference_files={str(config_path): reference_file(config_path)},
            force_alignment=last_status.get('force_alignment'),
            passive_capture=True, commands_sent=0, runtime_before=samples.copy(),
            preflight_counts=dict(counts), tf_counts=dict(tf_counts),
            camera_clock_synchronization='not calibrated; retain source and receive timestamps',
        )
        for rel in ('src/estimation_pkg/estimation_pkg/config.json',
                    'src/estimation_pkg/estimation_pkg/config_ROI_ref.json',
                    'src/launcher/config/apriltag.yaml',
                    'src/launcher/config/realsense_apriltag.yaml'):
            snapshot['reference_files'][rel] = reference_file(root / rel)
        directory = session.start(snapshot, allow_incomplete=True)
        started = True
        print(f'CAPTURE_DIRECTORY={directory}', flush=True)
        session.metadata['passive_validation_seconds'] = args.seconds
        session.metadata['recording_ready_ros_ns'] = node.get_clock().now().nanoseconds
        counts.clear()
        tf_counts.clear()
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline and session.active:
            rclpy.spin_once(node, timeout_sec=0.05)
            session.poll(ready=True)
        session.metadata['stop_requested_ros_ns'] = node.get_clock().now().nanoseconds
        session.metadata['passive_observation'] = dict(
            counts=dict(counts), tf_counts=dict(tf_counts), last_small_messages=samples)
        session.close()  # Signals only the recorder process owned by this object.
        print(json.dumps(dict(state=session.state, message_counts=session.metadata.get(
            'message_counts'), missing=session.metadata.get('selected_topics_without_messages'))),
            flush=True)
        save_json(directory / 'passive_observation.json', session.metadata['passive_observation'])
    finally:
        if started:
            session.close()
        node.destroy_node()
        rclpy.shutdown()

    report = finalize_session(directory)
    integrity = report.get('integrity') or {}
    print(json.dumps(dict(directory=str(directory), csv_status=report['status'],
                          integrity_status=integrity.get('status'),
                          gap_topics=integrity.get('required_topics_with_receive_gaps')),
                     ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'complete' and integrity.get('status') == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
