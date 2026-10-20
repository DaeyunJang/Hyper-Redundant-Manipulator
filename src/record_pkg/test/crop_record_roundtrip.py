#!/usr/bin/env python3
"""Crop PNG + numeric bag/CSV integration using only synthetic sources.

Run after building record_pkg with ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1.
Refuses a different/occupied domain. No camera, controller, motor, or launch
package is started. Synthetic data and logs are retained beneath /tmp.
"""

import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '183'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1.')

    import cv2
    from custom_interfaces.msg import SegmentAngle
    from geometry_msgs.msg import PointStamped
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    import rosbag2_py
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    artifacts = Path(tempfile.mkdtemp(prefix='hrm_crop_record_roundtrip_', dir='/tmp'))
    os.environ['ROS_LOG_DIR'] = str(artifacts / 'ros_log')
    print('Artifacts:', artifacts, flush=True)
    crop_topic = '/estimated_segment_crop_image'
    angle_topic = '/estimated_segment_angle'
    tip_topic = '/estimated_tip_position'
    config = json.loads((Path(__file__).resolve().parents[1]
                         / 'config/recording.json').read_text())
    config.update(topic_groups={'synthetic': [angle_topic, tip_topic]},
                  enabled_groups=['synthetic'], extra_topics=[],
                  required_topics=[angle_topic, tip_topic], metadata_nodes=[],
                  metadata_timeout_sec=1.0, required_max_gap_sec=2.0,
                  startup_timeout_sec=10.0, auto_export_csv=True,
                  max_cache_size_mb=4, max_bag_size_mb=16, min_free_disk_mb=32,
                  depth_archive={'enabled': False},
                  summary_csv=dict(enabled=True, max_time_difference_ms=25.0,
                                   startup_required_groups=['angles']),
                  image_archive=dict(enabled=True, topic=crop_topic,
                                     queue_size=16, required=True))
    config_path = artifacts / 'recording.json'
    config_path.write_text(json.dumps(config, indent=2))
    output_root = artifacts / 'sessions'
    rclpy.init()
    node = Node('crop_record_test_sources')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    recorder = None
    log = None
    statuses = []
    source_stamps = set()
    publish_crop = False

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_for(predicate, description, timeout=12):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), f'{description}; status={statuses[-2:]}; artifacts={artifacts}'

    try:
        spin_for(1.0)
        assert set(node.get_node_names()) <= {node.get_name()}, 'Domain 183 is occupied.'
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        crop_pub = node.create_publisher(Image, crop_topic, qos)
        angle_pub = node.create_publisher(SegmentAngle, angle_topic, 10)
        tip_pub = node.create_publisher(PointStamped, tip_topic, 10)
        node.create_subscription(
            String, '/data/record_status', lambda msg: statuses.append(json.loads(msg.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def publish():
            stamp = node.get_clock().now().to_msg()
            angle = SegmentAngle()
            angle.header.stamp = stamp
            angle.header.frame_id = 'hrm_base'
            for name in angle.get_fields_and_field_types():
                if name != 'header':
                    setattr(angle, name, [0.0] * 18)
            angle.tilt_relative[0] = 0.123
            angle.pan_relative[1] = -0.234
            angle_pub.publish(angle)
            tip = PointStamped()
            tip.header = angle.header
            tip.point.x, tip.point.y, tip.point.z = 0.080, -0.010, 0.020
            tip_pub.publish(tip)
            if publish_crop:
                image = Image(width=12, height=8, encoding='rgb8', step=36,
                              data=bytes([13, 57, 201]) * 12 * 8)
                image.header.stamp = stamp
                image.header.frame_id = 'camera_color_optical_frame'
                crop_pub.publish(image)
                source_stamps.add(stamp.sec * 10**9 + stamp.nanosec)

        node.create_timer(1.0 / 30, publish)
        client = node.create_client(SetBool, '/data/record')
        log = (artifacts / 'record_node.log').open('w')
        recorder = subprocess.Popen([
            'ros2', 'run', 'record_pkg', 'record', '--ros-args',
            '-p', f'config_file:={config_path}', '-p', f'output_root:={output_root}',
            '-p', 'contact_segment_id:=9',
        ], stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_for(client.service_is_ready, 'Recorder service missing')
        spin_for(1.0)
        assert not output_root.exists(), 'Recorder started without explicit Record request.'

        def request(value):
            future = client.call_async(SetBool.Request(data=value))
            wait_for(future.done, 'Record service response missing')
            return future.result()

        rejected = request(True)
        assert not rejected.success and crop_topic in rejected.message, rejected
        assert not output_root.exists(), 'Missing crop created a session.'
        publish_crop = True
        spin_for(0.8)
        accepted = request(True)
        assert accepted.success, accepted.message
        wait_for(lambda: statuses and statuses[-1]['state'] == 'recording',
                 'Healthy source streams never reached recording')
        directory = Path(statuses[-1]['directory'])
        assert not request(True).success, 'Duplicate Record must not overwrite a session.'
        spin_for(2.0)
        assert request(False).success
        wait_for(lambda: statuses[-1]['state'] in ('stopped', 'failed'),
                 'PNG/bag/CSV finalization did not complete', timeout=30)
        assert statuses[-1]['state'] == 'stopped', statuses[-1]
        assert statuses[-1]['data_quality'] == 'complete', statuses[-1]
        assert any(status['state'] == 'exporting' for status in statuses)

        archive_root = directory / 'images/hrm_crop'
        with (archive_root / 'index.csv').open() as handle:
            index = list(csv.DictReader(handle))
        assert len(index) >= 30, f'Only {len(index)} synthetic frames saved.'
        manifest = json.loads((archive_root / 'manifest.json').read_text())
        assert manifest['saved'] == len(index) == manifest['accepted']
        assert manifest['done'] and not manifest['pending']
        assert not manifest['errors'] and not manifest['finalization_errors']
        assert not manifest['dropped'], manifest
        png_names = {path.name for path in archive_root.glob('*.png')}
        assert png_names == {row['filename'] for row in index}
        for row in index:
            assert int(row['source_time_ns']) in source_stamps
            assert int(row['received_time_ns']) > 0
            assert row['frame_id'] == 'camera_color_optical_frame'
            assert int(row['contact_segment_id']) == 9
            pixels = cv2.imread(str(archive_root / row['filename']))
            assert pixels.shape == (8, 12, 3)
            assert (pixels == [201, 57, 13]).all(), 'Lossless RGB/BGR conversion changed colors.'
        spin_for(0.3)  # Crop publishing continues, but recording is over.
        assert len(list(archive_root.glob('*.png'))) == len(index)

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(directory / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        bag_topics = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
        assert set(bag_topics) == {angle_topic, tip_topic}, bag_topics
        assert 'sensor_msgs/msg/Image' not in bag_topics.values()
        csv_manifest = json.loads((directory / 'csv/manifest.json').read_text())
        assert csv_manifest['status'] == 'complete', csv_manifest
        for topic in (angle_topic, tip_topic):
            item = csv_manifest['topics'][topic]
            assert item['rows_written'] > 0
            assert (directory / 'csv' / item['file']).is_file()
        with (directory / 'csv/summary.csv').open() as handle:
            summary = list(csv.DictReader(handle))
        assert summary
        assert all(row['contact_segment_id'] == '9' for row in summary)
        assert all(abs(float(row['relative_angle_1']) - 0.123) < 1e-6 for row in summary)
        assert all(abs(float(row['relative_angle_2']) + 0.234) < 1e-6 for row in summary)
        assert all(float(row['relative_angle_3']) == 0 for row in summary)
        metadata = json.loads((directory / 'session.json').read_text())
        assert metadata['snapshot']['contact_segment_id'] == 9
        assert metadata['image_archive']['saved'] == len(index)
        assert not any('tag_camera' in name for name in bag_topics)
        print('PASS: explicit Record only; missing crop rejected; lossless timestamped crop PNGs; '
              'numeric-only bag and unchanged angle/label summary; post-Stop frames ignored.',
              flush=True)
        print('Session:', directory, flush=True)
    finally:
        if recorder is not None and recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid, signal.SIGTERM)
                recorder.wait(timeout=5)
        if log is not None:
            log.close()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
