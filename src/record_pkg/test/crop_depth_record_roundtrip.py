#!/usr/bin/env python3
"""Synthetic cropped RGB/depth archive across separate Record/Stop sessions.

Use ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 after sourcing the ROS workspace.
An occupied domain is rejected. Only a recorder and synthetic publishers run:
no real cameras, controllers, actuator topics, or Pause/Resume services.
Artifacts stay beneath a unique /tmp directory for inspection.
"""

import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '184'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1.')

    import cv2
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter
    from custom_interfaces.msg import SegmentAngle
    from geometry_msgs.msg import PointStamped, WrenchStamped
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    import rosbag2_py
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    source_root = Path(__file__).resolve().parents[3]
    source_paths = [str(source_root / 'src' / name)
                    for name in ('record_pkg', 'estimation_pkg')]
    os.environ['PYTHONPATH'] = os.pathsep.join(
        source_paths + [os.environ.get('PYTHONPATH', '')])
    sys.path[:0] = source_paths
    from estimation_pkg.crop_depth import crop_depth_calibration

    artifacts = Path(tempfile.mkdtemp(prefix='hrm_crop_depth_record_', dir='/tmp'))
    os.environ['ROS_LOG_DIR'] = str(artifacts / 'ros_log')
    print('Artifacts:', artifacts, flush=True)
    crop_topic = '/estimated_segment_crop_image'
    depth_topic = '/estimated_segment_crop_depth/image_raw'
    metadata_topic = '/estimated_segment_crop_depth/metadata'
    angle_topic, tip_topic, force_topic = (
        '/estimated_segment_angle', '/estimated_tip_position', '/fts_data')
    numeric_topics = [angle_topic, tip_topic, force_topic]
    config = json.loads((source_root / 'src/record_pkg/config/recording.json').read_text())
    config.update(
        topic_groups={'synthetic': numeric_topics}, enabled_groups=['synthetic'],
        extra_topics=[], required_topics=numeric_topics, metadata_nodes=[],
        metadata_timeout_sec=1.0, required_max_gap_sec=2.0,
        startup_timeout_sec=10.0, auto_export_csv=True,
        summary_csv=dict(enabled=True, max_time_difference_ms=25.0,
                         startup_required_groups=['fts', 'angles']),
        max_cache_size_mb=4, max_bag_size_mb=16, min_free_disk_mb=32,
        image_archive=dict(enabled=True, topic=crop_topic, queue_size=16, required=True),
        depth_archive=dict(enabled=True, topic=depth_topic, metadata_topic=metadata_topic,
                           queue_size=16, required=True))
    config_path = artifacts / 'recording.json'
    config_path.write_text(json.dumps(config, indent=2))
    output_root = artifacts / 'sessions'
    rclpy.init()
    node = Node('crop_depth_record_test_sources')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    recorder = log = None
    statuses, source_phases, directories = [], {}, []
    phase, publish_depth, publish_images = 0, False, True
    depth_pixels = np.arange(96, dtype=np.uint16).reshape(8, 12) + 300
    depth_pixels[0, 0], depth_pixels[-1, -1] = 0, 65535
    camera_info = CameraInfo(width=32, height=24, distortion_model='plumb_bob')
    camera_info.header.frame_id = 'camera_color_optical_frame'
    camera_info.k = [300., 0., 16., 0., 310., 12., 0., 0., 1.]
    camera_info.p = [300., 0., 16., 0., 0., 310., 12., 0., 0., 0., 1., 0.]
    camera_info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    camera_info.d = [0., 0., 0., 0., 0.]

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_for(predicate, description, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), f'{description}; status={statuses[-2:]}; artifacts={artifacts}'

    try:
        spin_for(1.0)
        assert set(node.get_node_names()) <= {node.get_name()}, 'Domain 184 is occupied.'
        qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        crop_pub = node.create_publisher(Image, crop_topic, qos)
        depth_pub = node.create_publisher(Image, depth_topic, qos)
        metadata_pub = node.create_publisher(String, metadata_topic, qos)
        angle_pub = node.create_publisher(SegmentAngle, angle_topic, 10)
        tip_pub = node.create_publisher(PointStamped, tip_topic, 10)
        force_pub = node.create_publisher(WrenchStamped, force_topic, 10)
        node.create_subscription(
            String, '/data/record_status', lambda msg: statuses.append(json.loads(msg.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def publish():
            stamp = node.get_clock().now().to_msg()
            source = stamp.sec * 10**9 + stamp.nanosec
            source_phases[source] = phase
            angle = SegmentAngle()
            angle.header.stamp, angle.header.frame_id = stamp, 'hrm_base'
            for name in angle.get_fields_and_field_types():
                if name != 'header':
                    setattr(angle, name, [0.0] * 18)
            angle.tilt_relative[0], angle.pan_relative[1] = .123, -.234
            angle_pub.publish(angle)
            tip = PointStamped(header=angle.header)
            tip.point.x, tip.point.y, tip.point.z = .080, -.010, .020
            tip_pub.publish(tip)
            force = WrenchStamped()
            force.header.stamp, force.header.frame_id = stamp, 'fts_sensor'
            force.wrench.force.x = float(phase + 1)
            force_pub.publish(force)
            image = Image(width=12, height=8, encoding='rgb8', step=36,
                          data=bytes([13 + phase, 57, 201]) * 96)
            image.header.stamp, image.header.frame_id = stamp, 'camera_color_optical_frame'
            if publish_images:
                crop_pub.publish(image)
            if publish_depth and publish_images:
                depth = Image(header=image.header, width=12, height=8, encoding='16UC1',
                              is_bigendian=0, step=24, data=depth_pixels.tobytes())
                _, calibration = crop_depth_calibration(
                    camera_info, depth.header, depth.encoding, .001,
                    (24, 32), (4, 3, 12, 8))
                metadata_pub.publish(String(data=json.dumps(calibration)))
                depth_pub.publish(depth)

        node.create_timer(1.0 / 30, publish)
        client = node.create_client(SetBool, '/data/record')
        settings_client = node.create_client(SetParametersAtomically, '/record/set_parameters_atomically')
        log = (artifacts / 'record_node.log').open('w')
        recorder = subprocess.Popen([
            sys.executable, '-c', 'from record_pkg.record_node import main; main()',
            '--ros-args', '-p', f'config_file:={config_path}',
            '-p', f'output_root:={output_root}', '-p', 'contact_segment_id:=9',
        ], stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_for(client.service_is_ready, 'Recorder service missing')
        spin_for(.8)

        def request(value):
            future = client.call_async(SetBool.Request(data=value))
            wait_for(future.done, 'Record response missing')
            return future.result()

        def set_images(value):
            future = settings_client.call_async(SetParametersAtomically.Request(parameters=[
                Parameter('save_images', value=value).to_parameter_msg()]))
            wait_for(future.done, 'Image settings acknowledgement missing')
            return future.result().result

        rejected = request(True)
        assert not rejected.success and depth_topic in rejected.message, rejected
        assert not output_root.exists(), 'Missing required depth created a session.'
        publish_depth = True
        spin_for(.8)
        preserved = {}
        wait_for(settings_client.service_is_ready, 'Recorder parameter service missing')
        for phase in (0, 1, 2):
            publish_images = phase != 1
            assert set_images(publish_images).successful
            spin_for(.3)
            accepted = request(True)
            assert accepted.success, accepted.message
            wait_for(lambda: statuses and statuses[-1]['state'] == 'recording',
                     'Recording did not start')
            directory = Path(statuses[-1]['directory'])
            assert directory not in directories, 'Record overwrote a previous session.'
            directories.append(directory)
            assert not request(True).success, 'Duplicate Record overwrote active session.'
            assert not set_images(not publish_images).successful, 'Settings changed during capture.'
            spin_for(2.0)
            assert request(False).success
            wait_for(lambda: statuses[-1]['state'] in ('stopped', 'failed'),
                     'Post-stop finalization did not complete', timeout=35)
            assert statuses[-1]['state'] == 'stopped', statuses[-1]
            assert statuses[-1]['data_quality'] == 'complete', statuses[-1]
            metadata = json.loads((directory / 'session.json').read_text())
            assert metadata['snapshot']['contact_segment_id'] == 9
            assert metadata['snapshot']['save_images'] is publish_images
            assert 'recording_intervals' not in metadata
            effective = json.loads((directory / 'recording_config.json').read_text())
            assert effective['image_archive']['enabled'] is publish_images
            assert effective['depth_archive']['enabled'] is publish_images
            if not publish_images:
                assert not (directory / 'images').exists()
                postprocess = json.loads((directory / 'postprocess.json').read_text())
                assert postprocess['image_integrity']['status'] == 'disabled'
                assert postprocess['depth_integrity']['status'] == 'disabled'
            for name in (('hrm_crop', 'hrm_crop_depth') if publish_images else ()):
                root = directory / 'images' / name
                rows = read_csv(root / 'index.csv')
                manifest = json.loads((root / 'manifest.json').read_text())
                assert len(rows) >= 45, (name, len(rows))
                assert manifest['saved'] == manifest['accepted'] == len(rows)
                assert manifest['done'] and not manifest['pending']
                assert not manifest['errors'] and not manifest['dropped'], manifest
                assert not manifest['finalization_errors']
                assert 'recording_interval_id' not in rows[0]
                assert all(row['contact_segment_id'] == '9' for row in rows)
                assert {path.name for path in root.glob('*.png')} == {
                    row['filename'] for row in rows}
                for row in rows:
                    source = int(row['source_time_ns'])
                    assert source_phases[source] == phase
                    pixels = cv2.imread(str(root / row['filename']), cv2.IMREAD_UNCHANGED)
                    if name == 'hrm_crop':
                        assert pixels.shape == (8, 12, 3)
                        assert (pixels == [201, 57, 13 + phase]).all()
                    else:
                        assert pixels.dtype == np.uint16
                        assert np.array_equal(pixels, depth_pixels)
                        assert float(row['depth_scale_m_per_unit']) == .001
                        assert [int(row[f'roi_{key}']) for key in (
                            'x', 'y', 'width', 'height')] == [4, 3, 12, 8]
                        assert [int(row['source_width']), int(row['source_height'])] == [32, 24]
                        assert json.loads(row['k']) == [
                            300., 0., 12., 0., 310., 9., 0., 0., 1.]
                        assert json.loads(row['p']) == [
                            300., 0., 12., 0., 0., 310., 9., 0., 0., 0., 1., 0.]
                preserved[root / 'index.csv'] = hashlib.sha256(
                    (root / 'index.csv').read_bytes()).hexdigest()
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(directory / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
            topics = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
            assert set(topics) == set(numeric_topics), topics
            csv_manifest = json.loads((directory / 'csv/manifest.json').read_text())
            assert csv_manifest['status'] == 'complete'
            assert not csv_manifest['integrity']['required_topics_with_receive_gaps']
            for topic in numeric_topics:
                item = csv_manifest['topics'][topic]
                rows = read_csv(directory / 'csv' / item['file'])
                assert rows and 'recording_interval_id' not in rows[0]
            summary = read_csv(directory / 'csv/summary.csv')
            assert summary and 'recording_interval_id' not in summary[0]
            assert len(summary[0]) == 268, 'Existing numeric summary schema changed.'
            assert all(row['contact_segment_id'] == '9' for row in summary)
            assert all(abs(float(row['relative_angle_1']) - .123) < 1e-6 for row in summary)
            assert all(abs(float(row['relative_angle_2']) + .234) < 1e-6 for row in summary)
            assert all(float(row['relative_angle_3']) == 0 for row in summary)
            spin_for(.3)  # Sources continue; closed archives must remain unchanged.
            for path, digest in preserved.items():
                assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
        assert len(list(output_root.iterdir())) == 3
        assert not any(name == '/data/record_pause'
                       for name, _ in node.get_service_names_and_types())
        print('PASS: ON/OFF/ON Record/Stop sessions; OFF has no image files; exact uint16 depth and '
              'calibration; lossless RGB; numeric-only bag/CSV; no overwrite; no actuators.',
              flush=True)
        print('Sessions:', *directories, flush=True)
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
