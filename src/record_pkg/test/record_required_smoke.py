#!/usr/bin/env python3
"""Actual recorder lifecycle with missing/stalled required data, no hardware.

Source the built workspace and run with /usr/bin/python3. The test owns ROS
domain 80 (localhost), rejects an occupied domain, and saves artifacts in /tmp.
Only synthetic String/PoseStamped publishers and record_pkg are started.
"""

import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '80'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

TICK = '/record_test/tick'
POSE = '/record_test/pose'


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    artifacts = Path(tempfile.mkdtemp(prefix='hrm_required_smoke_', dir='/tmp'))
    os.environ['ROS_LOG_DIR'] = str(artifacts / 'ros_log')
    config = json.loads((Path(__file__).resolve().parents[1] / 'config/recording.json').read_text())
    config.update(topic_groups={'test': [TICK, POSE]}, enabled_groups=['test'],
                  image_archive={'enabled': False}, depth_archive={'enabled': False},
                  extra_topics=[], required_topics=[TICK, POSE], metadata_nodes=['/record'],
                  metadata_timeout_sec=1.0, required_max_gap_sec=2.0,
                  startup_timeout_sec=10.0, auto_export_csv=True,
                  max_cache_size_mb=4, max_bag_size_mb=16, min_free_disk_mb=64)
    config_path = artifacts / 'recording.json'
    config_path.write_text(json.dumps(config, indent=2))
    output_root = artifacts / 'sessions'
    rclpy.init()
    node = Node('record_required_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    recorder = None
    log = None
    statuses = []
    publish_pose = False
    sequence = 0
    source_pose_stamps = []

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)

    def wait_for(predicate, description, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        assert predicate(), f'{description}; artifacts={artifacts}, status={statuses[-1:]}'

    try:
        spin_for(0.3)
        assert set(node.get_node_names()) <= {'record_required_test'}, 'Test domain 80 is occupied.'
        tick_pub = node.create_publisher(String, TICK, 10)
        pose_pub = node.create_publisher(PoseStamped, POSE, 10)
        node.create_subscription(
            String, '/data/record_status', lambda message: statuses.append(json.loads(message.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def publish():
            nonlocal sequence
            sequence += 1
            tick_pub.publish(String(data=str(sequence)))
            if publish_pose:
                message = PoseStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = 'synthetic_base'
                message.pose.position.x = 1.0 + sequence / 1000.0
                message.pose.orientation.w = 1.0
                pose_pub.publish(message)
                source_pose_stamps.append(
                    message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)

        node.create_timer(0.05, publish)
        client = node.create_client(SetBool, '/data/record')
        log = (artifacts / 'record_node.log').open('w')
        recorder = subprocess.Popen([
            'ros2', 'run', 'record_pkg', 'record', '--ros-args',
            '-p', f'config_file:={config_path}', '-p', f'output_root:={output_root}',
        ], stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_for(client.service_is_ready, 'Record service did not start.')
        spin_for(0.7)

        def request(value):
            future = client.call_async(SetBool.Request(data=value))
            wait_for(future.done, 'Record service did not respond.', timeout=8)
            return future.result()

        assert node.count_publishers(POSE) == 1
        rejected = request(True)
        assert not rejected.success and POSE in rejected.message, rejected
        assert not output_root.exists() or not list(output_root.iterdir())
        publish_pose = True
        spin_for(0.8)
        accepted = request(True)
        assert accepted.success, accepted.message
        wait_for(lambda: statuses and statuses[-1]['state'] == 'recording',
                 'Healthy data did not become recording.')
        directory = Path(statuses[-1]['directory'])
        spin_for(2.5)
        publish_pose = False  # Publisher still exists; its data are now absent.
        last_pose_stamp = source_pose_stamps[-1]
        wait_for(lambda: POSE in (statuses[-1].get('live_health') or {}).get('missing_or_stale', []),
                 'Stalled required data did not produce a live warning.', timeout=5)
        assert statuses[-1]['state'] == 'recording', statuses[-1]
        assert 'DATA WARNING' in statuses[-1]['detail']
        assert node.count_publishers(POSE) == 1
        spin_for(0.7)
        stopped = request(False)
        assert stopped.success, stopped.message
        wait_for(lambda: statuses[-1]['state'] in ('stopped', 'failed'),
                 'Bag/CSV finalization did not finish.', timeout=20)
        assert statuses[-1]['state'] == 'stopped', statuses[-1]
        assert statuses[-1]['data_quality'] == 'incomplete', statuses[-1]
        assert any(status['state'] == 'exporting' for status in statuses)
        metadata = json.loads((directory / 'session.json').read_text())
        report = json.loads((directory / 'postprocess.json').read_text())
        assert report['status'] == 'complete'
        assert report['integrity']['status'] == 'incomplete'
        assert POSE in report['integrity']['required_topics_with_receive_gaps']
        assert list((directory / 'bag').glob('*.db3'))
        assert (directory / 'bag/metadata.yaml').is_file()
        manifest = json.loads((directory / 'csv/manifest.json').read_text())
        entry = manifest['topics'][POSE]
        with (directory / 'csv' / entry['file']).open() as handle:
            rows = list(csv.DictReader(handle))
        assert rows and len(rows) == entry['rows_written']
        assert len(rows) == report['integrity']['topics'][POSE]['message_count']
        captured_stamps = [int(row['source_time_ns']) for row in rows]
        assert len(set(captured_stamps)) == len(rows), 'Repeated samples were fabricated.'
        assert set(captured_stamps) <= set(source_pose_stamps)
        assert max(captured_stamps) <= last_pose_stamp
        assert all(float(row['pose.position.x']) > 1.0 for row in rows)
        assert all(row['header.frame_id'] == 'synthetic_base' for row in rows)
        assert metadata['live_health_events']
        print('PASS: required publisher without samples rejected; healthy start accepted; '
              'stalled data warned without stopping; bag+CSV finalized as incomplete; '
              'source-stamped rows preserved without stale/zero fill.')
        print('Artifacts:', directory)
    finally:
        if recorder is not None and recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=15)
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
