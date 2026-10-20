#!/usr/bin/env python3
"""Synthetic GUI settings → recorder → bag → aligned CSV, no physical hardware.

Requires built record_pkg/gui_py_pkg and a sourced ROS environment. Uses isolated
localhost domain 87 and refuses an occupied domain. Artifacts remain under /tmp.
"""

import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


def main():
    artifacts = Path(tempfile.mkdtemp(prefix='hrm_force_alignment_', dir='/tmp'))
    os.environ['ROS_LOG_DIR'] = str(artifacts / 'ros_log')
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from gui_py_pkg.force_alignment import request_record_start
    from record_pkg.force_alignment import create_alignment

    config = json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())
    topics = ['/fts_data', '/fts_data_kalman_filter']
    config.update(topic_groups={'test': topics}, enabled_groups=['test'], extra_topics=[],
                  image_archive={'enabled': False}, depth_archive={'enabled': False},
                  required_topics=topics, metadata_nodes=['/record'], metadata_timeout_sec=1.0,
                  auto_export_csv=True, min_free_disk_mb=64, max_cache_size_mb=4,
                  max_bag_size_mb=16)
    config_path = artifacts / 'recording.json'
    config_path.write_text(json.dumps(config))
    rclpy.init()
    node = Node('force_alignment_roundtrip')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    recorder = None
    log = None
    statuses = []

    def wait_for(predicate, description, timeout=12):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        assert predicate(), f'{description}; status={statuses[-1:]}; artifacts={artifacts}'

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)

    try:
        spin_for(0.5)
        assert set(node.get_node_names()) <= {'force_alignment_roundtrip'}, 'Domain 87 occupied.'
        publishers = [node.create_publisher(WrenchStamped, topic, 10) for topic in topics]

        def publish():
            for index, publisher in enumerate(publishers):
                msg = WrenchStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'test_sensor'
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (
                    [1.0, 2.0, 3.0] if index == 0 else [4.0, 5.0, 6.0])
                msg.wrench.torque.x = 7.0
                publisher.publish(msg)

        node.create_timer(1 / 30, publish)
        node.create_subscription(String, '/data/record_status',
                                 lambda msg: statuses.append(json.loads(msg.data)),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        params = node.create_client(SetParametersAtomically, '/record/set_parameters_atomically')
        record = node.create_client(SetBool, '/data/record')
        log = (artifacts / 'recorder.log').open('w')
        recorder = subprocess.Popen([
            'ros2', 'run', 'record_pkg', 'record', '--ros-args',
            '-p', f'config_file:={config_path}', '-p', f'output_root:={artifacts / "sessions"}',
        ], stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait_for(lambda: params.service_is_ready() and record.service_is_ready(),
                 'Services unavailable')
        spin_for(1.0)

        def set_axes(axes):
            future = params.call_async(SetParametersAtomically.Request(parameters=[
                Parameter('force_alignment_axes', value=axes).to_parameter_msg()]))
            wait_for(future.done, 'Parameter reply missing')
            return future.result().result

        # Two consecutive sessions deliberately use different mappings.
        directories = []
        for axes in (['+x', '-y', '-z'], ['+x', '-z', '+y']):
            alignment = create_alignment(True, axes)
            response = request_record_start(params, record, alignment)
            assert response is not None
            wait_for(response.done, 'GUI startup chain failed')
            assert response.result().success, response.result().message
            wait_for(lambda: statuses and statuses[-1]['state'] == 'recording',
                     'Recording not ready')
            directory = Path(statuses[-1]['directory'])
            assert directory not in directories
            directories.append(directory)
            assert statuses[-1]['force_alignment']['axes'] == axes
            assert not set_axes(['+x', '+y', '+z']).successful, 'Active alignment changed!'
            spin_for(1.0)
            stopped = record.call_async(SetBool.Request(data=False))
            wait_for(stopped.done, 'Stop reply missing')
            assert stopped.result().success
            wait_for(lambda: statuses[-1]['state'] in ('stopped', 'failed'), 'Export failed', 25)
            assert statuses[-1]['state'] == 'stopped', statuses[-1]
            session = json.loads((directory / 'session.json').read_text())
            assert session['snapshot']['force_alignment'] == alignment
            manifest = json.loads((directory / 'csv/manifest.json').read_text())
            assert manifest['status'] == 'complete'
            for index, topic in enumerate(topics):
                original = [1., 2., 3.] if index == 0 else [4., 5., 6.]
                expected = [original['xyz'.index(a[1])] * (1 if a[0] == '+' else -1)
                            for a in axes]
                with (directory / 'csv' / manifest['topics'][topic]['file']).open() as handle:
                    rows = list(csv.DictReader(handle))
                assert rows
                for row in rows:
                    assert [float(row[f'wrench.force.{a}']) for a in 'xyz'] == original
                    assert [float(row[f'aligned_f{a}']) for a in 'xyz'] == expected
                    assert row['header.frame_id'] == 'test_sensor'
                    assert row['aligned_frame_id'] == 'hrm_base'
                    assert float(row['wrench.torque.x']) == 7.0
                    assert row['aligned_force_valid'] == 'True'
                    assert int(row['source_time_ns']) > 0
            print('PASS session', axes, directory)
        assert not set_axes(['+x', '+z', '+y']).successful, 'Reflection was accepted!'
        assert set_axes(['+x', '+y', '+z']).successful
        # Later settings never change the old session snapshot or its CSV.
        first = json.loads((directories[0] / 'session.json').read_text())
        assert first['snapshot']['force_alignment']['axes'] == ['+x', '-y', '-z']
        print('PASS: GUI ACK chain, per-session freeze, two mappings, raw/Kalman preservation.')
        print('Artifacts:', artifacts)
    finally:
        if recorder is not None and recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
            try:
                recorder.wait(timeout=12)
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
