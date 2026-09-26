"""Recorder parameter/snapshot contract; no recorder subprocess or hardware."""

import importlib
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
pytest.importorskip('custom_interfaces.msg')
record_node = importlib.import_module('record_pkg.record_node')
capture = importlib.import_module('record_pkg.capture')
force_alignment = importlib.import_module('record_pkg.force_alignment')


@pytest.fixture
def node(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    config = json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())
    config.update(topic_groups={'test': ['/fts_data']}, enabled_groups=['test'],
                  image_archive={'enabled': False}, depth_archive={'enabled': False},
                  extra_topics=[], required_topics=[], metadata_nodes=[], min_free_disk_mb=1)
    path = tmp_path / 'recording.json'
    path.write_text(json.dumps(config))
    # Never run git, launch rosbag, or operate an external device from this test.
    monkeypatch.setattr(record_node.subprocess, 'run', Mock(return_value=Mock(stdout='test')))
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(capture.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(capture.shutil, 'which', lambda name: '/opt/ros/humble/bin/ros2')
    rclpy.init(args=['--ros-args', '-p', f'config_file:={path}',
                     '-p', f'output_root:={tmp_path / "sessions"}'])
    instance = None
    try:
        instance = record_node.RecordNode()
        yield instance
    finally:
        if instance is not None:
            process.poll.return_value = 0
            instance.close()
            instance.destroy_node()
        rclpy.shutdown()


def parameters(enabled, axes):
    return [Parameter('force_alignment_enabled', value=enabled),
            Parameter('force_alignment_axes', value=axes)]


def test_atomic_valid_update_and_invalid_update_rolls_back_both_parameters(node):
    baseline = node.current_force_alignment()
    assert baseline == force_alignment.create_alignment()
    rejected = node.set_parameters_atomically(parameters(True, ['+x', '+z', '+y']))
    assert not rejected.successful
    assert node.current_force_alignment() == baseline
    accepted = node.set_parameters_atomically(parameters(True, ['+x', '-z', '+y']))
    assert accepted.successful, accepted.reason
    assert node.current_force_alignment() == force_alignment.create_alignment(
        True, ['+x', '-z', '+y'])


@pytest.mark.parametrize('state', ['starting', 'recording', 'stopping', 'exporting'])
def test_alignment_cannot_change_in_any_busy_recording_state(node, state):
    baseline = node.current_force_alignment()
    node.session.state = state
    try:
        result = node.set_parameters_atomically(parameters(True, ['+x', '-z', '+y']))
        assert not result.successful
        assert 'frozen' in result.reason
        assert node.current_force_alignment() == baseline
    finally:
        node.session.state = 'idle'


def test_finalizer_presence_locks_settings_even_if_status_not_yet_exporting(node):
    node.finalizer = Mock()
    try:
        result = node.set_parameters_atomically(parameters(True, ['+x', '+y', '+z']))
        assert not result.successful
    finally:
        node.finalizer = None


def test_alignment_type_and_duplicate_axes_are_rejected(node):
    baseline = node.current_force_alignment()
    for candidate in ([Parameter('force_alignment_enabled', value=1)],
                      parameters(True, ['+x', '+x', '+z'])):
        result = node.set_parameters_atomically(candidate)
        assert not result.successful
        assert node.current_force_alignment() == baseline


def test_start_freezes_alignment_in_session_json_and_future_settings_are_independent(node):
    first = force_alignment.create_alignment(True, ['+x', '-z', '+y'])
    assert node.set_parameters_atomically(parameters(first['enabled'], first['axes'])).successful
    directory = node.session.start(node.snapshot(), allow_incomplete=True)
    assert json.loads((directory / 'session.json').read_text())['snapshot']['force_alignment'] == first
    node.status_pub = Mock()
    node.publish_status()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['force_alignment'] == first
    assert status['force_alignment_scope'] == 'session'
    assert not node.set_parameters_atomically(parameters(False, ['+x', '+y', '+z'])).successful
    # Model completed capture/export without creating an actual bag subprocess.
    node.session.process = None
    node.session.log_file.close()
    node.session.log_file = None
    node.session.state = 'stopped'
    second = force_alignment.create_alignment(True, ['-x', '-y', '+z'])
    assert node.set_parameters_atomically(parameters(second['enabled'], second['axes'])).successful
    assert node.snapshot()['force_alignment'] == second
    assert node.session.metadata['snapshot']['force_alignment'] == first
    assert json.loads((directory / 'session.json').read_text())['snapshot']['force_alignment'] == first
    node.publish_status()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['force_alignment'] == second
    assert status['force_alignment_scope'] == 'next_session'


def test_no_new_force_subscription_is_created(node):
    names = {subscription.topic_name for subscription in node.subscriptions}
    assert '/fts_data' not in names and '/fts_data_kalman_filter' not in names


def test_contact_label_is_validated_frozen_and_in_session_status(node):
    assert node.get_parameter('contact_segment_id').value == 0
    for bad in (-1, 19, 1.5, '9', True):
        assert not node.set_parameters_atomically([
            Parameter('contact_segment_id', value=bad)]).successful
    for value in (0, 18, 9):
        assert node.set_parameters_atomically([
            Parameter('contact_segment_id', value=value)]).successful
    directory = node.session.start(node.snapshot(), allow_incomplete=True)
    assert json.loads((directory / 'session.json').read_text())['snapshot'][
        'contact_segment_id'] == 9
    node.status_pub = Mock()
    node.publish_status()
    assert json.loads(node.status_pub.publish.call_args.args[0].data)[
        'contact_segment_id'] == 9
    assert not node.set_parameters_atomically([
        Parameter('contact_segment_id', value=0)]).successful
    assert node.get_parameter('contact_segment_id').value == 9
