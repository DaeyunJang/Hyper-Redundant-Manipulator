"""Recording contract and lifecycle tests without ROS or physical hardware."""

import copy
from datetime import datetime
import json
from pathlib import Path
import signal
from unittest.mock import Mock

import pytest
import yaml

from record_pkg.capture import (
    CaptureSession, image_archive_settings, qos_overrides, selected_topics,
)


@pytest.fixture
def config():
    return json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())


@pytest.fixture
def session(tmp_path, config, monkeypatch):
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr('record_pkg.capture.subprocess.Popen', Mock(return_value=process))
    monkeypatch.setattr('record_pkg.capture.shutil.which', lambda name: '/opt/ros/humble/bin/ros2')
    monkeypatch.setattr('record_pkg.capture.shutil.disk_usage', lambda path: Mock(free=100 * 1024**3))
    return CaptureSession(config, tmp_path)


def start(session):
    session.start({'published_topics': session.topics})


def finalize(session, counts=None):
    bag = session.directory / 'bag'
    bag.mkdir()
    counts = counts if counts is not None else {t: 5 for t in session.topics}
    data = {'rosbag2_bagfile_information': {'topics_with_message_count': [
        {'topic_metadata': {'name': name}, 'message_count': count}
        for name, count in counts.items()]}}
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(data))
    session.process.poll.return_value = 0
    session.poll()


def test_current_topics_preserve_numeric_signals_but_not_images(config):
    topics = selected_topics(config)
    assert '/fts_data' in topics and '/fts_data_kalman_filter' in topics
    assert '/estimated_segment_angle' in topics
    assert '/camera/camera/aligned_depth_to_color/image_raw' not in topics
    assert '/estimated_segment_crop_image' not in topics
    assert '/camera/camera/color/camera_info' in topics
    assert '/motor_command' in topics and '/tf_static' in topics
    assert '/estimated_segment_angle/relative' not in topics
    assert '/estimated_segment_angle/absolute' not in topics


def test_selection_deduplicates_and_validates(config):
    config['extra_topics'] = ['/fts_data']
    assert selected_topics(config).count('/fts_data') == 1
    config['required_topics'] += ['/not_recorded']
    with pytest.raises(ValueError, match='required'):
        selected_topics(config)


def test_camera_qos_and_latched_tf(config):
    qos = qos_overrides(selected_topics(config))
    assert qos['/tf_static']['durability'] == 'transient_local'
    assert qos['/camera/camera/color/camera_info']['reliability'] == 'best_effort'
    assert '/fts_data' not in qos  # Let rosbag adapt to its actual publisher.


def test_missing_required_publishers_refuses_without_creating_session(session):
    with pytest.raises(RuntimeError, match='Missing required'):
        session.start({'published_topics': []})
    assert not session.active and session.directory is None


def test_incomplete_is_explicit(session):
    session.start({'published_topics': []}, allow_incomplete=True)
    assert session.metadata['missing_required_at_start']


def test_no_shell_no_motor_publisher_and_no_replay(session):
    start(session)
    command = session.metadata['command']
    assert command[1:3] == ['bag', 'record']
    assert 'play' not in command and 'pub' not in command
    from record_pkg.capture import subprocess
    kwargs = subprocess.Popen.call_args.kwargs
    assert kwargs['start_new_session'] is True
    assert kwargs['stdin'] == subprocess.DEVNULL
    assert not kwargs.get('shell', False)


def test_start_twice_rejected_without_overwrite(session):
    start(session)
    original = session.directory
    with pytest.raises(RuntimeError, match='already'):
        start(session)
    assert session.directory == original


def test_session_name_local_timezone_and_utc_agree(session):
    start(session)
    stored = json.loads((session.directory / 'session.json').read_text())
    local = datetime.fromisoformat(stored['start_requested_local'])
    utc = datetime.fromisoformat(stored['start_requested_utc'])
    assert local.tzinfo is not None and local == utc
    assert local.strftime('%Y%m%d_%H%M%S_%f') == session.directory.name == stored['session_id']
    assert stored['schema_version'] == 2


def test_stop_once_waits_for_flush_then_records_counts(session):
    start(session)
    process = session.process
    session.poll(ready=True)
    assert session.state == 'recording'
    session.request_stop()
    session.request_stop()
    process.send_signal.assert_called_once_with(signal.SIGINT)
    assert session.state == 'stopping' and session.active
    finalize(session)
    assert session.state == 'stopped' and not session.active
    assert session.metadata['message_counts']['/estimated_segment_angle'] == 5
    assert session.metadata['required_topics_without_messages'] == []


def test_unexpected_exit_is_failed_even_if_exit_zero(session):
    start(session)
    finalize(session)
    assert session.state == 'failed'


def test_clean_exit_without_metadata_is_failed(session):
    start(session)
    session.request_stop()
    session.process.poll.return_value = 0
    session.poll()
    assert session.state == 'failed'


def test_empty_required_topic_reported(session):
    start(session)
    session.request_stop()
    finalize(session, counts={})
    assert 'INCOMPLETE' in session.detail
    assert session.metadata['required_topics_without_messages']


def test_new_session_never_overwrites_previous(session):
    start(session)
    previous = session.directory
    previous_data = copy.deepcopy(session.metadata)
    session.request_stop()
    finalize(session)
    session.start({'published_topics': session.topics})
    assert session.directory != previous
    assert json.loads((previous / 'session.json').read_text())['start_requested_utc'] == previous_data['start_requested_utc']


def test_low_disk_stops_own_recorder(session, monkeypatch):
    start(session)
    monkeypatch.setattr('record_pkg.capture.shutil.disk_usage', lambda p: Mock(free=0))
    session.poll()
    assert session.state == 'stopping'
    assert session.metadata['automatic_stop_reason'] == 'low_disk_space'


def test_popen_failure_keeps_failed_metadata(session, monkeypatch):
    monkeypatch.setattr('record_pkg.capture.subprocess.Popen', Mock(side_effect=OSError('test failure')))
    with pytest.raises(OSError, match='test failure'):
        start(session)
    assert session.state == 'failed' and not session.active
    assert json.loads((session.directory / 'session.json').read_text())['state'] == 'failed'


def test_metadata_write_failure_does_not_prevent_stop_signal(session, monkeypatch):
    start(session)
    monkeypatch.setattr('record_pkg.capture.save_json', Mock(side_effect=OSError('disk full')))
    session.request_stop()
    session.process.send_signal.assert_called_once_with(signal.SIGINT)
    assert session.state == 'stopping'
    assert 'Metadata write failed' in session.detail


def test_image_archive_legacy_opt_out_and_default_profile(config):
    assert image_archive_settings({})['enabled'] is False
    assert image_archive_settings(config) == dict(
        enabled=True, topic='/estimated_segment_crop_image', queue_size=16, required=True)


@pytest.mark.parametrize('settings', [None, [], {'enabled': 1}, {'required': 'true'},
                                    {'topic': 'relative'}, {'queue_size': 0},
                                    {'queue_size': True}, {'queue_size': 129}])
def test_invalid_image_archive_settings_rejected(settings):
    with pytest.raises(ValueError):
        image_archive_settings({'image_archive': settings})
