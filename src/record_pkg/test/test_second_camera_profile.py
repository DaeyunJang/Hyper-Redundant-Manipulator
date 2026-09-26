"""Lightweight crop-only image profile, without ROS nodes or hardware."""

import json
from pathlib import Path
import sqlite3
from unittest.mock import Mock

import pytest
import yaml

from record_pkg.bag_integrity import audit_bag
from record_pkg.capture import CaptureSession, qos_overrides, selected_topics


TAG_IMAGE = '/tag_camera/tag_camera/color/image_raw'
TAG_INFO = '/tag_camera/tag_camera/color/camera_info'
TAG_TOPICS = [
    TAG_INFO, '/detections', '/apriltag/tag0/pose', '/apriltag/tag1/pose',
    '/apriltag/tag1_in_tag0/pose',
]
REQUIRED_TOPICS = [
    '/camera/camera/color/camera_info', '/motor_state', '/loadcell_state',
    '/fts_data', '/fts_data_kalman_filter', '/wire_length',
    '/estimated_segment_angle', '/tool_endeffector_pose',
]


@pytest.fixture
def config():
    path = Path(__file__).parents[1] / 'config/recording.json'
    return json.loads(path.read_text())


def test_camera_calibration_and_tag_poses_retained_without_image_bag_payload(config):
    topics = selected_topics(config)
    assert config['topic_groups']['camera'] == [
        '/camera/camera/color/camera_info',
        '/camera/camera/aligned_depth_to_color/camera_info',
        '/estimated_segment_crop_depth/camera_info',
        '/estimated_segment_crop_depth/metadata',
    ]
    assert 'apriltag' in config['enabled_groups']
    assert config['required_topics'] == REQUIRED_TOPICS
    for topic in TAG_TOPICS:
        assert topic in config['topic_groups']['apriltag']
        assert topic not in config['required_topics']
        assert topics.count(topic) == 1
    assert '/camera/camera' in config['metadata_nodes']
    assert config['metadata_nodes'].count('/tag_camera/tag_camera') == 1
    assert not any('/tag_camera/' in topic and 'depth' in topic for topic in topics)
    assert TAG_IMAGE not in topics
    assert TAG_IMAGE not in config['required_topics']
    assert not any('image' in topic for topic in topics)
    assert not any('image' in topic for topic in config['required_topics'])


def test_only_crop_png_archive_is_enabled_by_default(config):
    assert config['image_archive'] == {
        'enabled': True, 'topic': '/estimated_segment_crop_image',
        'queue_size': 16, 'required': True,
    }
    topics = selected_topics(config)
    assert config['image_archive']['topic'] not in topics
    assert 'visualization' not in config['enabled_groups']
    assert set(config['topic_groups']['visualization']).isdisjoint(topics)
    assert config['auto_export_csv']
    assert config['summary_csv']['enabled']


def test_second_camera_uses_nonblocking_sensor_qos(config):
    overrides = qos_overrides(selected_topics(config))
    for topic in ('/camera/camera/color/camera_info', TAG_INFO):
        assert overrides[topic]['reliability'] == 'best_effort'
        assert overrides[topic]['durability'] == 'volatile'
        assert overrides[topic]['history'] == 'keep_last'


@pytest.mark.parametrize('tags_available', [False, True])
def test_start_allows_optional_tags_and_keeps_them_in_recorder_command(
        config, tmp_path, monkeypatch, tags_available):
    process = Mock()
    process.poll.return_value = 0
    popen = Mock(return_value=process)
    monkeypatch.setattr('record_pkg.capture.subprocess.Popen', popen)
    monkeypatch.setattr('record_pkg.capture.shutil.which', lambda _: '/mock/ros2')
    monkeypatch.setattr('record_pkg.capture.shutil.disk_usage',
                        lambda _: Mock(free=10**12))
    session = CaptureSession(config, tmp_path / 'capture')
    available = [topic for topic in session.topics
                 if tags_available or topic not in TAG_TOPICS]
    try:
        session.start({'published_topics': available})
        popen.assert_called_once()
        assert session.state == 'starting'
        assert session.metadata['missing_required_at_start'] == []
        assert session.metadata['missing_selected_at_start'] == (
            [] if tags_available else sorted(TAG_TOPICS))
        command = popen.call_args.args[0]
        assert all(command.count(topic) == 1 for topic in TAG_TOPICS)
    finally:
        session.close()  # The process is fully mocked; no ROS node is started.


@pytest.mark.parametrize('missing', REQUIRED_TOPICS)
def test_start_still_rejects_missing_required_data_before_any_process(
        config, tmp_path, monkeypatch, missing):
    popen = Mock(side_effect=AssertionError('No recorder process may be started.'))
    monkeypatch.setattr('record_pkg.capture.subprocess.Popen', popen)
    session = CaptureSession(config, tmp_path / 'capture')
    available = [topic for topic in session.topics if topic != missing]
    with pytest.raises(RuntimeError, match=missing):
        session.start({'published_topics': available})
    assert session.directory is None
    assert not (tmp_path / 'capture').exists()
    popen.assert_not_called()


def write_bag(tmp_path, samples):
    """Create only metadata/timestamps; no ROS or image payload is needed."""
    bag = tmp_path / 'bag'
    bag.mkdir()
    database = bag / 'bag_0.db3'
    with sqlite3.connect(str(database)) as connection:
        connection.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT)')
        connection.execute('CREATE TABLE messages '
                           '(id INTEGER PRIMARY KEY, topic_id INTEGER, '
                           'timestamp INTEGER, data BLOB)')
        for index, (topic, stamps) in enumerate(samples.items(), 1):
            connection.execute('INSERT INTO topics VALUES (?, ?)', (index, topic))
            connection.executemany(
                'INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)',
                [(index, stamp, b'payload deliberately not decoded') for stamp in stamps])
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3', 'relative_file_paths': [database.name],
            'topics_with_message_count': [
                {'topic_metadata': {'name': topic}, 'message_count': len(stamps)}
                for topic, stamps in samples.items()],
        },
    }))
    return bag


@pytest.mark.parametrize('tag_case', ['absent', 'intermittent', 'continuous'])
def test_optional_tag_absence_or_gaps_do_not_make_bag_incomplete(
        config, tmp_path, tag_case):
    stamps = [second * 1_000_000_000 for second in range(1, 6)]
    samples = {topic: stamps for topic in config['required_topics']}
    if tag_case != 'absent':
        tag_stamps = [stamps[0], stamps[-1]] if tag_case == 'intermittent' else stamps
        samples.update({topic: tag_stamps for topic in TAG_TOPICS})
    bag = write_bag(tmp_path, samples)
    report = audit_bag(bag, config['required_topics'], selected_topics=selected_topics(config))
    assert report['scan_complete']
    assert report['status'] == 'complete'
    assert report['required_topics_without_messages'] == []
    assert report['required_topics_with_receive_gaps'] == []
    for topic in TAG_TOPICS:
        assert report['topics'][topic]['required'] is False
        assert (topic in report['absent_selected_topics']) == (tag_case == 'absent')
        assert report['topics'][topic]['message_count'] == (
            0 if tag_case == 'absent' else (2 if tag_case == 'intermittent' else 5))
        if tag_case == 'intermittent':
            assert report['topics'][topic]['internal_gaps_over_limit'] == 1


@pytest.mark.parametrize('topic', ['/motor_state', '/fts_data', '/loadcell_state'])
@pytest.mark.parametrize('missing', [False, True])
def test_required_sensor_absence_or_gaps_still_make_bag_incomplete(
        config, tmp_path, topic, missing):
    stamps = [second * 1_000_000_000 for second in range(1, 6)]
    samples = {name: stamps for name in config['required_topics']}
    samples[topic] = [] if missing else [stamps[0], stamps[-1]]
    bag = write_bag(tmp_path, samples)
    report = audit_bag(bag, config['required_topics'], selected_topics=selected_topics(config))
    assert report['scan_complete']
    assert report['status'] == 'incomplete'
    assert report['required_topics_without_messages'] == ([topic] if missing else [])
    assert report['required_topics_with_receive_gaps'] == ([] if missing else [topic])


def test_tag_group_can_be_disabled_without_changing_other_requirements(config):
    tag_topics = set(config['topic_groups']['apriltag'])
    config['enabled_groups'].remove('apriltag')
    assert tag_topics.isdisjoint(selected_topics(config))
    assert config['required_topics'] == REQUIRED_TOPICS
