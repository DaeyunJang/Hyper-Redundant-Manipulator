"""Crop archive lifecycle contracts; no actual recorder process or hardware."""

import importlib
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
Image = pytest.importorskip('sensor_msgs.msg').Image
SetBool = pytest.importorskip('std_srvs.srv').SetBool
pytest.importorskip('custom_interfaces.msg')
record_node = importlib.import_module('record_pkg.record_node')
capture = importlib.import_module('record_pkg.capture')

CROP = '/estimated_segment_crop_image'


class FakeArchive:
    """Controllable non-blocking worker, with no filesystem/thread operations."""

    def __init__(self, directory, topic=CROP, contact_segment_id=0, queue_size=16):
        self.directory = directory
        self.topic = topic
        self.contact_segment_id = contact_segment_id
        self.queue_size = queue_size
        self.done = False
        self.enqueued = []
        self.stop_requests = 0
        self.errors = 0

    def enqueue(self, message, received_ns):
        self.enqueued.append((message, received_ns))
        return True

    def request_stop(self):
        self.stop_requests += 1

    def report(self):
        return dict(topic=self.topic, done=self.done, received=len(self.enqueued),
                    accepted=len(self.enqueued), saved=len(self.enqueued), dropped=0,
                    errors=self.errors, finalization_errors=0, pending=0,
                    error_details=['synthetic PNG write failure'] if self.errors else [])

    def close(self, timeout=5):
        self.request_stop()
        self.done = True
        return True


@pytest.fixture
def node(request, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '183')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    config = json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())
    config.update(topic_groups={'test': ['/fts_data']}, enabled_groups=['test'],
                  extra_topics=[], required_topics=[], metadata_nodes=[], min_free_disk_mb=1,
                  depth_archive={'enabled': False})
    if getattr(request, 'param', True):
        config['image_archive'] = dict(enabled=True, topic=CROP, queue_size=16, required=True)
    else:
        config.pop('image_archive', None)
    path = tmp_path / 'recording.json'
    path.write_text(json.dumps(config))
    monkeypatch.setattr(record_node.subprocess, 'run', Mock(return_value=Mock(stdout='test')))
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(capture.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(capture.shutil, 'which', lambda name: '/opt/ros/humble/bin/ros2')
    monkeypatch.setattr(importlib.import_module('record_pkg.image_archive'),
                        'ImageArchive', FakeArchive)
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


def image(sequence=1):
    message = Image(width=2, height=2, encoding='rgb8', step=6, data=[1, 2, 3] * 4)
    message.header.stamp.sec = sequence
    message.header.frame_id = 'camera_color_optical_frame'
    return message


def call_record(node, value):
    return node.record_callback(SetBool.Request(data=value), SetBool.Response())


def test_enabled_crop_subscription_exists_without_recording_or_disk_creation(node):
    assert CROP in {subscription.topic_name for subscription in node.subscriptions}
    assert node.image_archive is None
    assert not node.session.active
    assert node.session.directory is None
    node.receive_archive_image(image())
    assert node.image_archive is None
    assert node.session.directory is None
    assert CROP not in node.health.stale([CROP])


def test_required_crop_is_checked_live_but_not_selected_for_rosbag(node):
    assert CROP in node.required_live_topics()
    assert CROP not in node.session.topics
    assert CROP not in node.heavy_topics
    assert CROP in node.health.stale(node.required_live_topics())
    node.receive_archive_image(image())
    assert CROP not in node.health.stale(node.required_live_topics())


@pytest.mark.parametrize('state,expected', [
    ('idle', False), ('starting', True), ('recording', True),
    ('stopping', False), ('exporting', False), ('stopped', False), ('failed', False),
])
def test_callback_only_enqueues_in_capture_states(node, state, expected):
    archive = node.image_archive = FakeArchive('/unused')
    node.session.state = state
    message = image(3)
    node.session.process = Mock()
    try:
        node.receive_archive_image(message)
    finally:
        node.session.process = None
    assert bool(archive.enqueued) is expected
    if expected:
        assert archive.enqueued[0][0] is message
        assert archive.enqueued[0][1] > 0
    assert CROP not in node.health.stale([CROP])


def test_repeated_source_stamp_does_not_hide_a_stalled_crop_source(node):
    node.receive_archive_image(image(4))
    node.health.last_progress[CROP] = 0
    node.receive_archive_image(image(4))
    assert CROP in node.health.stale([CROP])
    node.receive_archive_image(image(5))
    assert CROP not in node.health.stale([CROP])


def test_start_rejects_crop_without_progressing_samples(node):
    response = call_record(node, True)
    assert not response.success
    assert CROP in response.message
    assert node.image_archive is None
    assert node.session.directory is None


def test_explicit_start_creates_archive_with_frozen_contact_label(node):
    assert node.set_parameters_atomically([Parameter('contact_segment_id', value=9)]).successful
    node.receive_archive_image(image(1))
    response = call_record(node, True)
    assert response.success, response.message
    archive = node.image_archive
    assert archive.contact_segment_id == 9
    assert archive.topic == CROP
    assert archive.queue_size == 16
    assert archive.directory == node.session.directory
    assert node.session.metadata['snapshot']['contact_segment_id'] == 9
    assert not archive.enqueued, 'Pre-Record crop must not be copied into the new session.'
    node.receive_archive_image(image(2))
    assert len(archive.enqueued) == 1
    assert not node.set_parameters_atomically([
        Parameter('contact_segment_id', value=0)]).successful
    assert call_record(node, False).success
    node.receive_archive_image(image(3))
    assert len(archive.enqueued) == 1
    assert archive.stop_requests >= 1


def test_archive_construction_error_stops_owned_capture_and_reports_failure(node, monkeypatch):
    node.receive_archive_image(image())
    monkeypatch.setattr(importlib.import_module('record_pkg.image_archive'),
                        'ImageArchive', Mock(side_effect=OSError('synthetic disk denied')))
    response = call_record(node, True)
    assert not response.success
    assert 'synthetic disk denied' in response.message
    assert node.session.state == 'stopping'
    assert node.session.stop_started is not None
    assert node.session.metadata['image_archive']['status'] == 'failed'
    assert node.image_archive is None


def test_drain_locks_label_alignment_and_new_record_even_with_closed_bag(node):
    node.image_archive = FakeArchive('/unused')
    node.session.state = 'stopped'
    assert node.image_archive_busy()
    assert node.force_alignment_locked()
    result = node.set_parameters_atomically([Parameter('contact_segment_id', value=9)])
    assert not result.successful
    response = call_record(node, True)
    assert not response.success
    assert node.session.directory is None


def test_stop_does_not_wait_for_worker_and_new_images_are_not_queued(node):
    archive = node.image_archive = FakeArchive('/unused')
    node.session.state = 'recording'
    node.session.process = Mock()
    try:
        node.receive_archive_image(image(1))
        response = call_record(node, False)
    finally:
        node.session.process = None
    assert response.success
    assert archive.stop_requests >= 1
    assert not archive.done
    # Bag ownership is absent in this fixture, so model its post-stop state.
    node.session.state = 'stopping'
    node.receive_archive_image(image(2))
    assert len(archive.enqueued) == 1


@pytest.mark.parametrize('node', [False], indirect=True)
def test_legacy_profile_without_archive_creates_no_crop_subscriber(node):
    assert CROP not in {subscription.topic_name for subscription in node.subscriptions}
    assert node.image_archive is None
    assert CROP not in node.required_live_topics()
    assert not node.image_archive_busy()


def test_done_archive_does_not_hold_experiment_settings_when_session_is_idle(node):
    archive = node.image_archive = FakeArchive('/unused')
    archive.done = True
    assert not node.image_archive_busy()
    assert node.set_parameters_atomically([Parameter('contact_segment_id', value=18)]).successful


def test_finalizer_waits_for_png_drain_and_only_starts_once(node):
    archive = node.image_archive = FakeArchive('/unused')
    node._archive_finalize_pending = True
    node._bag_closed_state = 'stopped'
    node.start_finalizer = Mock()
    node.poll_archive_finalization()
    assert node.session.state == 'stopping'
    assert not node.start_finalizer.called
    assert node.session.metadata['image_archive']['done'] is False
    archive.done = True
    node.poll_archive_finalization()
    assert node.session.state == 'stopped'
    assert node.session.metadata['image_archive']['done'] is True
    node.start_finalizer.assert_called_once()
    node.poll_archive_finalization()
    node.start_finalizer.assert_called_once()


def test_failed_bag_is_not_promoted_to_success_when_png_drain_finishes(node):
    archive = node.image_archive = FakeArchive('/unused')
    archive.done = True
    node._archive_finalize_pending = True
    node._bag_closed_state = 'failed'
    node.start_finalizer = Mock()
    node.poll_archive_finalization()
    assert node.session.state == 'failed'
    assert not node.start_finalizer.called


def test_writer_error_is_visible_in_status_without_claiming_complete(node):
    archive = node.image_archive = FakeArchive('/unused')
    archive.errors = 1
    node.session.state = 'recording'
    node.poll_archive_finalization()
    node.status_pub = Mock()
    node.publish_status()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['image_archive']['errors'] == 1
    assert 'IMAGE WARNING' in status['detail']
    assert status['data_quality'] != 'complete'


@pytest.mark.parametrize('image_status,report_status,exit_code,state', [
    ('complete', 'complete', 0, 'stopped'),
    ('incomplete', 'complete', 0, 'stopped'),
    ('error', 'failed', 2, 'failed'),
])
def test_final_data_quality_includes_png_integrity(
        node, tmp_path, image_status, report_status, exit_code, state):
    node.session.directory = tmp_path / 'closed_session'
    node.session.directory.mkdir()
    report = dict(status=report_status, integrity={'status': 'complete'},
                  image_integrity={'status': image_status})
    (node.session.directory / 'postprocess.json').write_text(json.dumps(report))
    node.finalizer = Mock(returncode=exit_code)
    node.finalizer.poll.return_value = exit_code
    node.finalizer_log = Mock()
    node.poll_finalizer()
    assert node.session.state == state
    expected = 'complete' if image_status == 'complete' else 'incomplete'
    assert node.session.metadata['data_quality'] == expected
