"""Recorder crop-depth lifecycle; mocked workers/processes, no device I/O."""

import importlib
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
Image = pytest.importorskip('sensor_msgs.msg').Image
String = pytest.importorskip('std_msgs.msg').String
pytest.importorskip('custom_interfaces.msg')

from test_record_image_archive import FakeArchive, call_record  # noqa: E402

record_node = importlib.import_module('record_pkg.record_node')
capture = importlib.import_module('record_pkg.capture')
DEPTH = '/estimated_segment_crop_depth/image_raw'
CALIBRATION = '/estimated_segment_crop_depth/metadata'


class FakeDepthArchive(FakeArchive):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.metadata = []
        self.calibration_errors = 0

    def update_metadata(self, message):
        self.metadata.append(message)

    def report(self):
        return dict(super().report(), calibration_errors=self.calibration_errors)


@pytest.fixture
def node(request, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    config = json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())
    config.update(topic_groups={'test': ['/fts_data']}, enabled_groups=['test'],
                  extra_topics=[], required_topics=[], metadata_nodes=[], min_free_disk_mb=1,
                  image_archive=dict(enabled=True, required=False,
                                     topic='/estimated_segment_crop_image', queue_size=16),
                  depth_archive=dict(enabled=True, required=getattr(request, 'param', False),
                                     topic=DEPTH, metadata_topic=CALIBRATION, queue_size=16))
    path = tmp_path / 'recording.json'
    path.write_text(json.dumps(config))
    monkeypatch.setattr(record_node.subprocess, 'run', Mock(return_value=Mock(stdout='test')))
    process = Mock()
    process.poll.return_value = None
    monkeypatch.setattr(capture.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(capture.shutil, 'which', lambda name: '/opt/ros/humble/bin/ros2')
    monkeypatch.setattr(importlib.import_module('record_pkg.image_archive'),
                        'ImageArchive', FakeArchive)
    monkeypatch.setattr(importlib.import_module('record_pkg.depth_archive'),
                        'DepthArchive', FakeDepthArchive)
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


def depth(sequence=1):
    message = Image(width=2, height=2, encoding='16UC1', step=4, data=[1, 0] * 4)
    message.header.stamp.sec = sequence
    message.header.frame_id = 'camera_color_optical_frame'
    return message


@pytest.mark.parametrize('node', [True], indirect=True)
def test_images_off_needs_no_crop_and_preserves_numeric_session(node):
    from record_pkg.image_archive_audit import audit_image_archive
    from record_pkg.depth_archive_audit import audit_depth_archive
    topics = list(node.session.topics)
    assert node.set_parameters_atomically([Parameter('save_images', value=False)]).successful
    assert call_record(node, True).success
    assert node.session.topics == topics
    assert node.image_archive is None and node.depth_archive is None
    assert DEPTH not in node.required_live_topics()
    assert '/estimated_segment_crop_image' not in node.required_live_topics()
    node.receive_archive_depth(depth())
    directory = node.session.directory
    assert not (directory / 'images').exists()
    config = json.loads((directory / 'recording_config.json').read_text())
    assert config['save_images'] is False
    assert not config['image_archive']['enabled'] and not config['depth_archive']['enabled']
    assert node.session.metadata['snapshot']['save_images'] is False
    assert audit_image_archive(directory, config, node.session.metadata)['status'] == 'disabled'
    assert audit_depth_archive(directory, config, node.session.metadata)['status'] == 'disabled'
    assert not node.set_parameters_atomically([Parameter('save_images', value=True)]).successful


def test_image_selection_restores_original_profile_without_changing_numeric_topics(node):
    topics = list(node.session.topics)
    for enabled in (False, True, False, True):
        assert node.set_parameters_atomically([Parameter('save_images', value=enabled)]).successful
        node.apply_image_recording_settings()
        assert node.image_archive_settings['enabled'] is enabled
        assert node.depth_archive_settings['enabled'] is enabled
        assert node.session.topics == topics
        assert node.snapshot()['save_images'] is enabled


@pytest.mark.parametrize('state', ['starting', 'recording', 'stopping', 'exporting'])
def test_image_selection_locked_through_export(node, state):
    node.session.state = state
    try:
        assert not node.set_parameters_atomically([Parameter('save_images', value=False)]).successful
        assert node.get_parameter('save_images').value is True
    finally:
        node.session.state = 'idle'


def test_image_selection_type_validation_is_atomic(node):
    result = node.set_parameters_atomically([
        Parameter('contact_segment_id', value=9), Parameter('save_images', value=0)])
    assert not result.successful
    assert node.get_parameter('save_images').value is True
    assert node.get_parameter('contact_segment_id').value == 0


def test_depth_subscribers_do_not_create_archive_or_pause_service_at_startup(node):
    names = {subscription.topic_name for subscription in node.subscriptions}
    assert {DEPTH, CALIBRATION}.issubset(names)
    assert node.depth_archive is None and node.image_archive is None
    assert node.session.directory is None and not node.session.active
    services = {service.srv_name for service in node.services}
    assert '/data/record' in services
    assert not any(name.endswith(('/pause', '/resume', '/is_paused')) for name in services)
    assert 'recording_intervals' not in node.snapshot()


@pytest.mark.parametrize('node', [True], indirect=True)
def test_required_depth_needs_both_image_and_metadata_progress(node):
    assert {DEPTH, CALIBRATION}.issubset(node.required_live_topics())
    response = call_record(node, True)
    assert not response.success and DEPTH in response.message
    node.receive_archive_depth(depth())
    response = call_record(node, True)
    assert not response.success and CALIBRATION in response.message
    node.receive_depth_metadata(String(data='{"source_time_ns":1000000000}'))
    response = call_record(node, True)
    assert response.success, response.message


def test_optional_missing_depth_does_not_prevent_record_and_is_visible(node):
    assert DEPTH not in node.required_live_topics()
    assert CALIBRATION not in node.required_live_topics()
    response = call_record(node, True)
    assert response.success, response.message
    node.status_pub = Mock()
    node.publish_status()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert 'no crop frames saved yet' in status['detail']
    assert status['state'] == 'starting'


def test_pre_record_metadata_prefills_worker_but_pre_record_images_are_not_replayed(node):
    assert node.set_parameters_atomically([Parameter('contact_segment_id', value=9)]).successful
    calibration = String(data='{"source_time_ns":1000000000}')
    node.receive_depth_metadata(calibration)
    node.receive_archive_depth(depth())
    assert node.depth_archive is None
    assert node.session.directory is None
    response = call_record(node, True)
    assert response.success, response.message
    archive = node.depth_archive
    assert archive.metadata == [calibration]
    assert archive.enqueued == []
    assert archive.contact_segment_id == 9
    assert archive.topic == DEPTH
    assert 'recording_intervals' not in node.session.metadata


def test_metadata_prefill_cache_is_bounded_without_any_image_cache(node):
    for index in range(80):
        node.receive_depth_metadata(String(data=str(index)))
        node.receive_archive_depth(depth(index + 1))
    assert len(node.depth_metadata_cache) == 64
    assert node.depth_metadata_cache[0].data == '16'
    assert node.depth_archive is None
    assert call_record(node, True).success
    assert len(node.depth_archive.metadata) == 64
    assert node.depth_archive.enqueued == []


def test_depth_callback_only_enqueues_owned_starting_and_recording_states(node):
    archive = node.depth_archive = FakeDepthArchive('/unused')
    node.session.process = Mock()
    try:
        for sequence, state in enumerate(
                ('idle', 'starting', 'recording', 'stopping', 'exporting', 'stopped', 'failed'), 1):
            node.session.state = state
            before = len(archive.enqueued)
            node.receive_archive_depth(depth(sequence))
            assert len(archive.enqueued) - before == int(state in ('starting', 'recording'))
        node.session.state = 'recording'
        node.session.stop_started = 1.0
        node.receive_archive_depth(depth(99))
        assert len(archive.enqueued) == 2
    finally:
        node.session.process = None
    assert DEPTH not in node.health.stale([DEPTH])


def test_stop_closes_both_archives_and_rejects_post_stop_depth(node):
    assert call_record(node, True).success
    node.receive_archive_depth(depth(1))
    assert len(node.depth_archive.enqueued) == 1
    assert call_record(node, False).success
    assert node.image_archive.stop_requests >= 1
    assert node.depth_archive.stop_requests >= 1
    assert not node.depth_archive.done  # Callback never waits for worker I/O.
    node.receive_archive_depth(depth(2))
    assert len(node.depth_archive.enqueued) == 1


def test_csv_and_setting_unlock_wait_for_depth_even_after_color_finishes(node):
    node.image_archive = FakeArchive('/unused')
    node.image_archive.done = True
    node.depth_archive = FakeDepthArchive('/unused')
    node._archive_finalize_pending = True
    node._bag_closed_state = 'stopped'
    node.start_finalizer = Mock(side_effect=lambda: setattr(node.session, 'state', 'exporting'))
    node.poll_archive_finalization()
    assert node.session.state == 'stopping'
    assert node.image_archive_busy()
    assert not node.start_finalizer.called
    assert not node.set_parameters_atomically([Parameter('contact_segment_id', value=8)]).successful
    assert not call_record(node, True).success
    node.depth_archive.done = True
    node.poll_archive_finalization()
    node.start_finalizer.assert_called_once()
    assert not node.image_archive_busy()
    assert not node.set_parameters_atomically([Parameter('contact_segment_id', value=8)]).successful
    node.session.state = 'stopped'  # Model completed CSV worker, not merely PNG drain.
    assert node.set_parameters_atomically([Parameter('contact_segment_id', value=8)]).successful


def test_depth_constructor_failure_stops_color_and_owned_numeric_bag(node, monkeypatch):
    monkeypatch.setattr(importlib.import_module('record_pkg.depth_archive'),
                        'DepthArchive', Mock(side_effect=OSError('depth archive disk denied')))
    response = call_record(node, True)
    assert not response.success and 'depth archive disk denied' in response.message
    assert node.session.state == 'stopping'
    assert node.session.stop_started is not None
    assert node.image_archive.stop_requests >= 1
    assert node.depth_archive is None
    assert node.session.metadata['depth_archive']['status'] == 'failed'


def test_calibration_errors_warn_and_prevent_complete_data_quality(node, tmp_path):
    archive = node.depth_archive = FakeDepthArchive('/unused')
    archive.calibration_errors = 1
    node.session.state = 'recording'
    node.poll_archive_finalization()
    node.status_pub = Mock()
    node.publish_status()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['depth_archive']['calibration_errors'] == 1
    assert 'DEPTH WARNING' in status['detail']
    assert status['data_quality'] != 'complete'
    archive.done = True
    node.session.directory = tmp_path / 'completed'
    node.session.directory.mkdir()
    (node.session.directory / 'postprocess.json').write_text(json.dumps({
        'status': 'complete', 'integrity': {'status': 'complete'},
        'image_integrity': {'status': 'complete'},
        'depth_integrity': {'status': 'incomplete', 'writer': {'calibration_errors': 1}}}))
    node.finalizer = Mock(returncode=0)
    node.finalizer.poll.return_value = 0
    node.finalizer_log = Mock()
    node.poll_finalizer()
    assert node.session.state == 'stopped'
    assert node.session.metadata['data_quality'] == 'incomplete'
