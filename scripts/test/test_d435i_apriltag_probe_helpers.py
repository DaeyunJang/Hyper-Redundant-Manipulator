"""No-ROS/no-hardware checks for the bounded live camera probe."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'test_d435i_apriltag_live.py'
SPEC = importlib.util.spec_from_file_location('d435i_probe_helpers', SOURCE)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def message(stamp=1_000_000_000, frame='tag_camera_color_optical_frame'):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=frame))


def transport_stats():
    stats = {name: PROBE.TopicStats() for name in PROBE.TOPICS}
    for name in PROBE.REQUIRED_TRANSPORT:
        stats[name].observe(message(), now=10.0)
    return stats


def test_rates_distinguish_source_timing_and_callback_timing():
    stats = PROBE.TopicStats()
    stats.observe(message(1_000_000_000), now=10.0)
    stats.observe(message(1_100_000_000), now=10.2)
    report = stats.report()
    assert report['source_rate_hz'] == pytest.approx(10.0)
    assert report['receive_rate_hz'] == pytest.approx(5.0)
    assert report['max_receive_gap_ms'] == pytest.approx(200.0)


def test_empty_detections_are_transport_success_not_tag1_success():
    stats = transport_stats()
    report = PROBE.summarize(stats, [{'tags': []}])
    assert report['transport_ok']
    assert report['tag1_requirement_ok']
    assert not report['tag1_detected']
    assert report['empty_detection_arrays'] == 1
    assert not PROBE.summarize(stats, [{'tags': []}], True)['tag1_requirement_ok']


def test_missing_detection_messages_are_transport_failure():
    stats = transport_stats()
    stats['detections'] = PROBE.TopicStats()
    assert PROBE.summarize(stats, [])['missing_transport'] == ['detections']


def test_detected_tags_need_pose_pipeline_and_same_frame_relative_pose():
    stats = transport_stats()
    rows = [{'tags': [{'id': 0}, {'id': 1}]}]
    report = PROBE.summarize(stats, rows)
    assert report['missing_detected_tag_poses'] == ['tag0', 'tag1', 'relative']
    for name in ('tag0', 'tag1', 'relative'):
        stats[name].observe(message(), now=10.0)
    assert PROBE.summarize(stats, rows)['pose_pipeline_ok']


def test_intersections_report_stamp_mismatches_without_fabricating_matches():
    stats = transport_stats()
    stats['rect'].observe(message(2_000_000_000), now=11.0)
    matches = PROBE.summarize(stats, [])['image_stamp_match']['rect_in_raw']
    assert matches == {'matched': 1, 'observed': 2, 'unmatched': 1, 'fraction': 0.5}


@pytest.mark.parametrize('extra', [
    ['--domain-id', '0'], ['--domain-id', '102'], ['--duration', '31'],
    ['--duration', '0'], ['--startup-timeout', '21'], ['--serial-no', '_949122070024'],
])
def test_unsafe_or_unbounded_arguments_are_rejected(extra):
    with pytest.raises(SystemExit):
        PROBE.parse_args(['--serial-no', '949122070024', '--output-dir', '/tmp/unused'] + extra)


def test_exact_serial_is_not_coerced_and_default_domain_is_private():
    args = PROBE.parse_args(['--serial-no', '043422251095', '--output-dir', '/tmp/unused'])
    assert args.serial_no == '043422251095'
    assert args.domain_id == 96
    assert args.duration == 15


def test_parameter_values_keep_bool_and_string_types():
    assert PROBE.parameter_value(SimpleNamespace(type=1, bool_value=False)) is False
    assert PROBE.parameter_value(SimpleNamespace(type=4, string_value='_00123')) == '_00123'
    assert PROBE.parameter_value(SimpleNamespace(type=0)) is None


def test_empty_optional_parameter_cannot_erase_valid_identity_readback():
    received, issues = {}, {}
    assert PROBE.accept_parameter_response(
        'device_type', [SimpleNamespace(type=4, string_value='D435i$')], received, issues)
    assert not PROBE.accept_parameter_response('enable_infra', [], received, issues)
    assert received == {'device_type': 'D435i$'}
    assert 'enable_infra' in issues
    assert 'serial_no' not in received  # Empty result is never successful identity verification.


def test_false_parameter_is_valid_but_uninitialized_value_is_not():
    received, issues = {}, {}
    assert PROBE.accept_parameter_response(
        'enable_depth', [SimpleNamespace(type=1, bool_value=False)], received, issues)
    assert not PROBE.accept_parameter_response(
        'serial_no', [SimpleNamespace(type=0)], received, issues)
    assert received['enable_depth'] is False
    assert 'serial_no' not in received


def test_probe_dds_default_does_not_leak_into_independent_launches(monkeypatch):
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)

    def configure():
        monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/probe/default.xml')
        return '/probe/default.xml'

    child_environment, profile = PROBE.configure_probe_transport(configure)
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in child_environment
    assert 'FASTDDS_DEFAULT_PROFILES_FILE' not in child_environment
    assert profile == '/probe/default.xml'
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/operator/custom.xml')
    child_environment, _ = PROBE.configure_probe_transport(lambda: '/operator/custom.xml')
    assert child_environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/operator/custom.xml'


def test_library_evidence_never_reads_process_outside_owned_group(tmp_path, monkeypatch):
    log = tmp_path / 'camera.log'
    log.write_text(
        '[realsense2_camera_node-1]: process started with pid [10]\n'
        '[realsense2_camera_node-2]: process started with pid [20]\n'
        '[other_node-1]: process started with pid [30]\n')
    monkeypatch.setattr(PROBE.os, 'getpgid', lambda pid: 500 if pid == 10 else 600)
    process = tmp_path / '10'
    process.mkdir()
    process.joinpath('maps').write_text(
        '000 rw-p 0 0 1 /workspace/.cuda_realsense/install/librealsense/lib/librealsense2.so\n'
        '000 rw-p 0 0 1 /usr/local/cuda/lib64/libcudart.so\n')
    result = PROBE.camera_library_evidence(SimpleNamespace(pid=500), log, tmp_path)
    assert result['cuda_sdk_path_loaded']
    assert len(result['driver_processes']) == 2
    assert result['driver_processes'][1] == {
        'pid': 20, 'skipped': 'Not in the owned launch process group.'}
