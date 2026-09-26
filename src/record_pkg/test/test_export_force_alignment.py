"""Frozen-session force-only CSV derivation; no publications or physical I/O."""

import copy
import csv
import hashlib
import json
from types import SimpleNamespace

import pytest

from record_pkg import export_csv
from record_pkg.force_alignment import create_alignment


def wrench_record(force=(1.0, 2.0, 3.0)):
    return {
        'header': {'stamp': {'sec': 12, 'nanosec': 34}, 'frame_id': 'original_sensor_frame'},
        'wrench': {'force': dict(zip(('x', 'y', 'z'), force)),
                   'torque': {'x': 4.0, 'y': 5.0, 'z': 6.0}},
    }


def read_rows(directory, manifest, topic):
    with (directory / 'csv' / manifest['topics'][topic]['file']).open(newline='') as handle:
        return list(csv.DictReader(handle))


def fake_export_session(tmp_path, monkeypatch, alignment='missing', records=None):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'metadata.yaml').write_text('test: true\n')
    records = records or [('/fts_data', wrench_record()),
                          ('/fts_data_kalman_filter', wrench_record((-2.0, 3.0, 4.0)))]
    snapshot = {} if alignment == 'missing' else {'force_alignment': alignment}
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': [topic for topic, _ in records],
        'snapshot': snapshot,
    }))
    # Current notes/settings must never override the frozen recording snapshot.
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'force_alignment': create_alignment(enabled=True, axes=['+x', '+y', '+z']),
        'notes': {'force_sensor_to_base_rotation': [[1, 0, 0], [0, 0, -1], [0, 1, 0]]},
    }))
    pending = [(topic, record, 13_000_000_000 + index)
               for index, (topic, record) in enumerate(records)]
    reader = SimpleNamespace(
        get_all_topics_and_types=lambda: [
            SimpleNamespace(name=topic, type='geometry_msgs/msg/WrenchStamped')
            for topic in dict(records)],
        has_next=lambda: bool(pending), read_next=lambda: pending.pop(0),
    )
    monkeypatch.setattr(export_csv, 'open_reader', lambda path: reader)
    monkeypatch.setattr(export_csv, 'make_decoder', lambda type_name: lambda value: value)
    import record_pkg.bag_integrity
    monkeypatch.setattr(record_pkg.bag_integrity, 'audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})
    return tmp_path


@pytest.mark.parametrize('axes, raw_expected, kf_expected', [
    (['+x', '-y', '-z'], [1.0, -2.0, -3.0], [-2.0, -3.0, -4.0]),
    (['+x', '-z', '+y'], [1.0, -3.0, 2.0], [-2.0, -4.0, 3.0]),
])
def test_enabled_alignment_adds_columns_without_changing_originals(
        tmp_path, monkeypatch, axes, raw_expected, kf_expected):
    alignment = create_alignment(enabled=True, axes=axes)
    fake_export_session(tmp_path, monkeypatch, alignment)
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    for topic, expected in [('/fts_data', raw_expected),
                            ('/fts_data_kalman_filter', kf_expected)]:
        row = read_rows(tmp_path, manifest, topic)[0]
        assert [float(row[f'aligned_f{axis}']) for axis in 'xyz'] == expected
        assert row['aligned_force_valid'] == 'True'
        assert row['aligned_frame_id'] == 'hrm_base'
        assert row['header.frame_id'] == 'original_sensor_frame'
        assert row['source_time_ns'] == '12000000034'
        assert [row[f'wrench.torque.{axis}'] for axis in 'xyz'] == ['4.0', '5.0', '6.0']
    raw = read_rows(tmp_path, manifest, '/fts_data')[0]
    assert [raw[f'wrench.force.{axis}'] for axis in 'xyz'] == ['1.0', '2.0', '3.0']
    assert raw['bag_receive_time_ns'] == '13000000000'
    saved = manifest['force_alignment']
    assert saved['axes'] == axes
    assert saved['matrix_base_from_sensor'] == alignment['matrix_base_from_sensor']
    assert saved['provenance'] == 'session.json:snapshot.force_alignment'
    assert not saved['torque_transformed'] and not saved['time_or_units_changed']
    assert 'does not certify CAN/sensor health' in saved['validity_scope']


@pytest.mark.parametrize('alignment', ['missing', create_alignment(enabled=False)])
def test_disabled_or_legacy_session_is_raw_only(tmp_path, monkeypatch, alignment):
    fake_export_session(tmp_path, monkeypatch, alignment)
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    assert not manifest['force_alignment']['enabled']
    assert not manifest['force_alignment']['derived_columns']
    for topic in ('/fts_data', '/fts_data_kalman_filter'):
        columns = read_rows(tmp_path, manifest, topic)[0]
        assert not any(key.startswith('aligned_') for key in columns)
    if alignment == 'missing':
        assert manifest['force_alignment']['provenance'] == 'absent_session_snapshot_raw_only'


@pytest.mark.parametrize('force', [
    (float('nan'), 2.0, 3.0), (1.0, float('inf'), 3.0),
    (1.0, 2.0, -float('inf')), ('bad', 2.0, 3.0), (True, 2.0, 3.0),
])
def test_invalid_force_preserves_row_and_leaves_derived_blank(tmp_path, monkeypatch, force):
    record = wrench_record(force)
    original = copy.deepcopy(record)
    fake_export_session(tmp_path, monkeypatch, create_alignment(enabled=True),
                        records=[('/fts_data', record)])
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    row = read_rows(tmp_path, manifest, '/fts_data')[0]
    assert row['wrench.force.x'] == str(original['wrench']['force']['x'])
    assert row['wrench.force.y'] == str(original['wrench']['force']['y'])
    assert row['wrench.force.z'] == str(original['wrench']['force']['z'])
    assert row['aligned_force_valid'] == 'False'
    assert [row[f'aligned_f{axis}'] for axis in 'xyz'] == ['', '', '']
    assert row['aligned_frame_id'] == 'hrm_base'
    assert row['wrench.torque.z'] == '6.0'


@pytest.mark.parametrize('topic, type_name', [
    ('/fts_data_offset', 'geometry_msgs/msg/WrenchStamped'),
    ('/estimated_external_force', 'geometry_msgs/msg/WrenchStamped'),
    ('/other_sensor', 'geometry_msgs/msg/WrenchStamped'),
    ('/fts_data', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
])
def test_only_the_two_named_wrench_topics_receive_derived_columns(topic, type_name):
    row = next(export_csv.message_rows(
        wrench_record(), type_name, 123, 0, topic=topic,
        force_alignment=create_alignment(enabled=True)))
    assert not any(key.startswith('aligned_') for key in row)


def test_finite_zero_force_is_numerically_valid_not_a_sensor_health_claim():
    row = next(export_csv.message_rows(
        wrench_record((0.0, 0.0, 0.0)), 'geometry_msgs/msg/WrenchStamped', 123, 0,
        topic='/fts_data', force_alignment=create_alignment(enabled=True)))
    assert row['aligned_force_valid'] is True
    assert [row[f'aligned_f{axis}'] for axis in 'xyz'] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('change', [
    {'axes': ['+x', '+x', '+z']}, {'axes': ['+x', '+y', '-z']},
    {'axes': ['x', '+y', '+z']}, {'enabled': 'true'},
    {'matrix_base_from_sensor': [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]},
    {'target_frame': 'wrong_frame'},
])
def test_invalid_snapshot_fails_before_opening_bag_or_creating_output(
        tmp_path, monkeypatch, change):
    alignment = create_alignment(enabled=True)
    alignment.update(change)
    fake_export_session(tmp_path, monkeypatch, alignment)

    def must_not_open(path):
        pytest.fail('Invalid frozen alignment must fail before the bag reader opens.')
    monkeypatch.setattr(export_csv, 'open_reader', must_not_open)
    with pytest.raises(ValueError):
        export_csv.export_session(tmp_path)
    assert not (tmp_path / 'csv').exists()


def test_real_offline_wrench_bag_alignment_leaves_bag_unchanged(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import WrenchStamped
    from rclpy.serialization import serialize_message
    bag = tmp_path / 'bag'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag.ConverterOptions('', ''))
    for index, topic in enumerate(('/fts_data', '/fts_data_kalman_filter')):
        message = WrenchStamped()
        message.header.frame_id = 'original_sensor_frame'
        message.header.stamp.sec, message.header.stamp.nanosec = 12, 34
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = 1.0, 2.0, 3.0
        message.wrench.torque.x = 4.0
        writer.create_topic(rosbag.TopicMetadata(
            name=topic, type='geometry_msgs/msg/WrenchStamped', serialization_format='cdr'))
        writer.write(topic, serialize_message(message), 13_000_000_000 + index)
    del writer
    paths = sorted(bag.glob('*.db3'))
    original_hashes = [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths]
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'snapshot': {'force_alignment': create_alignment(
            enabled=True, axes=['+x', '-z', '+y'])},
    }))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete', manifest
    for topic in ('/fts_data', '/fts_data_kalman_filter'):
        row = read_rows(tmp_path, manifest, topic)[0]
        assert [float(row[f'aligned_f{axis}']) for axis in 'xyz'] == [1.0, -3.0, 2.0]
        assert row['wrench.force.y'] == '2.0' and row['wrench.torque.x'] == '4.0'
        assert row['header.frame_id'] == 'original_sensor_frame'
        assert row['source_time_ns'] == '12000000034'
    assert [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths] == original_hashes
