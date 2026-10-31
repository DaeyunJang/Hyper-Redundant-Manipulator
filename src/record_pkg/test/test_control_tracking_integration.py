"""Offline recording integration for command-anchored control tracking CSVs."""

import csv
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from record_pkg import export_csv
from record_pkg.capture import selected_topics
from record_pkg.finalize_session import finalize_session


REFERENCE_TOPIC = '/control/reference'
REFERENCE_TYPE = 'custom_interfaces/msg/ControlReference'
POINT_TYPE = 'geometry_msgs/msg/PointStamped'
ESTIMATED_TIP = '/estimated_tip_position'
FK_TIP = '/kinematics/fk_tip_position'
CONTROL_TOPICS = (REFERENCE_TOPIC, '/position/sine_reference', '/position/sine_status')


def read_rows(path):
    with Path(path).open(newline='') as handle:
        return list(csv.DictReader(handle))


def stamp(ns):
    return {'sec': ns // 10**9, 'nanosec': ns % 10**9}


def point_record(ns, x):
    return {'header': {'stamp': stamp(ns), 'frame_id': 'hrm_base'},
            'point': {'x': x, 'y': -0.004, 'z': 0.002}}


def reference_record(mode, ns, feedback_ns=0):
    return {
        'header': {'stamp': stamp(ns), 'frame_id': 'hrm_base'},
        'feedback_stamp': stamp(feedback_ns), 'control_mode': mode,
        'pan_rad': -0.1, 'tilt_rad': 0.2,
        'cartesian_reference_valid': mode != 'kinematics',
        'nominal_position': {'x': 0.082, 'y': -0.003, 'z': 0.001},
        'target_position': {'x': 0.081, 'y': -0.004, 'z': 0.002},
        'motor_output_enabled': False,
    }


def transform_record(ns, child, x):
    return {
        'header': {'stamp': stamp(ns), 'frame_id': 'hrm_base'},
        'child_frame_id': child,
        'transform': {'translation': {'x': x, 'y': -0.004, 'z': 0.002},
                      'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    }


def tracking_samples(with_tf=True, with_angles=False):
    """The Position command has distinct generation and camera-feedback times."""
    records = []
    types = {REFERENCE_TOPIC: REFERENCE_TYPE, ESTIMATED_TIP: POINT_TYPE,
             FK_TIP: POINT_TYPE}
    for ns, x in ((1_000_000_000, 0.080), (1_500_000_000, 0.081),
                  (2_000_000_000, 0.099)):
        records.extend([(ESTIMATED_TIP, point_record(ns, x)),
                        (FK_TIP, point_record(ns, x + 0.001))])
        if with_tf:
            types['/tf'] = 'tf2_msgs/msg/TFMessage'
            records.append(('/tf', {'transforms': [
                transform_record(ns, 'hrm_fk_tip', x + 0.002),
                transform_record(ns, 'estimated_segment_center_19', x + 0.003),
            ]}))
        if ns == 1_000_000_000:
            records.append((REFERENCE_TOPIC, reference_record('kinematics', ns)))
        elif ns == 2_000_000_000:
            records.append((REFERENCE_TOPIC,
                            reference_record('position', ns, 1_500_000_000)))
        if with_angles:
            types['/estimated_segment_angle'] = 'custom_interfaces/msg/SegmentAngle'
            records.append(('/estimated_segment_angle', {
                'header': {'stamp': stamp(ns), 'frame_id': 'hrm_base'},
                **{name: [0.01] * 18 for name in (
                    'pan_relative', 'tilt_relative', 'pan_absolute', 'tilt_absolute',
                    'pan_angular_velocity_relative', 'tilt_angular_velocity_relative')},
            }))
    return [(topic, record, 10_000_000_000 + index)
            for index, (topic, record) in enumerate(records)], types


def prepare_session(directory, selected, config):
    (directory / 'bag').mkdir()
    (directory / 'bag' / 'metadata.yaml').write_text('test: true\n')
    (directory / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(selected),
    }))
    (directory / 'recording_config.json').write_text(json.dumps(config))


def patch_reader(monkeypatch, records, types):
    def reader(_):
        remaining = list(records)
        return SimpleNamespace(
            get_all_topics_and_types=lambda: [SimpleNamespace(name=name, type=value)
                                              for name, value in types.items()],
            has_next=lambda: bool(remaining), read_next=lambda: remaining.pop(0),
        )
    monkeypatch.setattr(export_csv, 'open_reader', reader)
    monkeypatch.setattr(export_csv, 'make_decoder', lambda _: lambda value: value)
    monkeypatch.setattr('record_pkg.bag_integrity.audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})


def assert_tracking_rows(directory, manifest, with_tf):
    report = manifest['control_tracking_csv']
    assert report['status'] == 'complete'
    assert report['rows_written'] == 2
    assert report['anchor_topic'] == REFERENCE_TOPIC
    rows = read_rows(directory / 'control_tracking.csv')
    assert [row['control_mode'] for row in rows] == ['kinematics', 'position']
    assert [row['source_time_ns'] for row in rows] == ['1000000000', '2000000000']
    assert [row['alignment_time_ns'] for row in rows] == ['1000000000', '1500000000']
    assert [row['alignment_time_basis'] for row in rows] == ['command_stamp', 'feedback_stamp']
    for row, expected_x in zip(rows, (0.080, 0.081)):
        assert float(row['pan_rad']) == -0.1
        assert float(row['tilt_rad']) == 0.2
        assert row['motor_output_enabled'] == 'False'
        assert float(row['estimated_tip.x_m']) == pytest.approx(expected_x)
        assert float(row['fk_tip.x_m']) == pytest.approx(expected_x + 0.001)
        for group in ('estimated_tip', 'fk_tip'):
            assert row[f'{group}.matched'] == 'True'
            assert row[f'{group}.source_time_ns'] == row['alignment_time_ns']
            assert row[f'{group}.time_difference_ms'] == '0.0'
            assert row[f'{group}.frame_id'] == 'hrm_base'
        for group in ('fk_tip_tf', 'estimated_tip_orientation'):
            assert row[f'{group}.matched'] == str(with_tf)
            assert row[f'{group}.qw'] == ('1.0' if with_tf else '')
    assert rows[0]['cartesian_reference_valid'] == 'False'
    assert rows[0]['nominal_x_m'] == rows[0]['target_x_m'] == ''
    assert rows[1]['cartesian_reference_valid'] == 'True'
    assert float(rows[1]['nominal_x_m']) == 0.082
    assert float(rows[1]['target_x_m']) == 0.081
    schema = json.loads((directory / 'control_tracking.schema.json').read_text())
    assert schema == report
    assert list(rows[0]) == report['columns']
    return rows


def test_control_topics_are_selected_once_and_remain_optional():
    config = json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())
    selected = selected_topics(config)
    assert config['control_tracking_csv']['enabled'] is True
    for topic in CONTROL_TOPICS:
        assert selected.count(topic) == 1
        assert topic in config['topic_groups']['motor_and_control']
        assert topic not in config['required_topics']


@pytest.mark.parametrize('summary_enabled', [False, True])
@pytest.mark.parametrize('with_tf', [False, True])
def test_tracking_export_runs_without_angle_samples_or_enabled_summary(
        tmp_path, monkeypatch, summary_enabled, with_tf):
    records, types = tracking_samples(with_tf=with_tf)
    prepare_session(tmp_path, types, {
        'required_topics': [], 'summary_csv': {'enabled': summary_enabled},
    })
    patch_reader(monkeypatch, records, types)
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete', manifest
    assert manifest['summary_csv']['status'] == (
        'no_angle_samples' if summary_enabled else 'disabled')
    assert_tracking_rows(tmp_path / 'csv', manifest, with_tf)
    original = read_rows(tmp_path / 'csv' / manifest['topics'][REFERENCE_TOPIC]['file'])
    assert len(original) == 2
    assert original[0]['nominal_position.x'] == '0.082'
    assert original[1]['feedback_stamp.nanosec'] == '500000000'
    assert json.loads((tmp_path / 'csv/manifest.json').read_text()) == manifest


def test_control_tracking_can_be_disabled_without_losing_original_references(
        tmp_path, monkeypatch):
    records, types = tracking_samples()
    prepare_session(tmp_path, types, {
        'required_topics': [], 'summary_csv': {'enabled': False},
        'control_tracking_csv': {'enabled': False},
    })
    patch_reader(monkeypatch, records, types)
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    assert manifest['control_tracking_csv'] == {'status': 'disabled'}
    assert manifest['topics'][REFERENCE_TOPIC]['rows_written'] == 2
    assert not (tmp_path / 'csv/control_tracking.csv').exists()
    assert not (tmp_path / 'csv/control_tracking.schema.json').exists()


def test_control_tracking_preserves_existing_angle_summary_bytes(tmp_path, monkeypatch):
    records, types = tracking_samples(with_angles=True)
    config = {'required_topics': [], 'summary_csv': {'enabled': True}}
    prepare_session(tmp_path, types, config)
    patch_reader(monkeypatch, records, types)
    enabled = export_csv.export_session(tmp_path)
    assert enabled['status'] == 'complete', enabled
    assert enabled['control_tracking_csv']['rows_written'] == 2
    assert enabled['summary_csv']['rows_written'] == 3
    summary_path = tmp_path / 'csv/summary.csv'
    rows = read_rows(summary_path)
    assert len(rows) == 3 and len(rows[0]) == 268

    config['control_tracking_csv'] = {'enabled': False}
    (tmp_path / 'recording_config.json').write_text(json.dumps(config))
    alternate = tmp_path / 'csv_without_tracking'
    disabled = export_csv.export_session(tmp_path, alternate)
    assert disabled['status'] == 'complete'
    assert disabled['control_tracking_csv']['status'] == 'disabled'
    assert summary_path.read_bytes() == (alternate / 'summary.csv').read_bytes()
    assert enabled['summary_csv'] == disabled['summary_csv']


def test_finalize_reports_generated_control_tracking_files(tmp_path, monkeypatch):
    records, types = tracking_samples()
    prepare_session(tmp_path, types, {
        'auto_export_csv': True, 'required_topics': [], 'summary_csv': {'enabled': False},
    })
    patch_reader(monkeypatch, records, types)
    report = finalize_session(tmp_path)
    assert report['status'] == report['csv_status'] == 'complete', report
    manifest = json.loads((tmp_path / 'csv/manifest.json').read_text())
    assert report['control_tracking_csv'] == manifest['control_tracking_csv']
    assert report['control_tracking_csv']['rows_written'] == 2
    assert json.loads((tmp_path / 'postprocess.json').read_text()) == report


def test_real_offline_bag_generates_control_tracking_without_angles(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    interfaces = pytest.importorskip('custom_interfaces.msg')
    if not hasattr(interfaces, 'ControlReference'):
        pytest.skip('ControlReference message has not been generated in this ROS overlay')
    from geometry_msgs.msg import PointStamped, TransformStamped
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage

    def populate(message, fields):
        for name, value in fields.items():
            if isinstance(value, dict):
                populate(getattr(message, name), value)
            else:
                setattr(message, name, value)
        return message

    records, types = tracking_samples()
    bag = tmp_path / 'bag'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag.ConverterOptions('', ''))
    for topic, type_name in types.items():
        writer.create_topic(rosbag.TopicMetadata(
            name=topic, type=type_name, serialization_format='cdr'))
    for topic, record, received_ns in records:
        if topic == '/tf':
            message = TFMessage(transforms=[populate(TransformStamped(), value)
                                            for value in record['transforms']])
        else:
            cls = interfaces.ControlReference if topic == REFERENCE_TOPIC else PointStamped
            message = populate(cls(), record)
        writer.write(topic, serialize_message(message), received_ns)
    del writer  # Flush metadata without creating a ROS node or replaying a command.
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(types),
    }))
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'required_topics': [], 'summary_csv': {'enabled': False},
    }))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete', manifest
    assert manifest['integrity']['status'] == 'complete'
    assert_tracking_rows(tmp_path / 'csv', manifest, with_tf=True)
