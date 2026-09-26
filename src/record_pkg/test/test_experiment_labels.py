"""Explicit labels and alternating same-sample relative angles, without hardware."""

import csv
import json

import pytest

from record_pkg import export_csv
from record_pkg.experiment_labels import (
    RELATIVE_COLUMNS, relative_angles, session_contact_segment_id,
    validate_contact_segment_id,
)
from record_pkg.summary_csv import export_summary


@pytest.mark.parametrize('value', [0, 9, 18])
def test_valid_labels(value):
    assert validate_contact_segment_id(value) == value


@pytest.mark.parametrize('value', [-1, 19, True, False, 1.5, '9', None])
def test_invalid_labels(value):
    with pytest.raises(ValueError):
        validate_contact_segment_id(value)


def test_absent_legacy_label_is_not_free_motion():
    assert session_contact_segment_id({}) == ''
    assert session_contact_segment_id({'snapshot': {'contact_segment_id': 0}}) == 0


def sample_angles(offset=0.):
    pan = [offset + i * -.125 for i in range(18)]
    tilt = [offset + i * .25 + 10 for i in range(18)]
    tilt[0] = 0.0  # Selected zeros despite nonzero values on the other axis.
    pan[1] = 0.0
    tilt[2] = -.25
    return pan, tilt


def test_fixed_axis_selection_sign_zero_order_and_original_arrays():
    pan, tilt = sample_angles()
    result = relative_angles(pan, tilt)
    assert list(result) == list(RELATIVE_COLUMNS)
    assert list(result.values()) == [tilt[i] if i % 2 == 0 else pan[i] for i in range(18)]
    assert result['relative_angle_1'] == 0
    assert result['relative_angle_2'] == 0
    assert result['relative_angle_3'] == -.25
    assert (pan, tilt) == sample_angles()
    row = next(export_csv.message_rows(
        {'pan_relative': pan, 'tilt_relative': tilt}, 'custom_interfaces/msg/SegmentAngle',
        123, 0, topic='/estimated_segment_angle', contact_segment_id=9))
    assert json.loads(row['pan_relative']) == pan
    assert json.loads(row['tilt_relative']) == tilt
    assert list(row)[-19:] == ['contact_segment_id', *RELATIVE_COLUMNS]
    assert [row[key] for key in RELATIVE_COLUMNS] == list(result.values())


def test_bad_selected_axis_never_falls_back_to_other_axis():
    result = relative_angles([1.] * 17, [2.] * 18)
    assert result['relative_angle_1'] == 2.
    assert result['relative_angle_2'] == ''
    pan, tilt = sample_angles()
    tilt[0] = float('nan')
    assert relative_angles(pan, tilt)['relative_angle_1'] == ''


def test_estimator_tilt_first_sparse_arrays_select_all_active_joints():
    expected = [(-1.) ** i * i * .01 for i in range(18)]
    tilt = [expected[i] if i % 2 == 0 else 0. for i in range(18)]
    pan = [expected[i] if i % 2 else 0. for i in range(18)]
    assert list(relative_angles(pan, tilt).values()) == expected


@pytest.mark.parametrize('label', [0, 9, 18])
def test_offline_bag_every_row_labeled_and_summary_appends_19_columns(tmp_path, label):
    rosbag = pytest.importorskip('rosbag2_py')
    from custom_interfaces.msg import SegmentAngle
    from geometry_msgs.msg import WrenchStamped
    from rclpy.serialization import serialize_message, deserialize_message

    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(tmp_path / 'bag'), storage_id='sqlite3'),
                rosbag.ConverterOptions('', ''))
    for topic, typename in (('/estimated_segment_angle', 'custom_interfaces/msg/SegmentAngle'),
                            ('/fts_data', 'geometry_msgs/msg/WrenchStamped')):
        writer.create_topic(rosbag.TopicMetadata(name=topic, type=typename, serialization_format='cdr'))
    for i, force_value in enumerate((123., 0., -45.)):
        angle, force = SegmentAngle(), WrenchStamped()
        angle.header.stamp.sec = force.header.stamp.sec = 20 + i
        angle.pan_relative, angle.tilt_relative = sample_angles(i)
        angle.pan_absolute, angle.tilt_absolute = [3.] * 18, [4.] * 18
        force.wrench.force.x = force_value
        for topic, message in (('/estimated_segment_angle', angle), ('/fts_data', force)):
            writer.write(topic, serialize_message(message), (20 + i) * 10**9)
    del writer
    (tmp_path / 'session.json').write_text(json.dumps({
        'snapshot': {'contact_segment_id': label}}))
    (tmp_path / 'recording_config.json').write_text(json.dumps({'required_topics': []}))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    directory = tmp_path / 'csv'
    for topic in ('/estimated_segment_angle', '/fts_data'):
        with (directory / manifest['topics'][topic]['file']).open() as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 3
        assert all(row['contact_segment_id'] == str(label) for row in rows)
    with (directory / 'summary.csv').open() as handle:
        raw_rows = list(csv.reader(handle))
    assert len(raw_rows[0]) == 268  # Selected summary; label/relative angles remain last.
    assert all(len(row) == len(raw_rows[0]) for row in raw_rows)
    assert raw_rows[0][-19:] == ['contact_segment_id', *RELATIVE_COLUMNS]
    with (directory / 'summary.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    assert [float(row['fts.fx']) for row in rows] == [123., 0., -45.]
    for i, row in enumerate(rows):
        pan, tilt = sample_angles(i)
        assert row['contact_segment_id'] == str(label)
        assert [float(row[f'pan_relative_{j:02d}_rad']) for j in range(1, 19)] == pan
        assert [float(row[f'tilt_relative_{j:02d}_rad']) for j in range(1, 19)] == tilt
        assert [float(row[key]) for key in RELATIVE_COLUMNS] == list(relative_angles(pan, tilt).values())
    # Export never modifies serialized originals.
    reader = export_csv.open_reader(tmp_path / 'bag')
    topic, serialized, _ = reader.read_next()
    assert topic == '/estimated_segment_angle'
    original = deserialize_message(serialized, SegmentAngle)
    assert list(original.pan_relative) == sample_angles()[0]
    assert list(original.tilt_relative) == sample_angles()[1]


def test_legacy_summary_label_remains_blank(tmp_path):
    name = 'angles.csv'
    pan, tilt = sample_angles()
    with (tmp_path / name).open('w') as handle:
        writer = csv.DictWriter(handle, ['source_time_ns', 'pan_relative', 'tilt_relative'])
        writer.writeheader()
        writer.writerow(dict(source_time_ns=1, pan_relative=json.dumps(pan),
                             tilt_relative=json.dumps(tilt)))
    export_summary(tmp_path, {'status': 'complete', 'topics': {
        '/estimated_segment_angle': {'file': name}}})
    with (tmp_path / 'summary.csv').open() as handle:
        row = next(csv.DictReader(handle))
    assert row['contact_segment_id'] == ''
