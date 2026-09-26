"""Wide inspection table preserves provenance and does not invent missing values."""

import csv
import hashlib
import json
import math

import pytest

from record_pkg import export_csv
from record_pkg.summary_csv import (
    ANGLE_FIELDS, ANGLE_TOPIC, RELATIVE_VELOCITY_COLUMNS, Timeline, angles,
    array_values, export_summary, orientation,
)


def stamp(ns, received=None, frame='hrm_base'):
    return dict(source_time_ns=ns, bag_receive_time_ns=received or ns + 50_000_000,
                **{'header.frame_id': frame})


def angle_row(ns=1_000_000_000, received=None):
    return dict(**stamp(ns, received), **{
        name: json.dumps([i * .01 + field for i in range(18)])
        for field, name in enumerate(ANGLE_FIELDS)})


def write_topic(directory, manifest, topic, rows):
    name = export_csv.topic_filename(topic)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (directory / name).open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    manifest['topics'][topic] = {'file': name}


def read_summary(path):
    with (path / 'summary.csv').open(newline='') as handle:
        return list(csv.DictReader(handle))


def pose_row(ns, **extras):
    return {**stamp(ns, frame='ID0'), 'pose.position.x': .12, 'pose.position.y': .03,
            'pose.position.z': -.01, 'pose.orientation.x': 0, 'pose.orientation.y': 0,
            'pose.orientation.z': math.sin(math.pi / 12),
            'pose.orientation.w': math.cos(math.pi / 12), **extras}


def test_all_channels_all_angles_and_uncompensated_pose_are_separate_columns(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [angle_row()])
    write_topic(tmp_path, manifest, '/motor_state', [{**stamp(1_004_000_000),
                'actual_position': '[1,2,3,4]', 'actual_velocity': '[5,6,7,8]'}])
    write_topic(tmp_path, manifest, '/loadcell_state', [{**stamp(1_002_000_000),
                'stress': '[0,200,300,400]', 'output_voltage': '[1,2,3,4]'}])
    write_topic(tmp_path, manifest, '/apriltag/tag1_in_tag0/pose', [pose_row(1_005_000_000)])
    write_topic(tmp_path, manifest, '/estimated_tip_position', [{**stamp(1_000_000_000),
                'point.x': .082, 'point.y': 0, 'point.z': .003}])
    write_topic(tmp_path, manifest, '/estimated_tip_position_unscaled', [{
        **stamp(1_000_000_000), 'point.x': .09, 'point.y': 0, 'point.z': .004}])
    # Wrong stamp must not attach another frame's estimate to this angle.
    write_topic(tmp_path, manifest, '/kinematics/fk_tip_position', [{**stamp(1_001_000_000),
                'point.x': .08, 'point.y': 0, 'point.z': .002}])
    write_topic(tmp_path, manifest, '/tf', [{
        **stamp(1_000_000_000), 'child_frame_id': 'hrm_fk_tip',
        'transform.translation.x': .081, 'transform.translation.y': 0,
        'transform.translation.z': 0, 'transform.rotation.x': 0,
        'transform.rotation.y': 0, 'transform.rotation.z': 0, 'transform.rotation.w': 1}])
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in tmp_path.glob('*.csv')}
    report = export_summary(tmp_path, manifest)
    row = read_summary(tmp_path)[0]
    assert report['rows_written'] == 1
    assert row['motor.Motor #4 position [count]'] == '4.0'
    assert row['loadcell.Loadcell #1 tension'] == '0.0'
    assert row['motor.time_difference_ms'] == '4.0'
    assert row['pan_relative_18_rad'] == '0.17'
    assert row['tilt_absolute_18_rad'] == '3.17'
    assert row['relative_angular_velocity_1_rad_s'] == '6.0'
    assert row['relative_angular_velocity_2_rad_s'] == '4.01'
    assert row['relative_angular_velocity_18_rad_s'] == '4.17'
    assert not any('angular_velocity_absolute' in key for key in row)
    assert not any(key.startswith('estimated_tip_unscaled.') for key in row)
    assert 'estimated_tip_unscaled' not in report['groups']
    assert row['estimated_tip.x_m'] == '0.082'
    assert row['fk_tip.x_m'] == '' and row['fk_tip.matched'] == 'False'
    assert row['fk_tip_tf.x_m'] == '0.081' and row['fk_tip_tf.qw'] == '1.0'
    assert row['tag_id0_to_id1.frame_id'] == 'ID0'
    assert row['tag_id0_to_id1.x_m'] == '0.12'  # No mounting compensation.
    assert float(row['tag_id0_to_id1.yaw_deg']) == pytest.approx(30)
    assert row['tag_id0_to_id1.detection_guard'] == 'unavailable'
    assert row['fts.fx'] == '' and row['fts.matched'] == 'False'
    assert len(row) == len(set(row)) == len(report['columns']) == 268
    assert [key for key in row if key.startswith('relative_angular_velocity_')] == list(
        RELATIVE_VELOCITY_COLUMNS)
    with (tmp_path / 'summary.csv').open(newline='') as handle:
        assert all(len(raw_row) == 268 for raw_row in csv.reader(handle))
    for name, digest in before.items():
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == digest


def test_active_relative_velocities_keep_zero_sign_order_and_same_sample():
    sample = angle_row()
    pan = [-0.1 * (i + 1) if i % 2 else 999. for i in range(18)]
    tilt = [-0.2 * (i + 1) if not i % 2 else 888. for i in range(18)]
    tilt[0], pan[1] = 0., 0.
    sample['pan_angular_velocity_relative'] = json.dumps(pan)
    sample['tilt_angular_velocity_relative'] = json.dumps(tilt)
    result = angles(sample)
    assert [result[key] for key in RELATIVE_VELOCITY_COLUMNS] == [
        tilt[i] if i % 2 == 0 else pan[i] for i in range(18)]
    # Original relative AND absolute pan/tilt angles still have all 18 columns.
    for field in ANGLE_FIELDS[:4]:
        assert [result[f'{field}_{i + 1:02d}_rad'] for i in range(18)] == json.loads(
            sample[field])


@pytest.mark.parametrize('invalid', ['[]', '[1,2]', 'not-json', json.dumps([None] * 18),
                                     json.dumps([float('nan')] * 18)])
def test_missing_selected_velocity_does_not_fall_back_to_other_axis(invalid):
    sample = angle_row()
    sample['tilt_angular_velocity_relative'] = invalid
    result = angles(sample)
    for i, key in enumerate(RELATIVE_VELOCITY_COLUMNS):
        assert result[key] == ('' if i % 2 == 0 else 4 + i * .01)


def test_detection_failure_is_not_filled_from_nearby_valid_pose(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [angle_row()])
    write_topic(tmp_path, manifest, '/detections', [
        {**stamp(990_000_000), 'detections': '[{"id":0},{"id":1}]'},
        {**stamp(1_000_000_000), 'detections': '[{"id":0}]'},
    ])
    write_topic(tmp_path, manifest, '/apriltag/tag1_in_tag0/pose', [pose_row(990_000_000)])
    export_summary(tmp_path, manifest)
    row = read_summary(tmp_path)[0]
    assert row['tag_id0_to_id1.detection_guard'] == 'missing_or_not_both_detected'
    assert row['tag_id0_to_id1.x_m'] == ''
    assert row['tag_id0_to_id1.matched'] == 'False'


def test_pose_must_share_selected_detection_stamp(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [angle_row()])
    write_topic(tmp_path, manifest, '/detections', [
        {**stamp(1_000_000_000), 'detections': '[{"id":0},{"id":1}]'}])
    write_topic(tmp_path, manifest, '/apriltag/tag1_in_tag0/pose', [pose_row(999_000_000)])
    export_summary(tmp_path, manifest)
    row = read_summary(tmp_path)[0]
    assert row['tag_id0_to_id1.detection_guard'] == 'both_detected'
    assert row['tag_id0_to_id1.matched'] == 'False'


def test_headerless_uses_receive_time_but_invalid_stamped_data_does_not(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [angle_row(received=2_000_000_000)])
    write_topic(tmp_path, manifest, '/wire_length', [
        {**stamp('', 2_001_000_000), 'data': '[1,2,3,4]'}])
    write_topic(tmp_path, manifest, '/motor_state', [
        {**stamp(0, 2_001_000_000), 'actual_position': '[1,2,3,4]'}])
    export_summary(tmp_path, manifest)
    row = read_summary(tmp_path)[0]
    assert row['wire.time_basis'] == 'receive'
    assert row['wire.source_time_ns'] == ''
    assert row['wire.time_difference_ms'] == '1.0'
    assert row['wire.Cable #4 length'] == '4.0'
    assert row['motor.matched'] == 'False'


def test_stale_samples_are_blank_not_held_and_duplicate_anchors_are_reported(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC,
                [angle_row(), angle_row(), angle_row(1_100_000_000), angle_row(1)])
    write_topic(tmp_path, manifest, '/apriltag/tag1_in_tag0/pose', [pose_row(1_000_000_000)])
    report = export_summary(tmp_path, manifest)
    rows = read_summary(tmp_path)
    assert len(rows) == 2 and report['nonincreasing_anchor_timestamps'] == 2
    assert rows[0]['tag_id0_to_id1.matched'] == 'True'
    assert rows[1]['tag_id0_to_id1.matched'] == 'False'
    assert rows[1]['tag_id0_to_id1.qw'] == ''


def test_no_angles_produces_explicit_empty_table_and_never_overwrites(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    report = export_summary(tmp_path, manifest)
    assert report['status'] == 'no_angle_samples'
    assert read_summary(tmp_path) == []
    before = (tmp_path / 'summary.csv').read_bytes()
    with pytest.raises(FileExistsError):
        export_summary(tmp_path, manifest)
    assert (tmp_path / 'summary.csv').read_bytes() == before


def test_timeline_rejects_clock_resets_and_matches_nearest_not_just_previous():
    timeline = Timeline([stamp(0), stamp(10), stamp(10), stamp(5), stamp(30)])
    assert timeline.nearest(27, 10)[0] == 30
    assert timeline.nearest(40, 2) is None
    assert timeline.nearest(20, 10) is None
    assert timeline.stats['invalid_time'] == 1
    assert timeline.stats['nonincreasing_time'] == 2
    assert timeline.stats['backwards_anchor'] == 1


@pytest.mark.parametrize('value', [-1, float('nan'), float('inf'), 101, True])
def test_bad_tolerance_creates_no_files(tmp_path, value):
    with pytest.raises(ValueError):
        export_summary(tmp_path, {'status': 'complete', 'topics': {}}, value)
    assert not list(tmp_path.iterdir())


def test_invalid_arrays_and_quaternions_stay_blank_without_inventing_identity():
    assert array_values({'data': '[1,2]'}, 'data', 4) == [''] * 4
    assert array_values({'data': '[0,"nan",false,4]'}, 'data', 4) == [0., '', '', 4.]
    row = orientation({f'r.{axis}': 0 for axis in 'xyzw'}, 'r.')
    assert row['qw'] == 0 and row['roll_deg'] == ''


def test_frozen_aligned_force_columns_are_copied_not_recomputed(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [angle_row()])
    write_topic(tmp_path, manifest, '/fts_data', [{
        **stamp(1_000_000_000), 'wrench.force.x': 1, 'wrench.force.y': 2,
        'wrench.force.z': 3, 'aligned_fx': 1, 'aligned_fy': -3, 'aligned_fz': 2,
        'aligned_frame_id': 'hrm_base', 'aligned_force_valid': 'True'}])
    export_summary(tmp_path, manifest)
    row = read_summary(tmp_path)[0]
    assert [row[f'fts.f{a}'] for a in 'xyz'] == ['1.0', '2.0', '3.0']
    assert [row[f'fts.aligned_f{a}'] for a in 'xyz'] == ['1.0', '-3.0', '2.0']


@pytest.mark.parametrize('enabled', [True, False])
def test_real_offline_bag_export_automatically_adds_summary_when_enabled(tmp_path, enabled):
    rosbag = pytest.importorskip('rosbag2_py')
    from custom_interfaces.msg import SegmentAngle
    from geometry_msgs.msg import PointStamped, WrenchStamped
    from rclpy.serialization import serialize_message

    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(tmp_path / 'bag'), storage_id='sqlite3'),
                rosbag.ConverterOptions('', ''))
    for topic, type_name in ((ANGLE_TOPIC, 'custom_interfaces/msg/SegmentAngle'),
                             ('/fts_data', 'geometry_msgs/msg/WrenchStamped'),
                             ('/estimated_tip_position', 'geometry_msgs/msg/PointStamped')):
        writer.create_topic(rosbag.TopicMetadata(
            name=topic, type=type_name, serialization_format='cdr'))
    angle, force, tip = SegmentAngle(), WrenchStamped(), PointStamped()
    for message in (angle, force, tip):
        message.header.stamp.sec = 20
        message.header.frame_id = 'hrm_base'
    for field in ANGLE_FIELDS:
        setattr(angle, field, [0.1] * 18)
    force.header.stamp.nanosec = 1_000_000
    force.wrench.force.y = 123.
    tip.point.x = .081
    for index, (topic, message) in enumerate((
            (ANGLE_TOPIC, angle), ('/fts_data', force), ('/estimated_tip_position', tip))):
        writer.write(topic, serialize_message(message), 21_000_000_000 + index)
    del writer
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'required_topics': [], 'summary_csv': {'enabled': enabled}}))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete'
    if enabled:
        assert manifest['summary_csv']['rows_written'] == 1
        row = read_summary(tmp_path / 'csv')[0]
        assert row['fts.fy'] == '123.0' and row['fts.time_difference_ms'] == '1.0'
        assert row['estimated_tip.x_m'] == '0.081'
        assert row['pan_relative_18_rad'] == '0.1'
    else:
        assert manifest['summary_csv']['status'] == 'disabled'
        assert not (tmp_path / 'csv/summary.csv').exists()
    with (tmp_path / 'csv' / manifest['topics'][ANGLE_TOPIC]['file']).open() as handle:
        original = next(csv.DictReader(handle))
    assert json.loads(original['pan_relative']) == [0.1] * 18
    for field in ANGLE_FIELDS[4:]:
        assert json.loads(original[field]) == [0.1] * 18
