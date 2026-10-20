"""Trim only incomplete startup anchors, without concealing later signal gaps."""

import csv
import hashlib
import json
from types import SimpleNamespace

import pytest

from record_pkg import export_csv
from record_pkg.export_csv import topic_filename
from record_pkg.summary_csv import ANGLE_FIELDS, ANGLE_TOPIC, export_summary


GROUPS = ['motor', 'loadcell', 'fts', 'wire', 'angles']
BASE = 1_000_000_000
STEP = 100_000_000


def stamp(index=0):
    return {'source_time_ns': BASE + index * STEP,
            'bag_receive_time_ns': BASE + index * STEP + 50_000_000,
            'header.frame_id': 'hrm_base'}


def angle(index=0):
    return {**stamp(index), **{
        field: json.dumps([0.0 if i % 3 == 0 else -0.01 * i for i in range(18)])
        for field in ANGLE_FIELDS}}


def motor(index=0, positions=None):
    return {**stamp(index), 'actual_position': json.dumps(
        [0, -10, 20, -30] if positions is None else positions)}


def loadcell(index=0):
    return {**stamp(index), 'stress': '[0, -2, 3, 4]'}


def force(index=0):
    return {**stamp(index), 'wrench.force.x': 0,
            'wrench.force.y': -2, 'wrench.force.z': 3}


def wire(index=0):
    return {**stamp(index), 'source_time_ns': '', 'data': '[0,-2,3,4]'}


def write_topic(directory, manifest, topic, rows):
    filename = topic_filename(topic)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (directory / filename).open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    manifest['topics'][topic] = {'file': filename}


def dataset(directory, count=4, motor_indices=None):
    manifest = {'status': 'complete', 'topics': {}, 'contact_segment_id': 9}
    write_topic(directory, manifest, ANGLE_TOPIC, [angle(i) for i in range(count)])
    for topic, convert in (('/loadcell_state', loadcell), ('/fts_data', force),
                           ('/wire_length', wire)):
        write_topic(directory, manifest, topic, [convert(i) for i in range(count)])
    indices = range(count) if motor_indices is None else motor_indices
    write_topic(directory, manifest, '/motor_state', [motor(i) for i in indices])
    return manifest


def read_rows(directory):
    with (directory / 'summary.csv').open(newline='') as handle:
        return list(csv.DictReader(handle))


def export(directory, manifest, groups=None):
    return export_summary(directory, manifest, startup_required_groups=(
        GROUPS if groups is None else groups))


def test_delayed_motor_skips_only_prefix_keeps_original_times_and_sources(tmp_path):
    manifest = dataset(tmp_path, motor_indices=[2, 3])
    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in tmp_path.glob('*.csv')}
    report = export(tmp_path, manifest)
    rows = read_rows(tmp_path)
    assert report['status'] == 'complete'
    assert report['rows_written'] == 2
    assert [int(row['source_time_ns']) for row in rows] == [BASE + 2 * STEP, BASE + 3 * STEP]
    assert [float(row['elapsed_s']) for row in rows] == pytest.approx([.2, .3])
    assert rows[0]['bag_receive_time_ns'] == str(BASE + 2 * STEP + 50_000_000)
    assert rows[0]['contact_segment_id'] == '9'
    assert rows[0]['motor.Motor #2 position [count]'] == '-10.0'
    startup = report['startup']
    assert startup['enabled'] is True
    assert startup['required_groups'] == GROUPS
    assert startup['ready'] is True
    assert startup['rows_skipped'] == 2
    assert startup['first_written_source_time_ns'] == BASE + 2 * STEP
    assert startup['first_written_bag_receive_time_ns'] == BASE + 2 * STEP + 50_000_000
    assert startup['first_written_elapsed_s'] == pytest.approx(.2)
    assert startup['skipped_by_group']['motor'] == 2
    assert startup['later_incomplete_rows'] == 0
    # Required startup signals do not include tag detection or exact-stamp tip estimates.
    assert rows[0]['tag_id0_to_id1.matched'] == 'False'
    assert rows[0]['estimated_tip.matched'] == 'False'
    assert rows[0]['fk_tip.matched'] == 'False'
    with (tmp_path / 'summary.csv').open(newline='') as handle:
        assert all(len(row) == 268 for row in csv.reader(handle))
    assert len(report['columns']) == len(set(report['columns'])) == 268
    for path, digest in hashes.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest


def test_later_missing_motor_is_retained_not_silently_joined_away(tmp_path):
    manifest = dataset(tmp_path, count=5, motor_indices=[1, 2, 4])
    report = export(tmp_path, manifest)
    rows = read_rows(tmp_path)
    assert [int(row['source_time_ns']) for row in rows] == [BASE + i * STEP for i in range(1, 5)]
    assert rows[2]['motor.matched'] == 'False'
    assert rows[2]['motor.Motor #1 position [count]'] == ''
    assert rows[3]['motor.matched'] == 'True'
    assert report['startup']['rows_skipped'] == 1
    assert report['startup']['later_incomplete_rows'] == 1


def test_all_groups_must_match_same_anchor_not_just_have_started(tmp_path):
    manifest = dataset(tmp_path, count=3, motor_indices=[1])
    write_topic(tmp_path, manifest, '/fts_data', [force(0), force(2)])
    report = export(tmp_path, manifest)
    assert report['status'] == 'no_common_start'
    assert report['rows_written'] == 0
    assert report['startup']['ready'] is False
    assert report['startup']['rows_skipped'] == 3
    assert report['startup']['skipped_by_group']['motor'] == 2
    assert report['startup']['skipped_by_group']['fts'] == 1
    assert read_rows(tmp_path) == []
    with (tmp_path / 'summary.csv').open(newline='') as handle:
        raw = list(csv.reader(handle))
    assert len(raw) == 1 and len(raw[0]) == 268


@pytest.mark.parametrize('offset_ns,skipped', [(25_000_000, 0), (25_000_001, 1)])
def test_startup_keeps_original_25ms_association_tolerance(tmp_path, offset_ns, skipped):
    manifest = dataset(tmp_path, count=2)
    first = motor(0)
    first['source_time_ns'] += offset_ns
    first['bag_receive_time_ns'] += offset_ns
    write_topic(tmp_path, manifest, '/motor_state', [first, motor(1)])
    report = export(tmp_path, manifest)
    assert report['startup']['rows_skipped'] == skipped
    assert report['rows_written'] == 2 - skipped


def test_headerless_wire_requires_anchor_receive_time_not_source_time(tmp_path):
    manifest = dataset(tmp_path, count=2)
    invalid = angle(0)
    invalid['bag_receive_time_ns'] = ''
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [invalid, angle(1)])
    report = export(tmp_path, manifest)
    assert report['startup']['rows_skipped'] == 1
    assert report['startup']['skipped_by_group']['wire'] == 1
    assert report['startup']['skipped_by_group']['motor'] == 0


@pytest.mark.parametrize('positions', [[1, 2, 3], [1, 2, 3, 4, 5],
                                       [0, None, 2, 3], [0, float('nan'), 2, 3],
                                       [0, float('inf'), 2, 3], [0, True, 2, 3]])
def test_invalid_required_position_skips_even_if_timestamp_matched(tmp_path, positions):
    manifest = dataset(tmp_path, count=2)
    write_topic(tmp_path, manifest, '/motor_state', [motor(0, positions), motor(1)])
    report = export(tmp_path, manifest)
    assert report['rows_written'] == 1
    assert report['startup']['rows_skipped'] == 1
    assert report['startup']['skipped_by_group']['motor'] == 1
    assert read_rows(tmp_path)[0]['source_time_ns'] == str(BASE + STEP)


def test_optional_motor_and_loadcell_fields_do_not_prevent_start(tmp_path):
    manifest = dataset(tmp_path, count=1)
    report = export(tmp_path, manifest)
    row = read_rows(tmp_path)[0]
    assert report['startup']['rows_skipped'] == 0
    assert row['motor.Motor #1 position [count]'] == '0.0'
    assert row['motor.Motor #1 velocity'] == ''
    assert row['motor.Motor #1 acceleration'] == ''
    assert row['motor.Motor #1 torque'] == ''
    assert row['loadcell.Loadcell #1 tension'] == '0.0'
    assert row['loadcell.Loadcell #1 voltage'] == ''
    assert row['fts.tx'] == ''  # Only force XYZ is a required startup quantity.


@pytest.mark.parametrize('group,topic,key,value', [
    ('loadcell', '/loadcell_state', 'stress', '[1,2,3]'),
    ('fts', '/fts_data', 'wrench.force.y', 'nan'),
    ('wire', '/wire_length', 'data', '[1,2,null,4]'),
])
def test_other_required_groups_check_finite_complete_numeric_fields(
        tmp_path, group, topic, key, value):
    manifest = dataset(tmp_path, count=2)
    factory = {'loadcell': loadcell, 'fts': force, 'wire': wire}[group]
    bad = factory(0)
    bad[key] = value
    write_topic(tmp_path, manifest, topic, [bad, factory(1)])
    report = export(tmp_path, manifest)
    assert report['rows_written'] == 1
    assert report['startup']['skipped_by_group'][group] == 1


def test_active_angles_only_preserve_fixed_tilt_pan_signs_and_zero(tmp_path):
    manifest = dataset(tmp_path, count=2)
    good = angle(1)
    pan = [None if i % 2 == 0 else -.1 * i for i in range(18)]
    tilt = [0 if i == 0 else -.2 * i if i % 2 == 0 else None for i in range(18)]
    good['pan_relative'], good['tilt_relative'] = json.dumps(pan), json.dumps(tilt)
    # Absolute directions and angular velocities are not startup prerequisites.
    for field in ANGLE_FIELDS:
        if field not in ('pan_relative', 'tilt_relative'):
            good[field] = ''
    bad = angle(0)
    bad_tilt = [0.] * 18
    bad_tilt[0] = None
    bad['tilt_relative'] = json.dumps(bad_tilt)
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [bad, good])
    report = export(tmp_path, manifest)
    rows = read_rows(tmp_path)
    assert report['startup']['skipped_by_group']['angles'] == 1
    assert len(rows) == 1
    assert [float(rows[0][f'relative_angle_{i + 1}']) for i in range(18)] == [
        tilt[i] if i % 2 == 0 else pan[i] for i in range(18)]
    assert rows[0]['relative_angle_1'] == '0.0'
    assert rows[0]['relative_angle_2'] == '-0.1'


@pytest.mark.parametrize('group,topic,factory', [
    ('fts_kalman', '/fts_data_kalman_filter', force),
    ('wire_velocity', '/wire_length_velocity', wire),
])
def test_explicit_additional_groups_are_optional_unless_requested(tmp_path, group, topic, factory):
    manifest = dataset(tmp_path, count=2)
    write_topic(tmp_path, manifest, topic, [factory(1)])
    report = export(tmp_path, manifest, GROUPS + [group])
    assert report['startup']['rows_skipped'] == 1
    assert report['startup']['skipped_by_group'][group] == 1


@pytest.mark.parametrize('group,topic', [
    ('fts', '/fts_data'), ('fts_kalman', '/fts_data_kalman_filter')])
@pytest.mark.parametrize('bad_valid,bad_component', [('False', 2), ('', 2), ('True', 'nan')])
def test_enabled_force_alignment_requires_valid_aligned_vector(
        tmp_path, group, topic, bad_valid, bad_component):
    manifest = dataset(tmp_path, count=2)
    manifest['force_alignment'] = {'enabled': True}
    bad = {**force(0), 'aligned_fx': 0, 'aligned_fy': bad_component, 'aligned_fz': -3,
           'aligned_force_valid': bad_valid, 'aligned_frame_id': 'hrm_base'}
    good = {**force(1), 'aligned_fx': 0, 'aligned_fy': -2, 'aligned_fz': 3,
            'aligned_force_valid': 'True', 'aligned_frame_id': 'hrm_base'}
    write_topic(tmp_path, manifest, topic, [bad, good])
    report = export(tmp_path, manifest, ['angles', group])
    assert report['startup']['rows_skipped'] == 1
    assert report['startup']['skipped_by_group'][group] == 1
    assert read_rows(tmp_path)[0][f'{group}.aligned_fy'] == '-2.0'


def test_disabled_alignment_does_not_require_nonexistent_aligned_columns(tmp_path):
    manifest = dataset(tmp_path, count=1)
    manifest['force_alignment'] = {'enabled': False}
    report = export(tmp_path, manifest)
    assert report['startup']['ready'] is True
    assert report['rows_written'] == 1
    assert read_rows(tmp_path)[0]['fts.aligned_fx'] == ''


def test_unrequested_fts_does_not_gate_even_when_alignment_enabled(tmp_path):
    manifest = dataset(tmp_path, count=1)
    manifest['force_alignment'] = {'enabled': True}
    report = export(tmp_path, manifest, ['motor', 'angles'])
    assert report['rows_written'] == 1


def test_no_anchor_samples_remains_distinct_from_no_common_start(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    report = export(tmp_path, manifest)
    assert report['status'] == 'no_angle_samples'
    assert report['startup']['rows_skipped'] == 0
    assert report['startup']['ready'] is False
    assert read_rows(tmp_path) == []


def test_only_invalid_anchor_stamps_do_not_count_as_startup_skips(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    bad = angle()
    bad['source_time_ns'] = 0
    write_topic(tmp_path, manifest, ANGLE_TOPIC, [bad])
    report = export(tmp_path, manifest)
    assert report['status'] == 'no_angle_samples'
    assert report['invalid_anchor_timestamps'] == 1
    assert report['startup']['rows_skipped'] == 0


def test_explicit_empty_groups_preserve_legacy_rows_with_missing_signals(tmp_path):
    manifest = dataset(tmp_path, count=3, motor_indices=[2])
    report = export(tmp_path, manifest, [])
    assert report['rows_written'] == 3
    assert report['startup']['enabled'] is False
    assert report['startup']['rows_skipped'] == 0
    assert read_rows(tmp_path)[0]['motor.matched'] == 'False'


def test_no_frozen_configuration_retains_legacy_export_behavior(tmp_path):
    manifest = dataset(tmp_path, count=2, motor_indices=[1])
    report = export_summary(tmp_path, manifest)
    assert report['rows_written'] == 2
    assert report['startup']['required_groups'] == []


@pytest.mark.parametrize('override,expected', [(None, 1), ([], 2)])
def test_frozen_recording_configuration_and_explicit_override(tmp_path, override, expected):
    directory = tmp_path / 'csv'
    directory.mkdir()
    manifest = dataset(directory, count=2, motor_indices=[1])
    session_path = tmp_path / 'session.json'
    session_path.write_text('{}')
    manifest['session_metadata_source'] = str(session_path)
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'summary_csv': {'startup_required_groups': GROUPS}}))
    report = export_summary(directory, manifest, startup_required_groups=override)
    assert report['rows_written'] == expected
    assert report['startup']['required_groups'] == (GROUPS if override is None else [])


@pytest.mark.parametrize('invalid', ['motor', {}, [True], [None], ['tags'], ['motor', 'motor'],
                                     ['tag_id0_to_id1'], ['estimated_tip'], ['fk_tip']])
def test_invalid_startup_groups_rejected_before_output_creation(tmp_path, invalid):
    manifest = dataset(tmp_path, count=1)
    with pytest.raises(ValueError):
        export_summary(tmp_path, manifest, startup_required_groups=invalid)
    assert not (tmp_path / 'summary.csv').exists()
    assert not (tmp_path / 'summary.schema.json').exists()


def test_invalid_frozen_startup_configuration_rejected_before_output(tmp_path):
    directory = tmp_path / 'csv'
    directory.mkdir()
    manifest = dataset(directory, count=1)
    session_path = tmp_path / 'session.json'
    session_path.write_text('{}')
    manifest['session_metadata_source'] = str(session_path)
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'summary_csv': {'startup_required_groups': 'motor'}}))
    with pytest.raises(ValueError):
        export_summary(directory, manifest)
    assert not (directory / 'summary.csv').exists()
    assert not (directory / 'summary.schema.json').exists()


@pytest.mark.parametrize('groups,have_motor,status,summary_status,rows_written', [
    (GROUPS, True, 'complete', 'complete', 1),
    (GROUPS, False, 'partial', 'no_common_start', 0),
    (None, False, 'complete', 'complete', 2),
    (['angles'], False, 'complete', 'complete', 2),
])
def test_automatic_export_uses_frozen_groups_and_preserves_all_original_rows(
        tmp_path, monkeypatch, groups, have_motor, status, summary_status, rows_written):
    import record_pkg.bag_integrity

    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'metadata.yaml').write_text('test: true\n')
    types = {ANGLE_TOPIC: 'custom_interfaces/msg/SegmentAngle',
             '/motor_state': 'custom_interfaces/msg/MotorState',
             '/loadcell_state': 'custom_interfaces/msg/LoadcellState',
             '/fts_data': 'geometry_msgs/msg/WrenchStamped',
             '/wire_length': 'std_msgs/msg/Float32MultiArray'}
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(types),
        'snapshot': {'contact_segment_id': 9}}))
    settings = {'enabled': True}
    if groups is not None:
        settings['startup_required_groups'] = groups
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'required_topics': [], 'summary_csv': settings}))
    records = []
    for index in range(2):
        source = BASE + index * STEP
        received = source + 50_000_000
        header = {'stamp': {'sec': source // 1_000_000_000,
                            'nanosec': source % 1_000_000_000}, 'frame_id': 'hrm_base'}
        messages = {
            ANGLE_TOPIC: {'header': header, **{field: [0.] * 18 for field in ANGLE_FIELDS}},
            '/loadcell_state': {'header': header, 'stress': [1., 2., 3., 4.]},
            '/fts_data': {'header': header, 'wrench': {'force': {'x': 0., 'y': -2., 'z': 3.}}},
            '/wire_length': {'data': [1., 2., 3., 4.]}}
        if have_motor and index == 1:
            messages['/motor_state'] = {'header': header, 'actual_position': [0, -10, 20, -30]}
        records.extend((topic, message, received) for topic, message in messages.items())
    monkeypatch.setattr(record_pkg.bag_integrity, 'audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})
    reader = SimpleNamespace(
        get_all_topics_and_types=lambda: [SimpleNamespace(name=name, type=typename)
                                          for name, typename in types.items()],
        has_next=lambda: bool(records), read_next=lambda: records.pop(0))
    monkeypatch.setattr(export_csv, 'open_reader', lambda path: reader)
    monkeypatch.setattr(export_csv, 'make_decoder', lambda typename: lambda record: record)
    report = export_csv.export_session(tmp_path)
    assert report['status'] == status
    assert report['summary_csv']['status'] == summary_status
    assert report['summary_csv']['rows_written'] == rows_written
    assert report['summary_csv']['startup']['required_groups'] == (groups or [])
    assert len(read_rows(tmp_path / 'csv')) == rows_written
    assert report['topics'][ANGLE_TOPIC]['rows_written'] == 2
    assert report['topics']['/fts_data']['rows_written'] == 2
    with (tmp_path / 'csv' / report['topics'][ANGLE_TOPIC]['file']).open(newline='') as handle:
        original_angles = list(csv.DictReader(handle))
    assert [int(row['source_time_ns']) for row in original_angles] == [BASE, BASE + STEP]
    saved_manifest = json.loads((tmp_path / 'csv/manifest.json').read_text())
    assert saved_manifest['status'] == status
    assert saved_manifest['summary_csv']['status'] == summary_status


def test_cli_reports_no_common_start_as_nonzero_without_discarding_sources(tmp_path):
    from record_pkg.summary_csv import main

    manifest = dataset(tmp_path, count=1, motor_indices=[])
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    result = main([str(tmp_path), '--startup-required-groups', 'motor', 'angles'])
    assert result == 2
    assert json.loads((tmp_path / 'summary.schema.json').read_text())['status'] == 'no_common_start'
    assert (tmp_path / manifest['topics'][ANGLE_TOPIC]['file']).is_file()
