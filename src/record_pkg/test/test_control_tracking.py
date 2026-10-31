"""Command rows and independently stamped observed poses keep honest provenance."""

import csv
import hashlib
import json
import math

import pytest

from record_pkg import control_tracking
from record_pkg.export_csv import topic_filename


NS = 1_000_000_000
GROUPS = ('estimated_tip', 'fk_tip', 'fk_tip_tf', 'estimated_tip_orientation')


def command(ns=NS, mode='kinematics', feedback=0, **changes):
    return {
        'source_time_ns': ns, 'bag_receive_time_ns': ns + 30_000_000,
        'header.frame_id': 'hrm_base', 'feedback_stamp.sec': feedback // NS,
        'feedback_stamp.nanosec': feedback % NS, 'control_mode': mode,
        'pan_rad': -.125, 'tilt_rad': 0, 'motor_output_enabled': False,
        'cartesian_reference_valid': mode != 'kinematics',
        'nominal_position.x': 0, 'nominal_position.y': -.01, 'nominal_position.z': 0,
        'target_position.x': 0, 'target_position.y': -.02, 'target_position.z': .03,
        **changes,
    }


def point(ns=NS, **changes):
    return {
        'source_time_ns': ns, 'bag_receive_time_ns': ns + 20_000_000,
        'header.frame_id': 'hrm_base', 'point.x': .08, 'point.y': 0, 'point.z': -.01,
        **changes,
    }


def transform(ns=NS, child='hrm_fk_tip', **changes):
    return {
        'source_time_ns': ns, 'bag_receive_time_ns': ns + 40_000_000,
        'header.frame_id': 'hrm_base', 'child_frame_id': child,
        'transform.translation.x': .08, 'transform.translation.y': 0,
        'transform.translation.z': -.01, 'transform.rotation.x': 0,
        'transform.rotation.y': 0, 'transform.rotation.z': -math.sqrt(.5),
        'transform.rotation.w': math.sqrt(.5), **changes,
    }


def write_topic(directory, manifest, topic, rows):
    filename = topic_filename(topic)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (directory / filename).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    manifest['topics'][topic] = {'file': filename}


def write_poses(directory, manifest, stamps=(NS,)):
    for topic in ('/estimated_tip_position', '/kinematics/fk_tip_position'):
        write_topic(directory, manifest, topic, [point(ns) for ns in stamps])
    write_topic(directory, manifest, '/tf', [
        transform(ns, child, **{'transform.translation.x': (
            777 if child == 'estimated_segment_center_19' else .08)})
        for ns in stamps for child in ('hrm_fk_tip', 'estimated_segment_center_19')])


def read_rows(directory):
    with (directory / 'control_tracking.csv').open(newline='') as stream:
        return list(csv.DictReader(stream))


def test_modes_zero_sign_frames_and_independent_pose_times(tmp_path):
    manifest = {'status': 'complete', 'topics': {}, 'contact_segment_id': 0}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [
        command(NS), command(2 * NS, 'position', NS),
        command(3 * NS, 'admittance', NS, motor_output_enabled=True),
    ])
    write_poses(tmp_path, manifest)
    write_topic(tmp_path, manifest, '/tf', [
        transform(NS + 1_000_000),
        transform(NS + 2_000_000, 'estimated_segment_center_19',
                  **{'transform.translation.x': 777}),
    ])
    (tmp_path / 'summary.csv').write_text('existing,summary\n1,2\n')
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    before = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in tmp_path.iterdir()}
    report = control_tracking.export_control_tracking(tmp_path)
    rows = read_rows(tmp_path)
    assert report['rows_written'] == 3
    assert report == json.loads((tmp_path / report['schema_file']).read_text())
    assert report['status'] == 'complete'
    assert rows[0]['alignment_time_basis'] == 'command_stamp'
    assert rows[1]['alignment_time_basis'] == rows[2]['alignment_time_basis'] == 'feedback_stamp'
    assert rows[0]['feedback_time_ns'] == '0'
    assert rows[0]['feedback_stamp_valid'] == 'False'
    assert rows[0]['motor_output_enabled'] == 'False'
    assert rows[2]['motor_output_enabled'] == 'True'
    assert rows[0]['nominal_y_m'] == rows[0]['target_y_m'] == ''
    assert rows[0]['cartesian_reference_usable'] == 'False'
    assert rows[0]['target_minus_fk_valid'] == 'False'
    for index, row in enumerate(rows):
        assert row['source_time_ns'] == str((index + 1) * NS)
        assert row['alignment_time_ns'] == str(NS)
        assert row['elapsed_s'] == str(float(index))
        assert row['pan_rad'] == '-0.125' and row['tilt_rad'] == '0.0'
        assert row['contact_segment_id'] == '0'
        assert row['estimated_tip.y_m'] == '0.0'
        assert row['fk_tip.z_m'] == '-0.01'
        assert row['fk_tip_tf.time_difference_ms'] == '1.0'
        assert row['estimated_tip_orientation.time_difference_ms'] == '2.0'
        assert float(row['fk_tip_tf.yaw_deg']) == pytest.approx(-90)
        assert not any(key.startswith('estimated_tip_orientation.x_') for key in row)
        for group in GROUPS:
            assert row[f'{group}.numeric_valid'] == row[f'{group}.matched'] == 'True'
            assert row[f'{group}.frame_id'] == 'hrm_base'
    for row in rows[1:]:
        assert row['nominal_y_m'] == '-0.01'
        assert row['target_y_m'] == '-0.02'
        assert row['target_x_m'] == '0.0'
        assert float(row['target_minus_fk_y_m']) == pytest.approx(-.02)
        assert float(row['target_minus_fk_z_m']) == pytest.approx(.04)
        assert row['target_minus_fk_valid'] == 'True'
    for name, digest in before.items():
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == digest


def test_mode_switch_clock_offset_has_independent_cursors(tmp_path):
    manifest = {'status': 'partial', 'topics': {}}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [
        command(NS), command(NS + 100_000_000, 'position', NS - 100_000_000),
        command(NS + 200_000_000),
        command(NS + 300_000_000, 'admittance', NS),
        command(NS + 400_000_000, 'position', NS - 50_000_000),
        command(NS + 500_000_000, 'position', NS + 100_000_000),
    ])
    write_poses(tmp_path, manifest, (NS - 100_000_000, NS, NS + 100_000_000, NS + 200_000_000))
    report = control_tracking.export_control_tracking(tmp_path, manifest)
    rows = read_rows(tmp_path)
    assert report['rows_written'] == 6 and report['backwards_alignment_rows'] == 1
    assert rows[1]['fk_tip.source_time_ns'] == str(NS - 100_000_000)
    assert rows[1]['alignment_time_backwards'] == 'False'
    assert rows[2]['fk_tip.source_time_ns'] == str(NS + 200_000_000)
    assert rows[3]['fk_tip.source_time_ns'] == str(NS)
    for group in GROUPS:
        assert rows[4][f'{group}.matched'] == 'False'
        assert rows[4][f'{group}.source_time_ns'] == ''
        assert rows[5][f'{group}.matched'] == 'True'
        stats = report['groups'][group]['association_counts']
        assert stats['feedback_stamp']['backwards_anchor'] == 1
        assert stats['command_stamp']['matched'] == 2
    assert rows[4]['alignment_time_backwards'] == 'True'


@pytest.mark.parametrize('stamps,selected', [
    ((NS - 10_000_000, NS + 10_000_000), NS - 10_000_000),
    ((NS - 10_000_000, NS, NS + 10_000_000), NS),
    ((NS - 20_000_000, NS + 5_000_000), NS + 5_000_000),
    ((1, 50 * NS), 1),
    ((50 * NS,), 50 * NS),
])
def test_nearest_pose_no_age_cutoff(tmp_path, stamps, selected):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [command()])
    write_poses(tmp_path, manifest, stamps)
    report = control_tracking.export_control_tracking(tmp_path, manifest)
    row = read_rows(tmp_path)[0]
    for group in GROUPS:
        assert row[f'{group}.source_time_ns'] == str(selected)
        assert float(row[f'{group}.time_difference_ms']) == (selected - NS) / 1e6
        assert report['groups'][group]['max_time_difference_ms'] is None


def test_missing_topics_keep_all_commands_and_fixed_columns(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [
        command(), command(2 * NS, 'position', NS)])
    report = control_tracking.export_control_tracking(tmp_path, manifest)
    for row in read_rows(tmp_path):
        assert list(row) == control_tracking.columns() == report['columns']
        assert row['contact_segment_id'] == ''
        for group in GROUPS:
            assert row[f'{group}.matched'] == 'False'
            assert row[f'{group}.numeric_valid'] == 'False'
            assert row[f'{group}.source_time_ns'] == ''


@pytest.mark.parametrize('legacy_topic', [None, '/surgical_tool_pose', '/position_controller'])
def test_no_commands_still_writes_schema_header_and_cli_succeeds(tmp_path, legacy_topic, capsys):
    manifest = {'status': 'complete', 'topics': {}}
    if legacy_topic:
        write_topic(tmp_path, manifest, legacy_topic, [command()])
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    assert control_tracking.main([str(tmp_path)]) == 0
    assert 'no_commands' in capsys.readouterr().out
    assert read_rows(tmp_path) == []
    schema = json.loads((tmp_path / 'control_tracking.schema.json').read_text())
    assert schema['status'] == 'no_commands' and schema['rows_written'] == 0
    with (tmp_path / 'control_tracking.csv').open(newline='') as stream:
        assert next(csv.reader(stream)) == control_tracking.columns()


def test_bad_anchor_stamps_frames_flags_and_numbers_are_preserved(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [
        command(0), command(-NS), command(NS, **{'header.frame_id': 'camera'}),
        command(2 * NS, 'position', 0),
        command(3 * NS, 'position', NS, **{'feedback_stamp.nanosec': NS}),
        command(4 * NS, 'position', NS, cartesian_reference_valid=False),
        command(5 * NS, 'unknown'),
        command(6 * NS, 'position', NS, pan_rad='nan', tilt_rad='inf',
                **{'target_position.y': 'NaN', 'nominal_position.z': '-inf'}),
        command(6 * NS), command(5 * NS),
    ])
    write_poses(tmp_path, manifest)
    report = control_tracking.export_control_tracking(tmp_path, manifest)
    rows = read_rows(tmp_path)
    assert len(rows) == report['rows_written'] == 10
    assert report['invalid_command_timestamps'] == 2
    assert report['nonincreasing_command_timestamps'] == 2
    assert report['invalid_alignment_rows'] == 7
    assert rows[0]['source_time_ns'] == '0' and rows[1]['source_time_ns'] == str(-NS)
    assert rows[0]['elapsed_s'] == rows[1]['elapsed_s'] == ''
    for row in rows[:7]:
        assert row['alignment_time_valid'] == 'False'
        assert row['fk_tip.matched'] == 'False'
    assert rows[2]['command_frame_id'] == 'camera'
    assert rows[3]['feedback_time_ns'] == '0'
    assert rows[4]['feedback_time_ns'] == ''
    assert rows[5]['target_y_m'] == ''
    assert rows[7]['command_angles_valid'] == 'False'
    assert rows[7]['pan_rad'] == rows[7]['tilt_rad'] == rows[7]['target_y_m'] == ''
    assert rows[7]['target_numeric_valid'] == 'False'
    assert rows[7]['target_minus_fk_valid'] == 'False'
    assert rows[9]['alignment_time_backwards'] == 'True'


def test_wrong_frames_numeric_invalid_quaternion_and_invalid_source_stamps(tmp_path):
    manifest = {'status': 'complete', 'topics': {}}
    write_topic(tmp_path, manifest, control_tracking.REFERENCE_TOPIC, [
        command(NS, 'position', NS)])
    write_topic(tmp_path, manifest, '/estimated_tip_position', [
        point(0), point(-1), point(NS, **{'header.frame_id': 'camera'})])
    write_topic(tmp_path, manifest, '/kinematics/fk_tip_position', [
        point(NS, **{'point.z': 'nan'})])
    write_topic(tmp_path, manifest, '/tf', [
        transform(NS, **{'transform.rotation.z': 0, 'transform.rotation.w': 0}),
        transform(NS, 'estimated_segment_center_19', **{'transform.rotation.w': 'inf'}),
    ])
    report = control_tracking.export_control_tracking(tmp_path, manifest)
    row = read_rows(tmp_path)[0]
    assert row['estimated_tip.matched'] == row['estimated_tip.frame_valid'] == 'False'
    assert row['estimated_tip.source_time_ns'] == str(NS)
    assert row['estimated_tip.frame_id'] == 'camera'
    assert row['estimated_tip.x_m'] == ''
    assert report['groups']['estimated_tip']['frame_mismatch'] == 1
    stats = report['groups']['estimated_tip']['association_counts']['feedback_stamp']
    assert stats['invalid_time'] == 2
    assert row['fk_tip.matched'] == 'True' and row['fk_tip.numeric_valid'] == 'False'
    assert row['fk_tip.y_m'] == '0.0' and row['fk_tip.z_m'] == ''
    assert row['target_minus_fk_valid'] == 'False'
    assert row['fk_tip_tf.x_m'] == '0.08'
    for group in ('fk_tip_tf', 'estimated_tip_orientation'):
        assert row[f'{group}.matched'] == 'True' and row[f'{group}.numeric_valid'] == 'False'
        for field in ('qx', 'qy', 'qz', 'qw', 'roll_deg', 'pitch_deg', 'yaw_deg'):
            assert row[f'{group}.{field}'] == ''


@pytest.mark.parametrize('status', ['exporting', 'failed', 'recording', None])
def test_reject_unfinalized_exports_before_writing(tmp_path, status):
    with pytest.raises(ValueError, match='Finish per-topic'):
        control_tracking.export_control_tracking(tmp_path, {'status': status, 'topics': {}})
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('name', ['control_tracking.csv', 'control_tracking.schema.json'])
@pytest.mark.parametrize('symlink', [False, True])
def test_existing_output_never_overwritten_or_companion_created(tmp_path, name, symlink):
    existing = tmp_path / name
    if symlink:
        existing.symlink_to(tmp_path / 'missing-target')
    else:
        existing.write_bytes(b'keep this exact existing output')
    with pytest.raises(FileExistsError):
        control_tracking.export_control_tracking(tmp_path, {'status': 'complete', 'topics': {}})
    assert list(tmp_path.iterdir()) == [existing]
    if symlink:
        assert existing.is_symlink()
    else:
        assert existing.read_bytes() == b'keep this exact existing output'


def test_unsafe_topic_paths_rejected_with_failed_schema(tmp_path):
    directory = tmp_path / 'export'
    directory.mkdir()
    outside = tmp_path / 'outside.csv'
    outside.write_text('unchanged\n')
    manifest = {'status': 'complete', 'topics': {
        control_tracking.REFERENCE_TOPIC: {'file': '../outside.csv'}}}
    with pytest.raises(ValueError, match='Unsafe source CSV path'):
        control_tracking.export_control_tracking(directory, manifest)
    assert outside.read_text() == 'unchanged\n'
    schema = json.loads((directory / 'control_tracking.schema.json').read_text())
    assert schema['status'] == 'failed'


def test_matching_reads_only_two_sample_lookahead_per_stream(tmp_path, monkeypatch):
    consumed = {}

    def rows(directory, manifest, topic, stack, child=''):
        if topic == control_tracking.REFERENCE_TOPIC:
            return iter([command(NS), command(NS + 1, 'position', NS)])
        key = topic, child
        consumed.setdefault(key, 0)

        def samples():
            for index in range(1_000_000):
                consumed[key] += 1
                yield transform(NS + index * NS, child) if child else point(NS + index * NS)
        return samples()

    monkeypatch.setattr(control_tracking, 'csv_rows', rows)
    report = control_tracking.export_control_tracking(
        tmp_path, {'status': 'complete', 'topics': {}})
    assert report['rows_written'] == 2
    # Two independent bases each consume just the exact sample and one lookahead.
    assert len(consumed) == 4 and all(count == 4 for count in consumed.values())
