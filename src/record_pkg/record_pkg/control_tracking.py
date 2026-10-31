"""Offline command/pose inspection table from finalized per-topic CSV exports.

Every /control/reference row is retained. Matching is streaming nearest source
time, without interpolation, a maximum age, ROS imports, or hardware access.
"""

import argparse
from contextlib import ExitStack
import csv
import json
import math
from pathlib import Path

from record_pkg.experiment_labels import validate_contact_segment_id
from record_pkg.summary_csv import (
    Timeline, csv_rows, number, orientation, point, positive_ns, tf_pose,
)


REFERENCE_TOPIC = '/control/reference'
BASE_FRAME = 'hrm_base'
ALIGNMENT_BASES = ('command_stamp', 'feedback_stamp')
CARTESIAN_MODES = ('position', 'admittance')
GROUP_METADATA = (
    'matched', 'numeric_valid', 'frame_valid', 'source_time_ns',
    'bag_receive_time_ns', 'time_basis', 'time_difference_ms', 'frame_id',
)
ANCHOR_COLUMNS = (
    'source_time_ns', 'bag_receive_time_ns', 'elapsed_s', 'command_frame_id',
    'command_stamp_valid', 'command_frame_valid', 'feedback_time_ns',
    'feedback_stamp_valid', 'alignment_time_ns', 'alignment_time_basis',
    'alignment_time_valid', 'alignment_time_backwards', 'control_mode',
    'motor_output_enabled', 'cartesian_reference_valid', 'cartesian_reference_usable',
    'pan_rad', 'tilt_rad', 'command_angles_valid',
    'nominal_x_m', 'nominal_y_m', 'nominal_z_m', 'nominal_numeric_valid',
    'target_x_m', 'target_y_m', 'target_z_m', 'target_numeric_valid',
    'contact_segment_id',
)
ERROR_COLUMNS = (
    'target_minus_fk_y_m', 'target_minus_fk_z_m', 'target_minus_fk_yz_norm_m',
    'target_minus_fk_valid',
)


def boolean(value):
    """Retain explicit booleans; malformed or absent values remain blank."""
    if value is True or value in ('True', 'true'):
        return True
    if value is False or value in ('False', 'false'):
        return False
    return ''


def feedback_ns(row):
    """Combine the original ROS stamp, preserving an explicit zero as zero."""
    try:
        sec = int(row.get('feedback_stamp.sec', ''))
        nanosec = int(row.get('feedback_stamp.nanosec', ''))
    except (ValueError, TypeError):
        return ''
    if not 0 <= nanosec < 1_000_000_000:
        return ''
    return sec * 1_000_000_000 + nanosec


def specifications():
    """Independent observed point and orientation streams, never center positions."""
    return (
        ('estimated_tip', '/estimated_tip_position', point, ''),
        ('fk_tip', '/kinematics/fk_tip_position', point, ''),
        ('fk_tip_tf', '/tf', tf_pose, 'hrm_fk_tip'),
        ('estimated_tip_orientation', '/tf',
         lambda row: orientation(row, 'transform.rotation.'),
         'estimated_segment_center_19'),
    )


def columns():
    result = list(ANCHOR_COLUMNS)
    for prefix, _, convert, _ in specifications():
        result.extend(f'{prefix}.{key}' for key in convert({}))
        result.extend(f'{prefix}.{key}' for key in GROUP_METADATA)
    return result + list(ERROR_COLUMNS)


def command_row(anchor, contact_id, initial_ns):
    source_ns = positive_ns(anchor.get('source_time_ns'))
    feedback = feedback_ns(anchor)
    mode = anchor.get('control_mode', '')
    frame_valid = anchor.get('header.frame_id') == BASE_FRAME
    cartesian = boolean(anchor.get('cartesian_reference_valid'))
    cartesian_usable = mode in CARTESIAN_MODES and cartesian is True and frame_valid
    basis = ('command_stamp' if mode == 'kinematics' else
             'feedback_stamp' if mode in CARTESIAN_MODES else '')
    target = source_ns if basis == 'command_stamp' else positive_ns(feedback)
    alignment_valid = bool(
        basis and source_ns is not None and target is not None and frame_valid
        and (mode == 'kinematics' or cartesian_usable))
    row = dict(
        source_time_ns=anchor.get('source_time_ns', ''),
        bag_receive_time_ns=anchor.get('bag_receive_time_ns', ''),
        elapsed_s=((source_ns - initial_ns) / 1e9
                   if source_ns is not None and initial_ns is not None else ''),
        command_frame_id=anchor.get('header.frame_id', ''),
        command_stamp_valid=source_ns is not None, command_frame_valid=frame_valid,
        feedback_time_ns=feedback, feedback_stamp_valid=positive_ns(feedback) is not None,
        alignment_time_ns=target if target is not None and basis else '',
        alignment_time_basis=basis, alignment_time_valid=alignment_valid,
        alignment_time_backwards=False, control_mode=mode,
        motor_output_enabled=boolean(anchor.get('motor_output_enabled')),
        cartesian_reference_valid=cartesian,
        cartesian_reference_usable=cartesian_usable,
        pan_rad=number(anchor.get('pan_rad')), tilt_rad=number(anchor.get('tilt_rad')),
        contact_segment_id=contact_id, target_minus_fk_valid=False,
    )
    row['command_angles_valid'] = row['pan_rad'] != '' and row['tilt_rad'] != ''
    for name in ('nominal', 'target'):
        values = point(anchor if cartesian_usable else {}, f'{name}_position.')
        row.update({f'{name}_{key}': value for key, value in values.items()})
        row[f'{name}_numeric_valid'] = all(value != '' for value in values.values())
    return row


def sample_values(convert, sample):
    values = convert(sample)
    # A zero or incomplete quaternion is invalid, never an identity orientation.
    # Keep valid FK translation independently; original invalid values remain in TF CSV.
    if 'qw' in values and any(values[key] == '' for key in (
            'roll_deg', 'pitch_deg', 'yaw_deg')):
        for key in ('qx', 'qy', 'qz', 'qw', 'roll_deg', 'pitch_deg', 'yaw_deg'):
            values[key] = ''
    return values


def add_errors(row):
    """Effective target minus measured-joint FK point, in hrm_base Y/Z metres."""
    if (not row['target_numeric_valid'] or not row['fk_tip.matched']
            or not row['fk_tip.numeric_valid']):
        return
    y = row['target_y_m'] - row['fk_tip.y_m']
    z = row['target_z_m'] - row['fk_tip.z_m']
    norm = math.hypot(y, z)
    if not all(math.isfinite(value) for value in (y, z, norm)):
        return
    row.update(target_minus_fk_y_m=y, target_minus_fk_z_m=z,
               target_minus_fk_yz_norm_m=norm, target_minus_fk_valid=True)


def export_control_tracking(directory, manifest=None):
    """Create new CSV/schema files, returning their report; never modify inputs."""
    directory = Path(directory).resolve()
    if manifest is None:
        manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest.get('status') not in ('complete', 'partial'):
        raise ValueError('Finish per-topic CSV export before making control tracking')
    output = directory / 'control_tracking.csv'
    schema_path = directory / 'control_tracking.schema.json'
    if any(path.exists() or path.is_symlink() for path in (output, schema_path)):
        raise FileExistsError(
            'Control tracking already exists; original files were left untouched')
    contact_id = manifest.get('contact_segment_id', '')
    if contact_id != '':
        validate_contact_segment_id(contact_id)
    specs = specifications()
    schema = dict(
        schema_version=1, status='exporting', file=output.name,
        schema_file=schema_path.name, rows_written=0, source_manifest='manifest.json',
        source_export_status=manifest['status'],
        source_integrity_status=manifest.get('integrity', {}).get('status', 'not_checked'),
        anchor_topic=REFERENCE_TOPIC, columns=columns(), contact_segment_id=contact_id,
        first_valid_command_source_time_ns=None,
        invalid_command_timestamps=0, nonincreasing_command_timestamps=0,
        invalid_alignment_rows=0, backwards_alignment_rows=0,
        groups={prefix: dict(
            topic=topic, child_frame=child, required_frame=BASE_FRAME,
            time_basis='source', matching='nearest_unbounded',
            max_time_difference_ms=None, matched=0, unmatched=0,
            invalid_numeric=0, frame_mismatch=0, association_counts={})
            for prefix, topic, _, child in specs},
        notes=[
            'One output row per /control/reference row, including invalid stamps or values. '
            'No startup gate or dependency on training-summary streams. No legacy command '
            'inference: absent reference telemetry produces a fixed header and no_commands.',
            'source_time_ns is command generation time; bag_receive_time_ns is recorder '
            'receipt. elapsed_s starts at the first positive command stamp. feedback_time_ns '
            'is the original feedback image stamp, or explicit zero in Kinematics.',
            'Kinematics alignment_time_basis=command_stamp; Position/Admittance uses '
            'feedback_stamp when Cartesian reference and stamps are valid. Each basis '
            'has independent streaming cursors. Backwards anchors within a basis keep '
            'their command row but have blank pose values and a reported flag.',
            'motor_output_enabled is a configured output gate, not acknowledgment or proof '
            'of motor execution. Commands describe generated references; Kinematics preview '
            'may subsequently be blocked. pan_rad/tilt_rad retain signed angles and zero.',
            'Nominal and effective target XYZ are metres in hrm_base. Kinematics has no '
            'Cartesian reference: its nominal/target cells are blank even if raw defaults '
            'were zero. Position/Admittance use explicit cartesian_reference_valid.',
            'Each point/orientation group independently selects nearest source time; past '
            'or future samples may be reused across arbitrarily long gaps or stream ends. '
            'There is no age cutoff, interpolation, clock synchronization, or latency fix. '
            'time_difference_ms is signed sample stamp minus alignment_time_ns.',
            'matched requires a temporal candidate and exact hrm_base parent/frame; numeric_valid '
            'additionally requires every numeric component in that group to be finite/usable. '
            'Wrong-frame candidates retain timestamps/frame provenance but their values are '
            'blank and matched=false. Invalid fields stay blank; genuine zeros remain zero.',
            'fk_tip and fk_tip_tf are FK of observed/estimated joint angles, not command-angle '
            'FK. Their point and rotation may have different source and receipt timestamps.',
            'estimated_tip is the fitted tip boundary. estimated_tip_orientation copies only '
            'the last-link DH rotation from estimated_segment_center_19, never that center '
            'translation; it is not an independently observed tip roll.',
            'Quaternions retain valid source xyzw values. Euler degrees use normalized '
            'quaternions and extrinsic XYZ Rz(yaw) Ry(pitch) Rx(roll), not pan/tilt. '
            'Invalid quaternions produce blank quaternion/Euler cells, not identity rotation.',
            'target_minus_fk_y_m/z_m and yz_norm_m use effective target minus '
            '/kinematics/fk_tip_position in hrm_base. They are source-time associations, '
            'not execution acknowledgments or calibrated tracking accuracy.',
            'Invalid/nonincreasing source-stream stamps are skipped and counted by each '
            'basis cursor, never sorted across clock resets. Association counters describe '
            'rows consumed for matching, not a full independent source-stream audit.',
            'complete means table generation succeeded, not that every command or measurement '
            'is valid or recording is lossless. Per-topic CSVs, summary.csv, manifest and bag '
            'are untouched and authoritative; inspect source integrity and row validity.',
        ],
    )
    # Reserve both names exclusively. Never rewrite even a failed previous export.
    with output.open('x', newline='') as stream, schema_path.open('x') as schema_stream:
        try:
            with ExitStack() as stack:
                anchors = csv_rows(directory, manifest, REFERENCE_TOPIC, stack)
                timelines = {}
                last_alignment = {}
                last_command = None
                writer = csv.DictWriter(stream, fieldnames=schema['columns'])
                writer.writeheader()
                for anchor in anchors:
                    source_ns = positive_ns(anchor.get('source_time_ns'))
                    if source_ns is None:
                        schema['invalid_command_timestamps'] += 1
                    else:
                        if last_command is not None and source_ns <= last_command:
                            schema['nonincreasing_command_timestamps'] += 1
                        last_command = source_ns
                        if schema['first_valid_command_source_time_ns'] is None:
                            schema['first_valid_command_source_time_ns'] = source_ns
                    row = command_row(anchor, contact_id,
                                      schema['first_valid_command_source_time_ns'])
                    basis, target = row['alignment_time_basis'], row['alignment_time_ns']
                    if not row['alignment_time_valid']:
                        schema['invalid_alignment_rows'] += 1
                    elif basis in last_alignment and target < last_alignment[basis]:
                        row['alignment_time_backwards'] = True
                        schema['backwards_alignment_rows'] += 1
                    else:
                        last_alignment[basis] = target
                    if row['alignment_time_valid'] and basis not in timelines:
                        timelines[basis] = {
                            prefix: Timeline(csv_rows(directory, manifest, topic, stack, child))
                            for prefix, topic, _, child in specs}
                    for prefix, _, convert, _ in specs:
                        row.update({f'{prefix}.{key}': False
                                    for key in ('matched', 'numeric_valid', 'frame_valid')})
                        row[f'{prefix}.time_basis'] = 'source'
                        group = schema['groups'][prefix]
                        if not row['alignment_time_valid']:
                            group['unmatched'] += 1
                            continue
                        matched = timelines[basis][prefix].nearest(target, None)
                        if matched is None:
                            group['unmatched'] += 1
                            continue
                        stamp, sample = matched
                        row.update({f'{prefix}.{key}': sample.get(key, '') for key in (
                            'source_time_ns', 'bag_receive_time_ns')})
                        row[f'{prefix}.frame_id'] = sample.get('header.frame_id', '')
                        row[f'{prefix}.time_difference_ms'] = (stamp - target) / 1e6
                        if sample.get('header.frame_id') != BASE_FRAME:
                            group['frame_mismatch'] += 1
                            group['unmatched'] += 1
                            continue
                        row[f'{prefix}.frame_valid'] = True
                        row[f'{prefix}.matched'] = True
                        values = sample_values(convert, sample)
                        row.update({f'{prefix}.{key}': value for key, value in values.items()})
                        valid = all(value != '' for value in values.values())
                        row[f'{prefix}.numeric_valid'] = valid
                        group['matched'] += 1
                        group['invalid_numeric'] += not valid
                    add_errors(row)
                    writer.writerow(row)
                    schema['rows_written'] += 1
                for basis, groups in timelines.items():
                    for prefix, timeline in groups.items():
                        schema['groups'][prefix]['association_counts'][basis] = timeline.stats
            stream.flush()
            schema['status'] = 'complete' if schema['rows_written'] else 'no_commands'
        except Exception as exc:
            schema.update(status='failed', error=str(exc))
            raise
        finally:
            json.dump(schema, schema_stream, indent=2, ensure_ascii=False, allow_nan=False)
            schema_stream.write('\n')
    return schema


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_directory', type=Path,
                        help='Existing finalized topic CSV directory with manifest.json')
    options = parser.parse_args(args)
    try:
        report = export_control_tracking(options.csv_directory)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'Control tracking export failed: {exc}\n')
    print(f"Control tracking {report['status']}: {options.csv_directory / report['file']} "
          f"({report['rows_written']} rows)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
