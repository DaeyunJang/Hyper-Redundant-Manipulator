"""Timestamp-matched wide CSV, derived from lossless topic CSVs after recording.

No ROS imports, publications, training arrays, calibration or interpolation.
Streaming nearest-time joins use bounded memory and explicitly expose timing.
"""

import argparse
from contextlib import ExitStack
import csv
import json
import math
from pathlib import Path

from record_pkg.experiment_labels import (
    RELATIVE_COLUMNS, relative_angles, validate_contact_segment_id,
)


ANGLE_TOPIC = '/estimated_segment_angle'
POSITION_ANGLE_FIELDS = ('pan_relative', 'pan_absolute', 'tilt_relative', 'tilt_absolute')
ANGLE_FIELDS = POSITION_ANGLE_FIELDS + (
    'pan_angular_velocity_relative', 'pan_angular_velocity_absolute',
    'tilt_angular_velocity_relative', 'tilt_angular_velocity_absolute')
JOINT_COUNT = 18
MOTOR_COUNT = 4
RELATIVE_VELOCITY_COLUMNS = tuple(
    f'relative_angular_velocity_{i}_rad_s' for i in range(1, JOINT_COUNT + 1))


def number(value):
    if isinstance(value, bool):
        return ''
    try:
        result = float(value)
        return result if math.isfinite(result) else ''
    except (TypeError, ValueError):
        return ''


def array_values(row, key, size):
    try:
        values = json.loads(row.get(key, ''))
        if not isinstance(values, list) or len(values) != size:
            return [''] * size
        return [number(value) for value in values]
    except (TypeError, ValueError):
        return [''] * size


def channels(row, field, label):
    return {label.format(i + 1): value
            for i, value in enumerate(array_values(row, field, MOTOR_COUNT))}


def angles(row):
    result = {}
    for name in POSITION_ANGLE_FIELDS:
        result.update({f'{name}_{i + 1:02d}_rad': value
                       for i, value in enumerate(array_values(row, name, JOINT_COUNT))})
    # Reuse the same fixed odd-tilt/even-pan selection as relative_angle_1..18.
    # Copy this sample's relative velocity, without differentiating or filtering.
    velocities = relative_angles(
        array_values(row, 'pan_angular_velocity_relative', JOINT_COUNT),
        array_values(row, 'tilt_angular_velocity_relative', JOINT_COUNT))
    result.update(zip(RELATIVE_VELOCITY_COLUMNS, velocities.values()))
    return result


def motor(row):
    result = {}
    for field, label in (('actual_position', 'Motor #{} position [count]'),
                         ('actual_velocity', 'Motor #{} velocity'),
                         ('actual_acceleration', 'Motor #{} acceleration'),
                         ('actual_torque', 'Motor #{} torque')):
        result.update(channels(row, field, label))
    return result


def loadcell(row):
    return {**channels(row, 'stress', 'Loadcell #{} tension'),
            **channels(row, 'output_voltage', 'Loadcell #{} voltage')}


def wrench(row):
    result = {f'{kind}{axis}': number(row.get(f'wrench.{field}.{axis}'))
              for kind, field in (('f', 'force'), ('t', 'torque')) for axis in 'xyz'}
    for axis in 'xyz':
        key = f't{axis}'
        if result[key] != '':
            result[key] = f'{result[key]:.1f}'
    result.update({f'aligned_f{axis}': number(row.get(f'aligned_f{axis}')) for axis in 'xyz'})
    result.update(aligned_force_valid=row.get('aligned_force_valid', ''),
                  aligned_frame_id=row.get('aligned_frame_id', ''))
    return result


def point(row, prefix='point.'):
    return {f'{axis}_m': number(row.get(prefix + axis)) for axis in 'xyz'}


def orientation(row, prefix):
    result = {f'q{axis}': number(row.get(prefix + axis)) for axis in 'xyzw'}
    result.update(roll_deg='', pitch_deg='', yaw_deg='')
    if any(result[f'q{axis}'] == '' for axis in 'xyzw'):
        return result
    x, y, z, w = (result[f'q{axis}'] for axis in 'xyzw')
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm <= 1e-12:
        return result
    x, y, z, w = (value / norm for value in (x, y, z, w))
    result.update(
        roll_deg=math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))),
        pitch_deg=math.degrees(math.asin(max(-1., min(1., 2 * (w*y - z*x))))),
        yaw_deg=math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))),
    )
    return result


def pose(row):
    return {**point(row, 'pose.position.'), **orientation(row, 'pose.orientation.')}


def tf_pose(row):
    return {**point(row, 'transform.translation.'),
            **orientation(row, 'transform.rotation.')}


def specifications():
    """prefix, topic, conversion, timing basis, exact-source-only, child filter."""
    return [
        ('motor', '/motor_state', motor, 'source', False, ''),
        ('loadcell', '/loadcell_state', loadcell, 'source', False, ''),
        ('fts', '/fts_data', wrench, 'source', False, ''),
        ('fts_kalman', '/fts_data_kalman_filter', wrench, 'source', False, ''),
        ('wire', '/wire_length', lambda r: channels(r, 'data', 'Cable #{} length'),
         'receive', False, ''),
        ('wire_velocity', '/wire_length_velocity',
         lambda r: channels(r, 'data', 'Cable #{} velocity'), 'receive', False, ''),
        ('estimated_tip', '/estimated_tip_position', point, 'source', True, ''),
        ('fk_tip', '/kinematics/fk_tip_position', point, 'source', True, ''),
        ('fk_tip_tf', '/tf', tf_pose, 'source', True, 'hrm_fk_tip'),
        ('estimated_tip_orientation', '/tf',
         lambda r: orientation(r, 'transform.rotation.'), 'source', True,
         'estimated_segment_center_19'),
        ('tag_id0_to_id1', '/apriltag/tag1_in_tag0/pose', pose, 'source', False, ''),
    ]


def positive_ns(value):
    try:
        result = int(value)
        return result if result > 0 else None
    except (ValueError, TypeError):
        return None


def both_tags_detected(row):
    try:
        detections = json.loads(row.get('detections', ''))
        ids = set()
        for detection in detections:
            value = detection['id']
            ids.update(value if isinstance(value, list) else [value])
        return {0, 1}.issubset(ids)
    except (ValueError, TypeError, KeyError):
        return False


def csv_rows(directory, manifest, topic, stack, child=''):
    entry = manifest.get('topics', {}).get(topic, {})
    name = entry.get('file')
    if not name:
        return iter(())
    path = (directory / name).resolve()
    if path.parent != directory.resolve():
        raise ValueError(f'Unsafe source CSV path: {name}')
    reader = csv.DictReader(stack.enter_context(path.open(newline='')))
    return (row for row in reader if not child or row.get('child_frame_id') == child)


class Timeline:
    """Two-sample lookahead; duplicates/backwards clocks are skipped and counted."""

    def __init__(self, rows, basis='source'):
        self.rows = iter(rows)
        self.basis = basis
        self.key = 'source_time_ns' if basis == 'source' else 'bag_receive_time_ns'
        self.previous = None
        self.following = None
        self.last_stamp = None
        self.last_target = None
        self.stats = dict(invalid_time=0, nonincreasing_time=0, matched=0,
                          unmatched=0, backwards_anchor=0)
        self.advance()

    def advance(self):
        self.following = None
        for row in self.rows:
            stamp = positive_ns(row.get(self.key))
            if stamp is None:
                self.stats['invalid_time'] += 1
                continue
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.stats['nonincreasing_time'] += 1
                continue
            self.last_stamp = stamp
            self.following = (stamp, row)
            return

    def nearest(self, target, tolerance_ns):
        if target is None or (self.last_target is not None and target < self.last_target):
            self.stats['backwards_anchor'] += 1
            self.stats['unmatched'] += 1
            return None
        self.last_target = target
        while self.following is not None and self.following[0] <= target:
            self.previous = self.following
            self.advance()
        candidates = [item for item in (self.previous, self.following) if item is not None]
        best = min(candidates, key=lambda item: abs(item[0] - target)) if candidates else None
        if best is None or abs(best[0] - target) > tolerance_ns:
            self.stats['unmatched'] += 1
            return None
        self.stats['matched'] += 1
        return best


def startup_requirements(manifest, groups):
    """Core training fields, not every optional diagnostic/pose column.

    Missing settings in a legacy session retain its original export behavior.
    A configured missing topic must not silently remove a startup requirement.
    """
    if groups is None:
        source = manifest.get('session_metadata_source')
        config_path = Path(source).parent / 'recording_config.json' if source else None
        settings = {}
        if config_path is not None and config_path.is_file():
            config = json.loads(config_path.read_text())
            settings = config.get('summary_csv', {})
        if not isinstance(settings, dict):
            raise ValueError('summary_csv settings must be an object.')
        groups = settings.get('startup_required_groups', [])
    fields = {
        'motor': [f'motor.Motor #{i} position [count]' for i in range(1, 5)],
        'loadcell': [f'loadcell.Loadcell #{i} tension' for i in range(1, 5)],
        'fts': [f'fts.f{axis}' for axis in 'xyz'],
        'fts_kalman': [f'fts_kalman.f{axis}' for axis in 'xyz'],
        'wire': [f'wire.Cable #{i} length' for i in range(1, 5)],
        'wire_velocity': [f'wire_velocity.Cable #{i} velocity' for i in range(1, 5)],
        'angles': list(RELATIVE_COLUMNS),
    }
    if (not isinstance(groups, (list, tuple))
            or any(not isinstance(group, str) or group not in fields for group in groups)
            or len(set(groups)) != len(groups)):
        raise ValueError('startup_required_groups must list distinct core signal groups: '
                         + ', '.join(fields))
    required = {group: fields[group] for group in groups}
    alignment = manifest.get('force_alignment', {})
    if alignment.get('enabled', False):
        for group in ('fts', 'fts_kalman'):
            if group in required:
                required[group] += [f'{group}.aligned_f{axis}' for axis in 'xyz']
    return required


def missing_startup_groups(row, required):
    missing = []
    for group, columns in required.items():
        if (group != 'angles' and row.get(f'{group}.matched') is not True
                or any(number(row.get(column)) == '' for column in columns)):
            missing.append(group)
        elif (f'{group}.aligned_fx' in columns
              and row.get(f'{group}.aligned_force_valid') not in (True, 'True', 'true')):
            missing.append(group)
    return missing


def export_summary(directory, manifest=None, max_time_difference_ms=25.0,
                   startup_required_groups=None):
    """Add summary.csv/schema without modifying/overwriting any original CSV."""
    directory = Path(directory).resolve()
    if (isinstance(max_time_difference_ms, bool)
            or not math.isfinite(max_time_difference_ms)
            or not 0 <= max_time_difference_ms <= 100):
        raise ValueError('Summary tolerance must be finite and in 0..100 ms')
    manifest = manifest or json.loads((directory / 'manifest.json').read_text())
    if manifest.get('status') not in ('complete', 'partial'):
        raise ValueError('Finish per-topic CSV export before making the summary')
    required = startup_requirements(manifest, startup_required_groups)
    output, schema_path = directory / 'summary.csv', directory / 'summary.schema.json'
    if any(path.exists() or path.is_symlink() for path in (output, schema_path)):
        raise FileExistsError('Summary already exists; original files were left untouched')
    specs = specifications()
    contact_id = manifest.get('contact_segment_id', '')
    if contact_id != '':
        validate_contact_segment_id(contact_id)
    columns = ['source_time_ns', 'bag_receive_time_ns', 'elapsed_s', 'angle_frame_id']
    for prefix, _, convert, _, _, _ in specs:
        columns += [f'{prefix}.{name}' for name in convert({})]
        if prefix == 'wire_velocity':
            columns += list(angles({}))
    columns += ['tag_id0_to_id1.detection_guard', 'tag_id0_to_id1.detection_source_time_ns']
    for prefix, _, _, _, _, _ in specs:
        columns += [f'{prefix}.{name}' for name in (
            'matched', 'source_time_ns', 'bag_receive_time_ns', 'time_basis',
            'time_difference_ms', 'frame_id')]
    columns += ['contact_segment_id', *RELATIVE_COLUMNS]
    schema = dict(
        status='exporting', file=output.name, schema_file=schema_path.name, rows_written=0,
        source_manifest='manifest.json', source_export_status=manifest.get('status'),
        source_integrity_status=manifest.get('integrity', {}).get('status', 'not_checked'),
        anchor_topic=ANGLE_TOPIC, max_time_difference_ms=max_time_difference_ms,
        purpose='Timestamp-matched training source/inspection table; feature selection, '
                'later-gap handling and physical calibration remain training preparation tasks',
        columns=columns, invalid_anchor_timestamps=0, nonincreasing_anchor_timestamps=0,
        contact_segment_id=contact_id,
        groups={p: dict(topic=t, time_basis=b, exact_source_stamp_only=e, child_frame=c)
                for p, t, _, b, e, c in specs},
        startup=dict(
            enabled=bool(required), required_groups=list(required), required_columns=required,
            rows_skipped=0, skipped_by_group={group: 0 for group in required},
            ready=not required, first_written_source_time_ns=None,
            first_written_bag_receive_time_ns=None, first_written_elapsed_s=None,
            later_incomplete_rows=0),
        notes=[
            'Startup gating removes only the leading rows until all configured core '
            'signals are time-matched and finite in the same row. Tag/tip poses are '
            'not gates. Zero and negative signals are valid. Later missing rows remain '
            'blank/unmatched, never removed or filled. Original timestamps and elapsed_s '
            'origin are unchanged; schema.startup records the skipped prefix and start.',
            'contact_segment_id is the frozen operator experiment label, independent of '
            'sensor force; missing legacy labels stay blank (not automatically zero).',
            'relative_angle_1..18 select odd tilt / even pan from the same angle anchor '
            'in radians. Original pan/tilt columns remain unchanged; never select by magnitude.',
            'relative_angular_velocity_1_rad_s..18_rad_s select the same odd tilt / even '
            'pan axes from relative angular velocities in that anchor. Preserve sign and '
            'zero; no numerical differentiation or fallback to the other axis. All four '
            'original velocity arrays remain in the angle topic CSV and bag.',
            'Per-topic CSVs and bag are unchanged and authoritative; summary drops high-rate '
            'samples between angle anchors. Nearest association may use past OR future data '
            'and reuse a sample. No interpolation or unbounded hold-last.',
            'Summary complete means file generation succeeded, NOT lossless recording. '
            'Inspect source manifest integrity/errors before trusting a dataset.',
            '25 ms default matching for sensors/tag poses does not synchronize camera clocks '
            'or compensate filter/sensor latency. Blank means no close sample, not zero.',
            'When /detections CSV exists, the nearest detection frame gates the tag pose: '
            'both ID0/ID1 must be detected and pose stamp must match that frame exactly. '
            'Explicit empty/one-tag frames are NOT filled from neighboring valid poses. '
            'Without detections, guard=unavailable and only bounded pose-time association '
            'is possible; undetected image frames cannot be inferred.',
            'matched only describes temporal association, NOT sensor validity or calibration. '
            'Invalid numeric fields/array lengths are blank; original rows remain in topic CSV.',
            'Headerless wire topics use bag receive time against ANGLE RECEIVE time explicitly; '
            'not source-time aligned. Stamped streams with zero stamps are not matched.',
            'Tips and DH orientations require exact angle source stamps. fk_tip_tf retains '
            'FK position and orientation even if PointStamped is absent. Never relabel '
            'legacy headerless /tool_endeffector_pose with a fabricated source stamp.',
            'All pan/tilt relative AND absolute arrays have 18 physical-joint-indexed columns '
            'in rad, plus 18 active-axis relative angular velocities in rad/s. Relative odd joints '
            'are tilt, even are pan; inactive axis zero preserved. Absolute arrays are '
            'direction angles in hrm_base, NOT sums of relative joint angles.',
            'Motor and loadcell channels 1..4 preserve source order; motor positions are '
            'encoder counts. Other sensor units are unchanged; consult session metadata.',
            'Position XYZ is m. Quaternions retain source xyzw. Derived Euler degrees are '
            'extrinsic XYZ: Rz(yaw) Ry(pitch) Rx(roll), NOT pan/tilt; singularities/wrap apply.',
            'tag_id0_to_id1 is uncorrected ID1 in ID0; no tag-to-physical-base/tip mounting '
            'rotation/translation or frame relabeling. No 4x4 matrix redundancy added.',
            'estimated_tip is the length-constrained fitted boundary, not segment center. '
            'estimated_tip_orientation is filtered DH orientation of last link (center_19), '
            'not an independent depth-measured roll; no center position is used as tip.',
            'The unscaled estimated tip is excluded from this summary only; its original '
            'topic CSV and bag recording are unchanged.',
            'fts aligned components, if present, copy frozen-session derivation from source '
            'CSV only. No new force or torque calibration is applied here.',
            'F/T sensor tx/ty/tz are formatted to one decimal in CSV, including Kalman '
            'channels; full-precision serialized measurements remain in rosbag.',
            'Event topics, raw images/clouds, all intermediate TF/markers, settings and other '
            'diagnostics stay in per-topic CSV/bag; they are not repeated into this summary.',
            'Nonincreasing timestamps are skipped/reported, never sorted across clock resets. '
            'First-to-last rates and matching cannot establish true camera exposure timing.',
        ],
    )
    try:
        with ExitStack() as stack:
            anchor_rows = csv_rows(directory, manifest, ANGLE_TOPIC, stack)
            timelines = {p: Timeline(csv_rows(directory, manifest, t, stack, c), b)
                         for p, t, _, b, _, c in specs}
            detection_guard = bool(manifest.get('topics', {}).get('/detections', {}).get('file'))
            detections = Timeline(csv_rows(directory, manifest, '/detections', stack))
            stream = stack.enter_context(output.open('x', newline=''))
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            previous_ns = initial_ns = None
            for anchor in anchor_rows:
                source_ns = positive_ns(anchor.get('source_time_ns'))
                if source_ns is None:
                    schema['invalid_anchor_timestamps'] += 1
                    continue
                if previous_ns is not None and source_ns <= previous_ns:
                    schema['nonincreasing_anchor_timestamps'] += 1
                    continue
                previous_ns = source_ns
                if initial_ns is None:
                    initial_ns = source_ns
                row = dict(source_time_ns=source_ns,
                           bag_receive_time_ns=anchor.get('bag_receive_time_ns', ''),
                           elapsed_s=(source_ns - initial_ns) / 1e9,
                           angle_frame_id=anchor.get('header.frame_id', ''), **angles(anchor))
                row['contact_segment_id'] = contact_id
                row.update(relative_angles(
                    array_values(anchor, 'pan_relative', JOINT_COUNT),
                    array_values(anchor, 'tilt_relative', JOINT_COUNT)))
                for prefix, _, convert, basis, exact, _ in specs:
                    target = (source_ns if basis == 'source'
                              else positive_ns(anchor.get('bag_receive_time_ns')))
                    tolerance = 0 if exact else round(max_time_difference_ms * 1e6)
                    if prefix == 'tag_id0_to_id1' and detection_guard:
                        detected = detections.nearest(target, tolerance)
                        if detected is not None:
                            row['tag_id0_to_id1.detection_source_time_ns'] = detected[0]
                        if detected is not None and both_tags_detected(detected[1]):
                            row['tag_id0_to_id1.detection_guard'] = 'both_detected'
                            matched = timelines[prefix].nearest(detected[0], 0)
                        else:
                            row['tag_id0_to_id1.detection_guard'] = 'missing_or_not_both_detected'
                            matched = None
                            timelines[prefix].stats['unmatched'] += 1
                    else:
                        matched = timelines[prefix].nearest(target, tolerance)
                        if prefix == 'tag_id0_to_id1':
                            row['tag_id0_to_id1.detection_guard'] = 'unavailable'
                    row[f'{prefix}.matched'] = matched is not None
                    row[f'{prefix}.time_basis'] = basis
                    if matched is None:
                        continue
                    stamp, sample = matched
                    row.update({f'{prefix}.{key}': value for key, value in convert(sample).items()})
                    row.update({f'{prefix}.{key}': sample.get(key, '') for key in (
                        'source_time_ns', 'bag_receive_time_ns')})
                    row[f'{prefix}.time_difference_ms'] = (stamp - target) / 1e6
                    row[f'{prefix}.frame_id'] = sample.get('header.frame_id', '')
                startup = schema['startup']
                missing = missing_startup_groups(row, required)
                if not startup['ready'] and missing:
                    startup['rows_skipped'] += 1
                    for group in missing:
                        startup['skipped_by_group'][group] += 1
                    continue
                if startup['first_written_source_time_ns'] is None:
                    startup.update(
                        ready=True, first_written_source_time_ns=source_ns,
                        first_written_bag_receive_time_ns=positive_ns(
                            row['bag_receive_time_ns']),
                        first_written_elapsed_s=row['elapsed_s'])
                elif missing:
                    startup['later_incomplete_rows'] += 1
                writer.writerow(row)
                schema['rows_written'] += 1
            for prefix, timeline in timelines.items():
                schema['groups'][prefix]['association_counts'] = timeline.stats
            schema['detection_guard'] = dict(
                available=detection_guard, association_counts=detections.stats)
        schema['status'] = ('complete' if schema['rows_written'] else
                            'no_common_start' if schema['startup']['rows_skipped'] else
                            'no_angle_samples')
    except Exception as exc:
        schema.update(status='failed', error=str(exc))
        raise
    finally:
        with schema_path.open('x') as handle:
            json.dump(schema, handle, indent=2, ensure_ascii=False, allow_nan=False)
            handle.write('\n')
    return schema


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_directory', type=Path,
                        help='Existing topic CSV directory with manifest.json')
    parser.add_argument('--max-time-difference-ms', type=float, default=25.0)
    parser.add_argument('--startup-required-groups', nargs='*', default=None,
                        help='Override startup core groups; no values disables the gate. '
                             'Default: frozen session config, or no gate for legacy sessions.')
    options = parser.parse_args(args)
    try:
        report = export_summary(options.csv_directory,
                                max_time_difference_ms=options.max_time_difference_ms,
                                startup_required_groups=options.startup_required_groups)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'Summary export failed: {exc}\n')
    print(f"Summary {report['status']}: {options.csv_directory / report['file']} "
          f"({report['rows_written']} rows)")
    return 2 if report['status'] == 'no_common_start' else 0


if __name__ == '__main__':
    raise SystemExit(main())
