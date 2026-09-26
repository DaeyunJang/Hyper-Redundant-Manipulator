#!/usr/bin/env python3
"""Offline, no-fill tip CSV export and diagnostics from a finalized rosbag.

No ROS node is initialized and no camera, motor, parameter, or service is used.
Tag mounting transforms are intentionally NOT guessed. ID0 and hrm_base poses
cannot be subtracted as a physical accuracy measurement without calibration.
"""

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

import numpy as np


POINT_TOPICS = {
    '/estimated_tip_position': 'estimated_tip_base',
    '/estimated_tip_position_unscaled': 'estimated_tip_unscaled_base',
    '/kinematics/fk_tip_position': 'fk_tip_direct',
}
POSE_TOPIC = '/apriltag/tag1_in_tag0/pose'
MARKER_TOPIC = '/estimated_segment_reconstruction_markers'
ANGLE_TOPIC = '/estimated_segment_angle'
SELECTED_TOPICS = set(POINT_TOPICS) | {
    POSE_TOPIC, MARKER_TOPIC, ANGLE_TOPIC, '/tf', '/detections'}
TIME_FIELDS = ['source_time_ns', 'bag_receive_time_ns', 'frame_id', 'child_frame_id',
               'provenance']
POSITION_FIELDS = ['x_m', 'y_m', 'z_m', 'x_mm', 'y_mm', 'z_mm', 'origin_distance_mm']
ORIENTATION_FIELDS = ['qx', 'qy', 'qz', 'qw', 'roll_xyz_deg', 'pitch_xyz_deg',
                      'yaw_xyz_deg', 'orientation_change_from_first_deg']
TANGENT_FIELDS = ['direction_x', 'direction_y', 'direction_z',
                  'direction_azimuth_deg', 'direction_elevation_deg']
ANGLE_FIELDS = ['pan_relative', 'tilt_relative', 'pan_absolute', 'tilt_absolute']
CSV_SCHEMAS = {
    'tag_tip_id0': TIME_FIELDS + POSITION_FIELDS + ORIENTATION_FIELDS,
    'estimated_tip_base': TIME_FIELDS + POSITION_FIELDS,
    'estimated_tip_unscaled_base': TIME_FIELDS + POSITION_FIELDS,
    'fk_tip_base': TIME_FIELDS + POSITION_FIELDS + ORIENTATION_FIELDS
    + ['orientation_provenance'],
    'estimated_tip_orientation_base': TIME_FIELDS + ORIENTATION_FIELDS,
    'estimated_boundary_tangent_base': TIME_FIELDS + POSITION_FIELDS + TANGENT_FIELDS,
    'joint_angles': TIME_FIELDS + ANGLE_FIELDS,
}


def field(value, key, default=None):
    return value.get(key, default) if isinstance(value, dict) else getattr(value, key, default)


def xyz(value):
    result = np.array([float(field(value, axis)) for axis in 'xyz'])
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite XYZ')
    return result


def stamped(header, received_ns, provenance, child=''):
    stamp = field(header, 'stamp')
    sec, nanosec = int(field(stamp, 'sec')), int(field(stamp, 'nanosec'))
    source = sec * 1_000_000_000 + nanosec
    frame = field(header, 'frame_id')
    if source <= 0 or not 0 <= nanosec < 1_000_000_000 or not frame:
        raise ValueError('Invalid source timestamp or frame')
    return dict(source_time_ns=source, bag_receive_time_ns=int(received_ns),
                frame_id=frame, child_frame_id=child, provenance=provenance)


def position_columns(point):
    vector = xyz(point)
    result = {f'{axis}_m': float(value) for axis, value in zip('xyz', vector)}
    result.update({f'{axis}_mm': float(1000 * value) for axis, value in zip('xyz', vector)})
    result['origin_distance_mm'] = float(np.linalg.norm(vector) * 1000)
    return result


def quaternion_columns(orientation):
    quaternion = np.array([float(field(orientation, axis)) for axis in 'xyzw'])
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or norm <= 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = quaternion / norm
    # Extrinsic xyz / conventional roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
    # These are NOT alternating DH pan/tilt joint angles.
    result = dict(zip(('qx', 'qy', 'qz', 'qw'), map(float, (x, y, z, w))))
    result.update(
        roll_xyz_deg=math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
        pitch_xyz_deg=math.degrees(math.asin(float(np.clip(2 * (w * y - z * x), -1, 1)))),
        yaw_xyz_deg=math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
    )
    return result


def quaternion(row):
    return np.array([row[name] for name in ('qx', 'qy', 'qz', 'qw')])


def rotation_change_deg(first, current):
    # abs(dot) treats q and -q as the same rotation.
    return math.degrees(2 * math.acos(float(np.clip(abs(np.dot(first, current)), 0, 1))))


def rotation_x_direction(row):
    x, y, z, w = quaternion(row)
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])


class CaptureData:
    def __init__(self):
        self.rows = defaultdict(list)
        self.seen = defaultdict(set)
        self.counts = Counter()
        self.detections = Counter()
        self.detected_ids = Counter()

    def add(self, channel, row):
        key = (row['frame_id'], row['child_frame_id'], row['source_time_ns'])
        if key in self.seen[channel]:
            self.counts[f'{channel}:duplicate_source_stamp'] += 1
            return
        self.seen[channel].add(key)
        self.rows[channel].append(row)

    def guarded(self, label, action):
        try:
            action()
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
            self.counts[f'{label}:invalid_sample'] += 1

    def consume(self, topic, message, received_ns):
        self.counts[f'{topic}:received_messages'] += 1
        if topic in POINT_TOPICS:
            self.guarded(topic, lambda: self.add(POINT_TOPICS[topic], dict(
                **stamped(field(message, 'header'), received_ns, topic),
                **position_columns(field(message, 'point')))))
        elif topic == POSE_TOPIC:
            def pose():
                value = field(message, 'pose')
                self.add('tag_tip_id0', dict(
                    **stamped(field(message, 'header'), received_ns, topic, 'ID1'),
                    **position_columns(field(value, 'position')),
                    **quaternion_columns(field(value, 'orientation'))))
            self.guarded(topic, pose)
        elif topic == '/tf':
            for transform in field(message, 'transforms', []):
                child = field(transform, 'child_frame_id')
                if child not in ('hrm_fk_tip', 'estimated_segment_center_19'):
                    continue

                def pose_tf(transform=transform, child=child):
                    value = field(transform, 'transform')
                    row = dict(**stamped(field(transform, 'header'), received_ns,
                                         f'/tf:{child}', child),
                               **quaternion_columns(field(value, 'rotation')))
                    if child == 'hrm_fk_tip':
                        row.update(position_columns(field(value, 'translation')))
                        self.add('fk_tip_tf', row)
                    else:
                        # Preserve orientation ONLY: this frame origin is a CENTER,
                        # not the final boundary tip. Never export its position here.
                        self.add('estimated_tip_orientation_base', row)
                self.guarded(f'/tf:{child}', pose_tf)
        elif topic == MARKER_TOPIC:
            for marker in field(message, 'markers', []):
                if (field(marker, 'ns') != 'curve_tangents' or field(marker, 'id') != 119
                        or field(marker, 'action', 0) != 0):
                    continue

                def tangent(marker=marker):
                    points = field(marker, 'points', [])
                    if len(points) != 2:
                        raise ValueError('Arrow must have start/end points')
                    direction = xyz(points[1]) - xyz(points[0])
                    norm = np.linalg.norm(direction)
                    if norm <= 1e-12:
                        raise ValueError('Zero tangent')
                    direction /= norm
                    row = dict(**stamped(field(marker, 'header'), received_ns,
                                         f'{MARKER_TOPIC}:curve_tangents/119'),
                               **position_columns(points[0]))
                    row.update(dict(zip(('direction_x', 'direction_y', 'direction_z'),
                                        map(float, direction))))
                    row.update(direction_azimuth_deg=math.degrees(math.atan2(
                        direction[1], direction[0])),
                        direction_elevation_deg=math.degrees(math.atan2(
                            direction[2], math.hypot(direction[0], direction[1]))))
                    self.add('estimated_boundary_tangent_base', row)
                self.guarded(MARKER_TOPIC, tangent)
        elif topic == ANGLE_TOPIC:
            def angles():
                row = stamped(field(message, 'header'), received_ns, topic)
                for name in ANGLE_FIELDS:
                    values = np.asarray(field(message, name), dtype=float)
                    if values.ndim != 1 or len(values) != 18 or not np.all(np.isfinite(values)):
                        raise ValueError('Expected 18 finite angles per array')
                    row[name] = json.dumps(values.tolist())
                self.add('joint_angles', row)
            self.guarded(topic, angles)
        elif topic == '/detections':
            def detections():
                row = stamped(field(message, 'header'), received_ns, topic)
                key = (row['frame_id'], row['source_time_ns'])
                if key in self.seen['detections']:
                    self.counts['detections:duplicate_source_stamp'] += 1
                    return
                self.seen['detections'].add(key)
                ids = set()
                for detection in field(message, 'detections', []):
                    value = field(detection, 'id')
                    ids.update(int(item) for item in value) if isinstance(
                        value, (tuple, list)) else ids.add(int(value))
                self.detections['frames'] += 1
                self.detections['empty_frames'] += int(not ids)
                self.detections['both_0_and_1_frames'] += int({0, 1} <= ids)
                self.detected_ids.update(ids)
            self.guarded(topic, detections)

    def finalized(self):
        result = {name: sorted((dict(row) for row in rows),
                               key=lambda row: row['source_time_ns'])
                  for name, rows in self.rows.items()}
        if result.get('fk_tip_direct'):
            fk = result['fk_tip_direct']
            transforms = {(row['frame_id'], row['source_time_ns']): row
                          for row in result.get('fk_tip_tf', [])}
            for row in fk:
                transform = transforms.get((row['frame_id'], row['source_time_ns']))
                if transform:
                    for name in ORIENTATION_FIELDS[:-1]:
                        row[name] = transform[name]
                    row['orientation_provenance'] = transform['provenance']
                row['child_frame_id'] = 'hrm_fk_tip'
        else:
            fk = result.get('fk_tip_tf', [])
            for row in fk:
                row['orientation_provenance'] = row['provenance']
        result['fk_tip_base'] = fk
        for name in CSV_SCHEMAS:
            rows = result.setdefault(name, [])
            initial_quaternions = {}
            for row in rows:
                if 'qw' in row:
                    key = (row['frame_id'], row['child_frame_id'])
                    first = initial_quaternions.setdefault(key, quaternion(row))
                    row['orientation_change_from_first_deg'] = rotation_change_deg(
                        first, quaternion(row))
        return result


def nearest_pairs(left, right, tolerance_ns):
    """Left-driven nearest source-time samples in hrm_base only; never fill gaps.

    Exact stamps win. A right sample may serve two nearby left samples, which is
    reported in the summary; this is association, not resampling/interpolation.
    """
    if tolerance_ns < 0:
        raise ValueError('Matching tolerance must not be negative')
    right = sorted((row for row in right if row['frame_id'] == 'hrm_base'),
                   key=lambda row: row['source_time_ns'])
    stamps = [row['source_time_ns'] for row in right]
    for row in left:
        if row['frame_id'] != 'hrm_base' or not right:
            continue
        index = int(np.searchsorted(stamps, row['source_time_ns']))
        candidates = [item for item in (index - 1, index) if 0 <= item < len(right)]
        best = min(candidates, key=lambda item: abs(stamps[item] - row['source_time_ns']))
        if abs(stamps[best] - row['source_time_ns']) <= tolerance_ns:
            yield row, right[best]


def numerical_summary(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return dict(mean=float(np.mean(values)), std=float(np.std(values)),
                min=float(np.min(values)), max=float(np.max(values)),
                rms=float(np.sqrt(np.mean(values**2))))


def series_summary(rows):
    result = dict(samples=len(rows), frames=sorted({row['frame_id'] for row in rows}),
                  source_rate_hz=None)
    if not rows:
        return result
    stamps = [row['source_time_ns'] for row in rows]
    span = (max(stamps) - min(stamps)) / 1e9
    result.update(first_source_time_ns=min(stamps), last_source_time_ns=max(stamps),
                  source_duration_sec=span)
    if span > 0:
        result['source_rate_hz'] = (len(rows) - 1) / span
    # Mixing frames would make the following statistics meaningless.
    if len(result['frames']) != 1:
        result['statistics_unavailable_reason'] = 'Mixed coordinate frames'
        return result
    if all('x_m' in row for row in rows):
        points = np.array([[row[f'{axis}_mm'] for axis in 'xyz'] for row in rows])
        result.update(position_mean_mm=np.mean(points, axis=0).tolist(),
                      position_std_mm=np.std(points, axis=0).tolist(),
                      origin_distance_mm=numerical_summary(np.linalg.norm(points, axis=1)),
                      consecutive_position_step_mm=numerical_summary(
                          np.linalg.norm(np.diff(points, axis=0), axis=1)))
    orientations = [row['orientation_change_from_first_deg']
                    for row in rows if 'orientation_change_from_first_deg' in row]
    result['orientation_change_from_first_deg'] = numerical_summary(orientations)
    quaternion_rows = [quaternion(row) for row in rows if 'qw' in row]
    if quaternion_rows:
        # Sign-invariant chordal quaternion mean, not an average of wrapped Euler
        # angles. Small static scatter is summarized geodesically around it.
        quaternions = np.asarray(quaternion_rows)
        _, eigenvectors = np.linalg.eigh(quaternions.T @ quaternions)
        mean = eigenvectors[:, -1]
        if mean[-1] < 0:
            mean = -mean
        result['orientation_mean_quaternion_xyzw'] = mean.tolist()
        result['orientation_geodesic_from_mean_deg'] = numerical_summary(
            [rotation_change_deg(mean, value) for value in quaternions])
    return result


def comparisons(series, tolerance_ns):
    rows, tangent_rows = [], []
    for depth, fk in nearest_pairs(
            series['estimated_tip_base'], series['fk_tip_base'], tolerance_ns):
        delta = np.array([depth[f'{axis}_mm'] - fk[f'{axis}_mm'] for axis in 'xyz'])
        row = dict(estimated_source_time_ns=depth['source_time_ns'],
                   fk_source_time_ns=fk['source_time_ns'], frame_id='hrm_base',
                   source_skew_ms=(fk['source_time_ns'] - depth['source_time_ns']) / 1e6,
                   difference_norm_mm=float(np.linalg.norm(delta)))
        row.update({f'estimated_minus_fk_{axis}_mm': float(value)
                    for axis, value in zip('xyz', delta)})
        rows.append(row)
    for tangent, fk in nearest_pairs(
            series['estimated_boundary_tangent_base'],
            [row for row in series['fk_tip_base'] if 'qw' in row], tolerance_ns):
        direction = np.array([tangent[f'direction_{axis}'] for axis in 'xyz'])
        angle = math.degrees(math.acos(float(np.clip(
            np.dot(direction, rotation_x_direction(fk)), -1, 1))))
        tangent_rows.append(dict(tangent_source_time_ns=tangent['source_time_ns'],
                                 fk_source_time_ns=fk['source_time_ns'],
                                 source_skew_ms=(fk['source_time_ns']
                                                 - tangent['source_time_ns']) / 1e6,
                                 tangent_vs_fk_x_angle_deg=angle, frame_id='hrm_base'))
    return rows, tangent_rows


NOTES = [
    'All positions are metres (also explicit mm columns), quaternions are xyzw.',
    'Euler columns use extrinsic xyz: Rz(yaw) Ry(pitch) Rx(roll), in degrees. '
    'They are not pan/tilt joint angles; quaternion/geodesic values avoid Euler wrapping.',
    'tag_tip_id0 is ID1 origin in ID0, NOT physical tip in hrm_base. '
    'Needed calibration: B_T_tip = B_T_ID0 @ ID0_T_ID1 @ ID1_T_tip.',
    'Tag-to-estimated absolute position/orientation error is unavailable without both '
    'mounting transforms. No fitted/assumed offsets are applied.',
    'estimated_tip_base is the final fitted boundary after known-length scaling; '
    'unscaled is its fitted endpoint before scaling, NOT raw depth ground truth.',
    'estimated_tip_orientation_base is the filtered DH orientation at '
    'estimated_segment_center_19. Its center position is deliberately omitted.',
    'Estimated center orientation and FK tip orientation use the same filtered joint '
    'angles, so agreement is by construction, not independent angular validation.',
    'Boundary tangent comes from ARROW endpoints curve_tangents/119, not the '
    'identity marker.pose.orientation. A tangent does not measure axial roll.',
    'Source-time nearest matching uses hrm_base only, no interpolation or missing-value '
    'fill; exact source stamps take priority. Right samples may be reused.',
    'Cross-camera source clocks must be verified before future dynamic comparison. '
    'This report does not compare tag and depth trajectories as calibrated error.',
    'Orientation change magnitudes from each series first sample are invariant to '
    'constant mounting rotations, but are not absolute cross-frame orientation errors.',
    'Angular scatter is also reported as geodesic distance from the sign-invariant '
    'chordal quaternion mean, not an average or difference of wrapped Euler angles.',
    'Standard deviation is jitter only if the robot was stationary. Otherwise it '
    'includes real motion. Origin distances are descriptive, not accuracy errors.',
]


def write_csv(path, rows, columns):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def write_analysis(data, output, tolerance_ns=25_000_000):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    series = data.finalized()
    for name, columns in CSV_SCHEMAS.items():
        write_csv(output / f'{name}.csv', series[name], columns)
    position_pairs, tangent_pairs = comparisons(series, tolerance_ns)
    write_csv(output / 'depth_fk_comparison.csv', position_pairs,
              ['estimated_source_time_ns', 'fk_source_time_ns', 'frame_id', 'source_skew_ms',
               'estimated_minus_fk_x_mm', 'estimated_minus_fk_y_mm',
               'estimated_minus_fk_z_mm', 'difference_norm_mm'])
    write_csv(output / 'boundary_tangent_fk_comparison.csv', tangent_pairs,
              ['tangent_source_time_ns', 'fk_source_time_ns', 'frame_id', 'source_skew_ms',
               'tangent_vs_fk_x_angle_deg'])
    unique_fk = {row['fk_source_time_ns'] for row in position_pairs}
    summary = dict(
        source_tolerance_ms=tolerance_ns / 1e6,
        streams={name: series_summary(series[name]) for name in CSV_SCHEMAS},
        counters=dict(data.counts), detections=dict(data.detections),
        detected_id_frame_counts=dict(data.detected_ids),
        fk_position_source=('/kinematics/fk_tip_position'
                            if data.rows.get('fk_tip_direct') else '/tf:hrm_fk_tip'),
        calibrated_tag_position_error_mm=None, calibrated_tag_orientation_error_deg=None,
        unavailable_reason='Unknown B_T_ID0 and ID1_T_tip mounting transforms',
        depth_fk=dict(matched=len(position_pairs), unique_fk_samples=len(unique_fk),
                      exact_source_matches=sum(row['source_skew_ms'] == 0
                                               for row in position_pairs),
                      unmatched_estimated_samples=len(series['estimated_tip_base'])
                      - len(position_pairs),
                      difference_norm_mm=numerical_summary(
                          [row['difference_norm_mm'] for row in position_pairs]),
                      source_skew_ms=numerical_summary(
                          [row['source_skew_ms'] for row in position_pairs])),
        boundary_tangent_fk=dict(matched=len(tangent_pairs), angle_deg=numerical_summary(
            [row['tangent_vs_fk_x_angle_deg'] for row in tangent_pairs])),
        notes=NOTES,
    )
    (output / 'summary.json').write_text(
        json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    lines = ['# Tip capture diagnostic (uncompensated)', '']
    for name, stats in summary['streams'].items():
        rate = stats['source_rate_hz']
        lines.append(f"- {name}: {stats['samples']} samples; "
                     + (f'{rate:.2f} Hz' if rate is not None else 'rate unavailable'))
    lines += ['', f"Depth/FK matched: {len(position_pairs)} (tolerance "
              f"{tolerance_ns / 1e6:g} ms); summary.json contains descriptive statistics.",
              '', 'Calibrated tag position/orientation error: **unavailable**.', '',
              '## Interpretation', ''] + ['- ' + note for note in NOTES]
    (output / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summary


def read_bag(source):
    """Deserialize only relevant small topics; ROS imports are offline-only."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    source = Path(source).expanduser().resolve()
    bag = source / 'bag' if (source / 'bag').is_dir() else source
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('Use a finalized session/bag directory containing metadata.yaml')
    session_file = (source if source != bag else bag.parent) / 'session.json'
    if session_file.is_file() and json.loads(session_file.read_text()).get('state') in (
            'starting', 'recording', 'stopping'):
        raise ValueError('Stop and finalize the recording before analysis')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = sorted(SELECTED_TOPICS & type_names.keys())
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    message_types = {topic: get_message(type_names[topic]) for topic in selected}
    data = CaptureData()
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        data.consume(topic, message, received_ns)
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path,
                        help='New output directory; existing directories are never overwritten')
    parser.add_argument('--tolerance-ms', type=float, default=25.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.tolerance_ms) or not 0 <= args.tolerance_ms <= 25.:
        parser.error('Source matching tolerance must be finite and in 0..25 ms')
    output = args.output or args.session / 'analysis'
    if output.exists() or output.is_symlink():
        parser.error(f'Output already exists: {output}')
    try:
        data = read_bag(args.session)
        summary = write_analysis(data, output, round(args.tolerance_ms * 1e6))
    except (ValueError, OSError, RuntimeError) as error:
        parser.exit(2, f'Analysis failed: {error}\n')
    print(json.dumps(dict(output=str(output.resolve()),
                          samples={name: stats['samples']
                                   for name, stats in summary['streams'].items()},
                          depth_fk=summary['depth_fk']), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
