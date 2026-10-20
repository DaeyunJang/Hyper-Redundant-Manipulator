"""Export finalized rosbag sessions without replaying or publishing any message.

Each original topic keeps its own sampling times and lossless JSON array cells.
An additional inspection summary associates nearby samples with explicit skew;
it does not replace original files or perform training preprocessing.
"""

import argparse
from collections.abc import Mapping
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

from record_pkg.experiment_labels import relative_angles, session_contact_segment_id


BINARY_TYPES = {
    'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/PointCloud2',
}
SKIP_TYPES = {'visualization_msgs/msg/Marker', 'visualization_msgs/msg/MarkerArray'}
SCALAR_PACKAGES = {
    'std_msgs', 'geometry_msgs', 'custom_interfaces', 'apriltag_msgs',
    'apriltag_ros', 'rcl_interfaces',
}


def json_value(value):
    """Keep arrays/nested records JSON-compatible, retaining nonfinite values."""
    if isinstance(value, Mapping):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (bytes, bytearray, memoryview)):
        data = bytes(value)
        return data[0] if len(data) == 1 else list(data)
    if isinstance(value, (list, tuple)) or hasattr(value, 'tolist'):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def compact_json(value):
    return json.dumps(json_value(value), ensure_ascii=False, allow_nan=False,
                      separators=(',', ':'))


def flatten(record, prefix=''):
    """Flatten objects; arrays remain one lossless, stable-schema JSON cell."""
    result = {}
    for key, value in record.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, Mapping):
            result.update(flatten(value, name))
        elif (isinstance(value, (list, tuple, bytes, bytearray, memoryview)) or
                hasattr(value, 'tolist')):
            result[name] = compact_json(value)
        else:
            result[name] = value
    return result


def pose_euler(record):
    """Derived extrinsic XYZ Euler angles; the original quaternion is retained."""
    quaternion = record['pose']['orientation']
    x, y, z, w = [float(quaternion[key]) for key in ('x', 'y', 'z', 'w')]
    norm = math.hypot(x, y, z, w)
    result = dict(derived_quaternion_valid=False, derived_roll_rad='',
                  derived_pitch_rad='', derived_yaw_rad='')
    if not math.isfinite(norm) or norm < 1e-12:
        return result
    x, y, z, w = [value / norm for value in (x, y, z, w)]
    result.update(
        derived_quaternion_valid=True,
        derived_roll_rad=math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        derived_pitch_rad=math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        derived_yaw_rad=math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )
    return result


def message_rows(record, type_name, received_ns, message_index, *, topic=None,
                 force_alignment=None, contact_segment_id=''):
    """One sample per row; TF is expanded into individually stamped transforms."""
    records = record['transforms'] if type_name == 'tf2_msgs/msg/TFMessage' else [record]
    for row_index, item in enumerate(records):
        source_stamp = item.get('header', {}).get('stamp')
        row = {
            'bag_receive_time_ns': received_ns,
            'source_time_ns': (source_stamp['sec'] * 10**9 + source_stamp['nanosec']
                               if source_stamp is not None else ''),
            'bag_message_index': message_index,
            'item_index': row_index,
        }
        if type_name in BINARY_TYPES:
            item = {key: value for key, value in item.items() if key != 'data'}
            row['payload_bytes'] = len(record['data'])
        row.update(flatten(item))
        # CSV presentation only: keep serialized rosbag values untouched.
        if (type_name == 'geometry_msgs/msg/WrenchStamped' and
                topic in ('/fts_data', '/fts_data_kalman_filter')):
            for axis in 'xyz':
                key = f'wrench.torque.{axis}'
                if key in row:
                    row[key] = f'{float(row[key]):.1f}'
        if type_name == 'geometry_msgs/msg/PoseStamped':
            row.update(pose_euler(item))
        if (force_alignment and force_alignment['enabled'] and
                type_name == 'geometry_msgs/msg/WrenchStamped' and
                topic in ('/fts_data', '/fts_data_kalman_filter')):
            from record_pkg.force_alignment import aligned_force_columns
            row.update(aligned_force_columns(item.get('wrench', {}).get('force', {}),
                                             force_alignment))
        row['contact_segment_id'] = contact_segment_id
        if topic == '/estimated_segment_angle':
            row.update(relative_angles(item.get('pan_relative'), item.get('tilt_relative')))
        yield row


def topic_policy(type_name):
    if type_name in SKIP_TYPES:
        return 'skipped_visualization'
    if type_name in BINARY_TYPES:
        return 'metadata_only'
    if (type_name in ('tf2_msgs/msg/TFMessage', 'sensor_msgs/msg/CameraInfo') or
            type_name.split('/')[0] in SCALAR_PACKAGES):
        return 'full_fields'
    return 'skipped_unsupported_type'


def topic_filename(topic):
    readable = re.sub(r'[^A-Za-z0-9_]+', '__', topic.strip('/'))[:140] or 'root'
    digest = hashlib.sha256(topic.encode()).hexdigest()[:12]
    return f'{readable}__{digest}.csv'


def save_manifest(path, manifest):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(json_value(manifest), indent=2, ensure_ascii=False,
                                    allow_nan=False) + '\n')
    temporary.replace(path)


def resolve_paths(source, output=None):
    """Require finalized directory metadata, which also supports split bags."""
    source = Path(source).expanduser().resolve()
    bag = source / 'bag' if (source / 'bag').is_dir() else source
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('Use a finalized session or bag directory containing metadata.yaml; '
                         'do not pass an individual .db3 file.')
    session_directory = source if bag != source else bag.parent
    session_path = session_directory / 'session.json'
    session = json.loads(session_path.read_text()) if session_path.is_file() else {}
    if session.get('state') in ('starting', 'recording', 'stopping'):
        raise ValueError('The session is still recording or flushing; stop it before exporting.')
    if output is None:
        output = (session_directory / 'csv'
                  if session_path.is_file() or bag != source else bag / 'csv')
    output = Path(output).expanduser().resolve()
    return bag, output, session, session_path if session_path.is_file() else None


def open_reader(bag):
    """Lazy ROS imports keep pure metadata/schema tests usable without ROS."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def message_record(message, exclude=()):
    """Convert typed ROS fields, keeping octets numeric instead of characters.

    Humble's message_to_ordereddict can turn ParameterValue.byte_array_value
    into characters; other builds expose one-byte bytes. Both mean integers
    0..255, not text. Reading ROS field types avoids confusing real strings.
    """
    def convert(value):
        if hasattr(value, 'get_fields_and_field_types'):
            return message_record(value)
        if isinstance(value, (list, tuple)):
            return [convert(item) for item in value]
        if hasattr(value, 'tolist'):
            return value.tolist()
        return value

    def octet(value):
        return ord(value) if isinstance(value, (bytes, str)) else int(value)

    record = {}
    for name, field_type in message.get_fields_and_field_types().items():
        if name in exclude:
            continue
        value = getattr(message, name)
        if field_type == 'octet':
            record[name] = octet(value)
        elif field_type.startswith(('sequence<octet', 'octet[')):
            record[name] = [octet(item) for item in value]
        else:
            record[name] = convert(value)
    return record


def make_decoder(type_name):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    message_type = get_message(type_name)

    def decode(serialized):
        message = deserialize_message(serialized, message_type)
        # Converting Image/PointCloud2 data to a Python list is unnecessarily
        # expensive. Extract only descriptive fields before mapping conversion.
        if type_name in BINARY_TYPES:
            result = message_record(message, exclude=('data',))
            # A memoryview avoids copying the payload; rows only inspect len().
            result['data'] = memoryview(message.data)
            return result
        return message_record(message)
    return decode


def session_force_alignment(session):
    """Use only the frozen operator selection, never current config or TF."""
    from record_pkg.force_alignment import create_alignment, validate_alignment
    snapshot = session.get('snapshot', {})
    if 'force_alignment' not in snapshot:
        return create_alignment(enabled=False), 'absent_session_snapshot_raw_only'
    return (validate_alignment(snapshot['force_alignment']),
            'session.json:snapshot.force_alignment')


def export_session(source, output=None):
    """Return a completed/partial manifest; never overwrite an existing export."""
    bag, directory, session, session_path = resolve_paths(source, output)
    if directory.exists():
        raise FileExistsError(f'Export already exists: {directory}; use --output NEW_DIR.')
    # Reject inconsistent or malformed enabled metadata before creating output;
    # silently treating it as identity would mislabel force targets for training.
    force_alignment, alignment_source = session_force_alignment(session)
    contact_segment_id = session_contact_segment_id(session)
    reader = open_reader(bag)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    directory.mkdir(parents=True, exist_ok=False)
    manifest = {
        'schema_version': 1, 'status': 'exporting',
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'bag_directory': str(bag), 'csv_directory': str(directory),
        'session_metadata_source': str(session_path) if session_path else None,
        'contact_segment_id': contact_segment_id,
        'force_alignment': {
            **force_alignment,
            'provenance': alignment_source,
            'original_ros_fields_preserved': True,
            'torque_transformed': False,
            'time_or_units_changed': False,
            'derived_columns': (['aligned_fx', 'aligned_fy', 'aligned_fz',
                                 'aligned_frame_id', 'aligned_force_valid']
                                if force_alignment['enabled'] else []),
            'frame_note': 'source_frame is the operator calibration label, not a TF lookup. '
                          'Original header.frame_id remains unchanged.',
            'validity_scope': 'aligned_force_valid checks numeric transformation only. '
                              'It does not certify CAN/sensor health, calibration or a real '
                              'force measurement; finite zero inputs are valid numerically.',
        },
        'conventions': {
            'experiment_label': 'contact_segment_id is frozen from session.json snapshot, '
                                'never inferred from force. Missing legacy labels are blank.',
            'relative_angles': 'relative_angle_1..18 select same-message tilt_relative on '
                               'odd 1-based joints and pan_relative on even joints; radians, '
                               'sign and true zero preserved; no fallback to another axis.',
            'sensor_torque_precision': '/fts_data and /fts_data_kalman_filter wrench.torque '
                                       'x/y/z are formatted to one decimal in CSV only. '
                                       'Original serialized values remain in rosbag.',
            'time': 'bag_receive_time_ns is receive time; source_time_ns is the original header '
                    'stamp (including zero), blank if headerless. No synchronization/resampling.',
            'arrays': 'All array fields are compact JSON cells preserving every entry and order.',
            'bytes': 'ROS octet arrays (including parameter byte arrays) use JSON integers 0..255.',
            'nonfinite': 'NaN/infinity in JSON cells use strings nan, inf, -inf; no zero fill.',
            'tf': 'One transform per row; each uses its own source stamp, parent and child frame.',
            'pose': 'Original quaternion preserved. Derived RPY radians use extrinsic XYZ '
                    '(Rz(yaw) Ry(pitch) Rx(roll)); Euler angles are nonunique at singularities.',
            'units': 'Original message units/frames unchanged; consult session metadata.',
            'force_alignment': 'Only enabled per-session snapshot settings add aligned force '
                               'columns to /fts_data and /fts_data_kalman_filter WrenchStamped. '
                               'Nonfinite/invalid force keeps the original row and blank derived '
                               'components with aligned_force_valid=false. No torque conversion.',
            'payloads': 'Payloads present in legacy bags stay there. New crop-only profiles '
                        'keep PNGs/index in images/hrm_crop and only numeric topics in bag; '
                        'see image_integrity and the frozen recording_config.json.',
            'missing': 'not_recorded means no stored topic, no_messages means stored topic with '
                       'zero messages. Neither is fabricated as a zero-valued sample.',
        },
        'topics': {},
    }
    for topic in sorted(set(topic_types) | set(session.get('selected_topics', []))):
        type_name = topic_types.get(topic)
        manifest['topics'][topic] = {
            'type': type_name, 'policy': topic_policy(type_name) if type_name else 'not_recorded',
            'status': 'pending' if type_name else 'not_recorded',
            'messages_seen': 0, 'rows_written': 0, 'errors': 0,
        }
    manifest_path = directory / 'manifest.json'
    save_manifest(manifest_path, manifest)
    writers, streams, decoders = {}, {}, {}
    fatal = None
    try:
        from record_pkg.bag_integrity import audit_bag
        config_root = session_path.parent if session_path else bag.parent
        config_path = config_root / 'recording_config.json'
        config = json.loads(config_path.read_text()) if config_path.is_file() else {}
        from record_pkg.image_archive_audit import audit_image_archive
        from record_pkg.depth_archive_audit import audit_depth_archive
        manifest['image_integrity'] = audit_image_archive(config_root, config, session)
        manifest['depth_integrity'] = audit_depth_archive(config_root, config, session)
        manifest['integrity'] = audit_bag(
            bag, config.get('required_topics', []),
            max_gap_sec=config.get('required_max_gap_sec', 2.0),
            selected_topics=session.get('selected_topics'),
            observation_start_ns=session.get('recording_ready_ros_ns'),
            observation_end_ns=session.get('stop_requested_ros_ns'))
        if session_path:
            with (directory / 'session_metadata.csv').open('x', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['key', 'value_json'])
                # Leave all leaf arrays as JSON, just as in per-topic files.

                def leaves(record, prefix=''):
                    for key, value in record.items():
                        name = f'{prefix}.{key}' if prefix else key
                        if isinstance(value, Mapping) and value:
                            yield from leaves(value, name)
                        else:
                            yield name, compact_json(value)
                writer.writerows(leaves(session))
            manifest['session_metadata_csv'] = 'session_metadata.csv'
        while reader.has_next():
            topic, serialized, received_ns = reader.read_next()
            entry = manifest['topics'][topic]
            entry['messages_seen'] += 1
            if entry['policy'].startswith('skipped_'):
                entry['status'] = entry['policy']
                continue
            if topic in decoders and decoders[topic] is None:
                entry['errors'] += 1
                continue
            if topic not in decoders:
                try:
                    decoders[topic] = make_decoder(entry['type'])
                except Exception as exc:
                    decoders[topic] = None
                    entry.update(status='unavailable_message_type', errors=1,
                                 first_error=str(exc))
                    continue
            try:
                record = decoders[topic](serialized)
                rows = message_rows(record, entry['type'], received_ns, entry['messages_seen'] - 1,
                                    topic=topic, force_alignment=force_alignment,
                                    contact_segment_id=contact_segment_id)
                for row in rows:
                    if topic not in writers:
                        filename = topic_filename(topic)
                        streams[topic] = (directory / filename).open('x', newline='')
                        writers[topic] = csv.DictWriter(streams[topic], fieldnames=list(row))
                        writers[topic].writeheader()
                        entry.update(file=filename, columns=list(row))
                    writers[topic].writerow(row)
                    entry['rows_written'] += 1
            except OSError:
                raise
            except Exception as exc:
                entry['errors'] += 1
                entry.setdefault('first_error', str(exc))
        for entry in manifest['topics'].values():
            if entry['status'] in ('not_recorded', 'unavailable_message_type'):
                continue
            if entry['errors']:
                entry['status'] = 'partial'
            elif entry['policy'].startswith('skipped_'):
                entry['status'] = entry['policy']
            elif not entry['messages_seen']:
                entry['status'] = 'no_messages'
            elif not entry['rows_written']:
                entry['status'] = 'no_items'
            else:
                entry['status'] = 'exported'
    except Exception as exc:
        fatal = exc
        manifest['fatal_error'] = str(exc)
    finally:
        for stream in streams.values():
            try:
                stream.close()
            except OSError as exc:
                fatal = exc
                manifest['fatal_error'] = str(exc)
        manifest['status'] = ('failed' if fatal else 'partial' if any(
            item['errors'] for item in manifest['topics'].values()) else 'complete')
        if manifest.get('image_integrity', {}).get('status') == 'error' and not fatal:
            manifest['status'] = 'partial'
        if manifest.get('depth_integrity', {}).get('status') == 'error' and not fatal:
            manifest['status'] = 'partial'
        manifest['completed_utc'] = datetime.now(timezone.utc).isoformat()
        save_manifest(manifest_path, manifest)
    if fatal is None:
        try:
            from record_pkg.summary_csv import export_summary
            settings = config.get('summary_csv', {})
            if settings.get('enabled', True):
                manifest['summary_csv'] = export_summary(
                    directory, manifest,
                    max_time_difference_ms=settings.get('max_time_difference_ms', 25.0),
                    startup_required_groups=settings.get('startup_required_groups', []))
                if manifest['summary_csv']['status'] == 'no_common_start':
                    # Preserve every original topic CSV but do not claim a usable
                    # training summary when required streams never overlap.
                    manifest['status'] = 'partial'
            else:
                manifest['summary_csv'] = {'status': 'disabled'}
        except Exception as exc:
            manifest['summary_csv'] = {'status': 'failed', 'error': str(exc)}
            manifest['status'] = 'partial'
        manifest['completed_utc'] = datetime.now(timezone.utc).isoformat()
        save_manifest(manifest_path, manifest)
    return manifest


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', help='Finalized session or bag directory (not .db3).')
    parser.add_argument('--output', help='New export directory; never overwrite existing data.')
    options = parser.parse_args(args)
    try:
        manifest = export_session(options.source, options.output)
    except Exception as exc:
        print(f'CSV export failed: {exc}', file=sys.stderr)
        return 2
    print(f"CSV export {manifest['status']}: {manifest['csv_directory']}")
    integrity = manifest.get('integrity', {})
    quality = integrity.get('status', 'not_checked')
    print(f'Dataset receive-time integrity: {quality} '
          '(CSV conversion success does not certify data quality).')
    missing = sorted(set(integrity.get('required_topics_without_messages', [])) |
                     set(integrity.get('required_topics_without_window_messages', [])))
    if missing:
        print('WARNING: required topics without samples: ' + ', '.join(missing))
    gaps = integrity.get('required_topics_with_receive_gaps', [])
    if gaps:
        print('WARNING: required topics with receive gaps: ' + ', '.join(gaps))
    for error in integrity.get('errors', []):
        print(f'Integrity error: {error}', file=sys.stderr)
    return 0 if manifest['status'] == 'complete' and quality != 'error' else 2


if __name__ == '__main__':
    sys.exit(main())
