"""Validate cropped depth file/index/calibration coverage after recording."""

import csv
import json
from pathlib import Path
import struct

import numpy as np

from record_pkg.depth_archive import validate_depth_metadata


def audit_depth_archive(directory, config, session):
    settings = config.get('depth_archive', {})
    if not settings.get('enabled', False):
        return {'status': 'disabled'}
    report = dict(status='error', directory='images/hrm_crop_depth', images_checked=0,
                  errors=[], warnings=[],
                  scope='File headers/dimensions/types, explicit units/calibration and '
                        'source gaps; not PNG pixel/CRC validation.')
    root = Path(directory) / 'images/hrm_crop_depth'
    try:
        start = session.get('recording_ready_ros_ns')
        end = session.get('stop_requested_ros_ns')
        for bound in (start, end):
            if bound is not None and (isinstance(bound, bool) or not isinstance(bound, int)):
                raise ValueError('Depth observation bounds must be integer ROS timestamps.')
        if start is not None and end is not None and start > end:
            raise ValueError('Depth observation start must not exceed stop time.')
        check_coverage = start is not None or end is not None
        window_count, first_received, last_received = 0, None, None
        manifest = json.loads((root / 'manifest.json').read_text())
        report['writer'] = manifest
        if (not manifest.get('done') or manifest.get('topic') != settings.get(
                'topic', '/estimated_segment_crop_depth/image_raw')):
            raise ValueError('Depth archive not finalized or topic differs.')
        expected_label = session.get('snapshot', {}).get('contact_segment_id')
        names, previous = set(), None
        max_gap = config.get('required_max_gap_sec', 2.0) * 1e9
        with (root / 'index.csv').open(newline='') as handle:
            for row in csv.DictReader(handle):
                name = row['filename']
                path = root / name
                if (Path(name).name != name or name in names
                        or path.resolve().parent != root.resolve()):
                    raise ValueError('Unsafe or duplicate depth filename.')
                names.add(name)
                calibration = dict(
                    schema_version=1, source_time_ns=int(row['source_time_ns']),
                    frame_id=row['frame_id'], encoding=row['input_encoding'],
                    width=int(row['width']), height=int(row['height']),
                    source_width=int(row['source_width']), source_height=int(row['source_height']),
                    depth_scale_m_per_unit=float(row['depth_scale_m_per_unit']),
                    intrinsics_reference='cropped_pixels',
                    roi={k: int(row[f'roi_{k}']) for k in ('x', 'y', 'width', 'height')},
                    **{k: json.loads(row[k]) for k in ('k', 'p', 'r', 'd')},
                    distortion_model=row['distortion_model'],
                )
                validate_depth_metadata(calibration)
                received = int(row['received_time_ns'])
                if received <= 0:
                    raise ValueError('Invalid depth receipt time.')
                if ((start is None or received >= start)
                        and (end is None or received <= end)):
                    window_count += 1
                    first_received = (received if first_received is None
                                      else min(first_received, received))
                    last_received = (received if last_received is None
                                     else max(last_received, received))
                if expected_label is not None and int(row['contact_segment_id']) != expected_label:
                    raise ValueError('Depth contact label differs from frozen label.')
                shape = (calibration['height'], calibration['width'])
                if row['input_encoding'] == '32FC1':
                    if path.suffix != '.npy':
                        raise ValueError('Float depth must use NPY.')
                    pixels = np.load(path, allow_pickle=False, mmap_mode='r')
                    if pixels.shape != shape or pixels.dtype != np.dtype('float32'):
                        raise ValueError('NPY depth shape/type differs from index.')
                    del pixels
                else:
                    with path.open('rb') as image:
                        header = image.read(29)
                    if (path.suffix != '.png' or len(header) != 29
                            or header[:8] != b'\x89PNG\r\n\x1a\n'
                            or header[12:16] != b'IHDR' or header[24:26] != bytes((16, 0))
                            or struct.unpack('>II', header[16:24]) != shape[::-1]):
                        raise ValueError('Depth PNG header/dimensions/type invalid.')
                source = calibration['source_time_ns']
                if previous is not None and source <= previous:
                    report['warnings'].append('Nonincreasing depth source timestamp.')
                if previous is not None and source - previous > max_gap:
                    report['warnings'].append('Depth source gap exceeded limit.')
                previous = source
                report['images_checked'] += 1
        files = {p.name for p in root.iterdir() if p.suffix in ('.png', '.npy')}
        if files != names or manifest.get('saved') != report['images_checked']:
            raise ValueError('Depth saved count/files/index differ.')
        for key in ('dropped', 'errors', 'finalization_errors', 'calibration_errors', 'pending',
                    'nonincreasing_source_timestamps'):
            if manifest.get(key, 0):
                report['warnings'].append(f'{key}={manifest[key]}')
        if session.get('depth_archive', {}).get('finalization_errors', 0):
            report['warnings'].append('Session reports depth finalization error.')
        if not report['images_checked']:
            report['warnings'].append('No cropped depth saved; 3D reprocessing unavailable.')
        if check_coverage:
            leading = None if start is None or first_received is None else first_received - start
            trailing = None if end is None or last_received is None else end - last_received
            report['observation_window'] = dict(
                start_ns=start, end_ns=end, message_count=window_count,
                first_receive_ns=first_received, last_receive_ns=last_received,
                leading_gap_ns=leading, trailing_gap_ns=trailing)
            if not window_count:
                report['warnings'].append('No cropped depth in the requested recording window.')
            if leading is not None and leading > max_gap:
                report['warnings'].append('Depth leading receive gap exceeded limit.')
            if trailing is not None and trailing > max_gap:
                report['warnings'].append('Depth trailing receive gap exceeded limit.')
        report['warnings'] = list(dict.fromkeys(report['warnings']))
        report['status'] = 'incomplete' if report['warnings'] else 'complete'
    except (OSError, ValueError, TypeError, KeyError) as error:
        report['errors'].append(str(error))
    return report
