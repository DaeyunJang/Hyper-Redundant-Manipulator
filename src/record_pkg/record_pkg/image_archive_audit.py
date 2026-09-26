"""Check crop PNG index/presence after stop, without decoding images or ROS replay."""

import csv
import json
from pathlib import Path
import struct

from record_pkg.capture import image_archive_settings


def audit_image_archive(directory, config, session):
    settings = image_archive_settings(config)
    if not settings['enabled']:
        return {'status': 'disabled'}
    root = Path(directory) / 'images' / 'hrm_crop'
    report = dict(status='error', directory='images/hrm_crop', images_checked=0,
                  errors=[], warnings=[],
                  scope='Index, counts, PNG headers/dimensions and source gaps; '
                        'not pixel-content/CRC validation or proof of every camera exposure.')
    try:
        manifest = json.loads((root / 'manifest.json').read_text())
        report['writer'] = manifest
        if not manifest.get('done') or manifest.get('topic') != settings['topic']:
            raise ValueError('PNG writer did not finish or topic differs from session settings.')
        expected_label = session.get('snapshot', {}).get('contact_segment_id')
        names = set()
        with (root / 'index.csv').open(newline='') as handle:
            for row in csv.DictReader(handle):
                name = row['filename']
                path = root / name
                if (Path(name).name != name or not name.endswith('.png') or name in names
                        or path.resolve().parent != root.resolve()):
                    raise ValueError('Duplicate/unsafe PNG filename in index.')
                names.add(name)
                if int(row['source_time_ns']) <= 0 or int(row['received_time_ns']) <= 0:
                    raise ValueError('Invalid image timestamp in index.')
                if not row['frame_id'] or row['input_encoding'].lower() not in (
                        'rgb8', 'bgr8', 'mono8'):
                    raise ValueError('Invalid image frame/encoding in index.')
                if (expected_label is not None
                        and int(row['contact_segment_id']) != expected_label):
                    raise ValueError('Image contact label differs from frozen session label.')
                with path.open('rb') as image:
                    header = image.read(24)
                if (len(header) != 24 or header[:8] != b'\x89PNG\r\n\x1a\n'
                        or header[12:16] != b'IHDR'):
                    raise ValueError(f'Invalid PNG header: {name}')
                if struct.unpack('>II', header[16:24]) != (int(row['width']), int(row['height'])):
                    raise ValueError(f'PNG dimensions differ from index: {name}')
                report['images_checked'] += 1
        if {path.name for path in root.glob('*.png')} != names:
            raise ValueError('PNG files and index differ (missing or unindexed images).')
        if manifest.get('saved') != report['images_checked']:
            raise ValueError('PNG saved count differs from index.')
        for key in ('dropped', 'errors', 'finalization_errors', 'pending',
                    'nonincreasing_source_timestamps'):
            if manifest.get(key, 0):
                report['warnings'].append(f'{key}={manifest[key]}')
        # Session report can contain a manifest-write failure absent from the manifest.
        if session.get('image_archive', {}).get('finalization_errors', 0):
            report['warnings'].append('Session reports image finalization failure.')
        if settings['required'] and not report['images_checked']:
            report['warnings'].append('No required crop images saved.')
        if manifest.get('largest_source_gap_ns', 0) > config.get('required_max_gap_sec', 2.) * 1e9:
            report['warnings'].append('Crop source-image gap exceeded required_max_gap_sec.')
        report['status'] = 'incomplete' if report['warnings'] else 'complete'
    except (OSError, ValueError, KeyError, TypeError) as error:
        report['errors'].append(str(error))
    return report
