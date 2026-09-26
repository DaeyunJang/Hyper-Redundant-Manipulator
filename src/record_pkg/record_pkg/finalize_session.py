"""Post-stop worker: inspect finalized bag, then optionally export read-only CSV."""

import argparse
import json
import os
from pathlib import Path

from record_pkg.bag_integrity import audit_bag
from record_pkg.capture import save_json, utc_now
from record_pkg.depth_archive_audit import audit_depth_archive
from record_pkg.image_archive_audit import audit_image_archive


def finalize_session(directory):
    directory = Path(directory).resolve()
    config = json.loads((directory / 'recording_config.json').read_text())
    session = json.loads((directory / 'session.json').read_text())
    report = dict(status='running', started_utc=utc_now())
    path = directory / 'postprocess.json'
    save_json(path, report)
    try:
        if config.get('auto_export_csv', True):
            from record_pkg.export_csv import export_session
            manifest = export_session(directory)
            report.update(csv_status=manifest['status'], csv_directory=str(directory / 'csv'),
                          integrity=manifest.get('integrity'),
                          image_integrity=manifest.get('image_integrity'),
                          depth_integrity=manifest.get('depth_integrity'),
                          summary_csv=manifest.get('summary_csv'),
                          status=manifest['status'])
        else:
            report.update(csv_status='disabled', status='complete', integrity=audit_bag(
                directory / 'bag', config['required_topics'],
                max_gap_sec=config.get('required_max_gap_sec', 2.0),
                selected_topics=session.get('selected_topics'),
                observation_start_ns=session.get('recording_ready_ros_ns'),
                observation_end_ns=session.get('stop_requested_ros_ns')))
            report['image_integrity'] = audit_image_archive(directory, config, session)
            report['depth_integrity'] = audit_depth_archive(directory, config, session)
        if (report.get('integrity') or {}).get('status') == 'error':
            report['status'] = 'failed'
        if (report.get('image_integrity') or {}).get('status') == 'error':
            report['status'] = 'failed'
        if (report.get('depth_integrity') or {}).get('status') == 'error':
            report['status'] = 'failed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
    report['completed_utc'] = utc_now()
    save_json(path, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session')
    args = parser.parse_args()
    if hasattr(os, 'nice'):
        os.nice(10)  # Background postprocessing should yield to acquisition/control.
    report = finalize_session(args.session)
    return 0 if report['status'] == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
