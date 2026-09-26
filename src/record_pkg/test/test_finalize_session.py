import json

import pytest

from record_pkg.finalize_session import finalize_session


@pytest.mark.parametrize('integrity_status, expected', [
    ('complete', 'complete'), ('incomplete', 'complete'), ('error', 'failed')])
def test_audit_without_csv_distinguishes_missing_data_from_failed_scan(tmp_path, monkeypatch,
                                                                    integrity_status, expected):
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'auto_export_csv': False, 'required_topics': ['/pose'], 'required_max_gap_sec': 2.0}))
    (tmp_path / 'session.json').write_text(json.dumps({
        'selected_topics': ['/pose'], 'recording_ready_ros_ns': 10, 'stop_requested_ros_ns': 20}))
    calls = []
    def audit(path, required, **kwargs):
        calls.append((path, required, kwargs))
        return {'status': integrity_status}
    monkeypatch.setattr('record_pkg.finalize_session.audit_bag', audit)
    report = finalize_session(tmp_path)
    assert report['status'] == expected
    assert report['csv_status'] == 'disabled'
    assert report['integrity']['status'] == integrity_status
    assert not (tmp_path / 'csv').exists()
    assert calls[0][2]['observation_start_ns'] == 10
    assert calls[0][2]['observation_end_ns'] == 20
    assert json.loads((tmp_path / 'postprocess.json').read_text()) == report


def test_failed_csv_is_reported_without_touching_bag(tmp_path, monkeypatch):
    (tmp_path / 'recording_config.json').write_text('{"auto_export_csv": true}')
    (tmp_path / 'session.json').write_text('{}')
    bag = tmp_path / 'bag'
    bag.mkdir()
    preserved = bag / 'bag_0.db3'
    preserved.write_bytes(b'untouched')
    def fail(path):
        raise RuntimeError('CSV output already exists')
    monkeypatch.setattr('record_pkg.export_csv.export_session', fail)
    report = finalize_session(tmp_path)
    assert report['status'] == 'failed' and 'already exists' in report['error']
    assert preserved.read_bytes() == b'untouched'


def test_no_common_summary_start_is_explicit_in_postprocess(tmp_path, monkeypatch):
    (tmp_path / 'recording_config.json').write_text('{"auto_export_csv": true}')
    (tmp_path / 'session.json').write_text('{}')
    summary = {'status': 'no_common_start', 'rows_written': 0,
               'startup': {'ready': False, 'rows_skipped': 20}}
    monkeypatch.setattr('record_pkg.export_csv.export_session', lambda path: {
        'status': 'partial', 'integrity': {'status': 'complete'},
        'image_integrity': {'status': 'disabled'},
        'depth_integrity': {'status': 'disabled'}, 'summary_csv': summary})
    report = finalize_session(tmp_path)
    assert report['status'] == 'partial'
    assert report['summary_csv'] == summary
    assert json.loads((tmp_path / 'postprocess.json').read_text())['summary_csv'] == summary
