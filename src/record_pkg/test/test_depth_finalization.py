"""Depth integrity is retained without discarding otherwise valid numeric CSV."""

import json

import pytest

from record_pkg import export_csv
from record_pkg.finalize_session import finalize_session
from test_export_csv import fake_session, patch_reader


@pytest.mark.parametrize('depth_status,expected', [
    ('complete', 'complete'), ('incomplete', 'complete'), ('error', 'failed')])
def test_finalizer_preserves_depth_audit_without_failing_optional_missing_depth(
        tmp_path, monkeypatch, depth_status, expected):
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'auto_export_csv': False, 'required_topics': [],
        'depth_archive': {'enabled': True, 'required': False}}))
    (tmp_path / 'session.json').write_text('{}')
    monkeypatch.setattr('record_pkg.finalize_session.audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})
    monkeypatch.setattr('record_pkg.finalize_session.audit_depth_archive',
                        lambda *args: {'status': depth_status})
    report = finalize_session(tmp_path)
    assert report['depth_integrity']['status'] == depth_status
    assert report['status'] == expected


@pytest.mark.parametrize('depth_status,expected', [
    ('complete', 'complete'), ('incomplete', 'complete'), ('error', 'partial')])
def test_export_includes_depth_audit_without_discarding_numeric_csv(
        tmp_path, monkeypatch, depth_status, expected):
    fake_session(tmp_path)
    patch_reader(monkeypatch, [('/mode', {'data': 5}, 100)],
                 {'/mode': 'std_msgs/msg/Int32'})
    monkeypatch.setattr('record_pkg.depth_archive_audit.audit_depth_archive',
                        lambda *args: {'status': depth_status})
    manifest = export_csv.export_session(tmp_path)
    assert manifest['depth_integrity']['status'] == depth_status
    assert manifest['status'] == expected
    assert manifest['topics']['/mode']['rows_written'] == 1
