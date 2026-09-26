"""Post-stop PNG archive checks against real synthetic writer output."""

import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest

from record_pkg.image_archive import ImageArchive, INDEX_FIELDS
from record_pkg.image_archive_audit import audit_image_archive


@pytest.fixture
def config():
    return {
        'image_archive': {'enabled': True, 'required': True,
                          'topic': '/estimated_segment_crop_image', 'queue_size': 16},
        'required_max_gap_sec': 2.0,
    }


@pytest.fixture
def session():
    return {'snapshot': {'contact_segment_id': 9}}


def record(directory, count=2):
    archive = ImageArchive(directory, contact_segment_id=9)
    data = np.arange(36, dtype=np.uint8).tobytes()
    for frame in range(count):
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=100, nanosec=frame * 33_333_333),
                frame_id='camera_color_optical_frame'),
            width=4, height=3, step=12, encoding='rgb8', data=data,
        )
        assert archive.enqueue(message, 101_000_000_000 + frame)
    assert archive.close()
    return archive.directory


def read_rows(root):
    with (root / 'index.csv').open() as handle:
        return list(csv.DictReader(handle))


def write_rows(root, rows):
    with (root / 'index.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def update_manifest(root, **changes):
    path = root / 'manifest.json'
    data = json.loads(path.read_text())
    data.update(changes)
    path.write_text(json.dumps(data))


def test_real_writer_archive_is_complete_with_expected_counts_and_labels(
        tmp_path, config, session):
    root = record(tmp_path)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'complete'
    assert report['images_checked'] == 2
    assert report['errors'] == report['warnings'] == []
    assert report['writer']['saved'] == 2
    assert report['writer']['contact_segment_id'] == 9
    assert [(row['width'], row['height']) for row in read_rows(root)] == [('4', '3')] * 2


@pytest.mark.parametrize('config', [{}, {'image_archive': {'enabled': False}}])
def test_legacy_or_disabled_profile_does_not_require_archive(tmp_path, config):
    assert audit_image_archive(tmp_path, config, {}) == {'status': 'disabled'}
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('missing', ['manifest.json', 'index.csv', 'png'])
def test_missing_manifest_index_or_referenced_png_is_error(
        tmp_path, config, session, missing):
    root = record(tmp_path)
    path = root / missing if missing != 'png' else next(root.glob('*.png'))
    path.unlink()
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert report['errors']


def test_orphan_png_is_error(tmp_path, config, session):
    root = record(tmp_path)
    (root / 'orphan.png').write_bytes(next(root.glob('*.png')).read_bytes())
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert 'unindexed' in report['errors'][0]


@pytest.mark.parametrize('field,value,error', [
    ('width', '5', 'dimensions'), ('height', '4', 'dimensions'),
    ('contact_segment_id', '8', 'frozen session label'),
    ('source_time_ns', '0', 'timestamp'), ('received_time_ns', '-1', 'timestamp'),
    ('frame_id', '', 'frame/encoding'), ('input_encoding', '16UC1', 'frame/encoding'),
    ('filename', '../outside.png', 'unsafe'), ('filename', '/tmp/outside.png', 'unsafe'),
])
def test_corrupted_index_or_unsafe_path_is_error(tmp_path, config, session, field, value, error):
    root = record(tmp_path)
    rows = read_rows(root)
    rows[0][field] = value
    write_rows(root, rows)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert error in report['errors'][0]


def test_duplicate_filename_is_error(tmp_path, config, session):
    root = record(tmp_path)
    rows = read_rows(root)
    rows[1]['filename'] = rows[0]['filename']
    write_rows(root, rows)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert 'Duplicate' in report['errors'][0]


def test_external_symlink_image_is_error(tmp_path, config, session):
    root = record(tmp_path)
    original = next(root.glob('*.png'))
    outside = tmp_path / 'elsewhere.png'
    original.rename(outside)
    original.symlink_to(outside)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert 'unsafe' in report['errors'][0]


@pytest.mark.parametrize('data', [b'', b'broken-png', bytes(24)])
def test_invalid_or_truncated_png_header_is_error(tmp_path, config, session, data):
    root = record(tmp_path)
    next(root.glob('*.png')).write_bytes(data)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert 'PNG header' in report['errors'][0]


@pytest.mark.parametrize('changes,error', [
    ({'saved': 3}, 'saved count'), ({'done': False}, 'did not finish'),
    ({'topic': '/wrong_topic'}, 'topic differs'),
])
def test_writer_manifest_mismatch_is_error(tmp_path, config, session, changes, error):
    root = record(tmp_path)
    update_manifest(root, **changes)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'error'
    assert error in report['errors'][0]


@pytest.mark.parametrize('field', [
    'dropped', 'errors', 'finalization_errors', 'pending', 'nonincreasing_source_timestamps',
])
def test_writer_reports_delivery_or_finalization_loss_as_incomplete(
        tmp_path, config, session, field):
    root = record(tmp_path)
    update_manifest(root, **{field: 1})
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'incomplete'
    assert report['errors'] == []
    assert report['warnings'] == [f'{field}=1']
    assert report['images_checked'] == 2


def test_large_source_gap_is_incomplete(tmp_path, config, session):
    root = record(tmp_path)
    update_manifest(root, largest_source_gap_ns=2_000_000_001)
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'incomplete'
    assert 'source-image gap' in report['warnings'][0]


def test_session_manifest_write_failure_survives_old_manifest(tmp_path, config, session):
    record(tmp_path)
    session['image_archive'] = {'finalization_errors': 1}
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == 'incomplete'
    assert 'Session reports' in report['warnings'][0]


@pytest.mark.parametrize('required,expected', [(True, 'incomplete'), (False, 'complete')])
def test_empty_archive_required_policy(tmp_path, config, session, required, expected):
    record(tmp_path, count=0)
    config['image_archive']['required'] = required
    report = audit_image_archive(tmp_path, config, session)
    assert report['status'] == expected
    assert report['images_checked'] == 0
    assert bool(report['warnings']) is required
