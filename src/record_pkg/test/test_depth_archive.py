"""Lossless raw depth, calibration association and bounded lifecycle tests."""

import csv
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import cv2
import numpy as np
import pytest

from record_pkg.depth_archive import DepthArchive, depth_pixels
from record_pkg.depth_archive_audit import audit_depth_archive


def sample(encoding='16UC1', big=False, padding=0, sec=12):
    floating = encoding == '32FC1'
    kind = 'f4' if floating else 'u2'
    values = np.array([[0, 1, 65535], [12, 1024, 4095]], dtype=kind)
    if floating:
        values[:] = [[0, np.nan, np.inf], [-np.inf, 0.234567, 12.25]]
    stored = values.astype(('>' if big else '<') + kind)
    rowbytes = 3 * stored.dtype.itemsize
    data = b''.join(row.tobytes() + bytes(padding) for row in stored)
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=34), frame_id='camera'),
        width=3, height=2, step=rowbytes + padding, encoding=encoding,
        is_bigendian=int(big), data=data)
    metadata = dict(
        schema_version=1, source_time_ns=sec * 1_000_000_000 + 34, frame_id='camera',
        encoding=encoding, width=3, height=2, source_width=100, source_height=80,
        roi=dict(x=10, y=20, width=3, height=2),
        depth_scale_m_per_unit=1.0 if floating else .001,
        intrinsics_reference='cropped_pixels',
        k=[200., 0., 40., 0., 200., 20., 0., 0., 1.],
        p=[200., 0., 40., 0., 0., 200., 20., 0., 0., 0., 1., 0.],
        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], d=[0.] * 5,
        distortion_model='plumb_bob')
    return msg, metadata, values


def rows(archive):
    with (archive.directory / 'index.csv').open() as handle:
        return list(csv.DictReader(handle))


def audit(directory, **options):
    return audit_depth_archive(directory, {'depth_archive': {'enabled': True, **options}},
                               {'snapshot': {'contact_segment_id': 9}})


@pytest.mark.parametrize('encoding', ['16UC1', 'mono16', '32FC1'])
@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('padding', [0, 3])
def test_exact_values_endian_padding_units_and_calibration(tmp_path, encoding, big, padding):
    message, metadata, values = sample(encoding, big, padding)
    archive = DepthArchive(tmp_path, contact_segment_id=9)
    assert archive.update_metadata(metadata)
    assert archive.enqueue(message, 99)
    assert archive.close()
    assert archive.report()['errors'] == 0
    assert archive.report()['saved'] == 1
    row = rows(archive)[0]
    path = archive.directory / row['filename']
    actual = (np.load(path, allow_pickle=False) if encoding == '32FC1'
              else cv2.imread(str(path), cv2.IMREAD_UNCHANGED))
    assert actual.dtype == values.dtype
    np.testing.assert_array_equal(actual, values)
    assert row['source_time_ns'] == '12000000034'
    assert row['contact_segment_id'] == '9'
    assert row['roi_x'] == '10' and row['roi_y'] == '20'
    assert float(row['depth_scale_m_per_unit']) == metadata['depth_scale_m_per_unit']
    assert json.loads(row['k']) == metadata['k']
    assert audit(tmp_path)['status'] == 'complete'


def test_metadata_may_arrive_after_image(tmp_path):
    message, metadata, _ = sample()
    archive = DepthArchive(tmp_path)
    assert archive.enqueue(message, 1)
    assert archive.update_metadata(SimpleNamespace(data=json.dumps(metadata)))
    assert archive.close()
    assert archive.report()['saved'] == 1


def test_missing_metadata_never_infers_scale(tmp_path):
    archive = DepthArchive(tmp_path)
    archive.enqueue(sample()[0], 1)
    assert archive.close()
    assert archive.report()['saved'] == 0
    assert archive.report()['errors'] == 1
    assert 'matching' in archive.report()['error_details'][0]


@pytest.mark.parametrize('field,value', [
    ('depth_scale_m_per_unit', 0), ('depth_scale_m_per_unit', float('nan')),
    ('intrinsics_reference', 'full_image'), ('k', [0.] * 9), ('width', -1),
])
def test_invalid_calibration_is_explicit(tmp_path, field, value):
    archive = DepthArchive(tmp_path)
    _, metadata, _ = sample()
    metadata[field] = value
    assert not archive.update_metadata(metadata)
    assert archive.close()
    assert archive.report()['calibration_errors'] == 1
    assert audit(tmp_path)['status'] == 'incomplete'


@pytest.mark.parametrize('change', ['frame', 'time', 'size', 'encoding'])
def test_wrong_calibration_cannot_be_used(tmp_path, change):
    message, metadata, _ = sample()
    if change == 'frame':
        metadata['frame_id'] = 'other_camera'
    elif change == 'time':
        metadata['source_time_ns'] += 1
    elif change == 'size':
        metadata['width'] = metadata['roi']['width'] = 4
    else:
        metadata['encoding'] = 'mono16'
    archive = DepthArchive(tmp_path)
    assert archive.update_metadata(metadata)
    archive.enqueue(message, 1)
    assert archive.close()
    assert archive.report()['saved'] == 0
    assert archive.report()['errors'] == 1


@pytest.mark.parametrize('field,value', [
    ('step', 1), ('data', b'\x00'), ('encoding', 'rgb8'), ('height', 0)])
def test_invalid_depth_payload(field, value):
    message = sample()[0]
    setattr(message, field, value)
    with pytest.raises(ValueError):
        depth_pixels(message)


def test_missing_depth_source_gap_is_reported(tmp_path):
    archive = DepthArchive(tmp_path, contact_segment_id=9)
    for sec in (12, 100):
        message, metadata, _ = sample(sec=sec)
        archive.update_metadata(metadata)
        archive.enqueue(message, sec)
    assert archive.close()
    assert archive.report()['largest_source_gap_ns'] == 88_000_000_000
    assert audit(tmp_path)['status'] == 'incomplete'


def test_queue_full_and_stop_drain(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = DepthArchive._save

    def blocked(self, *args):
        entered.set()
        assert release.wait(3)
        original(self, *args)

    monkeypatch.setattr(DepthArchive, '_save', blocked)
    archive = DepthArchive(tmp_path, queue_size=1)
    message, metadata, _ = sample()
    archive.update_metadata(metadata)
    try:
        archive.enqueue(message, 1)
        assert entered.wait(2)
        assert archive.enqueue(message, 2)
        assert not archive.enqueue(message, 3)
        archive.request_stop()
        assert not archive.enqueue(message, 4)
    finally:
        release.set()
        assert archive.close()
    assert archive.report()['saved'] == 2
    assert archive.report()['dropped'] == 2
    assert archive.report()['pending'] == 0


def test_optional_no_depth_is_visible_not_complete(tmp_path):
    archive = DepthArchive(tmp_path)
    assert archive.close()
    assert audit(tmp_path, required=False)['status'] == 'incomplete'


def test_missing_file_or_unsafe_index_is_error(tmp_path):
    archive = DepthArchive(tmp_path, contact_segment_id=9)
    message, metadata, _ = sample()
    archive.update_metadata(metadata)
    archive.enqueue(message, 1)
    assert archive.close()
    (archive.directory / rows(archive)[0]['filename']).unlink()
    assert audit(tmp_path)['status'] == 'error'


def test_stop_then_new_record_has_separate_files_and_fresh_source_statistics(tmp_path):
    for name, sec in (('first', 12), ('second', 100)):
        directory = tmp_path / name
        archive = DepthArchive(directory, contact_segment_id=9)
        message, metadata, _ = sample(sec=sec)
        archive.update_metadata(metadata)
        archive.enqueue(message, sec * 1_000_000_000)
        assert archive.close()
        assert archive.report()['largest_source_gap_ns'] == 0
        assert audit(directory)['status'] == 'complete'


@pytest.mark.parametrize('seconds,start,end,expected,warning', [
    ([12], 12, 20, 'incomplete', 'trailing'),
    ([18, 19, 20], 12, 20, 'incomplete', 'leading'),
    ([10, 11], 12, 20, 'incomplete', 'requested recording window'),
    ([21, 22], 12, 20, 'incomplete', 'requested recording window'),
    ([12, 14, 16, 18, 20], 12, 20, 'complete', None),
    ([12], None, None, 'complete', None),
    ([12], None, 20, 'incomplete', 'trailing'),
    ([18], 12, None, 'incomplete', 'leading'),
])
def test_depth_audit_detects_leading_and_trailing_coverage(
        tmp_path, seconds, start, end, expected, warning):
    archive = DepthArchive(tmp_path, contact_segment_id=9)
    for sec in seconds:
        message, metadata, _ = sample(sec=sec)
        archive.update_metadata(metadata)
        archive.enqueue(message, sec * 1_000_000_000)
    assert archive.close()
    session = {'snapshot': {'contact_segment_id': 9}}
    if start is not None:
        session['recording_ready_ros_ns'] = start * 1_000_000_000
    if end is not None:
        session['stop_requested_ros_ns'] = end * 1_000_000_000
    result = audit_depth_archive(
        tmp_path, {'depth_archive': {'enabled': True, 'required': False}}, session)
    assert result['status'] == expected
    if warning:
        assert any(warning in detail for detail in result['warnings'])
    if start is None and end is None:
        assert 'observation_window' not in result
    else:
        assert result['observation_window']['message_count'] == sum(
            (start is None or sec >= start) and (end is None or sec <= end) for sec in seconds)


def test_constructor_closes_index_on_header_write_failure(tmp_path, monkeypatch):
    opened = []
    original = Path.open

    def tracked(path, *args, **kwargs):
        handle = original(path, *args, **kwargs)
        opened.append(handle)
        return handle

    def failed_header(self):
        raise OSError('Synthetic index header write failure')

    monkeypatch.setattr(Path, 'open', tracked)
    monkeypatch.setattr(csv.DictWriter, 'writeheader', failed_header)
    with pytest.raises(OSError, match='header write failure'):
        DepthArchive(tmp_path)
    assert len(opened) == 1 and opened[0].closed
