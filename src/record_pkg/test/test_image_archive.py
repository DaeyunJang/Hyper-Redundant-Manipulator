"""Offline lossless crop archive and bounded recording lifecycle contracts."""

import csv
import json
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from record_pkg.image_archive import ImageArchive, INDEX_FIELDS, image_pixels


def image(encoding='rgb8', *, sec=12, nanosec=34, padding=0):
    rgb = np.array([[[1, 2, 240], [3, 150, 4]], [[70, 6, 7], [8, 9, 10]]],
                   dtype=np.uint8)
    pixels = rgb if encoding == 'rgb8' else rgb[:, :, ::-1]
    if encoding == 'mono8':
        pixels = rgb[:, :, 0]
    stride = pixels.shape[1] * (1 if encoding == 'mono8' else 3)
    rows = np.full((2, stride + padding), 231, dtype=np.uint8)
    rows[:, :stride] = pixels.reshape(2, stride)
    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
            frame_id='camera_color_optical_frame'),
        width=2, height=2, step=stride + padding, encoding=encoding,
        data=rows.tobytes(),
    )
    return msg, pixels


def index_rows(archive):
    with (archive.directory / 'index.csv').open() as stream:
        reader = csv.DictReader(stream)
        assert reader.fieldnames == list(INDEX_FIELDS)
        return list(reader)


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'mono8'])
@pytest.mark.parametrize('padding', [0, 5])
def test_pixels_lossless_including_color_order_stride_and_labels(tmp_path, encoding, padding):
    msg, original = image(encoding, padding=padding)
    archive = ImageArchive(tmp_path, contact_segment_id=9)
    assert archive.enqueue(msg, 15_000_000_123)
    assert archive.close()
    rows = index_rows(archive)
    assert len(rows) == 1
    row = rows[0]
    assert row == dict(
        source_time_ns='12000000034', received_time_ns='15000000123',
        frame_id='camera_color_optical_frame', filename='00000001_12_000000034.png',
        width='2', height='2', input_encoding=encoding,
        payload_bytes=str(len(msg.data)), contact_segment_id='9',
    )
    restored = cv2.imread(str(archive.directory / row['filename']), cv2.IMREAD_UNCHANGED)
    if encoding == 'rgb8':
        restored = restored[:, :, ::-1]
    np.testing.assert_array_equal(restored, original)
    assert archive.report()['saved'] == 1
    assert archive.report()['pending'] == 0
    assert archive.report()['errors'] == 0
    manifest = json.loads((archive.directory / 'manifest.json').read_text())
    assert manifest == archive.report()


def test_duplicate_timestamps_are_not_overwritten(tmp_path):
    archive = ImageArchive(tmp_path)
    for i in range(3):
        msg, _ = image()
        assert archive.enqueue(msg, 100 + i)
    assert archive.close()
    rows = index_rows(archive)
    assert len({row['filename'] for row in rows}) == 3
    assert {row['source_time_ns'] for row in rows} == {'12000000034'}
    assert archive.report()['nonincreasing_source_timestamps'] == 2
    assert archive.report()['first_received_time_ns'] == 100
    assert archive.report()['last_received_time_ns'] == 102


def test_queue_full_is_explicit_and_stop_drains_every_accepted_frame(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    save = ImageArchive._save

    def blocked_save(self, *args):
        entered.set()
        assert release.wait(3)
        save(self, *args)

    monkeypatch.setattr(ImageArchive, '_save', blocked_save)
    archive = ImageArchive(tmp_path, queue_size=1)
    msg, _ = image()
    try:
        assert archive.enqueue(msg, 1)
        assert entered.wait(2)
        assert archive.enqueue(msg, 2)
        assert not archive.enqueue(msg, 3)
        before = time.monotonic()
        archive.request_stop()
        assert time.monotonic() - before < 0.1
        assert not archive.done
        assert not archive.enqueue(msg, 4)
        report = archive.report()
        assert report['received'] == 4 and report['accepted'] == 2
        assert report['dropped'] == 2
        assert report['dropped_queue_full'] == 1
        assert report['rejected_after_stop'] == 1
        assert report['pending'] == 2
    finally:
        release.set()
        assert archive.close()
    assert len(index_rows(archive)) == 2
    assert archive.report()['saved'] == 2


def test_encoding_occurs_only_in_worker_not_enqueue(tmp_path, monkeypatch):
    threads = []
    imencode = cv2.imencode

    def track(*args, **kwargs):
        threads.append(threading.current_thread().name)
        return imencode(*args, **kwargs)

    monkeypatch.setattr(cv2, 'imencode', track)
    archive = ImageArchive(tmp_path)
    archive.enqueue(image()[0], 1)
    assert archive.close()
    assert threads == ['hrm-crop-png-writer']


def test_stop_after_timeout_cannot_skip_concurrently_accepted_frame(tmp_path, monkeypatch):
    """Reproduce enqueue/stop between Queue.get's timeout and the stop check."""
    entered, release = threading.Event(), threading.Event()
    original_get = queue.Queue.get

    def interrupted_get(self, *args, **kwargs):
        if not entered.is_set():
            entered.set()
            assert release.wait(3)
            raise queue.Empty
        return original_get(self, *args, **kwargs)

    monkeypatch.setattr(queue.Queue, 'get', interrupted_get)
    archive = ImageArchive(tmp_path)
    try:
        assert entered.wait(2)
        assert archive.enqueue(image()[0], 1)
        archive.request_stop()
    finally:
        release.set()
        assert archive.close()
    assert archive.report()['saved'] == 1
    assert archive.report()['pending'] == 0
    assert len(index_rows(archive)) == 1


def test_no_existing_archive_overwrite(tmp_path):
    existing = tmp_path / 'images/hrm_crop'
    existing.mkdir(parents=True)
    marker = existing / 'index.csv'
    marker.write_text('original')
    with pytest.raises(FileExistsError):
        ImageArchive(tmp_path)
    assert marker.read_text() == 'original'


@pytest.mark.parametrize('attribute,value', [
    ('width', 0), ('height', -1), ('step', 1), ('step', 8),
    ('data', b'\x00'), ('encoding', '16UC1'), ('width', 1.2),
])
def test_invalid_image_samples_are_not_silently_saved(tmp_path, attribute, value):
    archive = ImageArchive(tmp_path)
    message = image()[0]
    setattr(message, attribute, value)
    assert archive.enqueue(message, 1)
    assert archive.close()
    report = archive.report()
    assert report['errors'] == 1 and report['saved'] == 0
    assert report['error_details']
    assert index_rows(archive) == []
    assert list(archive.directory.glob('*.png')) == []


@pytest.mark.parametrize('sec,nanosec,frame,received', [
    (0, 0, 'camera', 1), (-1, 0, 'camera', 1), (1, 1_000_000_000, 'camera', 1),
    (1, 1, '', 1), (1, 1, 'camera', 0), (1, 1, 'camera', -1),
])
def test_invalid_timestamps_or_frame_are_visible_errors(tmp_path, sec, nanosec, frame, received):
    msg, _ = image(sec=sec, nanosec=nanosec)
    msg.header.frame_id = frame
    archive = ImageArchive(tmp_path)
    archive.enqueue(msg, received)
    assert archive.close()
    assert archive.report()['errors'] == 1
    assert archive.report()['saved'] == 0


def test_disk_write_failure_is_reported_worker_continues(tmp_path, monkeypatch):
    original_open = Path.open

    def fail_png(path, mode='r', *args, **kwargs):
        if path.suffix == '.png':
            raise OSError('Synthetic disk full')
        return original_open(path, mode, *args, **kwargs)

    archive = ImageArchive(tmp_path)
    monkeypatch.setattr(Path, 'open', fail_png)
    archive.enqueue(image()[0], 1)
    archive.enqueue(image()[0], 2)
    assert archive.close()
    report = archive.report()
    assert report['errors'] == 2 and report['saved'] == 0
    assert report['pending'] == 0
    assert 'Synthetic disk full' in report['error_details'][0]
    assert json.loads((archive.directory / 'manifest.json').read_text())['errors'] == 2


def test_png_encoding_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(cv2, 'imencode', lambda *args: (False, None))
    archive = ImageArchive(tmp_path)
    archive.enqueue(image()[0], 1)
    assert archive.close()
    assert archive.report()['errors'] == 1
    assert 'could not encode' in archive.report()['error_details'][0]


def test_manifest_failure_is_visible_in_runtime_report(tmp_path, monkeypatch):
    original_open = Path.open

    def fail_manifest(path, mode='r', *args, **kwargs):
        if path.name == 'manifest.json':
            raise OSError('Synthetic manifest write failure')
        return original_open(path, mode, *args, **kwargs)

    archive = ImageArchive(tmp_path)
    monkeypatch.setattr(Path, 'open', fail_manifest)
    archive.enqueue(image()[0], 1)
    assert archive.close()
    report = archive.report()
    assert report['saved'] == 1 and report['errors'] == 0
    assert report['finalization_errors'] == 1


def test_empty_session_has_index_and_manifest(tmp_path):
    archive = ImageArchive(tmp_path)
    assert archive.close()
    assert archive.done
    assert index_rows(archive) == []
    assert archive.report()['saved'] == 0
    assert (archive.directory / 'manifest.json').is_file()


@pytest.mark.parametrize('kwargs', [
    {'queue_size': 0}, {'queue_size': 1.2}, {'queue_size': True},
    {'contact_segment_id': -1}, {'contact_segment_id': 19},
    {'contact_segment_id': 1.5}, {'contact_segment_id': True}, {'topic': 'crop'},
])
def test_invalid_config_rejected_before_files_created(tmp_path, kwargs):
    with pytest.raises(ValueError):
        ImageArchive(tmp_path, **kwargs)
    assert list(tmp_path.iterdir()) == []


def test_label_stays_user_selected_across_zero_and_nonzero_pixels(tmp_path):
    archive = ImageArchive(tmp_path, contact_segment_id=18)
    msg, _ = image()
    archive.enqueue(msg, 1)
    zero, _ = image()
    zero.data = bytes(len(zero.data))
    archive.enqueue(zero, 2)
    assert archive.close()
    assert [row['contact_segment_id'] for row in index_rows(archive)] == ['18', '18']


def test_mono_shape_is_two_dimensional():
    msg, _ = image('mono8')
    assert image_pixels(msg).shape == (2, 2)
