"""Skeleton tip preview checks without ROS traffic, cameras, or robot hardware."""

from itertools import permutations
import os
import re
import threading
from unittest.mock import Mock

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from PyQt5.QtCore import QPoint, QRect
from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg import image_preview
from gui_py_pkg.image_preview import ImageCanvas, LatestImages
from gui_py_pkg.skeleton_preview import (
    TIP_CACHE_SIZE, TIP_MAX_AGE, TIP_TOPICS, SkeletonFrameBuffer,
    SkeletonPreviewFrame, format_skeleton_tips,
)


def skeleton_image(sequence=1, nanosec=123):
    """Return a camera exposure whose frame differs from the tip reference frame."""
    pixels = np.full((24, 32, 3), (11, 22, 33), dtype=np.uint8)
    message = CvBridge().cv2_to_imgmsg(pixels, 'rgb8')
    message.header.frame_id = 'camera_color_optical_frame'
    message.header.stamp.sec = sequence
    message.header.stamp.nanosec = nanosec
    return message


def tip_point(sequence=1, nanosec=123, frame='hrm_base',
              xyz=(0.0123, -0.0456, 0.0789)):
    message = PointStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = sequence
    message.header.stamp.nanosec = nanosec
    message.point.x, message.point.y, message.point.z = xyz
    return message


def complete_frame(sequence=1, received_at=10.0):
    return SkeletonPreviewFrame(
        skeleton_image(sequence), received_at,
        estimated_tip=tip_point(sequence),
        fk_tip=tip_point(sequence, xyz=(-0.0045, 0.0067, 0.0089)))


def signed_numbers(line):
    return re.findall(r'[+-]\d+\.\d+', line)


def assert_no_values(line):
    assert line.strip()
    assert not signed_numbers(line), line


def assert_unavailable(lines):
    assert len(lines) == 3
    assert lines[0] == 'hrm_base | XYZ [mm]'
    assert lines[1].startswith('Est:')
    assert lines[2].startswith('FK:')
    for line in lines[1:]:
        assert_no_values(line)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


class FakeReceiver:
    """Consume pending snapshots without constructing a ROS node or executor."""

    def __init__(self):
        self.pending = {}
        self.received_at = {}
        self.closed = False

    def queue(self, value, received_at=10.0):
        self.pending['skeleton'] = value
        self.received_at['skeleton'] = received_at

    def take_latest(self):
        latest, self.pending = self.pending, {}
        return latest, dict(self.received_at)

    def close(self):
        self.closed = True


@pytest.fixture
def panel(app, monkeypatch):
    receiver = FakeReceiver()
    clock = {'now': 10.0}
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: clock['now'])
    widget = image_preview.ImagePreviewPanel(None)
    widget.timer.stop()
    widget.resize(1000, 320)
    widget.show()
    app.processEvents()
    yield widget, receiver, clock
    widget.stop()
    widget.close()
    assert receiver.closed


def test_tip_topics_are_existing_read_only_point_streams():
    assert TIP_TOPICS == {
        'estimated_tip': '/estimated_tip_position',
        'fk_tip': '/kinematics/fk_tip_position',
    }
    assert TIP_CACHE_SIZE == 8
    assert TIP_MAX_AGE == 0.2
    assert SkeletonFrameBuffer().snapshot(10.0) is None


@pytest.mark.parametrize('order', list(permutations(('image', 'estimated_tip', 'fk_tip'))))
def test_exact_source_stamp_matches_in_every_callback_order(order):
    buffer = SkeletonFrameBuffer()
    image = skeleton_image()
    points = {'estimated_tip': tip_point(), 'fk_tip': tip_point()}
    for index, name in enumerate(order):
        if name == 'image':
            buffer.receive_image(image, now=10.0 + index * 0.01)
        else:
            buffer.receive_tip(name, points[name], now=10.0 + index * 0.01)
    frame = buffer.snapshot(10.03)
    assert frame.image is image
    assert frame.estimated_tip is points['estimated_tip']
    assert frame.fk_tip is points['fk_tip']
    assert signed_numbers(format_skeleton_tips(frame, 10.03)[1])


@pytest.mark.parametrize('kind', ['estimated_tip', 'fk_tip'])
@pytest.mark.parametrize('sequence,nanosec', [(2, 123), (1, 124), (0, 0)])
def test_buffer_does_not_attach_another_exposures_tip(kind, sequence, nanosec):
    buffer = SkeletonFrameBuffer()
    image = skeleton_image()
    buffer.receive_image(image, now=10.0)
    buffer.receive_tip(kind, tip_point(sequence, nanosec), now=10.01)
    frame = buffer.snapshot(10.02)
    assert frame.image is image
    assert getattr(frame, kind) is None


@pytest.mark.parametrize('kind,row', [('estimated_tip', 1), ('fk_tip', 2)])
def test_one_missing_tip_does_not_hide_the_other(kind, row):
    buffer = SkeletonFrameBuffer()
    buffer.receive_image(skeleton_image(), now=10.0)
    buffer.receive_tip(kind, tip_point(), now=10.01)
    lines = format_skeleton_tips(buffer.snapshot(10.02), 10.02)
    assert signed_numbers(lines[row]) == ['+12.3', '-45.6', '+78.9']
    assert_no_values(lines[3 - row])


def test_delayed_fk_selects_its_own_complete_image_then_expires_to_latest():
    buffer = SkeletonFrameBuffer()
    source, latest = skeleton_image(), skeleton_image(2)
    estimate, fk = tip_point(), tip_point()
    buffer.receive_image(source, now=10.0)
    buffer.receive_tip('estimated_tip', estimate, now=10.0)
    buffer.receive_image(latest, now=10.03)
    buffer.receive_tip('estimated_tip', tip_point(2), now=10.03)
    buffer.receive_tip('fk_tip', fk, now=10.04)
    paired = buffer.snapshot(10.05)
    assert paired.image is source
    assert paired.estimated_tip is estimate
    assert paired.fk_tip is fk
    fallback = buffer.snapshot(10.0 + TIP_MAX_AGE + 0.01)
    assert fallback.image is latest
    assert fallback.estimated_tip is not None
    assert fallback.fk_tip is None
    assert_no_values(format_skeleton_tips(fallback, 10.21)[2])


def test_newest_complete_snapshot_wins_over_an_older_complete_snapshot():
    buffer = SkeletonFrameBuffer()
    for sequence in (1, 2):
        image = skeleton_image(sequence)
        buffer.receive_image(image, now=10.0 + sequence * 0.01)
        buffer.receive_tip('estimated_tip', tip_point(sequence), now=10.03)
        buffer.receive_tip('fk_tip', tip_point(sequence), now=10.03)
    assert buffer.snapshot(10.04).image is image


@pytest.mark.parametrize('kind', ['estimated_tip', 'fk_tip'])
def test_repeated_image_stamp_cannot_revive_an_old_point_receipt(kind):
    buffer = SkeletonFrameBuffer()
    buffer.receive_tip(kind, tip_point(), now=10.0)
    later = 10.0 + TIP_MAX_AGE + 0.01
    buffer.receive_image(skeleton_image(), now=later)
    frame = buffer.snapshot(later)
    assert getattr(frame, kind) is None


def test_tip_cache_cannot_pair_points_after_its_source_image_expires():
    buffer = SkeletonFrameBuffer()
    buffer.receive_image(skeleton_image(), now=10.0)
    later = 10.0 + TIP_MAX_AGE + 0.01
    for kind in TIP_TOPICS:
        buffer.receive_tip(kind, tip_point(), now=later)
    assert_unavailable(format_skeleton_tips(buffer.snapshot(later), later))


def test_image_and_both_point_histories_are_bounded_and_expire_without_callbacks():
    buffer = SkeletonFrameBuffer()
    for sequence in range(1, TIP_CACHE_SIZE * 4):
        buffer.receive_image(skeleton_image(sequence), now=10.0)
        for kind in TIP_TOPICS:
            buffer.receive_tip(kind, tip_point(sequence), now=10.0)
        assert len(buffer.images) <= TIP_CACHE_SIZE
        assert all(len(cache) <= TIP_CACHE_SIZE for cache in buffer.points.values())
    assert len(buffer.images) == TIP_CACHE_SIZE
    assert all(len(cache) == TIP_CACHE_SIZE for cache in buffer.points.values())
    assert (1, 123) not in buffer.images
    assert all((1, 123) not in cache for cache in buffer.points.values())
    expired = buffer.snapshot(10.0 + TIP_MAX_AGE + 0.01)
    assert not buffer.images
    assert all(not cache for cache in buffer.points.values())
    assert expired.image.header.stamp.sec == TIP_CACHE_SIZE * 4 - 1
    assert expired.estimated_tip is None
    assert expired.fk_tip is None


def test_estimate_and_fk_from_different_images_are_not_presented_as_one_pair():
    buffer = SkeletonFrameBuffer()
    buffer.receive_image(skeleton_image(), now=10.0)
    buffer.receive_tip('estimated_tip', tip_point(), now=10.0)
    latest = skeleton_image(2)
    fk = tip_point(2)
    buffer.receive_image(latest, now=10.01)
    buffer.receive_tip('fk_tip', fk, now=10.01)
    snapshot = buffer.snapshot(10.02)
    assert snapshot.image is latest
    assert snapshot.estimated_tip is None
    assert snapshot.fk_tip is fk


@pytest.mark.parametrize('kind,row', [('estimated_tip', 1), ('fk_tip', 2)])
def test_expired_point_receipt_clears_only_its_row_on_a_fresh_image(kind, row):
    buffer = SkeletonFrameBuffer()
    buffer.receive_tip(kind, tip_point(), now=10.0)
    later = 10.0 + TIP_MAX_AGE + 0.01
    buffer.receive_image(skeleton_image(), now=later)
    other = 'fk_tip' if kind == 'estimated_tip' else 'estimated_tip'
    buffer.receive_tip(other, tip_point(), now=later)
    lines = format_skeleton_tips(buffer.snapshot(later), later)
    assert_no_values(lines[row])
    assert signed_numbers(lines[3 - row]) == ['+12.3', '-45.6', '+78.9']


def test_tip_callback_updates_snapshot_without_another_image(monkeypatch):
    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._latest, node._received_at = {}, {}
    node._tag_buffer = None
    node._skeleton_buffer = SkeletonFrameBuffer()
    node._skeleton_signature = None
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: 10.0)
    image = skeleton_image()
    node.receive('skeleton', image)
    first, _ = node.take_latest()
    assert first['skeleton'].estimated_tip is None
    estimate = tip_point()
    node.receive_tip('estimated_tip', estimate)
    second, times = node.take_latest()
    assert second['skeleton'].image is image
    assert second['skeleton'].estimated_tip is estimate
    fk = tip_point()
    node.receive_tip('fk_tip', fk)
    third, times = node.take_latest()
    assert third['skeleton'].fk_tip is fk
    assert node.take_latest() == ({}, times)
    monkeypatch.setattr(image_preview.time, 'monotonic',
                        lambda: 10.0 + TIP_MAX_AGE + 0.01)
    expired, _ = node.take_latest()
    assert expired['skeleton'].estimated_tip is None
    assert expired['skeleton'].fk_tip is None


def test_text_converts_metres_to_signed_millimetres_in_xyz_order():
    lines = format_skeleton_tips(complete_frame(), 10.01)
    assert len(lines) == 3
    assert lines[0] == 'hrm_base | XYZ [mm]'
    assert lines[1].startswith('Est:')
    assert lines[2].startswith('FK:')
    assert signed_numbers(lines[1]) == ['+12.3', '-45.6', '+78.9']
    assert signed_numbers(lines[2]) == ['-4.5', '+6.7', '+8.9']


def test_zero_tip_is_a_valid_measurement():
    zero = tip_point(xyz=(0.0, 0.0, 0.0))
    frame = SkeletonPreviewFrame(skeleton_image(), 10.0, zero, zero)
    lines = format_skeleton_tips(frame, 10.01)
    assert signed_numbers(lines[1]) == ['+0.0'] * 3
    assert signed_numbers(lines[2]) == ['+0.0'] * 3


@pytest.mark.parametrize('kind,row', [('estimated_tip', 1), ('fk_tip', 2)])
@pytest.mark.parametrize('xyz', [
    (float('nan'), 0.0, 0.0), (0.0, float('inf'), 0.0),
    (0.0, 0.0, float('-inf')), (float('1e308'), 0.0, 0.0),
])
def test_nonfinite_points_hide_only_the_invalid_row(kind, row, xyz):
    points = {'estimated_tip': tip_point(), 'fk_tip': tip_point()}
    points[kind] = tip_point(xyz=xyz)
    frame = SkeletonPreviewFrame(skeleton_image(), 10.0, **points)
    lines = format_skeleton_tips(frame, 10.01)
    assert_no_values(lines[row])
    assert 'invalid' in lines[row].lower()
    assert signed_numbers(lines[3 - row]) == ['+12.3', '-45.6', '+78.9']


@pytest.mark.parametrize('sequence,nanosec,frame', [
    (2, 123, 'hrm_base'), (1, 124, 'hrm_base'), (0, 0, 'hrm_base'),
    (1, 123, 'camera_color_optical_frame'), (1, 123, 'ID0'), (1, 123, ''),
])
def test_formatter_independently_rejects_wrong_stamp_and_reference_frame(
        sequence, nanosec, frame):
    point = tip_point(sequence, nanosec, frame)
    snapshot = SkeletonPreviewFrame(skeleton_image(), 10.0, point, point)
    assert_unavailable(format_skeleton_tips(snapshot, 10.01))


def test_empty_image_stamp_cannot_establish_a_point_pair():
    point = tip_point(sequence=0, nanosec=0)
    frame = SkeletonPreviewFrame(skeleton_image(0, 0), 10.0, point, point)
    assert_unavailable(format_skeleton_tips(frame, 10.01))


def test_missing_and_stale_snapshots_do_not_reuse_numeric_values():
    assert_unavailable(format_skeleton_tips(None, 10.0))
    frame = SkeletonPreviewFrame(skeleton_image(), 10.0)
    assert_unavailable(format_skeleton_tips(frame, 10.01))
    stale = format_skeleton_tips(complete_frame(), 10.0 + TIP_MAX_AGE + 0.01)
    assert_unavailable(stale)
    assert all('stale' in line.lower() for line in stale[1:])


def test_canvas_header_preserves_original_pixels_and_size_hints(app):
    canvas = ImageCanvas()
    try:
        minimum, hint = canvas.minimumSize(), canvas.sizeHint()
        pixels = np.full((24, 32, 3), (11, 22, 33), dtype=np.uint8)
        canvas.set_rgb(pixels)
        canvas.set_overlay(format_skeleton_tips(complete_frame(), 10.01))
        assert canvas.image.pixelColor(0, 0).getRgb() == (11, 22, 33, 255)
        assert canvas.minimumSize() == minimum
        assert canvas.sizeHint() == hint
    finally:
        canvas.close()


def test_panel_updates_each_tip_and_clears_stale_values_without_new_frame(panel):
    widget, receiver, clock = panel
    receiver.queue(complete_frame())
    widget.refresh()
    canvas = widget.canvases['skeleton']
    assert signed_numbers(canvas.overlay_lines[1]) == ['+12.3', '-45.6', '+78.9']
    assert signed_numbers(canvas.overlay_lines[2]) == ['-4.5', '+6.7', '+8.9']
    assert not widget.canvases['depth'].overlay_lines
    assert widget.canvases['tag'].overlay_lines[0] == 'ID0 -> ID1'
    clock['now'] += TIP_MAX_AGE + 0.01
    widget.refresh()
    assert_unavailable(canvas.overlay_lines)


def test_panel_point_only_arrivals_do_not_convert_the_rgb_image_again(panel, monkeypatch):
    widget, receiver, _ = panel
    convert = Mock(wraps=image_preview.preview_rgb)
    monkeypatch.setattr(image_preview, 'preview_rgb', convert)
    image = skeleton_image()
    receiver.queue(SkeletonPreviewFrame(image, 10.0))
    widget.refresh()
    assert_unavailable(widget.canvases['skeleton'].overlay_lines)
    estimate = tip_point()
    receiver.queue(SkeletonPreviewFrame(image, 10.0, estimated_tip=estimate))
    widget.refresh()
    lines = widget.canvases['skeleton'].overlay_lines
    assert signed_numbers(lines[1])
    assert_no_values(lines[2])
    receiver.queue(SkeletonPreviewFrame(image, 10.0, estimate, tip_point()))
    widget.refresh()
    assert signed_numbers(widget.canvases['skeleton'].overlay_lines[2])
    convert.assert_called_once()


def test_panel_conversion_failure_suppresses_values_and_next_image_recovers(panel):
    widget, receiver, _ = panel
    receiver.queue(complete_frame())
    widget.refresh()
    canvas = widget.canvases['skeleton']
    assert signed_numbers(canvas.overlay_lines[1])
    malformed = complete_frame(sequence=2)
    malformed.image.encoding = 'invalid-encoding'
    receiver.queue(malformed)
    widget.refresh()
    assert 'skeleton' in widget.errors
    assert_unavailable(canvas.overlay_lines)
    widget.refresh()
    assert_unavailable(canvas.overlay_lines)
    receiver.queue(complete_frame(sequence=3))
    widget.refresh()
    assert 'skeleton' not in widget.errors
    assert signed_numbers(canvas.overlay_lines[1]) == ['+12.3', '-45.6', '+78.9']


def test_legacy_image_only_receiver_clears_previous_tip_values(panel):
    widget, receiver, _ = panel
    receiver.queue(complete_frame())
    widget.refresh()
    image = skeleton_image(2)
    receiver.queue(image)
    widget.refresh()
    assert 'skeleton' not in widget.errors
    assert widget.skeleton_frame.image is image
    assert_unavailable(widget.canvases['skeleton'].overlay_lines)


def test_tip_header_stays_inside_existing_three_preview_layout(panel, app):
    widget, receiver, _ = panel
    before = {key: canvas.geometry() for key, canvas in widget.canvases.items()}
    receiver.queue(complete_frame())
    widget.refresh()
    app.processEvents()
    assert set(widget.canvases) == {'depth', 'skeleton', 'tag'}
    assert (widget.width(), widget.height()) == (1000, 320)
    for key, canvas in widget.canvases.items():
        assert canvas.geometry() == before[key]
        bounds = QRect(canvas.mapTo(widget, QPoint(0, 0)), canvas.size())
        assert widget.rect().contains(bounds)
        assert canvas.width() >= 150 and canvas.height() >= 100
