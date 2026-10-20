"""Relative tag pose preview checks without ROS traffic or hardware access."""

from itertools import permutations
import math
import os
import re
import threading
import time
from unittest.mock import Mock

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg import image_preview
from gui_py_pkg.image_preview import (
    IMAGE_STALE_SECONDS, TAG_CACHE_SIZE, TAG_MATCH_MAX_AGE,
    TAG_RELATIVE_POSE_TOPIC, ImageCanvas, LatestImages, TagFrameBuffer,
    TagPreviewFrame, format_tag_pose,
)


def camera_image(sequence=1):
    """Return a small RGB exposure with a nonzero source timestamp."""
    message = CvBridge().cv2_to_imgmsg(np.zeros((24, 32, 3), np.uint8), 'rgb8')
    message.header.frame_id = 'tag_camera_color_optical_frame'
    message.header.stamp.sec = sequence
    message.header.stamp.nanosec = 123
    return message


def detections(sequence=1, ids=(0, 1)):
    message = AprilTagDetectionArray()
    message.header.frame_id = 'tag_camera_color_optical_frame'
    message.header.stamp.sec = sequence
    message.header.stamp.nanosec = 123
    message.detections = [AprilTagDetection(id=tag_id) for tag_id in ids]
    return message


def relative_pose(sequence=1, frame='ID0', position=(0.0123, -0.0456, 0.0789),
                  quaternion=(0.0, 0.0, 0.0, 1.0)):
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = sequence
    message.header.stamp.nanosec = 123
    p, q = message.pose.position, message.pose.orientation
    p.x, p.y, p.z = position
    q.x, q.y, q.z, q.w = quaternion
    return message


def matched_frame(pose=None, received_at=10.0, ids=(0, 1)):
    return TagPreviewFrame(
        camera_image(), detections(ids=ids), received_at,
        'rectified', 'Detected ID 0, 1', pose=pose)


def signed_numbers(line):
    return [float(value) for value in re.findall(r'[+-]\d+\.\d+', line)]


def assert_unavailable(lines):
    assert len(lines) == 3
    assert lines[0] == 'ID0 -> ID1'
    assert any(lines[1:])
    assert not any(signed_numbers(line) for line in lines[1:])


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('order', list(permutations(('image', 'detections', 'pose'))))
def test_pose_matches_exact_exposure_in_every_callback_order(order):
    buffer = TagFrameBuffer()
    image, tags, pose = camera_image(), detections(), relative_pose()
    callbacks = {
        'image': (buffer.receive_image, image),
        'detections': (buffer.receive_detections, tags),
        'pose': (buffer.receive_pose, pose),
    }
    for index, name in enumerate(order):
        callback, message = callbacks[name]
        callback(message, now=10.0 + index * 0.01)
    buffer.receive_image(camera_image(2), now=10.03)
    frame = buffer.snapshot(10.04)
    assert frame.image is image
    assert frame.detections is tags
    assert frame.pose is pose
    assert TAG_RELATIVE_POSE_TOPIC == '/apriltag/tag1_in_tag0/pose'


@pytest.mark.parametrize('sequence,frame,nanosecond', [
    (2, 'ID0', 123), (1, 'ID0', 124), (1, 'ID1', 123),
    (1, 'tag_camera_color_optical_frame', 123), (1, '', 123), (0, 'ID0', 0),
])
def test_pose_rejects_wrong_source_stamp_and_reference_frame(sequence, frame, nanosecond):
    buffer = TagFrameBuffer()
    buffer.receive_image(camera_image(), now=10.0)
    buffer.receive_detections(detections(), now=10.0)
    pose = relative_pose(sequence, frame)
    pose.header.stamp.nanosec = nanosecond
    buffer.receive_pose(pose, now=10.0)
    assert buffer.snapshot(10.01).pose is None


@pytest.mark.parametrize('ids', [(), (0,), (1,), (0, 2), (1, 2)])
def test_pose_requires_both_tags_in_matching_detection_array(ids):
    buffer = TagFrameBuffer()
    buffer.receive_image(camera_image(), now=10.0)
    buffer.receive_detections(detections(ids=ids), now=10.0)
    buffer.receive_pose(relative_pose(), now=10.0)
    assert buffer.snapshot(10.01).pose is None


def test_pose_does_not_attach_to_raw_or_unmatched_images():
    for raw in (False, True):
        buffer = TagFrameBuffer()
        buffer.receive_image(camera_image(), raw=raw, now=10.0)
        buffer.receive_detections(detections(sequence=2), now=10.0)
        buffer.receive_pose(relative_pose(), now=10.0)
        frame = buffer.snapshot(10.01)
        assert frame.detections is None
        assert frame.pose is None


def test_missing_tag_in_new_exposure_clears_previous_pose():
    buffer = TagFrameBuffer()
    buffer.receive_image(camera_image(), now=10.0)
    buffer.receive_detections(detections(), now=10.0)
    buffer.receive_pose(relative_pose(), now=10.0)
    assert buffer.snapshot(10.01).pose is not None
    image = camera_image(2)
    buffer.receive_image(image, now=10.02)
    buffer.receive_detections(detections(2, ids=(0,)), now=10.02)
    frame = buffer.snapshot(10.03)
    assert frame.image is image
    assert frame.pose is None


def test_old_pose_receipt_is_not_revived_by_repeated_image_stamp():
    buffer = TagFrameBuffer()
    buffer.receive_pose(relative_pose(), now=10.0)
    later = 10.0 + TAG_MATCH_MAX_AGE + 0.05
    buffer.receive_image(camera_image(), now=later)
    buffer.receive_detections(detections(), now=later)
    frame = buffer.snapshot(later + 0.01)
    assert frame.detections is not None
    assert frame.pose is None


def test_pose_history_is_bounded_and_expires_without_new_messages():
    buffer = TagFrameBuffer()
    for sequence in range(1, TAG_CACHE_SIZE * 4):
        buffer.receive_pose(relative_pose(sequence), now=10.0)
        assert len(buffer.poses) <= TAG_CACHE_SIZE
    buffer.snapshot(10.0 + IMAGE_STALE_SECONDS + 0.1)
    assert not buffer.poses


def test_pose_callback_updates_snapshot_even_without_new_image(monkeypatch):
    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._latest, node._received_at = {}, {}
    node._tag_buffer = TagFrameBuffer()
    node._tag_signature = None
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: 10.0)
    node.receive('tag', camera_image())
    node.receive_detections(detections())
    first, _ = node.take_latest()
    assert first['tag'].pose is None
    pose = relative_pose()
    node.receive_pose(pose)
    latest, times = node.take_latest()
    assert latest['tag'].pose is pose
    assert node.take_latest() == ({}, times)
    monkeypatch.setattr(image_preview.time, 'monotonic',
                        lambda: 10.0 + TAG_MATCH_MAX_AGE + 0.1)
    expired, _ = node.take_latest()
    assert expired['tag'].pose is None


def test_pose_text_converts_meters_to_signed_millimeters():
    lines = format_tag_pose(matched_frame(relative_pose()), 10.01)
    assert len(lines) == 3
    assert lines[0] == 'ID0 -> ID1'
    assert lines[1].startswith('XYZ [mm]')
    assert lines[2].startswith('RPY [deg]')
    assert signed_numbers(lines[1]) == pytest.approx([12.3, -45.6, 78.9])
    assert signed_numbers(lines[2]) == pytest.approx([0.0, 0.0, 0.0])


@pytest.mark.parametrize('angles', [(90.0, 0.0, 0.0), (0.0, 30.0, 0.0),
                                   (0.0, 0.0, -90.0), (20.0, -30.0, 45.0)])
@pytest.mark.parametrize('scale', [1.0, 2.0, -1.0])
def test_pose_text_uses_normalized_xyzw_and_roll_pitch_yaw_degrees(angles, scale):
    roll, pitch, yaw = (math.radians(value) / 2 for value in angles)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    quaternion = (sr * cp * cy - cr * sp * sy,
                  cr * sp * cy + sr * cp * sy,
                  cr * cp * sy - sr * sp * cy,
                  cr * cp * cy + sr * sp * sy)
    pose = relative_pose(quaternion=tuple(scale * value for value in quaternion))
    lines = format_tag_pose(matched_frame(pose), 10.01)
    assert signed_numbers(lines[2]) == pytest.approx(angles, abs=0.05)


@pytest.mark.parametrize('position,quaternion', [
    ((float('nan'), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ((0.0, float('inf'), 0.0), (0.0, 0.0, 0.0, 1.0)),
    ((0.0, 0.0, float('-inf')), (0.0, 0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, float('nan'), 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, float('inf'), 1.0)),
])
def test_invalid_pose_text_never_renders_numeric_measurements(position, quaternion):
    pose = relative_pose(position=position, quaternion=quaternion)
    assert_unavailable(format_tag_pose(matched_frame(pose), 10.01))


def test_missing_and_stale_pose_text_clear_numeric_measurements():
    assert_unavailable(format_tag_pose(None, 10.01))
    assert_unavailable(format_tag_pose(matched_frame(), 10.01))
    frame = matched_frame(relative_pose())
    assert_unavailable(format_tag_pose(frame, 10.0 + IMAGE_STALE_SECONDS + 0.1))


def test_canvas_overlay_keeps_source_image_untouched(app):
    canvas = ImageCanvas()
    pixels = np.full((24, 32, 3), (11, 22, 33), dtype=np.uint8)
    canvas.set_rgb(pixels)
    lines = ('ID0 -> ID1', 'XYZ [mm] +1.0 +2.0 +3.0',
             'RPY [deg] +0.0 +0.0 +0.0')
    canvas.set_overlay(lines)
    assert canvas.overlay_lines == lines
    assert canvas.image.pixelColor(0, 0).getRgb() == (11, 22, 33, 255)
    canvas.set_overlay(())
    assert canvas.overlay_lines == ()
    canvas.close()


def test_panel_updates_overlay_and_clears_stale_pose_without_new_frame(app, monkeypatch):
    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    now = time.monotonic()
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: now)
    panel = image_preview.ImagePreviewPanel(None)
    panel.timer.stop()
    panel.resize(1000, 320)
    panel.show()
    app.processEvents()
    try:
        frame = matched_frame(relative_pose(), received_at=now)
        receiver.take_latest.return_value = ({'tag': frame}, {'tag': now})
        panel.refresh()
        assert signed_numbers(panel.canvases['tag'].overlay_lines[1]) == [12.3, -45.6, 78.9]
        assert not panel.canvases['depth'].overlay_lines
        assert panel.canvases['skeleton'].overlay_lines[0] == 'hrm_base | XYZ [mm]'
        receiver.take_latest.return_value = ({}, {'tag': now})
        monkeypatch.setattr(image_preview.time, 'monotonic',
                            lambda: now + IMAGE_STALE_SECONDS + 0.1)
        panel.refresh()
        assert_unavailable(panel.canvases['tag'].overlay_lines)
    finally:
        panel.stop()
        panel.close()


def test_panel_hides_pose_when_image_conversion_fails_and_recovers(app, monkeypatch):
    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    now = time.monotonic()
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: now)
    panel = image_preview.ImagePreviewPanel(None)
    panel.timer.stop()
    panel.resize(1000, 320)
    panel.show()
    app.processEvents()
    try:
        first = matched_frame(relative_pose(), received_at=now)
        receiver.take_latest.return_value = ({'tag': first}, {'tag': now})
        panel.refresh()
        assert signed_numbers(panel.canvases['tag'].overlay_lines[1])
        malformed = matched_frame(relative_pose(position=(0.3, 0.4, 0.5)),
                                  received_at=now)
        malformed.image.encoding = 'invalid-encoding'
        receiver.take_latest.return_value = ({'tag': malformed}, {'tag': now})
        panel.refresh()
        assert 'tag' in panel.errors
        assert_unavailable(panel.canvases['tag'].overlay_lines)
        recovered = matched_frame(relative_pose(), received_at=now)
        receiver.take_latest.return_value = ({'tag': recovered}, {'tag': now})
        panel.refresh()
        assert 'tag' not in panel.errors
        assert signed_numbers(panel.canvases['tag'].overlay_lines[1]) == [12.3, -45.6, 78.9]
    finally:
        panel.stop()
        panel.close()


def test_panel_pose_only_arrival_updates_text_without_converting_image_again(app, monkeypatch):
    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    now = time.monotonic()
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: now)
    panel = image_preview.ImagePreviewPanel(None)
    panel.timer.stop()
    panel.resize(1000, 320)
    panel.show()
    app.processEvents()
    canvas = panel.canvases['tag']
    render = Mock(wraps=canvas.set_rgb)
    monkeypatch.setattr(canvas, 'set_rgb', render)
    try:
        first = matched_frame(received_at=now)
        receiver.take_latest.return_value = ({'tag': first}, {'tag': now})
        panel.refresh()
        assert_unavailable(canvas.overlay_lines)
        second = TagPreviewFrame(first.image, first.detections, now,
                                 first.source, first.status, pose=relative_pose())
        receiver.take_latest.return_value = ({'tag': second}, {'tag': now})
        panel.refresh()
        render.assert_called_once()
        assert signed_numbers(canvas.overlay_lines[1]) == [12.3, -45.6, 78.9]
    finally:
        panel.stop()
        panel.close()
