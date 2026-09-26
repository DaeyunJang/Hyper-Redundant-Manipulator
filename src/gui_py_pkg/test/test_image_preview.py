"""GUI preview conversion/layout without camera, motor or ROS network access."""

import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from custom_interfaces.msg import LoadcellState
from sensor_msgs.msg import Image

from gui_py_pkg.image_preview import (
    IMAGE_TOPICS, TAG_CACHE_SIZE, ImageCanvas, LatestImages, TagFrameBuffer,
    TagPreviewFrame, header_key, overlay_tag_corners, preview_rgb,
)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('encoding', ['16UC1', '32FC1'])
def test_depth_near_red_far_blue_invalid_black(encoding):
    bridge = CvBridge()
    metres = np.array([[0.0, 0.1, 0.3, 0.5]], dtype=np.float32)
    values = ((metres * 1000).astype(np.uint16)
              if encoding == '16UC1' else metres)
    original = values.copy()
    msg = bridge.cv2_to_imgmsg(values, encoding=encoding)
    rgb = preview_rgb(msg, bridge, depth=True, near_m=0.1, far_m=0.5)
    assert np.all(rgb[0, 0] == 0)
    assert rgb[0, 1, 0] > rgb[0, 1, 2]  # red at near limit
    assert rgb[0, 3, 2] > rgb[0, 3, 0]  # blue at far limit
    np.testing.assert_array_equal(values, original)


def test_float_depth_nan_infinity_negative_are_black():
    bridge = CvBridge()
    values = np.array([[np.nan, np.inf, -np.inf, -0.1, 0.0]], np.float32)
    msg = bridge.cv2_to_imgmsg(values, encoding='32FC1')
    assert not preview_rgb(msg, bridge, depth=True).any()


@pytest.mark.parametrize('near,far', [(0.6, 0.1), (0.1, 0.1), (-1.0, 0.6)])
def test_bad_display_range_is_rejected(near, far):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(np.ones((4, 4), np.uint16), encoding='16UC1')
    with pytest.raises(ValueError, match='Depth range'):
        preview_rgb(msg, bridge, depth=True, near_m=near, far_m=far)


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8'])
def test_skeleton_encodings_and_preview_size(encoding):
    bridge = CvBridge()
    rgb = np.zeros((480, 848, 3), np.uint8)
    rgb[:, :, 0] = 255
    source = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
    pixels = bridge.imgmsg_to_cv2(source, desired_encoding=encoding)
    msg = bridge.cv2_to_imgmsg(pixels, encoding=encoding)
    result = preview_rgb(msg, bridge)
    assert result.shape == (362, 640, 3)
    if encoding != 'mono8':
        np.testing.assert_array_equal(result[0, 0], [255, 0, 0])
    assert result.flags.c_contiguous


def test_unsupported_depth_encoding():
    msg = Image(encoding='mono8')
    with pytest.raises(ValueError, match='Unsupported depth encoding'):
        preview_rgb(msg, CvBridge(), depth=True)


def test_depth_padded_rows_and_big_endian():
    values = np.array([[100, 500, 999], [500, 100, 999]], dtype='>u2')
    msg = Image(height=2, width=2, step=6, is_bigendian=1,
                encoding='16UC1', data=values.tobytes())
    rgb = preview_rgb(msg, CvBridge(), depth=True, near_m=0.1, far_m=0.5)
    assert rgb.shape == (2, 2, 3)
    assert rgb[0, 0, 0] > rgb[0, 0, 2]
    np.testing.assert_array_equal(rgb[0, 0], rgb[1, 1])


def test_latest_frame_slot_discards_backlog():
    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._latest = {}
    node._received_at = {}
    first, last, skeleton = object(), object(), object()
    node.receive('depth', first)
    node.receive('depth', last)
    node.receive('skeleton', skeleton)
    frames, times = node.take_latest()
    assert frames == {'depth': last, 'skeleton': skeleton}
    assert node.take_latest() == ({}, times)


def test_qimage_owns_numpy_buffer(app):
    canvas = ImageCanvas()
    pixels = np.full((20, 30, 3), [255, 0, 0], np.uint8)
    canvas.set_rgb(pixels)
    pixels[:] = 0
    assert canvas.image.pixelColor(0, 0).red() == 255


def test_motor_enable_sensor_zero_and_previews_have_correct_parents(app, monkeypatch):
    from gui_py_pkg import gui_node
    from gui_py_pkg import image_preview

    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    monkeypatch.setattr(gui_node.MyGUI, 'init_fts_plot', lambda self: None)
    node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False)
    window = gui_node.MyGUI(node)
    try:
        assert window.control_tab.isAncestorOf(window.motor_output_checkbox)
        assert not window.system_tab.isAncestorOf(window.motor_output_checkbox)
        assert not window.motor_output_checkbox.isChecked()
        assert window.sensor_raw_widget.isAncestorOf(window.zero_button)
        assert not window.control_tab.isAncestorOf(window.zero_button)
        assert window.control_tab.isAncestorOf(window.image_preview)
        window.zero_button.click()
        node.send_request_set_zero.assert_called_once()
        node.send_request_move_motor_direct.assert_not_called()
        assert IMAGE_TOPICS['depth'] == '/camera/camera/aligned_depth_to_color/image_raw'
        # A stale ROS name after exit must not enable External or Start.
        node.discovered_node_names.return_value = set()
        node.last_motor_state_time = None
        node.last_fts_data_time = None
        node.last_loadcell_data_time = None
        control = window.system_manager.processes['robot_control']
        control.status = Mock(return_value='graph_pending')
        window.system_manager.start = Mock()
        window.system_manager.stop = Mock()
        window.update_system_status()
        widgets = window.component_widgets['robot_control']
        assert widgets['status'].text() == '● STOPPED*'
        assert widgets['button'].text() == 'ROS cleanup…'
        assert not widgets['button'].isEnabled()
        window.toggle_component('robot_control')
        window.system_manager.start.assert_not_called()
        window.system_manager.stop.assert_not_called()
        control.status.return_value = 'stopped'
        window.update_system_status()
        assert widgets['button'].text() == 'Start'
        assert widgets['button'].isEnabled()
    finally:
        window.close()
    receiver.close.assert_called_once()


@pytest.mark.parametrize('values,expected', [
    ([10.0, 20.0, 30.0, 40.0], ['10.0', '20.0', '30.0', '40.0']),
    ([1.0, 2.0], ['1.0', '2.0', '—', '—']),
    ([], ['—', '—', '—', '—']),
])
def test_all_four_loadcell_channels_update_and_missing_values_clear(app, values, expected):
    from gui_py_pkg.gui_node import MyGUI

    surface = QWidget()
    node = SimpleNamespace(num_loadcells=4, loadcell_data=LoadcellState())
    view = SimpleNamespace(node=node, layout_global=QVBoxLayout(surface))
    MyGUI.init_loadcell_ui(view)
    assert [label.text() for label in view.lc_sub_label_list] == [
        f'Loadcell #{i} (g)' for i in range(1, 5)]
    assert all(field.isReadOnly() for field in view.lc_sub_line_edit_list)
    node.loadcell_data.stress = [100.0] * 4
    MyGUI.update_loadcell(view)
    node.loadcell_data.stress = values
    MyGUI.update_loadcell(view)
    assert [field.text() for field in view.lc_sub_line_edit_list] == expected
    surface.close()


def test_preview_reports_stale_images_and_bad_encoding(app, monkeypatch):
    from gui_py_pkg import image_preview

    receiver = Mock()
    receiver.take_latest.return_value = ({}, {'depth': time.monotonic() - 5})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    panel = image_preview.ImagePreviewPanel(None)
    panel.show()
    app.processEvents()
    try:
        panel.refresh()
        assert 'Stale' in panel.status_labels['depth'].text()
        assert 'No image' in panel.status_labels['skeleton'].text()
        receiver.take_latest.return_value = (
            {'depth': Image(encoding='mono8')}, {'depth': time.monotonic()})
        panel.refresh()
        assert 'Unsupported depth encoding' in panel.status_labels['depth'].text()
    finally:
        panel.stop()
        panel.close()


def tag_image(sequence=1, frame='tag_camera_color_optical_frame', width=640):
    """Build a small, precisely timestamped test camera exposure."""
    msg = CvBridge().cv2_to_imgmsg(np.zeros((120, width, 3), np.uint8), 'rgb8')
    msg.header.frame_id = frame
    msg.header.stamp.sec = sequence
    return msg


def tag_detections(sequence=1, ids=(0, 1), frame='tag_camera_color_optical_frame'):
    msg = AprilTagDetectionArray()
    msg.header.frame_id = frame
    msg.header.stamp.sec = sequence
    for tag_id in ids:
        detection = AprilTagDetection(id=tag_id)
        for corner, (x, y) in zip(detection.corners,
                                  [(100, 30), (200, 30), (200, 90), (100, 90)]):
            corner.x, corner.y = float(x), float(y)
        msg.detections.append(detection)
    return msg


@pytest.mark.parametrize('detections_first', [False, True])
def test_tag_matches_exact_source_exposure_in_either_arrival_order(detections_first):
    buffer = TagFrameBuffer()
    image, detections = tag_image(), tag_detections()
    callbacks = [(buffer.receive_image, image),
                 (buffer.receive_detections, detections)]
    for callback, msg in reversed(callbacks) if detections_first else callbacks:
        callback(msg, now=10.0)
    # A newer image does not receive the earlier image's corner locations.
    buffer.receive_image(tag_image(2), now=10.02)
    frame = buffer.snapshot(10.03)
    assert frame.image is image
    assert frame.detections is detections
    assert header_key(frame.image) == header_key(frame.detections)
    assert frame.source == 'rectified'
    assert frame.status == 'Detected ID 0, 1'


@pytest.mark.parametrize('sequence,frame', [
    (2, 'tag_camera_color_optical_frame'), (1, 'camera_color_optical_frame'),
    (0, ''),
])
def test_tag_rejects_stamp_frame_and_empty_header_mismatches(sequence, frame):
    buffer = TagFrameBuffer()
    buffer.receive_image(tag_image(), now=10)
    buffer.receive_detections(tag_detections(sequence, frame=frame), now=10)
    assert buffer.snapshot(10.01).detections is None


def test_empty_detection_array_clears_tag_outline():
    buffer = TagFrameBuffer()
    buffer.receive_image(tag_image(), now=10)
    buffer.receive_detections(tag_detections(), now=10)
    assert 'ID 0' in buffer.snapshot(10).status
    next_image = tag_image(2)
    buffer.receive_image(next_image, now=10.03)
    empty = tag_detections(2, ids=())
    buffer.receive_detections(empty, now=10.04)
    frame = buffer.snapshot(10.05)
    assert frame.image is next_image
    assert frame.detections is empty
    assert frame.status == 'No tags detected'


def test_late_detections_do_not_freeze_latest_camera_image():
    buffer = TagFrameBuffer()
    buffer.receive_image(tag_image(), now=10)
    buffer.receive_detections(tag_detections(), now=10.01)
    newest = tag_image(2)
    buffer.receive_image(newest, now=10.3)
    frame = buffer.snapshot(10.3)
    assert frame.image is newest
    assert frame.detections is None
    buffer.receive_image(tag_image(3), now=11.2)
    assert 'Detections stale' in buffer.snapshot(11.2).status


def test_old_detection_receipt_is_not_revived_by_repeated_image_header():
    buffer = TagFrameBuffer()
    buffer.receive_detections(tag_detections(), now=10)
    buffer.receive_image(tag_image(), now=10.3)
    assert buffer.snapshot(10.31).detections is None


def test_raw_fallback_never_uses_rectified_detections():
    buffer = TagFrameBuffer()
    image = tag_image()
    buffer.receive_image(image, raw=True, now=10)
    buffer.receive_detections(tag_detections(), now=10)
    assert buffer.wants_raw(10)
    frame = buffer.snapshot(10.01)
    assert frame.image is image
    assert frame.detections is None
    assert frame.source == 'raw'
    assert 'no overlay' in frame.status
    buffer.receive_image(tag_image(2), now=10.1)
    assert not buffer.wants_raw(10.2)
    assert buffer.snapshot(10.2).source == 'rectified'
    assert buffer.wants_raw(11.2)


def test_tag_history_bounded_and_pruned_without_new_frames():
    buffer = TagFrameBuffer()
    assert buffer.snapshot(10) is None
    for sequence in range(100):
        buffer.receive_image(tag_image(sequence + 1), now=10)
        buffer.receive_detections(tag_detections(sequence + 1), now=10)
    assert len(buffer.images) == TAG_CACHE_SIZE
    assert len(buffer.detections) == TAG_CACHE_SIZE
    frame = buffer.snapshot(12)
    assert frame.status == 'Image stale'
    assert frame.detections is None
    assert not buffer.images
    assert not buffer.detections


def test_overlay_scales_corners_with_resize_and_preserves_input():
    pixels = np.zeros((60, 320, 3), np.uint8)
    result = overlay_tag_corners(pixels, 640, 120, tag_detections(ids=(0,)))
    assert result.shape == pixels.shape
    assert result[15, 75].any()  # top edge: x=100..200 -> 50..100, y=30 -> 15
    assert result[45, 75].any()
    assert not result[55, 200].any()
    assert not pixels.any()


def test_overlay_invalid_corners_are_ignored_and_empty_array_has_no_edges():
    pixels = np.zeros((60, 320, 3), np.uint8)
    detections = tag_detections(ids=(0,))
    detections.detections[0].corners[0].x = float('nan')
    assert not overlay_tag_corners(pixels, 640, 120, detections).any()
    assert not overlay_tag_corners(pixels, 640, 120, tag_detections(ids=())).any()
    with pytest.raises(ValueError, match='dimensions'):
        overlay_tag_corners(pixels, 0, 120, detections)


def test_tag_snapshot_changes_only_for_new_image_detection_or_status():
    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._latest, node._received_at = {}, {}
    node._tag_buffer = TagFrameBuffer()
    node._tag_signature = None
    image, detections = tag_image(), tag_detections()
    node.receive('tag', image)
    latest, times = node.take_latest()
    assert latest['tag'].image is image
    assert latest['tag'].detections is None
    assert node.take_latest() == ({}, times)
    node.receive_detections(detections)
    latest, times = node.take_latest()
    assert latest['tag'].detections is detections
    assert node.take_latest() == ({}, times)


def test_raw_fallback_subscription_exists_only_without_rectification(monkeypatch):
    from gui_py_pkg import image_preview

    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._tag_buffer = TagFrameBuffer()
    node._started_at = 10.0
    node._raw_subscription = None
    node._image_qos = object()
    subscription = object()
    node.create_subscription = Mock(return_value=subscription)
    node.destroy_subscription = Mock()
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: 10.5)
    node._update_raw_subscription()
    node.create_subscription.assert_not_called()
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: 11.1)
    node._update_raw_subscription()
    node._update_raw_subscription()
    node.create_subscription.assert_called_once()
    assert node.create_subscription.call_args.args[1] == image_preview.TAG_RAW_TOPIC
    assert node._raw_subscription is subscription
    node._tag_buffer.receive_image(tag_image(), now=11.1)
    node._update_raw_subscription()
    node.destroy_subscription.assert_called_once_with(subscription)
    assert node._raw_subscription is None
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: 12.2)
    node._update_raw_subscription()
    assert node.create_subscription.call_count == 2


def test_roi_topics_do_not_enable_detector_or_fallback():
    node = LatestImages.__new__(LatestImages)
    node._lock = threading.Lock()
    node._latest, node._received_at = {}, {}
    node._tag_buffer = None
    node.create_subscription = Mock()
    node._update_raw_subscription()
    image = tag_image()
    node.receive('color', image)
    latest, _ = node.take_latest()
    assert latest == {'color': image}
    node.create_subscription.assert_not_called()


def test_tag_preview_three_tiles_status_and_matching_overlay(app, monkeypatch):
    from gui_py_pkg import image_preview

    receiver = Mock()
    now = time.monotonic()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    panel = image_preview.ImagePreviewPanel(None)
    panel.show()
    app.processEvents()
    try:
        assert list(panel.canvases) == ['depth', 'skeleton', 'tag']
        panel.refresh()
        assert panel.status_labels['tag'].text() == 'No image received'
        frame = TagPreviewFrame(tag_image(), tag_detections(), now,
                                'rectified', 'Detected ID 0, 1')
        receiver.take_latest.return_value = ({'tag': frame}, {'tag': now})
        panel.refresh()
        assert panel.tag_frame is frame
        assert 'ID 0, 1' in panel.status_labels['tag'].text()
        assert not panel.canvases['tag'].image.isNull()
        empty = TagPreviewFrame(tag_image(2), tag_detections(2, ids=()), now,
                                'rectified', 'No tags detected')
        receiver.take_latest.return_value = ({'tag': empty}, {'tag': now})
        panel.refresh()
        assert panel.status_labels['tag'].text() == 'No tags detected'
        assert panel.canvases['tag'].image.pixelColor(10, 10).red() == 0
        receiver.take_latest.return_value = ({}, {'tag': now - 5})
        panel.refresh()
        assert 'Stale' in panel.status_labels['tag'].text()
    finally:
        panel.stop()
        panel.close()
