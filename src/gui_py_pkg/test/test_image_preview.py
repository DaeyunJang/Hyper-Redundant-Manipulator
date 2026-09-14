"""GUI preview conversion/layout without camera, motor or ROS network access."""

import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from cv_bridge import CvBridge
import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from custom_interfaces.msg import LoadcellState
from sensor_msgs.msg import Image

from gui_py_pkg.image_preview import (
    IMAGE_TOPICS, ImageCanvas, LatestImages, preview_rgb,
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
