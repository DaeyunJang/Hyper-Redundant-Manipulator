"""Full-image ROI coordinates and atomic startup saves; mocked ROS, no hardware."""

import json
import os
from unittest.mock import Mock

from cv_bridge import CvBridge
import numpy as np
from PyQt5.QtCore import QPoint, QPointF, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QDialog
import pytest

from gui_py_pkg import roi_editor
from gui_py_pkg.roi_editor import (
    COLOR_TOPIC, RoiCanvas, RoiEditorDialog, load_roi_config, save_roi_config, validate_roi,
)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'config_ROI_ref.json'
    path.write_text(json.dumps({'x': 10, 'y': 20, 'w': 50, 'h': 40,
                                'other': {'label': 'do not erase', 'settings': [1, 2, 3]}}))
    return path


@pytest.mark.parametrize('update', [
    {'x': -1}, {'y': -1}, {'w': 0}, {'h': -1}, {'x': True},
    {'w': 2.5}, {'h': '10'}, {'x': None},
])
def test_roi_fields_require_strict_integers_and_positive_extent(update):
    value = {'x': 0, 'y': 0, 'w': 100, 'h': 100, **update}
    with pytest.raises(ValueError):
        validate_roi(value)


def test_roi_bounds_are_rejected_not_clipped():
    original = dict(x=150, y=20, w=51, h=40)
    with pytest.raises(ValueError, match='outside'):
        validate_roi(original, (200, 100))
    assert original == dict(x=150, y=20, w=51, h=40)
    assert validate_roi(dict(x=0, y=0, w=200, h=100), (200, 100))['w'] == 200


def test_load_only_roi_and_save_preserves_extras_mode_and_symlink(config, tmp_path):
    config.chmod(0o640)
    original = json.loads(config.read_text())
    linked = tmp_path / 'installed_roi.json'
    linked.symlink_to(config)
    assert load_roi_config(linked) == {key: original[key] for key in ('x', 'y', 'w', 'h')}
    changed = dict(x=5, y=6, w=100, h=80)
    save_roi_config(linked, changed)
    assert linked.is_symlink()
    assert load_roi_config(config) == changed
    assert json.loads(config.read_text())['other'] == original['other']
    assert config.stat().st_mode & 0o777 == 0o640
    assert not list(tmp_path.glob('.*.tmp'))


def test_failed_atomic_save_leaves_original_and_no_temporary_file(config, monkeypatch):
    original = config.read_bytes()

    def fail_replace(*args):
        raise OSError('simulated atomic replace error')
    monkeypatch.setattr(roi_editor.os, 'replace', fail_replace)
    with pytest.raises(OSError, match='simulated'):
        save_roi_config(config, dict(x=0, y=0, w=20, h=20))
    assert config.read_bytes() == original
    assert not list(config.parent.glob('.*.tmp'))


def test_canvas_letterbox_drag_clipping_reverse_and_original_pixel_scale(app):
    canvas = RoiCanvas()
    canvas.resize(400, 400)
    canvas.set_rgb(np.zeros((100, 200, 3), np.uint8))
    assert canvas.image_rect().x() == 0 and canvas.image_rect().y() == 100
    assert canvas.image_rect().width() == 400 and canvas.image_rect().height() == 200
    assert canvas.image_point(QPoint(100, 20)) is None
    start = canvas.image_point(QPoint(40, 140))
    end = canvas.image_point(QPoint(240, 260))
    assert start == QPointF(20, 20) and end == QPointF(120, 80)
    assert canvas.drag_roi(start, end) == dict(x=20, y=20, w=100, h=60)
    assert canvas.drag_roi(end, start) == canvas.drag_roi(start, end)
    clipped = canvas.image_point(QPoint(500, 500), clip=True)
    assert canvas.drag_roi(start, clipped) == dict(x=20, y=20, w=180, h=80)
    assert canvas.drag_roi(start, start) is None
    captured = []
    canvas.roi_changed.connect(captured.append)
    QTest.mousePress(canvas, Qt.LeftButton, pos=QPoint(40, 140))
    QTest.mouseRelease(canvas, Qt.LeftButton, pos=QPoint(240, 260))
    assert captured[-1] == dict(x=20, y=20, w=100, h=60)
    before = len(captured)
    QTest.mousePress(canvas, Qt.LeftButton, pos=QPoint(40, 20))
    QTest.mouseRelease(canvas, Qt.LeftButton, pos=QPoint(240, 260))
    assert len(captured) == before
    canvas.close()


def test_canvas_qimage_owns_full_resolution_buffer(app):
    canvas = RoiCanvas()
    pixels = np.full((100, 200, 3), [255, 0, 0], dtype=np.uint8)
    canvas.set_rgb(pixels)
    pixels[:] = 0
    assert canvas.image.size().width() == 200
    assert canvas.image.size().height() == 100
    assert canvas.image.pixelColor(0, 0).red() == 255
    canvas.close()


@pytest.fixture
def receiver_factory(monkeypatch):
    receivers = []

    def create(context, topics):
        assert topics == {'color': COLOR_TOPIC}
        receiver = Mock()
        receiver.take_latest.return_value = ({}, {})
        receivers.append(receiver)
        return receiver
    monkeypatch.setattr(roi_editor, 'LatestImages', create)
    return receivers


def supply_snapshot(dialog, receiver, width=200, height=100):
    message = CvBridge().cv2_to_imgmsg(np.zeros((height, width, 3), np.uint8), encoding='rgb8')
    receiver.take_latest.return_value = ({'color': message}, {})
    dialog.poll_snapshot()


def test_dialog_subscribes_only_when_shown_and_stops_after_one_frame(
        app, config, receiver_factory):
    dialog = RoiEditorDialog(None, load_roi_config(config), config)
    assert not receiver_factory
    assert not dialog.apply_button.isEnabled()
    assert dialog.selected_roi is None
    dialog.show()
    assert len(receiver_factory) == 1
    receiver = receiver_factory[0]
    supply_snapshot(dialog, receiver)
    assert dialog.apply_button.isEnabled()
    assert not dialog.timer.isActive()
    receiver.close.assert_called_once()
    assert dialog.receiver is None
    dialog.refresh_button.click()
    assert len(receiver_factory) == 2 and not dialog.apply_button.isEnabled()
    dialog.reject()
    receiver_factory[1].close.assert_called_once()
    assert not dialog.timer.isActive()
    dialog.close()
    receiver_factory[1].close.assert_called_once()


def test_numeric_edit_full_image_and_accept_without_save(app, config, receiver_factory):
    original = config.read_bytes()
    dialog = RoiEditorDialog(None, load_roi_config(config), config)
    dialog.show()
    supply_snapshot(dialog, receiver_factory[-1])
    dialog.spins['x'].setValue(160)
    assert not dialog.apply_button.isEnabled()
    assert dialog.current_roi()['w'] == 50
    dialog.accept()
    assert dialog.selected_roi is None and config.read_bytes() == original
    dialog.full_image_button.click()
    assert dialog.current_roi() == dict(x=0, y=0, w=200, h=100)
    assert dialog.canvas.roi == dialog.current_roi()
    dialog.save_checkbox.setChecked(False)
    dialog.accept()
    assert dialog.result() == QDialog.Accepted
    assert dialog.selected_roi == dict(x=0, y=0, w=200, h=100)
    assert config.read_bytes() == original
    assert dialog.closed


def test_accept_persists_only_after_confirmation_and_preserves_extras(
        app, config, receiver_factory):
    original = config.read_bytes()
    dialog = RoiEditorDialog(None, load_roi_config(config), config)
    dialog.show()
    supply_snapshot(dialog, receiver_factory[-1])
    assert dialog.save_checkbox.isChecked()
    dialog.spins['x'].setValue(30)
    assert config.read_bytes() == original
    dialog.accept()
    assert dialog.result() == QDialog.Accepted
    assert load_roi_config(config)['x'] == 30
    assert json.loads(config.read_text())['other']['label'] == 'do not erase'


def test_apply_guard_rechecks_before_saving_or_accepting(app, config, receiver_factory):
    original = config.read_bytes()
    guard = Mock(return_value='Stop the estimator before changing ROI.')
    dialog = RoiEditorDialog(None, load_roi_config(config), config, apply_guard=guard)
    dialog.show()
    supply_snapshot(dialog, receiver_factory[-1])
    dialog.spins['x'].setValue(30)
    dialog.accept()
    guard.assert_called_once()
    assert dialog.selected_roi is None and not dialog.closed
    assert config.read_bytes() == original
    assert 'Stop the estimator' in dialog.status.text()
    dialog.reject()


@pytest.mark.parametrize('action', ['reject', 'close', 'timeout'])
def test_no_image_cleanup_and_no_save(app, config, receiver_factory, action):
    original = config.read_bytes()
    dialog = RoiEditorDialog(None, load_roi_config(config), config)
    dialog.show()
    receiver = receiver_factory[-1]
    if action == 'timeout':
        dialog.snapshot_deadline = 0
        dialog.poll_snapshot()
        assert 'No full RGB' in dialog.status.text()
        assert not dialog.apply_button.isEnabled()
        dialog.close()
    else:
        getattr(dialog, action)()
    receiver.close.assert_called_once()
    assert not dialog.timer.isActive()
    assert dialog.selected_roi is None and config.read_bytes() == original


def test_failed_save_keeps_dialog_open_without_selected_roi(
        app, config, receiver_factory, monkeypatch):
    dialog = RoiEditorDialog(None, load_roi_config(config), config)
    dialog.show()
    supply_snapshot(dialog, receiver_factory[-1])
    monkeypatch.setattr(roi_editor, 'save_roi_config', Mock(side_effect=OSError('not writable')))
    dialog.accept()
    assert dialog.selected_roi is None and not dialog.closed
    assert 'not writable' in dialog.status.text()
    dialog.reject()
