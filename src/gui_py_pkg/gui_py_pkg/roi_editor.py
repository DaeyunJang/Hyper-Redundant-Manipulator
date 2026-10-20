"""Snapshot-based, startup-only estimator ROI editor; never controls hardware."""

import json
import math
import os
from pathlib import Path
import stat
import tempfile
import time

from cv_bridge import CvBridge
import numpy as np
from PyQt5.QtCore import QPointF, QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen
from PyQt5.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from gui_py_pkg.image_preview import LatestImages


COLOR_TOPIC = '/camera/camera/color/image_rect_raw'
ROI_KEYS = ('x', 'y', 'w', 'h')
MAX_PIXEL = 2**31 - 1


def validate_roi(roi, image_size=None):
    """Use top-left x/y and positive width/height; never truncate to fit."""
    if not isinstance(roi, dict):
        raise ValueError('ROI must be a JSON object with integer x, y, w and h.')
    result = {}
    for name in ROI_KEYS:
        value = roi.get(name)
        minimum = 0 if name in ('x', 'y') else 1
        if type(value) is not int or not minimum <= value <= MAX_PIXEL:
            raise ValueError(f'ROI {name} must be an integer from {minimum} to {MAX_PIXEL}.')
        result[name] = value
    if image_size is not None:
        width, height = image_size
        if result['x'] + result['w'] > width or result['y'] + result['h'] > height:
            raise ValueError(f'ROI is outside the full {width} × {height} image. '
                             'Adjust x/y/w/h or choose Full image; values are not clipped.')
    return result


def load_roi_config(path):
    """Read only the ROI fields; ignore but never remove other settings."""
    with Path(path).expanduser().open(encoding='utf-8') as handle:
        return validate_roi(json.load(handle))


def save_roi_config(path, roi):
    """Atomically update four keys in the resolved source, preserving symlinks."""
    roi = validate_roi(roi)
    destination = Path(path).expanduser().resolve(strict=True)
    with destination.open(encoding='utf-8') as handle:
        existing = json.load(handle)
    if not isinstance(existing, dict):
        raise ValueError('ROI configuration must contain a JSON object.')
    existing.update(roi)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{destination.name}.', suffix='.tmp', dir=destination.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, stat.S_IMODE(destination.stat().st_mode))
        with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
            descriptor = None
            json.dump(existing, handle, ensure_ascii=False, indent=4, allow_nan=False)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary.exists():
            temporary.unlink()


class RoiCanvas(QWidget):
    """Letterboxed full-resolution image with an original-pixel drag rectangle."""

    roi_changed = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = QImage()
        self.roi = None
        self.drag_start = None
        self.setMinimumSize(320, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.CrossCursor)

    def sizeHint(self):
        return QSize(800, 500)

    def set_rgb(self, pixels):
        pixels = np.ascontiguousarray(pixels)
        if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
            raise ValueError('ROI preview requires a nonempty RGB8 image.')
        height, width = pixels.shape[:2]
        if not width or not height:
            raise ValueError('ROI preview received an empty image.')
        self.image = QImage(pixels.data, width, height, pixels.strides[0],
                            QImage.Format_RGB888).copy()
        self.drag_start = None
        self.update()

    def set_roi(self, roi):
        self.roi = validate_roi(roi)
        self.update()

    def image_rect(self):
        if self.image.isNull():
            return QRectF()
        scale = min(self.width() / self.image.width(), self.height() / self.image.height())
        width, height = self.image.width() * scale, self.image.height() * scale
        return QRectF((self.width() - width) / 2, (self.height() - height) / 2, width, height)

    def image_point(self, point, *, clip=False):
        """Map widget/letterbox coordinates to image pixel edges, not centers."""
        area = self.image_rect()
        if area.isEmpty() or (not clip and not area.contains(QPointF(point))):
            return None
        x = (point.x() - area.x()) * self.image.width() / area.width()
        y = (point.y() - area.y()) * self.image.height() / area.height()
        return QPointF(max(0.0, min(float(self.image.width()), x)),
                       max(0.0, min(float(self.image.height()), y)))

    def drag_roi(self, start, end):
        """Cover the dragged pixel interval; end edges are exclusive."""
        if self.image.isNull() or start == end:
            return None
        x0, x1 = sorted((start.x(), end.x()))
        y0, y1 = sorted((start.y(), end.y()))
        left, top = math.floor(x0), math.floor(y0)
        right, bottom = math.ceil(x1), math.ceil(y1)
        if right <= left or bottom <= top:
            return None
        return dict(x=left, y=top, w=right - left, h=bottom - top)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_start = self.image_point(event.pos())

    def _drag_to(self, point):
        if self.drag_start is None:
            return
        end = self.image_point(point, clip=True)
        roi = self.drag_roi(self.drag_start, end) if end is not None else None
        if roi is not None:
            self.set_roi(roi)
            self.roi_changed.emit(dict(roi))

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self._drag_to(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_to(event.pos())
            self.drag_start = None

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#121820'))
        if self.image.isNull():
            painter.setPen(QColor('#b6c2cf'))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for full RGB snapshot…')
            return
        area = self.image_rect()
        painter.drawImage(area, self.image)
        if self.roi is None:
            return
        scale_x = area.width() / self.image.width()
        scale_y = area.height() / self.image.height()
        roi = self.roi
        valid = (roi['x'] + roi['w'] <= self.image.width() and
                 roi['y'] + roi['h'] <= self.image.height())
        painter.setClipRect(area)
        painter.setPen(QPen(QColor('#25e06e' if valid else '#ff5252'), 2))
        painter.drawRect(QRectF(area.x() + roi['x'] * scale_x,
                                area.y() + roi['y'] * scale_y,
                                roi['w'] * scale_x, roi['h'] * scale_y))


class RoiEditorDialog(QDialog):
    """Take one full RGB snapshot, edit ROI, then close its subscription."""

    SNAPSHOT_TIMEOUT_SEC = 6.0

    def __init__(self, context, roi, config_path, parent=None, apply_guard=None):
        super().__init__(parent)
        initial = validate_roi(roi)
        self.context = context
        self.config_path = Path(config_path)
        self.apply_guard = apply_guard
        self.selected_roi = None
        self.receiver = None
        self.snapshot_valid = False
        self.initial_request_made = False
        self.closed = False
        self.bridge = CvBridge()
        self.setWindowTitle('Estimator ROI — full RGB image')
        self.resize(1000, 760)
        layout = QVBoxLayout(self)
        explanation = QLabel(
            'Drag on the full image or edit pixel values. x/y are the top-left corner; '
            'w/h are width/height. This sets the next estimator launch only, '
            'not the camera stream or AprilTag input.')
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.canvas = RoiCanvas()
        self.canvas.set_roi(initial)
        self.canvas.roi_changed.connect(self._set_roi)
        layout.addWidget(self.canvas, 1)
        controls = QHBoxLayout()
        self.spins = {}
        for name in ROI_KEYS:
            controls.addWidget(QLabel(name))
            spin = QSpinBox()
            spin.setRange(0 if name in ('x', 'y') else 1, MAX_PIXEL)
            spin.setValue(initial[name])
            spin.valueChanged.connect(self._numeric_changed)
            self.spins[name] = spin
            controls.addWidget(spin)
        self.full_image_button = QPushButton('Full image')
        self.full_image_button.clicked.connect(self._full_image)
        controls.addWidget(self.full_image_button)
        self.refresh_button = QPushButton('Refresh snapshot')
        self.refresh_button.clicked.connect(self.request_snapshot)
        controls.addWidget(self.refresh_button)
        layout.addLayout(controls)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.save_checkbox = QCheckBox('Save as startup ROI')
        self.save_checkbox.setChecked(True)
        self.save_checkbox.setToolTip(f'Only Apply writes x/y/w/h to {self.config_path}.')
        layout.addWidget(self.save_checkbox)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.apply_button = self.buttons.button(QDialogButtonBox.Ok)
        self.apply_button.setText('Apply ROI')
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_snapshot)
        self._validate_selection()

    def showEvent(self, event):
        super().showEvent(event)
        if not self.initial_request_made:
            self.initial_request_made = True
            self.request_snapshot()

    def current_roi(self):
        return {name: spin.value() for name, spin in self.spins.items()}

    def _set_roi(self, roi):
        for name, value in roi.items():
            self.spins[name].blockSignals(True)
            self.spins[name].setValue(value)
            self.spins[name].blockSignals(False)
        self._numeric_changed()

    def _numeric_changed(self):
        self.canvas.set_roi(self.current_roi())
        self._validate_selection()

    def _full_image(self):
        if self.snapshot_valid:
            self._set_roi(dict(x=0, y=0, w=self.canvas.image.width(), h=self.canvas.image.height()))

    def _validate_selection(self):
        valid = False
        if not self.snapshot_valid:
            text = 'A full RGB snapshot is required to check ROI bounds before Apply.'
        else:
            try:
                validate_roi(self.current_roi(),
                             (self.canvas.image.width(), self.canvas.image.height()))
                text = (f'Full image: {self.canvas.image.width()} × {self.canvas.image.height()} '
                        'pixels. Green rectangle is the estimator ROI; no pixels are rescaled.')
                valid = True
            except ValueError as error:
                text = str(error)
        self.status.setText(text)
        self.status.setStyleSheet('color: #188038;' if valid else 'color: #d93025;')
        self.apply_button.setEnabled(valid)
        self.full_image_button.setEnabled(self.snapshot_valid)
        return valid

    def request_snapshot(self):
        if self.closed:
            return
        self._stop_receiver()
        self.snapshot_valid = False
        self._validate_selection()
        try:
            self.receiver = LatestImages(self.context, topics={'color': COLOR_TOPIC})
        except Exception as error:
            self.status.setText(f'Cannot subscribe to full RGB: {error}')
            return
        self.snapshot_deadline = time.monotonic() + self.SNAPSHOT_TIMEOUT_SEC
        self.status.setText(f'Waiting for a snapshot from {COLOR_TOPIC}…')
        self.timer.start(50)

    def poll_snapshot(self):
        if self.receiver is None:
            return
        latest, _ = self.receiver.take_latest()
        message = latest.get('color')
        if message is None:
            if time.monotonic() >= self.snapshot_deadline:
                self._stop_receiver()
                self.status.setText(
                    'No full RGB image received. Start the camera, then Refresh snapshot.')
            return
        # No ongoing subscriber or image conversion during rectangle editing.
        self._stop_receiver()
        try:
            pixels = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            if pixels.shape[:2] != (message.height, message.width):
                raise ValueError('Converted RGB dimensions do not match the full source image.')
            self.canvas.set_rgb(pixels)
            self.snapshot_valid = True
            self._validate_selection()
        except Exception as error:
            self.snapshot_valid = False
            self._validate_selection()
            self.status.setText(f'Cannot show full RGB snapshot: {error}')

    def _stop_receiver(self):
        self.timer.stop()
        receiver, self.receiver = self.receiver, None
        if receiver is not None:
            receiver.close()

    def accept(self):
        for spin in self.spins.values():
            spin.interpretText()
        if not self._validate_selection():
            return
        selected = validate_roi(self.current_roi(),
                                (self.canvas.image.width(), self.canvas.image.height()))
        if self.apply_guard is not None:
            try:
                blocked = self.apply_guard()
            except Exception as error:
                blocked = f'Cannot verify whether ROI changes are safe: {error}'
            if blocked:
                self.status.setText(str(blocked))
                self.status.setStyleSheet('color: #d93025;')
                return
        if self.save_checkbox.isChecked():
            try:
                save_roi_config(self.config_path, selected)
            except (OSError, ValueError, TypeError) as error:
                self.status.setText(f'ROI was not saved: {error}')
                self.status.setStyleSheet('color: #d93025;')
                return
        self.selected_roi = selected
        super().accept()

    def done(self, result):
        self.closed = True
        self._stop_receiver()
        super().done(result)

    def closeEvent(self, event):
        self.closed = True
        self._stop_receiver()
        super().closeEvent(event)
