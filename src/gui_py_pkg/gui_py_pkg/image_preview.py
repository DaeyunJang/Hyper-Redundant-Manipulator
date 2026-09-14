"""Read-only, latest-frame image previews; independent of motor callbacks."""

import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from PyQt5.QtCore import QRect, QSize, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter
from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QSizePolicy,
    QVBoxLayout, QWidget,
)
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


IMAGE_TOPICS = {
    'depth': '/camera/camera/aligned_depth_to_color/image_raw',
    'skeleton': '/estimated_segment_skeleton_image',
}


class LatestImages(Node):
    """Only subscribe/cache here. Qt owns image conversion and all widgets."""

    def __init__(self, context):
        """Start an image-only executor with sensor-compatible QoS."""
        # Do not inherit the gui_node name remap passed by ros2 launch.
        super().__init__('gui_image_preview', context=context,
                         use_global_arguments=False)
        self._lock = threading.Lock()
        self._latest = {}
        self._received_at = {}
        self._stop = threading.Event()
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_subscriptions = [
            self.create_subscription(
                Image, topic, lambda msg, key=key: self.receive(key, msg), qos)
            for key, topic in IMAGE_TOPICS.items()
        ]
        self.worker_executor = SingleThreadedExecutor(context=context)
        self.worker_executor.add_node(self)
        self.worker = threading.Thread(target=self._run,
                                       name='gui-image-receiver', daemon=True)
        self.worker.start()

    def receive(self, key, msg):
        """Replace a pending frame without doing image processing."""
        with self._lock:
            self._latest[key] = msg
            self._received_at[key] = time.monotonic()

    def take_latest(self):
        """Consume pending frames while retaining last-arrival timestamps."""
        with self._lock:
            latest, self._latest = self._latest, {}
            return latest, dict(self._received_at)

    def _run(self):
        try:
            while not self._stop.is_set() and self.context.ok():
                self.worker_executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass

    def close(self):
        """Join the image worker before releasing its ROS handles."""
        # Stop callbacks before destroying the node / shutting down ROS.
        self._stop.set()
        self.worker_executor.wake()
        self.worker.join()
        self.worker_executor.shutdown()
        self.destroy_node()


def preview_rgb(msg, bridge, *, depth=False, near_m=0.07, far_m=0.6,
                max_width=640):
    """Convert for display only; never change measured/published depth data."""
    if depth and msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
    pixels = bridge.imgmsg_to_cv2(
        msg, desired_encoding='passthrough' if depth else 'rgb8')
    if pixels.size == 0:
        raise ValueError('Empty image')
    height, width = pixels.shape[:2]
    if width > max_width:
        pixels = cv2.resize(pixels, (max_width, max(1, height * max_width // width)),
                            interpolation=cv2.INTER_NEAREST)
    if not depth:
        return np.ascontiguousarray(pixels)
    if not (0.0 <= near_m < far_m):
        raise ValueError('Depth range requires 0 <= near < far')
    # ROS depth images: uint16 millimetres, or float32 metres.
    metres = pixels.astype(np.float32)
    if msg.encoding == '16UC1':
        metres *= 0.001
    valid = np.isfinite(metres) & (metres > 0.0)
    normalized = np.zeros(metres.shape, dtype=np.float32)
    normalized[valid] = np.clip(
        (far_m - metres[valid]) / (far_m - near_m), 0.0, 1.0)
    colors_bgr = cv2.applyColorMap(
        (normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
    colors_bgr[~valid] = 0
    return cv2.cvtColor(colors_bgr, cv2.COLOR_BGR2RGB)


class ImageCanvas(QWidget):
    """Aspect-preserving image surface that does not force the window wider."""

    def __init__(self):
        """Create an empty, resizable preview surface."""
        super().__init__()
        self.image = QImage()
        self.setMinimumSize(200, 140)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def sizeHint(self):
        """Suggest a compact preview size to the Qt layout."""
        return QSize(400, 230)

    def set_rgb(self, pixels):
        """Own the image bytes and schedule painting in the Qt thread."""
        height, width = pixels.shape[:2]
        # QImage must own its bytes after the temporary NumPy array is released.
        self.image = QImage(pixels.data, width, height, pixels.strides[0],
                            QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self, event):
        """Letterbox the image without changing its aspect ratio."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#121820'))
        if self.image.isNull():
            painter.setPen(QColor('#b6c2cf'))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for image…')
            return
        size = self.image.size().scaled(self.size(), Qt.KeepAspectRatio)
        area = QRect(0, 0, size.width(), size.height())
        area.moveCenter(self.rect().center())
        painter.drawImage(area, self.image)


class ImagePreviewPanel(QGroupBox):
    """Show independent depth and skeleton streams with freshness indicators."""

    def __init__(self, context):
        """Build display controls and start the bounded preview timer."""
        super().__init__('Camera / skeleton preview')
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.near = QDoubleSpinBox()
        self.far = QDoubleSpinBox()
        for spinbox, value in ((self.near, 0.07), (self.far, 0.6)):
            spinbox.setRange(0.01, 10.0)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.01)
            spinbox.setSuffix(' m')
            spinbox.setValue(value)
        controls.addWidget(QLabel('Depth: near (red)'))
        controls.addWidget(self.near)
        controls.addWidget(QLabel('far (blue)'))
        controls.addWidget(self.far)
        controls.addStretch(1)
        layout.addLayout(controls)
        self.canvases = {}
        self.status_labels = {}
        self.errors = {}
        views = QHBoxLayout()
        for key, title in (('depth', 'Aligned depth'), ('skeleton', 'HRM skeleton')):
            box = QGroupBox(title)
            column = QVBoxLayout(box)
            box.setToolTip(IMAGE_TOPICS[key])
            canvas = ImageCanvas()
            status = QLabel('No image received')
            status.setWordWrap(True)
            column.addWidget(canvas, 1)
            column.addWidget(status)
            self.canvases[key] = canvas
            self.status_labels[key] = status
            views.addWidget(box, 1)
        layout.addLayout(views, 1)
        self.bridge = CvBridge()
        self.receiver = LatestImages(context)
        self.last_depth = None
        self.range_dirty = False
        self.near.valueChanged.connect(self.range_changed)
        self.far.valueChanged.connect(self.range_changed)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(34)  # Display ceiling ~30 Hz; no queue of old images.

    def range_changed(self):
        """Request recoloring even when no new depth frame arrives."""
        self.range_dirty = True

    def refresh(self):
        """Render at most one pending frame per topic in each timer tick."""
        latest, received_at = self.receiver.take_latest()
        now = time.monotonic()
        if 'depth' in latest:
            self.last_depth = latest['depth']
        elif self.range_dirty and self.last_depth is not None:
            latest['depth'] = self.last_depth
        self.range_dirty = False
        # Keep the subscriptions drained even while scrolled out of view.
        if self.visibleRegion().isEmpty():
            return
        for key, msg in latest.items():
            try:
                pixels = preview_rgb(msg, self.bridge, depth=key == 'depth',
                                     near_m=self.near.value(), far_m=self.far.value())
                self.canvases[key].set_rgb(pixels)
                self.errors.pop(key, None)
            except Exception as error:
                self.errors[key] = str(error)
        for key, label in self.status_labels.items():
            age = now - received_at[key] if key in received_at else None
            if key in self.errors:
                text, color = self.errors[key], '#d93025'
            elif age is None:
                text, color = 'No image received', '#777777'
            elif age > 1.0:
                text, color = f'Stale — last received {age:.1f}s ago', '#d93025'
            else:
                text, color = '● Receiving', '#188038'
            if label.text() != text:
                label.setText(text)
                label.setStyleSheet(f'color: {color};')

    def stop(self):
        """Stop preview work before the GUI shuts down its ROS context."""
        self.timer.stop()
        self.receiver.close()
