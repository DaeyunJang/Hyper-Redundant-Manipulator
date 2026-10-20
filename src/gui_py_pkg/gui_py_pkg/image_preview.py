"""Read-only, latest-frame image previews; independent of motor callbacks."""

from collections import OrderedDict
from dataclasses import dataclass
import math
import threading
import time

from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
from PyQt5.QtCore import QRect, QSize, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QImage, QPainter
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

from gui_py_pkg.skeleton_preview import (
    TIP_TOPICS, SkeletonFrameBuffer, SkeletonPreviewFrame, format_skeleton_tips,
)


IMAGE_TOPICS = {
    'depth': '/camera/camera/aligned_depth_to_color/image_raw',
    'skeleton': '/estimated_segment_skeleton_image',
    'tag': '/tag_camera/tag_camera/color/image_rect',
}
TAG_RAW_TOPIC = '/tag_camera/tag_camera/color/image_raw'
TAG_DETECTIONS_TOPIC = '/detections'
TAG_RELATIVE_POSE_TOPIC = '/apriltag/tag1_in_tag0/pose'
TAG_CACHE_SIZE = 8
TAG_MATCH_MAX_AGE = 0.2
IMAGE_STALE_SECONDS = 1.0


def header_key(msg):
    """Identify the exact camera exposure, not just a nearby receipt time."""
    stamp = msg.header.stamp
    if not msg.header.frame_id or (stamp.sec == 0 and stamp.nanosec == 0):
        return None  # Empty headers cannot establish a safe image/detection pair.
    return msg.header.frame_id, stamp.sec, stamp.nanosec


@dataclass(frozen=True)
class TagPreviewFrame:
    """A display-only image and detections belonging to exactly that image."""

    image: object
    detections: object
    received_at: float
    source: str
    status: str
    pose: object = None


class TagFrameBuffer:
    """Bound a little asynchronous matching history; never re-run detection."""

    def __init__(self):
        """Keep at most eight references per image/detection/relative-pose stream."""
        self.images = OrderedDict()
        self.detections = OrderedDict()
        self.poses = OrderedDict()
        self.latest_rect = None
        self.latest_raw = None
        self.last_detection_at = None

    def _trim(self, now):
        for cache in (self.images, self.detections, self.poses):
            for key, (_, received) in list(cache.items()):
                if now - received > IMAGE_STALE_SECONDS:
                    del cache[key]
            while len(cache) > TAG_CACHE_SIZE:
                cache.popitem(last=False)

    def receive_image(self, msg, *, raw=False, now=None):
        """Cache message references only; the Qt thread does the rendering."""
        now = time.monotonic() if now is None else now
        if raw:
            self.latest_raw = (msg, now)
        else:
            self.latest_rect = (msg, now)
            key = header_key(msg)
            if key is not None:
                self.images[key] = (msg, now)
                self.images.move_to_end(key)
        self._trim(now)

    def receive_detections(self, msg, *, now=None):
        """Store empty detection arrays too: they mean 'no tags', not stale."""
        now = time.monotonic() if now is None else now
        self.last_detection_at = now
        key = header_key(msg)
        if key is not None:
            self.detections[key] = (msg, now)
            self.detections.move_to_end(key)
        self._trim(now)

    def wants_raw(self, now):
        """Subscribe to raw RGB only while the rectified stream is absent."""
        return (self.latest_rect is None or
                now - self.latest_rect[1] > IMAGE_STALE_SECONDS)

    def receive_pose(self, msg, *, now=None):
        """ID1 in ID0 uses the image stamp but a different (ID0) parent frame."""
        now = time.monotonic() if now is None else now
        key = header_key(msg)
        if key is not None and key[0] == 'ID0':
            self.poses[key[1:]] = (msg, now)
            self.poses.move_to_end(key[1:])
        self._trim(now)

    def snapshot(self, now):
        """Prefer a fresh exact pair, otherwise keep showing the latest image."""
        self._trim(now)
        if self.latest_rect and now - self.latest_rect[1] <= IMAGE_STALE_SECONDS:
            # A detector can finish a few milliseconds after its image arrives.
            # Select its source image, never its corners on a newer exposure.
            for key, (msg, received) in reversed(self.images.items()):
                if now - received > TAG_MATCH_MAX_AGE:
                    continue
                if (key in self.detections and
                        now - self.detections[key][1] <= TAG_MATCH_MAX_AGE):
                    detections = self.detections[key][0]
                    ids = sorted({int(tag.id) for tag in detections.detections})
                    status = ('Detected ID ' + ', '.join(map(str, ids))
                              if ids else 'No tags detected')
                    pose = None
                    cached = self.poses.get(key[1:])
                    if (0 in ids and 1 in ids and cached is not None
                            and now - cached[1] <= TAG_MATCH_MAX_AGE):
                        pose = cached[0]
                    return TagPreviewFrame(msg, detections, received,
                                           'rectified', status, pose)
            msg, received = self.latest_rect
            if self.last_detection_at is None:
                status = 'Waiting for detections'
            elif now - self.last_detection_at > IMAGE_STALE_SECONDS:
                status = 'Detections stale — no overlay'
            else:
                status = 'Waiting for matching detections'
            return TagPreviewFrame(msg, None, received, 'rectified', status)
        if self.latest_raw and now - self.latest_raw[1] <= IMAGE_STALE_SECONDS:
            msg, received = self.latest_raw
            return TagPreviewFrame(msg, None, received, 'raw', 'Raw RGB — no overlay')
        sources = [(name, frame) for name, frame in (
            ('rectified', self.latest_rect), ('raw', self.latest_raw)) if frame]
        if sources:
            source, (msg, received) = max(sources, key=lambda item: item[1][1])
            return TagPreviewFrame(msg, None, received, source, 'Image stale')
        return None


class LatestImages(Node):
    """Only subscribe/cache here. Qt owns image conversion and all widgets."""

    def __init__(self, context, topics=None):
        """Start an image-only executor with sensor-compatible QoS."""
        # Do not inherit the gui_node name remap passed by ros2 launch.
        node_name = 'gui_image_preview' if topics is None else 'gui_roi_preview'
        super().__init__(node_name, context=context,
                         use_global_arguments=False)
        self._lock = threading.Lock()
        self._latest = {}
        self._received_at = {}
        self._stop = threading.Event()
        self._tag_buffer = TagFrameBuffer() if topics is None else None
        self._tag_signature = None
        self._skeleton_buffer = SkeletonFrameBuffer() if topics is None else None
        self._skeleton_signature = None
        self._raw_subscription = None
        self._started_at = time.monotonic()
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_subscriptions = [
            self.create_subscription(
                Image, topic, lambda msg, key=key: self.receive(key, msg), qos)
            for key, topic in (IMAGE_TOPICS if topics is None else topics).items()
        ]
        self._image_qos = qos
        self.detection_subscription = (
            self.create_subscription(AprilTagDetectionArray, TAG_DETECTIONS_TOPIC,
                                     self.receive_detections, qos)
            if self._tag_buffer is not None else None)
        self.pose_subscription = (
            self.create_subscription(PoseStamped, TAG_RELATIVE_POSE_TOPIC,
                                     self.receive_pose, qos)
            if self._tag_buffer is not None else None)
        self.tip_subscriptions = [
            self.create_subscription(
                PointStamped, topic, lambda msg, kind=kind: self.receive_tip(kind, msg), qos)
            for kind, topic in TIP_TOPICS.items()
        ] if self._skeleton_buffer is not None else []
        self.worker_executor = SingleThreadedExecutor(context=context)
        self.worker_executor.add_node(self)
        self.worker = threading.Thread(target=self._run,
                                       name='gui-image-receiver', daemon=True)
        self.worker.start()

    def receive(self, key, msg):
        """Replace a pending frame without doing image processing."""
        with self._lock:
            if key in ('tag', 'tag_raw') and self._tag_buffer is not None:
                self._tag_buffer.receive_image(msg, raw=key == 'tag_raw')
                return
            if key == 'skeleton' and getattr(self, '_skeleton_buffer', None) is not None:
                self._skeleton_buffer.receive_image(msg)
                return
            self._latest[key] = msg
            self._received_at[key] = time.monotonic()

    def receive_detections(self, msg):
        """Cache detection results on the image-only executor, not motor callbacks."""
        with self._lock:
            self._tag_buffer.receive_detections(msg)

    def receive_pose(self, msg):
        """Cache a small existing pose message; never query TF or run detection."""
        with self._lock:
            self._tag_buffer.receive_pose(msg)

    def receive_tip(self, kind, msg):
        """Only cache PointStamped telemetry; never recompute FK or query TF."""
        with self._lock:
            self._skeleton_buffer.receive_tip(kind, msg)

    def take_latest(self):
        """Consume pending frames while retaining last-arrival timestamps."""
        with self._lock:
            latest, self._latest = self._latest, {}
            if getattr(self, '_tag_buffer', None) is not None:
                frame = self._tag_buffer.snapshot(time.monotonic())
                if frame is not None:
                    signature = (id(frame.image), id(frame.detections),
                                 id(frame.pose), frame.status)
                    if signature != self._tag_signature:
                        latest['tag'] = frame
                        self._tag_signature = signature
                    self._received_at['tag'] = frame.received_at
            if getattr(self, '_skeleton_buffer', None) is not None:
                frame = self._skeleton_buffer.snapshot(time.monotonic())
                if frame is not None:
                    signature = (id(frame.image), id(frame.estimated_tip), id(frame.fk_tip))
                    if signature != self._skeleton_signature:
                        latest['skeleton'] = frame
                        self._skeleton_signature = signature
                    self._received_at['skeleton'] = frame.received_at
            return latest, dict(self._received_at)

    def _update_raw_subscription(self):
        """Switch fallback subscription on this executor, never from the Qt thread."""
        if self._tag_buffer is None:
            return
        now = time.monotonic()
        with self._lock:
            want_raw = (now - self._started_at >= IMAGE_STALE_SECONDS and
                        self._tag_buffer.wants_raw(now))
        if want_raw and self._raw_subscription is None:
            self._raw_subscription = self.create_subscription(
                Image, TAG_RAW_TOPIC, lambda msg: self.receive('tag_raw', msg),
                self._image_qos)
        elif not want_raw and self._raw_subscription is not None:
            self.destroy_subscription(self._raw_subscription)
            self._raw_subscription = None

    def _run(self):
        try:
            while not self._stop.is_set() and self.context.ok():
                self.worker_executor.spin_once(timeout_sec=0.1)
                self._update_raw_subscription()
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


def overlay_tag_corners(pixels, source_width, source_height, detections):
    """Draw existing detector corners on resized RGB; never modify source pixels."""
    if source_width <= 0 or source_height <= 0:
        raise ValueError('Tag source dimensions must be positive')
    result = pixels.copy()
    height, width = result.shape[:2]
    scale = np.array([width / source_width, height / source_height])
    for detection in detections.detections:
        corners = np.array([(point.x, point.y) for point in detection.corners])
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            continue
        clipped = np.clip(corners * scale, [0, 0], [width - 1, height - 1])
        corners = np.rint(clipped).astype(np.int32)
        color = (80, 255, 110) if detection.id == 0 else (255, 210, 60)
        cv2.polylines(result, [corners], True, color, 1, cv2.LINE_AA)
        x, y = corners.min(axis=0)
        cv2.putText(result, f'ID {detection.id}',
                    (min(int(x), max(0, width - 50)), max(12, int(y) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return result


def format_tag_pose(frame, now):
    """Display ID1 in ID0: mm and extrinsic XYZ Euler (RPY) degrees, not HRM pose."""
    title = 'ID0 -> ID1'
    units = 'XYZ [mm] / RPY [deg]'
    if frame is None:
        return title, 'Waiting for tag image', units
    if now - frame.received_at > TAG_MATCH_MAX_AGE:
        return title, 'Pose stale - no current values', units
    if frame.source != 'rectified' or frame.detections is None:
        return title, 'Waiting for matched detections', units
    image_key = header_key(frame.image)
    if image_key is None or image_key != header_key(frame.detections):
        return title, 'Image / detection mismatch', units
    ids = {int(tag.id) for tag in frame.detections.detections}
    if not {0, 1}.issubset(ids):
        return title, 'Both ID0 and ID1 required', units
    if frame.pose is None:
        return title, 'Waiting for matching pose', units
    pose_key = header_key(frame.pose)
    if pose_key != ('ID0', *image_key[1:]):
        return title, 'Pose frame / time mismatch', units
    p, q = frame.pose.pose.position, frame.pose.pose.orientation
    xyz = [value * 1000.0 for value in (p.x, p.y, p.z)]
    quaternion = (q.x, q.y, q.z, q.w)
    norm = math.hypot(*quaternion)
    if (not all(math.isfinite(value) for value in (*xyz, *quaternion, norm))
            or norm <= 1e-12):
        return title, 'Invalid pose - no values', units
    x, y, z, w = (value / norm for value in quaternion)
    # R_ID0_ID1 = Rz(yaw) Ry(pitch) Rx(roll); Euler wraps/singularities apply.
    rpy = [math.degrees(value) for value in (
        math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
        math.asin(max(-1., min(1., 2 * (w*y - z*x)))),
        math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)),
    )]
    return (title,
            'XYZ [mm] ' + ' '.join(f'{value:+.1f}' for value in xyz),
            'RPY [deg] ' + ' '.join(f'{value:+.1f}' for value in rpy))


class ImageCanvas(QWidget):
    """Aspect-preserving image surface that does not force the window wider."""

    def __init__(self):
        """Create an empty, resizable preview surface."""
        super().__init__()
        self.image = QImage()
        self.overlay_lines = ()
        self.setMinimumSize(150, 100)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def sizeHint(self):
        """Suggest a compact preview size to the Qt layout."""
        return QSize(260, 110)

    def set_rgb(self, pixels):
        """Own the image bytes and schedule painting in the Qt thread."""
        height, width = pixels.shape[:2]
        # QImage must own its bytes after the temporary NumPy array is released.
        self.image = QImage(pixels.data, width, height, pixels.strides[0],
                            QImage.Format_RGB888).copy()
        self.update()

    def set_overlay(self, lines):
        """Reserve a small in-canvas header, without growing the widget/layout."""
        lines = tuple(lines)
        if self.overlay_lines != lines:
            self.overlay_lines = lines
            self.update()

    def paintEvent(self, event):
        """Letterbox the image without changing its aspect ratio."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#121820'))
        header_height = 0
        if self.overlay_lines:
            font = QFont(self.font())
            font.setPixelSize(12)
            width = max(QFontMetrics(font).horizontalAdvance(line)
                        for line in self.overlay_lines)
            font.setPixelSize(max(8, min(12, int(12 * (self.width() - 12) / max(1, width)))))
            painter.setFont(font)
            header_height = QFontMetrics(font).height() * len(self.overlay_lines) + 8
        available = self.rect().adjusted(0, header_height, 0, 0)
        if self.image.isNull():
            painter.setPen(QColor('#b6c2cf'))
            painter.drawText(available, Qt.AlignCenter, 'Waiting for image…')
        else:
            size = self.image.size().scaled(self.size(), Qt.KeepAspectRatio)
            area = QRect(0, 0, size.width(), size.height())
            area.moveCenter(self.rect().center())
            # Use the existing top letterbox where possible. Otherwise shrink the
            # image inside this canvas, preserving aspect and all camera pixels.
            if area.top() < header_height:
                size = self.image.size().scaled(available.size(), Qt.KeepAspectRatio)
                area = QRect(0, 0, size.width(), size.height())
                area.moveCenter(available.center())
            painter.drawImage(area, self.image)
        if self.overlay_lines:
            painter.setPen(QColor('#e2edf7'))
            painter.drawText(QRect(6, 4, self.width() - 12, header_height - 8),
                             Qt.AlignLeft | Qt.AlignTop, '\n'.join(self.overlay_lines))


class ImagePreviewPanel(QGroupBox):
    """Show depth, skeleton and matched AprilTag overlays without control traffic."""

    def __init__(self, context):
        """Build display controls and start the bounded preview timer."""
        # Each of the three tiles has its own title; a second outer title costs
        # a full row on the 1366x768 control console without adding information.
        super().__init__()
        self.setFlat(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)
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
        views.setSpacing(4)
        for key, title in (('depth', 'Aligned depth'), ('skeleton', 'HRM skeleton'),
                           ('tag', 'Tag camera / detections')):
            box = QGroupBox(title)
            column = QVBoxLayout(box)
            column.setContentsMargins(4, 4, 4, 4)
            column.setSpacing(2)
            box.setToolTip(IMAGE_TOPICS[key])
            canvas = ImageCanvas()
            if key == 'tag':
                canvas.set_overlay(format_tag_pose(None, time.monotonic()))
                canvas.setToolTip(
                    TAG_RELATIVE_POSE_TOPIC + '\nID1 position in ID0: X, Y, Z [mm].\n'
                    'Orientation: Roll, Pitch, Yaw [deg], Rz(yaw) Ry(pitch) Rx(roll).\n'
                    'Same image timestamp only. Tag mounting offsets are not compensated.\n'
                    'Euler angles can wrap at +/-180 degrees and are singular at pitch +/-90.')
            elif key == 'skeleton':
                canvas.set_overlay(format_skeleton_tips(None, time.monotonic()))
                canvas.setToolTip(
                    'hrm_base coordinates: X, Y, Z [mm].\n'
                    'Est: /estimated_tip_position (length-constrained fitted boundary tip).\n'
                    'FK: /kinematics/fk_tip_position (robot_control measured-angle FK tip).\n'
                    'Only exact image source-stamp matches, at most 0.2 s old by receipt.\n'
                    'Missing/stale/invalid values are not filled from another frame.')
            status = QLabel('No image received')
            status.setWordWrap(True)
            status.setMaximumHeight(32)
            column.addWidget(canvas, 1)
            column.addWidget(status)
            self.canvases[key] = canvas
            self.status_labels[key] = status
            views.addWidget(box, 1)
        layout.addLayout(views, 1)
        self.bridge = CvBridge()
        self.receiver = LatestImages(context)
        self.last_depth = None
        self.tag_frame = None
        self.skeleton_frame = None
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
                if key == 'tag':
                    previous = self.tag_frame
                    self.tag_frame = msg
                    if (previous is not None and previous.image is msg.image
                            and previous.detections is msg.detections and key not in self.errors):
                        continue  # Pose-only arrival updates text, not the RGB pixels.
                    msg = msg.image
                elif key == 'skeleton':
                    previous = self.skeleton_frame
                    self.skeleton_frame = (msg if isinstance(msg, SkeletonPreviewFrame)
                                           else SkeletonPreviewFrame(
                                               msg, received_at.get('skeleton', now)))
                    msg = self.skeleton_frame.image
                    if previous is not None and previous.image is msg and key not in self.errors:
                        continue  # A tip-only update does not reconvert the image.
                pixels = preview_rgb(msg, self.bridge, depth=key == 'depth',
                                     near_m=self.near.value(), far_m=self.far.value(),
                                     max_width=min(640, self.canvases[key].width()))
                if key == 'tag' and self.tag_frame.detections is not None:
                    pixels = overlay_tag_corners(pixels, msg.width, msg.height,
                                                 self.tag_frame.detections)
                self.canvases[key].set_rgb(pixels)
                self.errors.pop(key, None)
            except Exception as error:
                self.errors[key] = str(error)
        tag_overlay = (('ID0 -> ID1', 'Image unavailable - no values', 'XYZ [mm] / RPY [deg]')
                       if 'tag' in self.errors else format_tag_pose(self.tag_frame, now))
        self.canvases['tag'].set_overlay(tag_overlay)
        skeleton_overlay = (('hrm_base | XYZ [mm]', 'Est: image unavailable',
                             'FK: image unavailable') if 'skeleton' in self.errors
                            else format_skeleton_tips(self.skeleton_frame, now))
        self.canvases['skeleton'].set_overlay(skeleton_overlay)
        for key, label in self.status_labels.items():
            age = now - received_at[key] if key in received_at else None
            if key in self.errors:
                text, color = self.errors[key], '#d93025'
            elif age is None:
                text, color = 'No image received', '#777777'
            elif age > IMAGE_STALE_SECONDS:
                text, color = f'Stale — last received {age:.1f}s ago', '#d93025'
            elif key == 'tag' and self.tag_frame is not None:
                text = self.tag_frame.status
                color = ('#188038' if self.tag_frame.detections is not None
                         else '#a56400')
            else:
                text, color = '● Receiving', '#188038'
            if label.text() != text:
                label.setText(text)
                label.setToolTip(text)
                label.setStyleSheet(f'color: {color};')

    def stop(self):
        """Stop preview work before the GUI shuts down its ROS context."""
        self.timer.stop()
        self.receiver.close()
