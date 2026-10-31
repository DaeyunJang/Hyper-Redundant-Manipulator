"""Display-only cache and probability heatmap for joint force/contact inference."""

from dataclasses import dataclass
import math
import time

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)


@dataclass(frozen=True)
class ForceIdSnapshot:
    """No motor commands and no substituted zero-force measurements."""

    valid: bool = False
    status: str = 'Waiting'
    force_mn: tuple = ()
    probabilities: tuple = ()
    mode: str = ''
    model_name: str = ''
    force_scope: str = ''
    contact_id: int = -1
    control_contact_id: int = -1
    load_state: int = 0
    tip_force_valid: bool = False
    body_contact_enabled: bool = False
    body_contact_detected: bool = False
    heatmap_center_id: int = -1
    id_tol: int = 2
    samples_in_window: int = 0
    window_samples: int = 0
    source_stamp: int = None  # Newest selected input time [ns], not image time.
    received_at: float = None  # Monotonic receipt; independent of ROS clock.


class ForceIdPreview:
    """Once this source appears, stale/invalid data never fall back to legacy data."""

    def __init__(self, timeout_sec=0.5):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('force_id_timeout_sec must be finite and positive')
        self.timeout_sec = timeout_sec
        self.received_at = None
        self.value = ForceIdSnapshot()

    @property
    def seen(self):
        return self.received_at is not None

    def receive(self, message, now=None):
        self.received_at = time.monotonic() if now is None else now
        try:
            # Old diagnostic messages without stamps can still populate numeric
            # labels, but cannot be associated with an image overlay.
            stamp = getattr(message.header, 'stamp', None)
            source_ns = None
            if (stamp is not None and stamp.sec >= 0
                    and 0 <= stamp.nanosec < 1_000_000_000):
                source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                source_ns = source_ns if source_ns > 0 else None
            shared = dict(
                status=str(message.status), model_name=str(message.model_name),
                mode=str(message.mode), force_scope=str(message.force_scope),
                body_contact_enabled=bool(message.body_contact_enabled),
                id_tol=int(message.id_tol), samples_in_window=int(message.samples_in_window),
                window_samples=int(message.window_samples),
                source_stamp=source_ns, received_at=self.received_at)
            if not message.valid:
                self.value = ForceIdSnapshot(**shared)
                return
            force = tuple(float(getattr(message.external_force, axis)) * 1000.0
                          for axis in 'xyz')
            probabilities = tuple(float(p) for p in message.probabilities)
            contact = int(message.contact_id)
            control = int(message.control_contact_id)
            center = int(message.heatmap_center_id)
            state = int(message.load_state)
            if (message.header.frame_id != 'hrm_base'
                    or not all(math.isfinite(v) for v in force)
                    or len(probabilities) != 19
                    or not all(math.isfinite(p) and 0 <= p <= 1 for p in probabilities)
                    or not math.isclose(sum(probabilities), 1.0, abs_tol=1e-5)
                    or shared['mode'] not in ('class0', 'masked')
                    or not 0 <= contact <= 18 or not -1 <= control <= 18
                    or not 1 <= center <= 18 or state not in (0, 1, 2)
                    or not 0 <= shared['id_tol'] <= 18):
                raise ValueError('invalid force/contact message')
            self.value = ForceIdSnapshot(
                valid=True, force_mn=force, probabilities=probabilities,
                contact_id=contact, control_contact_id=control, load_state=state,
                tip_force_valid=bool(message.tip_force_valid),
                body_contact_detected=bool(message.body_contact_detected),
                heatmap_center_id=center, **shared)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.value = ForceIdSnapshot(status='Invalid message / frame')

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        if self.received_at is not None and (
                now < self.received_at or now - self.received_at > self.timeout_sec):
            return ForceIdSnapshot(status='Stale')
        return self.value


def heatmap_values(snapshot):
    """Original p_i within the candidate band; never conditional renormalization."""
    if not snapshot.valid:
        return (None,) * 18
    return tuple(snapshot.probabilities[index]
                 if abs(index - snapshot.heatmap_center_id) <= snapshot.id_tol else None
                 for index in range(1, 19))


class ForceIdMonitor(QWidget):
    """Small 18-cell visualization; body settings enable monitoring, not motion."""

    settings_requested = pyqtSignal(bool, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._last_rendered = None
        self.last_snapshot = ForceIdSnapshot()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        controls = QHBoxLayout()
        controls.setSpacing(3)
        controls.addWidget(QLabel('Contact ID'))
        self.body_checkbox = QCheckBox('Body monitor')
        self.body_checkbox.setToolTip(
            'Body-contact monitoring is visual only, not motor output or a control gate. '
            'By default, robot_control uses valid inferred force in ADMITTANCE '
            'independently of the displayed contact ID; an ID mistake does not stop '
            'force feedback. Optional admittance/use_contact_id_gate=true enables the '
            'ID-gated policy. This checkbox does not change that controller setting.')
        self.tolerance = QSpinBox()
        self.tolerance.setRange(0, 18)
        self.tolerance.setValue(2)
        self.tolerance.setPrefix('±')
        self.tolerance.setToolTip('Display raw p_i around the positive-ID argmax; '
                                  'this is not measured ±n accuracy.')
        controls.addStretch(1)
        controls.addWidget(self.body_checkbox)
        controls.addWidget(self.tolerance)
        layout.addLayout(controls)
        cells = QHBoxLayout()
        cells.setSpacing(1)
        self.cells = []
        for segment_id in range(1, 19):
            cell = QLabel(str(segment_id))
            font = cell.font()
            font.setPixelSize(11)
            cell.setFont(font)
            cell.setAlignment(Qt.AlignCenter)
            cell.setMinimumWidth(0)
            cell.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            cell.setFixedHeight(23)
            cells.addWidget(cell, 1)
            self.cells.append(cell)
        layout.addLayout(cells)
        self.status_label = QLabel('Force / ID: waiting')
        status_font = self.status_label.font()
        status_font.setPixelSize(11)
        self.status_label.setFont(status_font)
        self.status_label.setWordWrap(True)
        self.status_label.setMaximumHeight(self.status_label.fontMetrics().lineSpacing() * 2 + 2)
        self.status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        layout.addWidget(self.status_label)
        self.settings_label = QLabel('ID: display only (default) · force → ADMITTANCE')
        self.settings_label.setToolTip(
            'Default force control assumes the experiment physically loads the tip. '
            'The model output is not a separately extracted segment-18 force. '
            'Motor enable, control mode and other safety checks remain independent.')
        self.settings_label.setStyleSheet('color: #666; font-size: 9px;')
        self.settings_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        layout.addWidget(self.settings_label)
        self.body_checkbox.toggled.connect(self._settings_changed)
        self.tolerance.valueChanged.connect(self._settings_changed)
        self.refresh(ForceIdSnapshot())

    def _settings_changed(self):
        self.settings_requested.emit(self.body_checkbox.isChecked(), self.tolerance.value())

    def substitutions(self):
        return {'body_contact_enabled': self.body_checkbox.isChecked(),
                'force_id_tol': self.tolerance.value()}

    def refresh(self, snapshot):
        self.last_snapshot = snapshot
        if snapshot == self._last_rendered:
            return
        self._last_rendered = snapshot
        for index, (cell, probability) in enumerate(zip(self.cells, heatmap_values(snapshot)), 1):
            if probability is None:
                style = 'background: #eeeeee; color: #777; border: 1px solid #ddd;'
            else:
                # Color has fixed 0..1 scale, not a per-frame maximum or band normalization.
                faded = round(255 * (1.0 - probability))
                border = '#174ea6' if index == snapshot.heatmap_center_id else '#ddd'
                style = (f'background: rgb(255, {faded}, {faded}); color: black; '
                         f'border: 1px solid {border};')
            if cell.styleSheet() != style:
                cell.setStyleSheet(style)
            cell.setToolTip(f'ID {index}: p={snapshot.probabilities[index]:.4f} '
                            '(raw softmax; diagnostic even when unloaded)'
                            if snapshot.valid else snapshot.status)
        if not snapshot.valid:
            progress = (f' ({snapshot.samples_in_window}/{snapshot.window_samples})'
                        if snapshot.window_samples else '')
            self.status_label.setText(f'Force / ID: {snapshot.status}{progress}')
            self.status_label.setToolTip(
                f'{snapshot.status}{progress}\nNo usable inference. Not a no-load measurement.')
            return
        state = {0: 'Load unknown', 1: 'Unloaded', 2: 'Loaded'}[snapshot.load_state]
        control = ('—' if snapshot.control_contact_id < 0 else str(snapshot.control_contact_id))
        band = sum(p for p in heatmap_values(snapshot) if p is not None)
        p0 = (f'p0={snapshot.probabilities[0]:.2f}' if snapshot.mode == 'class0'
              else 'p0 unavailable')
        policy = 'ON' if snapshot.body_contact_enabled else 'OFF'
        self.status_label.setText(
            f'{state} · raw ID {snapshot.contact_id} · control ID {control}\n'
            f'{p0} · band Σp={band:.2f} · Body monitor {policy}')
        self.status_label.setToolTip(
            f'Model: {snapshot.model_name}\nForce scope: {snapshot.force_scope}; '
            'displayed XYZ are diagnostic hrm_base forces in mN.\n'
            f'ID-gated tip eligibility: {snapshot.tip_force_valid} '
            '(policy diagnostic, not control authorization).\n'
            'Default admittance/use_contact_id_gate=false: ID/load/body labels are '
            'diagnostic, and valid Y/Z force can continue to the admittance controller '
            'subject to its other checks. Setting the gate true restores ID-gated '
            'eligibility. The default assumes physically tip-only loading; this output '
            'does not isolate a segment-18 force from multiple contacts.\n'
            'With inferred-force range policy enabled, Unloaded means all force '
            'components are inside the configured no-load ranges; it is not proof '
            'of zero physical force. Raw classifier ID/probabilities remain unchanged '
            'and can disagree with the force-derived load state.\n'
            'Σp is probability mass in the displayed band, not real-time accuracy. '
            'Unloaded control ID18 is an operating default, not detected tip contact.')
