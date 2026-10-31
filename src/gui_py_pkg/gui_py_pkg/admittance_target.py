"""Read and explicitly apply admittance force targets without motor commands."""

import math
import time

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QWidget
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter


TARGET = '/robot_control'
AXES = ('y', 'z')
TARGET_NAMES = tuple(f'admittance/target_force_{axis}_N' for axis in AXES)
PARAMETER_NAMES = TARGET_NAMES + ('admittance/force_sign', 'admittance/max_force_N')
REQUEST_TIMEOUT_SECONDS = 3.0
REFRESH_SECONDS = 2.0
DISPLAY_LIMIT_MN = 1.e9


class AdmittanceTargetPanel(QWidget):
    """One row of signed mN targets, backed by confirmed atomic ROS parameters."""

    settings_changed = pyqtSignal()

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._clients = {
            'read': node.create_client(GetParameters, TARGET + '/get_parameters'),
            'write': node.create_client(
                SetParametersAtomically, TARGET + '/set_parameters_atomically'),
        }
        self._kind = self._future = self._request = None
        self._purpose = None
        self._deadline = self._next_refresh = 0.0
        self._timed_out = self._loaded = self._updating = self._closed = False
        self._unknown = False
        self._available = None
        self._was_available = False
        self._snapshot = None
        self._desired = None
        self._edited_keys = set()

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        title = QLabel('Admittance F* [mN]')
        title.setToolTip(
            'hrm_base force ON robot; during Admittance Apply affects motion immediately. '
            'Targets use the controller convention. Red plot targets account for force_sign.')
        row.addWidget(title)
        self.targets = {}
        for axis in AXES:
            row.addWidget(QLabel('F' + axis))
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(-DISPLAY_LIMIT_MN, DISPLAY_LIMIT_MN)
            spin.setSingleStep(10.0)
            spin.setFixedWidth(88)
            spin.setToolTip(
                'Signed hrm_base force ON the robot, in mN. Read before editing. '
                'Edits take effect only after Apply; startup zero is a placeholder.')
            spin.valueChanged.connect(lambda unused_value, key=axis: self._edited(key))
            self.targets[axis] = spin
            row.addWidget(spin)
        self.read_button = QPushButton('Read')
        self.apply_button = QPushButton('Apply')
        for button in (self.read_button, self.apply_button):
            button.setFixedWidth(45)
            row.addWidget(button)
        self.read_button.setToolTip('Read targets, force_sign and max_force_N from robot_control.')
        self.apply_button.setToolTip(
            'Atomically apply both force targets. During Admittance this can immediately '
            'change actual robot motion. Does not enable motors or switch modes.')
        self.limits_label = QLabel('sign ? / max ? mN')
        self.limits_label.setToolTip(
            'Read-only controller force_sign and target vector norm limit.')
        row.addWidget(self.limits_label)
        self.status_label = QLabel('Read first')
        self.status_label.setFixedWidth(80)
        row.addWidget(self.status_label)
        row.addStretch(1)
        self.setMaximumHeight(30)
        self.read_button.clicked.connect(self.read_settings)
        self.apply_button.clicked.connect(self.apply_settings)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        self._status('Read first', 'Startup zero values are placeholders, not confirmed targets.')
        self._controls()

    @property
    def busy(self):
        return self._kind is not None

    @property
    def confirmed_force_sign(self):
        """Return the confirmed controller force sign, or None when uncertain."""
        return self._snapshot[2] if self._confirmed() else None

    def confirmed_target_mn(self):
        """Return confirmed force ON robot targets in controller convention, in mN."""
        if not self._confirmed():
            return None
        return tuple(value * 1000.0 for value in self._snapshot[:2])

    def target_plot_mn(self):
        """Return targets in the raw, unnegated model's hrm_base force convention."""
        targets = self.confirmed_target_mn()
        if targets is None:
            return None
        return tuple(value / self._snapshot[2] for value in targets)

    def _confirmed(self):
        return (self._loaded and self._snapshot is not None and not self._unknown
                and self._available is not False and not self._closed)

    def _status(self, short, detail, error=False):
        self.status_label.setText(short)
        self.status_label.setToolTip(detail)
        self.status_label.setStyleSheet('color: #d93025;' if error else 'color: #666;')

    def _controls(self):
        available = self._available is not False and not self._closed
        idle = available and not self.busy
        self.read_button.setEnabled(idle)
        self.apply_button.setEnabled(idle and self._loaded)
        editable = available and self._loaded and (
            not self.busy or (self._kind == 'read' and self._purpose == 'refresh'))
        for spin in self.targets.values():
            spin.setEnabled(editable)

    def _edited(self, key):
        if not self._updating and self._loaded:
            self._edited_keys.add(key)
            self._status('Edited',
                         'Not applied. Apply can immediately change motion in Admittance.')

    def update_nodes(self, nodes):
        """Discover and read the controller; discovery never applies targets."""
        if self._closed:
            return
        available = TARGET in nodes
        appeared = available and not self._was_available
        self._available = self._was_available = available
        if not available:
            self._loaded = False
            if self._kind == 'write' and self._future is not None:
                self._timed_out = self._unknown = True
                self._status('Unknown', 'Controller disappeared during Apply. Await its ACK, '
                             'then Read before another Apply.', True)
            else:
                self._clear_pending()
                self._status('Offline',
                             'Start robot_control, then read its current force targets.')
            self.settings_changed.emit()
        self._controls()
        if appeared and not self.busy:
            self.read_settings()

    def _clear_pending(self):
        self._kind = self._future = self._request = self._purpose = None
        self._timed_out = False

    def _begin(self, kind, request, purpose=None):
        self._kind, self._request, self._purpose = kind, request, purpose
        self._future = None
        self._timed_out = False
        self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        if purpose != 'refresh':
            label = 'Applying' if kind == 'write' else (
                'Verifying' if purpose == 'verify' else 'Reading')
            self._status(label, 'Awaiting controller parameter response.')
        self._controls()
        self.poll()

    def _read(self, purpose):
        self._begin('read', GetParameters.Request(names=list(PARAMETER_NAMES)), purpose)

    def read_settings(self):
        if not self._closed and not self.busy and self._available is not False:
            self._read('manual')

    @staticmethod
    def _validate(values):
        if len(values) != len(PARAMETER_NAMES) or not all(math.isfinite(v) for v in values):
            raise ValueError('Expected four finite double-typed admittance parameters.')
        y, z, sign, maximum = values
        if sign not in (-1.0, 1.0) or maximum <= 0.0:
            raise ValueError('force_sign must be +/-1 and max_force_N must be positive.')
        if math.hypot(y, z) > maximum:
            raise ValueError('Target force vector norm exceeds controller max_force_N.')
        if any(abs(value) * 1000.0 > DISPLAY_LIMIT_MN for value in (y, z)):
            raise ValueError('Target exceeds the GUI display range; values cannot be clamped.')

    def apply_settings(self):
        if self._closed or self.busy or not self._loaded or self._available is False:
            return
        try:
            # Preserve server precision for fields the operator has not edited.
            desired = tuple(self.targets[axis].value() / 1000.0
                            if axis in self._edited_keys else self._snapshot[index]
                            for index, axis in enumerate(AXES))
            self._validate(desired + self._snapshot[2:])
            self._desired = desired
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=float(value)).to_parameter_msg()
                for name, value in zip(TARGET_NAMES, desired)])
            self._begin('write', request)
        except Exception as error:
            self._status('Invalid', str(error), True)
            self._controls()

    def _accept_read(self, response, purpose):
        values = response.values
        if (len(values) != len(PARAMETER_NAMES)
                or any(value.type != ParameterType.PARAMETER_DOUBLE for value in values)):
            raise ValueError('Controller must return four double-typed admittance parameters.')
        numbers = tuple(value.double_value for value in values)
        self._validate(numbers)
        self._updating = True
        try:
            for index, axis in enumerate(AXES):
                # Refresh uses the latest server values without overwriting pending edits.
                if purpose != 'refresh' or axis not in self._edited_keys:
                    self.targets[axis].setValue(numbers[index] * 1000.0)
        finally:
            self._updating = False
        changed = numbers != self._snapshot
        self._snapshot = numbers
        self._loaded = True
        self._unknown = False
        self._next_refresh = time.monotonic() + REFRESH_SECONDS
        if purpose != 'refresh':
            self._edited_keys.clear()
        self.limits_label.setText(f'sign {numbers[2]:+g} / max {numbers[3] * 1000.0:g} mN')
        self.limits_label.setToolTip(
            f'Read-only force_sign={numbers[2]:g}; target vector norm must be '
            f'<= {numbers[3]:.12g} N. Raw model plot targets are F*/force_sign.')
        if purpose == 'verify':
            matched = numbers[:2] == self._desired
            self._status('Applied' if matched else 'Changed',
                         'Both force targets acknowledged and confirmed by controller readback.'
                         if matched else 'Controller readback differs from the requested targets; '
                         'showing the confirmed current values.', not matched)
        elif purpose != 'refresh' or self._edited_keys or changed:
            self._status('Edited' if self._edited_keys else 'Read OK',
                         'Pending edits are not applied; plot lines show confirmed targets.'
                         if self._edited_keys else
                         'Read controller targets, force_sign and max_force_N; no write made.')
        self.settings_changed.emit()

    def poll(self):
        """Poll futures without blocking, executing ROS, or changing Qt off-thread."""
        if self._closed:
            return
        if not self.busy:
            if (self._loaded and self._available is True
                    and time.monotonic() >= self._next_refresh):
                self._read('refresh')
            return
        kind, purpose = self._kind, self._purpose
        try:
            if self._future is not None and self._future.done():
                late = self._timed_out or time.monotonic() > self._deadline
                response = self._future.result()
                self._clear_pending()
                if kind == 'read' and late:
                    self._loaded = False
                    self._status('Read again', 'Read response arrived after its deadline; '
                                 'read current values again.', True)
                    self.settings_changed.emit()
                elif kind == 'read':
                    self._accept_read(response, purpose)
                elif late or self._available is False:
                    self._loaded = False
                    self._unknown = True
                    self._status('Read again', 'Late Apply ACK: successful='
                                 + str(response.result.successful) + '. '
                                 + (response.result.reason or '')
                                 + ' Read current values before another Apply.', True)
                    self.settings_changed.emit()
                elif response.result.successful:
                    self._loaded = False
                    self._unknown = True
                    self._read('verify')
                    return
                else:
                    self._unknown = False
                    self._status('Rejected', response.result.reason or
                                 'Controller rejected both force targets.', True)
                    self._next_refresh = time.monotonic() + REFRESH_SECONDS
                    self.settings_changed.emit()
                self._controls()
                return
            if time.monotonic() > self._deadline:
                self._loaded = False
                if kind == 'write' and self._future is not None:
                    self._timed_out = self._unknown = True
                    self._status('Unknown', 'Apply ACK timed out; targets may still take effect. '
                                 'No overlapping write is allowed. Await the ACK, then Read. '
                                 'If the controller died, restart the GUI after recovery.', True)
                else:
                    sent = self._future is not None
                    self._clear_pending()
                    self._status('Read again' if sent else 'Offline',
                                 'Read response timed out; read again.' if sent else
                                 'Parameter service unavailable; no write was sent.', True)
                self.settings_changed.emit()
                self._controls()
                return
            if self._future is None and self._clients[kind].service_is_ready():
                # A dispatched write may take effect before its ACK arrives.
                if kind == 'write':
                    self._unknown = True
                    self.settings_changed.emit()
                self._future = self._clients[kind].call_async(self._request)
                self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        except Exception as error:
            self._loaded = False
            self._unknown = self._unknown or kind == 'write'
            self._clear_pending()
            self._status('Read again',
                         ('Apply outcome unknown; read controller targets. ' if self._unknown
                          else 'Parameter read failed. ') + str(error), True)
            self.settings_changed.emit()
            self._controls()

    def shutdown(self):
        """Stop polling without writing, retrying, or sending motor/mode commands."""
        self._closed = True
        self._loaded = False
        self.timer.stop()
        self.settings_changed.emit()
        self._controls()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)
