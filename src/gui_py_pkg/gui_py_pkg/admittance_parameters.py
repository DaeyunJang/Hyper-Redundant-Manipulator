"""Explicitly edit the existing admittance M/B/K parameters, without mode changes."""

import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QDialog, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter

from .admittance_tuning_dialog import AdmittanceTuningDialog


TARGET = '/robot_control'
KEYS = ('m', 'b', 'k')
GAIN_NAMES = ('admittance/M_d', 'admittance/B_d', 'admittance/K_d')
PARAMETER_NAMES = GAIN_NAMES + ('control_mode',)
ADMITTANCE_MODE = 4
REQUEST_TIMEOUT_SECONDS = 3.0
REFRESH_SECONDS = 2.0
DISPLAY_LIMIT = 1.e9


class AdmittanceParametersPanel(QWidget):
    """Compact two-row, read-first M/B/K controls with atomic verified writes."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._clients = {
            'read': node.create_client(GetParameters, TARGET + '/get_parameters'),
            'write': node.create_client(
                SetParametersAtomically, TARGET + '/set_parameters_atomically'),
        }
        self._kind = self._future = self._request = self._purpose = None
        self._snapshot = self._desired = self._mode = None
        self._deadline = self._next_refresh = 0.0
        self._timed_out = self._loaded = self._updating = self._closed = False
        self._available = None
        self._was_available = False
        self._edited_keys = set()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        fields = QHBoxLayout()
        fields.setSpacing(4)
        self.parameters = {}
        for key, label, unit, step in (
                ('m', 'M', 'kg (virtual mass)', 0.01),
                ('b', 'B', 'N s/m (damping)', 0.1),
                ('k', 'K', 'N/m (stiffness)', 1.0)):
            caption = QLabel(label)
            caption.setToolTip(unit)
            fields.addWidget(caption)
            spin = QDoubleSpinBox()
            # Keep ordinary values readable next to the step arrows in a compact panel.
            # Unedited Apply values still come from the full-precision server snapshot.
            spin.setDecimals(4)
            spin.setStyleSheet('font-size: 9pt;')
            spin.setRange(0.0, DISPLAY_LIMIT)
            spin.setSingleStep(step)
            spin.setFixedSize(80, 23)
            spin.setToolTip(
                f'{GAIN_NAMES[KEYS.index(key)]}: {unit}. Read before editing. '
                'Existing shared Y/Z parameter; no independent-axis or matrix change. '
                'Apply is disabled in Admittance mode.')
            spin.valueChanged.connect(lambda unused_value, name=key: self._edited(name))
            self.parameters[key] = spin
            fields.addWidget(spin)
        fields.addStretch(1)
        layout.addLayout(fields)
        actions = QHBoxLayout()
        actions.setSpacing(4)
        actions.addWidget(QLabel('Admittance'))
        self.read_button = QPushButton('Read')
        self.apply_button = QPushButton('Apply')
        self.tune_button = QPushButton('Tune')
        for button in (self.read_button, self.apply_button, self.tune_button):
            button.setFixedSize(45, 23)
            actions.addWidget(button)
        self.tune_button.setFixedWidth(42)
        self.read_button.setToolTip('Read M/B/K and control_mode; never changes robot state.')
        self.apply_button.setToolTip(
            'Atomically apply M/B/K only. Leave Admittance mode first. '
            'Never changes control mode, target force, limits, or motor enable.')
        self.tune_button.setToolTip(
            'Calculate M/B/K candidates from assumed correction requirements. '
            'No robot action; Use candidate only fills the editors, never applies settings.')
        self.status_label = QLabel('Read first')
        self.status_label.setMinimumWidth(90)
        actions.addWidget(self.status_label)
        actions.addStretch(1)
        layout.addLayout(actions)
        self.setMaximumHeight(48)
        self.read_button.clicked.connect(self.read_settings)
        self.apply_button.clicked.connect(self.apply_settings)
        self.tune_button.clicked.connect(self.open_tuning)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        self._status('Read first', 'Startup zeros are placeholders, not controller settings.')
        self._controls()

    @property
    def busy(self):
        return self._kind is not None

    def _status(self, short, detail, error=False):
        self.status_label.setText(short)
        self.status_label.setToolTip(detail)
        color = '#d93025' if error else '#666'
        self.status_label.setStyleSheet(f'font-size: 9pt; color: {color};')

    def _controls(self):
        available = self._available is not False and not self._closed
        idle = available and not self.busy
        self.read_button.setEnabled(idle)
        self.apply_button.setEnabled(
            idle and self._loaded and self._mode != ADMITTANCE_MODE)
        self.tune_button.setEnabled(idle and self._loaded)
        editable = available and self._loaded and (
            not self.busy or (self._kind == 'read' and self._purpose == 'refresh'))
        for spin in self.parameters.values():
            spin.setEnabled(editable)

    def _mode_status(self):
        self._status('Leave Admit',
                     'Leave Admittance mode before applying M/B/K. Pending edits are '
                     'retained. The GUI never changes mode automatically.')

    def _edited(self, key):
        if not self._updating and self._loaded:
            self._edited_keys.add(key)
            if self._mode == ADMITTANCE_MODE:
                self._mode_status()
            else:
                self._status('Edited', 'Not applied. Apply changes only M/B/K parameters.')

    def update_nodes(self, nodes):
        if self._closed:
            return
        available = TARGET in nodes
        appeared = available and not self._was_available
        self._available = self._was_available = available
        if not available:
            self._loaded = False
            self._mode = None
            if self._kind == 'write' and self._future is not None:
                self._timed_out = True
                self._status('Unknown', 'Controller disappeared during Apply. Await the ACK, '
                             'then Read before another Apply.', True)
            else:
                self._clear_pending()
                self._status('Offline', 'Start robot_control, then read M/B/K.')
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

    def open_tuning(self):
        """Stage an explicitly accepted calculation; never issue a service call here."""
        if (self._closed or self.busy or not self._loaded
                or self._available is False or self._snapshot is None):
            return
        dialog = AdmittanceTuningDialog(self._snapshot, self)
        try:
            if dialog.exec_() != QDialog.Accepted or dialog.candidate is None:
                return
            # The node could have disappeared while the modal calculator was open.
            if self._closed or not self._loaded or self._available is False:
                return
            candidate = dialog.candidate
            self._updating = True
            try:
                for key, value in zip(KEYS, (candidate.mass, candidate.damping,
                                             candidate.stiffness)):
                    self.parameters[key].setValue(value)
            finally:
                self._updating = False
            self._edited_keys.update(KEYS)
            if self._mode == ADMITTANCE_MODE:
                self._mode_status()
            else:
                self._status('Candidate', 'Calculated M/B/K staged only, not applied. '
                             'Review values and use Apply explicitly.')
            self._controls()
        finally:
            dialog.deleteLater()

    @staticmethod
    def _validate(values):
        invalid = any(not math.isfinite(value) or value < 0.0 or value > DISPLAY_LIMIT
                      for value in values)
        if (len(values) != 3 or invalid
                or not any(value > 0.0 for value in values)):
            raise ValueError('M/B/K must be finite, nonnegative, not all zero, '
                             'and within the GUI display range.')

    def apply_settings(self):
        if self._closed or self.busy or not self._loaded or self._available is False:
            return
        if self._mode == ADMITTANCE_MODE:
            self._mode_status()
            return
        try:
            desired = tuple(self.parameters[key].value() if key in self._edited_keys
                            else self._snapshot[index] for index, key in enumerate(KEYS))
            self._validate(desired)
            self._desired = desired
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=float(value)).to_parameter_msg()
                for name, value in zip(GAIN_NAMES, desired)])
            self._begin('write', request)
        except Exception as error:
            self._status('Invalid', str(error), True)
            self._controls()

    def _accept_read(self, response, purpose):
        values = response.values
        if (len(values) != 4
                or any(value.type != ParameterType.PARAMETER_DOUBLE for value in values[:3])
                or values[3].type != ParameterType.PARAMETER_INTEGER):
            raise ValueError('Expected three DOUBLE M/B/K parameters and INTEGER control_mode.')
        numbers = tuple(value.double_value for value in values[:3])
        mode = values[3].integer_value
        self._validate(numbers)
        if mode not in (1, 2, 3, 4):
            raise ValueError('control_mode must be 1..4; Apply disabled for unknown mode.')
        changed = numbers != self._snapshot or mode != self._mode
        self._updating = True
        try:
            for index, key in enumerate(KEYS):
                if purpose != 'refresh' or key not in self._edited_keys:
                    self.parameters[key].setValue(numbers[index])
        finally:
            self._updating = False
        self._snapshot, self._mode, self._loaded = numbers, mode, True
        self._next_refresh = time.monotonic() + REFRESH_SECONDS
        if purpose != 'refresh':
            self._edited_keys.clear()
        if mode == ADMITTANCE_MODE:
            self._mode_status()
        elif purpose == 'verify':
            matched = numbers == self._desired
            self._status('Applied' if matched else 'Changed',
                         'M/B/K acknowledged and confirmed by readback.' if matched else
                         'Controller readback differs from requested M/B/K; '
                         'showing confirmed current settings.', not matched)
        elif purpose != 'refresh' or changed or self._edited_keys:
            self._status('Edited' if self._edited_keys else 'Read OK',
                         'Pending edits are not applied.' if self._edited_keys else
                         'M/B/K and control mode read; no write made.')

    def poll(self):
        """Poll without blocking Qt, starting ROS executors, or automatic writes."""
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
                    self._status('Read again', 'Late read response; read current settings.', True)
                elif kind == 'read':
                    self._accept_read(response, purpose)
                elif late or self._available is False:
                    self._loaded = False
                    self._status('Read again', 'Late Apply ACK: successful='
                                 + str(response.result.successful) + '. '
                                 + (response.result.reason or '')
                                 + ' Read before another Apply.', True)
                elif response.result.successful:
                    self._loaded = False
                    self._read('verify')
                    return
                else:
                    self._status('Rejected', response.result.reason or
                                 'Controller rejected M/B/K. No automatic mode change.', True)
                    self._next_refresh = time.monotonic() + REFRESH_SECONDS
                self._controls()
                return
            if time.monotonic() > self._deadline:
                self._loaded = False
                if kind == 'write' and self._future is not None:
                    self._timed_out = True
                    self._status('Unknown', 'Apply ACK timed out; settings may still change. '
                                 'No overlapping write is allowed. Await ACK, then Read. '
                                 'If the controller died, restart GUI after recovery.', True)
                else:
                    sent = self._future is not None
                    self._clear_pending()
                    self._status('Read again' if sent else 'Offline',
                                 'Read timed out; read again.' if sent else
                                 'Parameter service unavailable; no write sent.', True)
                self._controls()
                return
            if self._future is None and self._clients[kind].service_is_ready():
                self._future = self._clients[kind].call_async(self._request)
                self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        except Exception as error:
            self._loaded = False
            self._clear_pending()
            self._status('Read again',
                         ('Apply outcome unknown; read controller settings. ' if kind == 'write'
                          else 'Parameter read failed. ') + str(error), True)
            self._controls()

    def shutdown(self):
        self._closed = True
        self._loaded = False
        self.timer.stop()
        self._controls()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)
