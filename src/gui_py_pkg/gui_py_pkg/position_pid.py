"""Compact, nonblocking runtime position-PD gain controls; no motor commands."""

import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QWidget)
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter


REQUEST_TIMEOUT_SECONDS = 3.0
TARGET = '/robot_control'
GAIN_KEYS = tuple((axis, term) for axis in ('pan', 'tilt') for term in ('p', 'i', 'd'))
GAIN_NAMES = tuple(
    f'position_control/pid_controller_{axis}/{term}_gain' for axis, term in GAIN_KEYS)


class PositionPIDPanel(QWidget):
    """Read first, then explicitly apply six gains atomically; position I is zero."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        compact_font = self.font()
        compact_font.setPointSizeF(min(compact_font.pointSizeF(), 9.0))
        self.setFont(compact_font)
        self._clients = {
            'read': node.create_client(GetParameters, TARGET + '/get_parameters'),
            'write': node.create_client(
                SetParametersAtomically, TARGET + '/set_parameters_atomically'),
        }
        self._kind = None
        self._future = None
        self._request = None
        self._deadline = 0.0
        self._timed_out = False
        self._loaded = False
        self._available = None
        self._was_available = False
        self._updating = False
        self._closed = False
        self._desired = None
        self._snapshot = {}
        self._edited_keys = set()

        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(3)
        grid.setVerticalSpacing(2)
        title = QLabel('Position\nPD')
        title.setToolTip('Runtime gains only. Read current values before Apply. '
                         'No mode, target or motor-enable changes.')
        grid.addWidget(title, 0, 0, 2, 1)
        self.gains = {}
        for row, axis in enumerate(('pan', 'tilt')):
            grid.addWidget(QLabel('pan (Z)' if axis == 'pan' else 'tilt (Y)'), row, 1)
            for column, term in enumerate(('p', 'i', 'd')):
                grid.addWidget(QLabel(term.upper()), row, 2 + 2 * column)
                spin = QDoubleSpinBox()
                spin.setDecimals(4)
                spin.setRange(0.0, 1000000.0)
                spin.setSingleStep(0.5 if term == 'p' else 0.01)
                spin.setValue({'p': 5.0, 'i': 0.0, 'd': 0.05}[term])
                # The PD and admittance settings share one horizontal strip.
                # Keep gain precision and all controls; I remains read-only.
                spin.setFixedWidth(76 if term != 'i' else 56)
                if term == 'i':
                    spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
                spin.setFixedHeight(23)
                spin.setToolTip(
                    'Position controller currently requires I=0 (PD only).'
                    if term == 'i' else
                    'Displayed startup value is a placeholder, not an applied gain. '
                    'Read the controller, edit, then Apply. '
                    + ('P: rad/(m*s). ' if term == 'p' else 'D: rad/m. ')
                    + 'PD output is angular rate (rad/s); speed-limit the rate, '
                    'then integrate once using feedback dt: delta_angle = rate * dt.')
                spin.valueChanged.connect(
                    lambda unused_value, key=(axis, term): self._edited(key))
                self.gains[(axis, term)] = spin
                grid.addWidget(spin, row, 3 + 2 * column)
        self.read_button = QPushButton('Read')
        self.apply_button = QPushButton('Apply')
        for button in (self.read_button, self.apply_button):
            button.setFixedWidth(45)
            button.setFixedHeight(23)
        self.read_button.setToolTip('Read both axes from the running robot_control node.')
        self.apply_button.setToolTip(
            'Atomically apply P/I/D for both axes. I remains zero. '
            'Changes feedback gains immediately; does not enable motors or switch modes.')
        grid.addWidget(self.read_button, 0, 8)
        grid.addWidget(self.apply_button, 1, 8)
        self.status_label = QLabel('Read first')
        self.status_label.setFixedWidth(70)
        self.status_label.setWordWrap(True)
        grid.addWidget(self.status_label, 0, 9, 2, 1)
        grid.setColumnStretch(10, 1)
        self.setMaximumHeight(62)
        self.read_button.clicked.connect(self.read_settings)
        self.apply_button.clicked.connect(self.apply_settings)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        self._status('Read first', 'Startup values are placeholders, not controller settings.')
        self._controls()

    @property
    def busy(self):
        return self._kind is not None

    def _status(self, short, detail, error=False):
        self.status_label.setText(short)
        self.status_label.setToolTip(detail)
        self.status_label.setStyleSheet('color: #d93025;' if error else 'color: #666;')

    def _controls(self):
        available = self._available is not False and not self._closed
        idle = available and not self.busy
        self.read_button.setEnabled(idle)
        self.apply_button.setEnabled(idle and self._loaded)
        for (unused_axis, term), spin in self.gains.items():
            spin.setEnabled(idle and self._loaded and term != 'i')

    def _edited(self, key):
        if not self._updating and self._loaded:
            self._edited_keys.add(key)
            self._status('Edited', 'Not applied. Apply explicitly changes both axes together.')

    def update_nodes(self, nodes):
        """Read once when the controller appears. Never apply on discovery."""
        if self._closed:
            return
        available = TARGET in nodes
        appeared = available and not self._was_available
        self._available = self._was_available = available
        if not available:
            self._loaded = False
            if self._kind == 'write' and self._future is not None:
                # A disappearing DDS node is not proof that its write failed.
                self._timed_out = True
                self._status('Unknown', 'Controller disappeared during Apply. '
                             'Await the outstanding ACK, then Read before another Apply.', True)
            else:
                self._clear_pending()
                self._status('Offline', 'Start robot_control, then Read its current gains.')
        self._controls()
        if appeared and not self.busy:
            self.read_settings()

    def _clear_pending(self):
        self._kind = self._future = self._request = None
        self._timed_out = False

    def _begin(self, kind, request):
        self._kind, self._request = kind, request
        self._future = None
        self._timed_out = False
        self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        self._status('Reading' if kind == 'read' else 'Applying',
                     'Awaiting controller parameter response; no motor/mode/target command.')
        self._controls()
        self.poll()

    def read_settings(self):
        if not self._closed and not self.busy and self._available is not False:
            self._begin('read', GetParameters.Request(names=list(GAIN_NAMES)))

    def apply_settings(self):
        if self._closed or self.busy or not self._loaded or self._available is False:
            return
        try:
            # Unedited fields preserve the server's full precision, even when
            # the compact spinbox display rounds its last decimal places.
            values = [self.gains[key].value() if key in self._edited_keys
                      else self._snapshot[key] for key in GAIN_KEYS]
            self._validate(values)
            self._desired = values
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=float(value)).to_parameter_msg()
                for name, value in zip(GAIN_NAMES, values)])
            self._begin('write', request)
        except Exception as error:
            self._status('Invalid', str(error), True)
            self._controls()

    def _validate(self, values):
        if len(values) != len(GAIN_KEYS):
            raise ValueError('Expected all six position gains.')
        for key, value in zip(GAIN_KEYS, values):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError('All gains must be finite and nonnegative.')
            if key[1] == 'i' and value != 0.0:
                raise ValueError('Position controller requires I=0; no write was made.')
            if value > self.gains[key].maximum():
                raise ValueError('Gain exceeds this GUI display range; do not silently clamp it.')

    def poll(self):
        """Poll futures only: no blocking waits, spin calls or background Qt writes."""
        if self._closed or not self.busy:
            return
        kind = self._kind
        try:
            if self._future is not None and self._future.done():
                late = self._timed_out
                response = self._future.result()
                self._clear_pending()
                if kind == 'read':
                    values = response.values
                    if (len(values) != len(GAIN_KEYS)
                            or any(v.type != ParameterType.PARAMETER_DOUBLE for v in values)):
                        raise ValueError('Controller must return six double-typed gains.')
                    numbers = [v.double_value for v in values]
                    self._validate(numbers)
                    self._updating = True
                    try:
                        for key, value in zip(GAIN_KEYS, numbers):
                            self.gains[key].setValue(value)
                    finally:
                        self._updating = False
                    self._loaded = True
                    self._snapshot = dict(zip(GAIN_KEYS, numbers))
                    self._edited_keys.clear()
                    self._status('Read OK', 'Read current gains from robot_control; no write made.')
                elif late or self._available is False:
                    self._loaded = False
                    reason = response.result.reason or 'No reason supplied.'
                    self._status('Read again',
                                 'Late Apply ACK: successful=' + str(response.result.successful)
                                 + '. ' + reason + ' Read current values before another Apply.', True)
                elif response.result.successful:
                    self._snapshot = dict(zip(GAIN_KEYS, self._desired))
                    self._edited_keys.clear()
                    self._status('Applied', 'Controller acknowledged all six gains atomically.')
                else:
                    self._status('Rejected', response.result.reason or
                                 'Controller rejected the complete gain update.', True)
                self._controls()
                return
            if time.monotonic() > self._deadline:
                self._loaded = False
                if kind == 'write' and self._future is not None:
                    self._timed_out = True
                    self._status('Unknown', 'Apply ACK timed out; it may still take effect. '
                                 'No overlapping write is allowed. Await the ACK, then Read. '
                                 'If the controller died, restart the GUI after it is recovered.', True)
                else:
                    sent = self._future is not None
                    self._clear_pending()
                    self._status('Read again' if sent else 'Offline',
                                 'Read response timed out; read again.' if sent else
                                 'Parameter service unavailable; no write was sent.', True)
                self._controls()
                return
            if self._future is None and self._clients[kind].service_is_ready():
                self._future = self._clients[kind].call_async(self._request)
                self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        except Exception as error:
            self._loaded = False
            self._clear_pending()
            self._status('Read again',
                         ('Apply outcome unknown; read controller values. ' if kind == 'write'
                          else 'Parameter read failed. ') + str(error), True)
            self._controls()

    def shutdown(self):
        """Stop polling; closing the GUI never sends or retries a gain update."""
        self._closed = True
        self.timer.stop()
        self._loaded = False
        self._controls()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)
