"""Nonblocking, explicit live RealSense exposure controls; no motor traffic."""

from dataclasses import dataclass
import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QWidget,
)
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter


REQUEST_TIMEOUT_SECONDS = 3.0
TARGETS = (('HRM', '/camera/camera'), ('Tag', '/tag_camera/tag_camera'))


@dataclass(frozen=True)
class ExposureSettings:
    """A parameter snapshot, not a measurement of AE's actual exposure."""

    node: str
    module: str
    auto: bool
    value: int
    minimum: int
    maximum: int
    step: int

    @property
    def unit_ms(self):
        return 0.1 if self.module == 'rgb_camera' else 0.001

    def raw_value(self, milliseconds):
        if not math.isfinite(milliseconds):
            raise ValueError('Exposure must be finite.')
        value = round(milliseconds / self.unit_ms)
        if (not math.isclose(value * self.unit_ms, milliseconds, abs_tol=1e-7)
                or not self.minimum <= value <= self.maximum
                or (value - self.minimum) % self.step):
            raise ValueError('Exposure is outside the camera range or supported step.')
        return value


class CameraExposurePanel(QWidget):
    """Single compact row; all response handling and widget work is on Qt's thread."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.settings = None
        self._clients = {}
        self._future = None
        self._pending_request = None
        self._stage = None
        self._deadline = 0.0
        self._target = None
        self._module = None
        self._range = None
        self._desired = None
        self._write_started = False
        self._available_nodes = None
        self._was_available = False
        self._updating = False

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 2, 0, 2)
        row.setSpacing(4)
        row.addWidget(QLabel('Exposure'))
        self.target_combo = QComboBox()
        for label, target in TARGETS:
            self.target_combo.addItem(label, target)
        self.target_combo.setCurrentIndex(1)
        self.target_combo.setToolTip('Select the running camera node, not the next-launch model.')
        self.auto_checkbox = QCheckBox('Auto')
        self.auto_checkbox.setToolTip(
            'Checked: automatic exposure. Unchecked: manual exposure. '
            'Changing this box alone does not apply settings.')
        self.exposure_spin = QDoubleSpinBox()
        self.exposure_spin.setDecimals(3)
        self.exposure_spin.setSuffix(' ms')
        self.exposure_spin.setRange(0.001, 1000.)
        self.exposure_spin.setMaximumWidth(106)
        self.read_button = QPushButton('Read')
        self.apply_button = QPushButton('Apply')
        for button in (self.read_button, self.apply_button):
            button.setFixedWidth(48)
        self.status_label = QLabel('Read first')
        self.status_label.setMinimumWidth(0)
        self.status_label.setMaximumWidth(72)
        for widget in (self.target_combo, self.auto_checkbox, self.exposure_spin,
                       self.read_button, self.apply_button, self.status_label):
            row.addWidget(widget)
        row.addStretch(1)
        self.read_button.clicked.connect(self.read_settings)
        self.apply_button.clicked.connect(self.apply_settings)
        self.target_combo.currentIndexChanged.connect(self._target_changed)
        self.auto_checkbox.toggled.connect(self._edited)
        self.exposure_spin.valueChanged.connect(self._edited)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        self._controls()

    @property
    def busy(self):
        return self._stage is not None

    def _status(self, short, detail, error=False):
        self.status_label.setText(short)
        self.status_label.setStyleSheet('color: #d93025;' if error else 'color: #666;')
        self.status_label.setToolTip(detail)
        self.setToolTip(detail)

    def _controls(self):
        available = (self._available_nodes is None
                     or self.target_combo.currentData() in self._available_nodes)
        loaded = (self.settings is not None
                  and self.settings.node == self.target_combo.currentData())
        self.target_combo.setEnabled(not self.busy)
        self.read_button.setEnabled(not self.busy and available)
        self.auto_checkbox.setEnabled(not self.busy and loaded and available)
        self.exposure_spin.setEnabled(not self.busy and loaded and available
                                      and not self.auto_checkbox.isChecked())
        self.apply_button.setEnabled(not self.busy and loaded and available)

    def _edited(self, *_):
        if not self._updating and self.settings is not None:
            self._status('Edited', 'Not applied yet. Press Apply to change only this camera.')
        self._controls()

    def _target_changed(self, *_):
        if self.busy:
            self._fail(RuntimeError('Camera selection changed; pending request canceled.'))
        self.settings = None
        self._was_available = False
        self._status('Read first', 'Read the selected running camera before applying settings.')
        self._controls()
        if (self._available_nodes is not None
                and self.target_combo.currentData() in self._available_nodes):
            self._was_available = True
            self.read_settings()

    def update_nodes(self, nodes):
        """Read once when this camera appears, never auto-apply after a restart."""
        self._available_nodes = set(nodes)
        available = self.target_combo.currentData() in nodes
        appeared = available and not self._was_available
        self._was_available = available
        if not available and not self.busy:
            self.settings = None
            self._status('Offline', 'Start the selected camera, then Read its exposure settings.')
        self._controls()
        if appeared and not self.busy:
            self.read_settings()

    def _request(self, service, request_type, request, stage):
        key = (self._target, service)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(
                request_type, self._target + '/' + service)
        client = self._clients[key]
        if client.service_is_ready():
            self._future = client.call_async(request)
            self._stage = stage
            self._pending_request = None
        else:
            # A newly created DDS service client needs discovery time. Wait via
            # Qt polling, never wait_for_service() on the GUI thread.
            self._future = None
            self._stage = 'wait_service'
            self._pending_request = (client, request, stage)
        self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
        self._controls()

    def _begin(self, desired=None):
        self._target = self.target_combo.currentData()
        self._desired = desired
        self._write_started = False
        self._status('Reading', f'Reading {self._target}; no camera writes yet.')
        try:
            self._request('list_parameters', ListParameters,
                          ListParameters.Request(prefixes=[], depth=0), 'list')
        except Exception as error:
            self._fail(error)

    def read_settings(self):
        if not self.busy:
            self._begin()

    def apply_settings(self):
        if self.busy or self.settings is None:
            return
        try:
            auto = self.auto_checkbox.isChecked()
            raw = None if auto else self.settings.raw_value(self.exposure_spin.value())
            # Re-read module/range before writes; the device may have restarted.
            self._begin((self.settings.module, auto, raw))
        except Exception as error:
            self._fail(error)

    def _get_values(self, stage):
        self._request('get_parameters', GetParameters, GetParameters.Request(names=[
            self._module + '.enable_auto_exposure', self._module + '.exposure']), stage)

    def _set_value(self, name, value, stage):
        self._status('Applying', f'Applying {name} to {self._target}; awaiting camera ACK.')
        self._request('set_parameters', SetParameters, SetParameters.Request(parameters=[
            Parameter(self._module + '.' + name, value=value).to_parameter_msg()]), stage)
        self._write_started = True

    def poll(self):
        """No service waits, spin loops or ROS callbacks that mutate Qt widgets."""
        if not self.busy:
            return
        try:
            if time.monotonic() > self._deadline:
                raise TimeoutError('Camera request timed out; press Read to check its state.')
            if self._stage == 'wait_service':
                client, request, stage = self._pending_request
                if client.service_is_ready():
                    self._future = client.call_async(request)
                    self._stage = stage
                    self._pending_request = None
                    self._deadline = time.monotonic() + REQUEST_TIMEOUT_SECONDS
                return
            if not self._future.done():
                return
            response = self._future.result()
            stage = self._stage
            self._future = None
            if stage == 'list':
                names = set(response.result.names)
                if {'rgb_camera.exposure', 'rgb_camera.enable_auto_exposure'} <= names:
                    self._module = 'rgb_camera'
                elif {'depth_module.exposure', 'depth_module.enable_auto_exposure',
                      'depth_module.color_profile'} <= names:
                    self._module = 'depth_module'  # D405 shares color and depth exposure.
                else:
                    raise ValueError('No supported color exposure controls on this camera.')
                self._request('describe_parameters', DescribeParameters,
                              DescribeParameters.Request(names=[
                                  self._module + '.exposure',
                                  self._module + '.enable_auto_exposure']), 'describe')
            elif stage == 'describe':
                if (len(response.descriptors) != 2
                        or any(item.read_only for item in response.descriptors)):
                    raise ValueError('Camera exposure parameters are unavailable or read-only.')
                descriptor, auto_descriptor = response.descriptors
                if (descriptor.type != ParameterType.PARAMETER_INTEGER
                        or auto_descriptor.type != ParameterType.PARAMETER_BOOL
                        or not descriptor.integer_range):
                    raise ValueError('Unsupported camera exposure type or missing range.')
                limits = descriptor.integer_range[0]
                if limits.from_value <= 0 or limits.to_value < limits.from_value:
                    raise ValueError('Invalid camera exposure range.')
                self._range = (limits.from_value, limits.to_value, max(1, limits.step))
                self._get_values('read')
            elif stage in ('read', 'verify'):
                if (len(response.values) != 2
                        or response.values[0].type != ParameterType.PARAMETER_BOOL
                        or response.values[1].type != ParameterType.PARAMETER_INTEGER):
                    raise ValueError('Camera returned invalid exposure values.')
                snapshot = ExposureSettings(
                    self._target, self._module,
                    response.values[0].bool_value, response.values[1].integer_value, *self._range)
                if stage == 'read' and self._desired is not None:
                    module, auto, raw = self._desired
                    if snapshot.module != module:
                        raise ValueError('Camera sensor changed; Read its settings and retry.')
                    if not auto:
                        snapshot.raw_value(raw * snapshot.unit_ms)
                    # Exposure writes disable AE in the SDK. Never write exposure after Auto ON.
                    self._set_value('enable_auto_exposure', auto, 'set_auto')
                else:
                    if stage == 'verify':
                        _, auto, raw = self._desired
                        if snapshot.auto != auto or (not auto and snapshot.value != raw):
                            raise ValueError('Camera parameter readback differs from the request.')
                    self._finish(snapshot, applied=stage == 'verify')
            elif stage in ('set_auto', 'set_exposure'):
                if len(response.results) != 1 or not response.results[0].successful:
                    reasons = '; '.join(item.reason for item in response.results)
                    raise RuntimeError('Camera rejected the setting: ' + (reasons or 'no ACK'))
                if stage == 'set_auto' and not self._desired[1]:
                    self._set_value('exposure', self._desired[2], 'set_exposure')
                else:
                    self._get_values('verify')
        except Exception as error:
            self._fail(error)

    def _finish(self, snapshot, *, applied):
        self.settings = snapshot
        self._stage = self._future = None
        self._write_started = False
        self._updating = True
        self.exposure_spin.setRange(snapshot.minimum * snapshot.unit_ms,
                                    snapshot.maximum * snapshot.unit_ms)
        self.exposure_spin.setSingleStep(snapshot.step * snapshot.unit_ms)
        self.exposure_spin.setValue(snapshot.value * snapshot.unit_ms)
        self.auto_checkbox.setChecked(snapshot.auto)
        self._updating = False
        detail = (f'{snapshot.node}: {snapshot.module}.exposure = {snapshot.value}; '
                  f'1 driver unit = {snapshot.unit_ms:g} ms. '
                  'While Auto is ON, the displayed value is a stored manual setting, '
                  'not the measured automatic exposure. Changes are live only; '
                  'camera restart uses launch defaults. Gain/FPS settings are unchanged. '
                  'Long exposures may reduce frame rate (30 FPS has a 33.3 ms frame period).')
        if snapshot.module == 'depth_module':
            detail += ' D405: this changes both color and depth exposure.'
        self.exposure_spin.setToolTip(detail)
        self._status('Applied' if applied else 'Read OK', detail)
        self._controls()

    def _fail(self, error):
        if self._future is not None:
            self._future.cancel()
        detail = str(error)
        if self._write_started:
            detail += ' Settings may have partially changed; Read again. No automatic rollback.'
        self._stage = self._future = None
        self._pending_request = None
        self.settings = None
        self._status('Error', detail, error=True)
        self._controls()
