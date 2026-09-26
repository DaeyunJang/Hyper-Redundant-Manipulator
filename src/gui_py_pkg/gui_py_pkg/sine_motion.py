"""Explicit, nonblocking controls for the controller's existing IK sine motion."""

import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QWidget
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


TIMEOUT_SECONDS = 3.0


class SineMotionPanel(QWidget):
    """Only Start applies settings; Stop never changes mode or enables motors."""

    def __init__(self, node, parent=None, kinematics_selected=None):
        super().__init__(parent)
        self._kinematics_selected = kinematics_selected or (lambda: True)
        self._parameters = node.create_client(
            SetParametersAtomically, '/robot_control/set_parameters_atomically')
        self._motion = node.create_client(SetBool, '/kinematics/sine_motion')
        self._status_message = None
        self._seen_status = None
        self._subscription = node.create_subscription(
            String, '/kinematics/sine_status', self.receive_status,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._pending = None
        self._future = None
        self._deadline = 0.0
        self._timed_out = False
        self._stop_future = None
        self._stop_deadline = 0.0
        self._stop_timed_out = False
        self._stop_again = False
        self._cancel_start = False
        self._active = False
        self._unknown = False

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        row.addWidget(QLabel('IK sine'))
        self.amplitude, self.phase = {}, {}
        for axis, phase in (('pan', 90.), ('tilt', 0.)):
            row.addSpacing(4)
            row.addWidget(QLabel(axis.capitalize() + ' A'))
            self.amplitude[axis] = self._spin(0., 45., 45., '°')
            self.amplitude[axis].setToolTip(
                'Amplitude [deg], centred on 0°. 0 means target 0°, not hold current pose.')
            row.addWidget(self.amplitude[axis])
            row.addWidget(QLabel('φ'))
            self.phase[axis] = self._spin(-360., 360., phase, '°')
            self.phase[axis].setToolTip('Phase [deg]. Pan φ − Tilt φ is the phase difference.')
            row.addWidget(self.phase[axis])
        row.addSpacing(4)
        row.addWidget(QLabel('T'))
        self.period = self._spin(1., 3600., 20., ' s')
        self.period.setToolTip('Common period [s]; each angle = A sin(2πt/T + φ).')
        row.addWidget(self.period)
        self.start_button = QPushButton('Start')
        self.stop_button = QPushButton('Stop')
        self.start_button.setFixedWidth(45)
        self.stop_button.setFixedWidth(45)
        self.start_button.setToolTip(
            'Apply settings then start IK sine. Kinematics mode required. '
            'Does not enable physical motor output.')
        self.stop_button.setToolTip(
            'Stop new sine targets; retain the last target. No zero return. '
            'NOT a hardware emergency stop.')
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        self.status_label = QLabel('Idle')
        self.status_label.setFixedWidth(65)
        row.addWidget(self.status_label)
        row.addStretch(1)
        self.start_button.clicked.connect(self.start_motion)
        self.stop_button.clicked.connect(self.stop_motion)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        self._controls()

    @staticmethod
    def _spin(minimum, maximum, value, suffix):
        spin = QDoubleSpinBox()
        spin.setDecimals(1)
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setFixedWidth(72)
        return spin

    def receive_status(self, message):
        # ROS callbacks only replace a reference. Qt's poll updates widgets.
        self._status_message = message

    def _status(self, short, detail, error=False):
        self.status_label.setText(short)
        self.status_label.setToolTip(detail)
        self.status_label.setStyleSheet('color: #d93025;' if error else 'color: #188038;')
        self.setToolTip(detail)

    def _controls(self):
        idle = not (self._pending or self._stop_future or self._active or self._unknown)
        for widget in (*self.amplitude.values(), *self.phase.values(), self.period):
            widget.setEnabled(idle)
        self.start_button.setEnabled(idle and self._kinematics_selected())
        # Stop remains accessible in every operation mode and uncertainty state.
        self.stop_button.setEnabled(True)

    def start_motion(self):
        if self._pending or self._stop_future or self._active or self._unknown:
            return
        try:
            if not self._kinematics_selected():
                raise ValueError('Select Kinematics mode first; no automatic mode switching.')
            if not self._parameters.service_is_ready() or not self._motion.service_is_ready():
                raise RuntimeError('Services unavailable. Start robot_control, then retry.')
            if any(2 * math.pi * spin.value() / self.period.value() > 15.
                   for spin in self.amplitude.values()):
                raise ValueError('Peak speed >15 deg/s. Increase period or reduce amplitude.')
            parameters = [
                Parameter('motion.sine.' + axis,
                          value=[0., self.amplitude[axis].value(), self.phase[axis].value()]
                          ).to_parameter_msg() for axis in ('tilt', 'pan')]
            parameters.append(Parameter(
                'motion.sine.period_sec', value=self.period.value()).to_parameter_msg())
            self._future = self._parameters.call_async(
                SetParametersAtomically.Request(parameters=parameters))
            self._pending = 'configure'
            self._deadline = time.monotonic() + TIMEOUT_SECONDS
            self._timed_out = self._cancel_start = False
            self._status('Applying', 'Applying all sine settings atomically; not started yet.')
        except Exception as error:
            self._status('Error', str(error), True)
        self._controls()

    def stop_motion(self):
        self._cancel_start = True
        # Keep pending config too: a late write must settle before another Start
        # can apply a new waveform. Its continuation is canceled, not its Future.
        self._unknown = True
        self._request_stop()
        self._controls()

    def _request_stop(self):
        if self._stop_future is not None and not self._stop_timed_out:
            self._stop_again = True
            return
        try:
            if not self._motion.service_is_ready():
                raise RuntimeError('Stop NOT confirmed: service unavailable. Retry Stop.')
            self._stop_future = self._motion.call_async(SetBool.Request(data=False))
            self._stop_deadline = time.monotonic() + TIMEOUT_SECONDS
            self._stop_timed_out = False
            self._stop_again = False
            self._status('Stopping', 'Waiting for Stop acknowledgement; not an emergency stop.')
        except Exception as error:
            self._unknown = True
            self._status('Unknown', str(error), True)

    def _poll_pending(self, now):
        if self._pending is None:
            return
        stage = self._pending
        if not self._future.done():
            if now <= self._deadline or self._timed_out:
                return
            self._timed_out = True
            if stage == 'configure':
                self._cancel_start = True
                self._status('Timeout', 'Settings timeout; no Start sent. Awaiting reply.', True)
            else:
                # A delivered Start cannot be undone by canceling its local Future.
                # Keep watching it and issue another Stop once it resolves.
                self._unknown = self._cancel_start = True
                self._request_stop()
                self._status('Unknown', 'Start timeout. Stop requested; awaiting reply.', True)
            return
        future = self._future
        self._pending = self._future = None
        try:
            try:
                result = future.result()
            except Exception:
                self._unknown = True
                raise
            if stage == 'configure':
                if self._cancel_start:
                    if self._unknown:
                        self._request_stop()
                    else:
                        self._cancel_start = False
                        self._status('Idle', 'Late settings reply received; no Start sent.')
                    return
                if not result.result.successful:
                    raise ValueError(result.result.reason)
                if not self._kinematics_selected():
                    raise ValueError('Mode changed while applying settings; no Start sent.')
                if not self._motion.service_is_ready():
                    raise RuntimeError('Sine service unavailable; settings applied, not started.')
                # Assign stage first: a transport exception has an uncertain outcome.
                stage = 'start'
                self._seen_status = self._status_message
                self._future = self._motion.call_async(SetBool.Request(data=True))
                self._pending = 'start'
                self._deadline = now + TIMEOUT_SECONDS
                self._timed_out = False
                self._status('Starting', 'Waiting for Start acknowledgement.')
            elif self._cancel_start:
                self._request_stop()  # After late Start: do not trust an earlier Stop ACK.
            elif result.success:
                self._active = True
                self._status('Dry run' if result.message.startswith('DRY_RUN') else 'Running',
                             result.message)
            else:
                self._status('Rejected', result.message, True)
                self._seen_status = None  # May already be running from another client.
        except Exception as error:
            if stage == 'start':
                self._unknown = self._cancel_start = True
                self._request_stop()
                self._status('Unknown', 'Start result uncertain: ' + str(error), True)
            else:
                self._status('Unknown' if self._unknown else 'Rejected', str(error), True)

    def _poll_stop(self, now):
        if self._stop_future is None:
            return
        if not self._stop_future.done():
            if now > self._stop_deadline and not self._stop_timed_out:
                self._stop_timed_out = True
                self._unknown = True
                self._status('Unknown', 'Stop NOT confirmed (timeout). Retry Stop.', True)
            return
        future = self._stop_future
        self._stop_future = None
        try:
            result = future.result()
            if self._stop_again:
                self._request_stop()
            elif not result.success:
                raise RuntimeError(result.message)
            elif self._pending is not None:
                self._status('Stopping', 'Stop acknowledged; awaiting earlier request reply.')
            else:
                self._unknown = self._active = self._cancel_start = False
                self._seen_status = self._status_message
                self._status('Stopped', result.message)
        except Exception as error:
            self._unknown = True
            self._status('Unknown', 'Stop NOT confirmed: ' + str(error), True)

    def poll(self):
        now = time.monotonic()
        self._poll_pending(now)
        self._poll_stop(now)
        message = self._status_message
        if message is not None and message is not self._seen_status:
            if not self._pending and self._stop_future is None and not self._unknown:
                self._seen_status = message
                text = message.data
                if text.startswith(('RUNNING', 'DRY_RUN', 'STOPPED')):
                    self._active = not text.startswith('STOPPED')
                    self._status('Stopped' if not self._active else
                                 ('Dry run' if text.startswith('DRY_RUN') else 'Running'), text)
        self._controls()
