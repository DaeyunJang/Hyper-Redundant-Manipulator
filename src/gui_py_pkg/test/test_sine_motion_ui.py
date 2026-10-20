"""Sine controls are tested with fake services, never with motor hardware."""

from concurrent.futures import Future
from dataclasses import dataclass
import os

from PyQt5.QtWidgets import QApplication
import pytest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from gui_py_pkg import sine_motion


CONFIG_SERVICE = '/robot_control/set_parameters_atomically'
MOTION_SERVICE = '/kinematics/sine_motion'
STATUS_TOPIC = '/kinematics/sine_status'


@dataclass
class PendingCall:
    service: str
    service_type: object
    request: object
    future: Future


class FakeClient:
    def __init__(self, node, service_type, service):
        self.node = node
        self.service_type = service_type
        self.service = service

    def service_is_ready(self):
        return self.service not in self.node.unavailable

    def wait_for_service(self, *args, **kwargs):
        raise AssertionError('A GUI action must not block waiting for ROS services.')

    def call_async(self, request):
        assert isinstance(request, self.service_type.Request)
        future = Future()
        self.node.calls.append(PendingCall(
            self.service, self.service_type, request, future))
        return future


class FakeNode:
    def __init__(self):
        self.calls = []
        self.clients = {}
        self.subscriptions = {}
        self.unavailable = set()

    def create_client(self, service_type, service, **kwargs):
        assert (service_type, service) in (
            (SetParametersAtomically, CONFIG_SERVICE), (SetBool, MOTION_SERVICE))
        client = FakeClient(self, service_type, service)
        self.clients[service] = client
        return client

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        assert message_type is String
        assert topic == STATUS_TOPIC
        self.subscriptions[topic] = (callback, qos)
        return object()

    @property
    def motion_calls(self):
        return [call for call in self.calls if call.service == MOTION_SERVICE]

    def answer(self, call, successful=True, reason=''):
        response = call.service_type.Response()
        if call.service_type is SetParametersAtomically:
            response.result.successful = successful
            response.result.reason = reason
        else:
            response.success = successful
            response.message = reason
        call.future.set_result(response)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(sine_motion.time, 'monotonic', lambda: now[0])
    node = FakeNode()
    widget = sine_motion.SineMotionPanel(node)
    yield widget, node, now
    widget.close()


def detail(widget):
    return (widget.status_label.text() + ' ' + widget.status_label.toolTip()).lower()


def parameters(call):
    result = {}
    for parameter in call.request.parameters:
        value = parameter.value
        assert value.type in (
            ParameterType.PARAMETER_DOUBLE, ParameterType.PARAMETER_DOUBLE_ARRAY)
        result[parameter.name] = (
            value.double_value if value.type == ParameterType.PARAMETER_DOUBLE
            else list(value.double_array_value))
    return result


def begin_start(widget, node):
    widget.start_motion()
    assert len(node.calls) == 1
    assert node.calls[0].service == CONFIG_SERVICE
    node.answer(node.calls[0])
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True]
    return node.motion_calls[0]


def test_initial_defaults_and_no_automatic_requests(panel):
    widget, node, unused = panel
    assert set(widget.amplitude) == {'pan', 'tilt'}
    assert set(widget.phase) == {'pan', 'tilt'}
    for axis in ('pan', 'tilt'):
        assert widget.amplitude[axis].value() == 45.0
        assert widget.amplitude[axis].minimum() == 0.0
        assert widget.amplitude[axis].maximum() == 45.0
        assert widget.phase[axis].minimum() == -360.0
        assert widget.phase[axis].maximum() == 360.0
    assert widget.phase['pan'].value() == 90.0
    assert widget.phase['tilt'].value() == 0.0
    assert widget.period.value() == 20.0
    assert widget.period.minimum() >= 1.0
    widget.poll()
    assert not node.calls


def test_editing_values_never_sends_motion_or_parameters(panel):
    widget, node, unused = panel
    widget.amplitude['pan'].setValue(0.0)
    widget.phase['tilt'].setValue(-90.0)
    widget.period.setValue(30.0)
    widget.poll()
    assert not node.calls


def test_start_applies_all_three_parameters_atomically_before_enabling(panel):
    widget, node, unused = panel
    widget.start_button.click()
    assert len(node.calls) == 1
    assert node.calls[0].service == CONFIG_SERVICE
    assert parameters(node.calls[0]) == {
        'motion.sine.pan': [0.0, 45.0, 90.0],
        'motion.sine.tilt': [0.0, 45.0, 0.0],
        'motion.sine.period_sec': 20.0,
    }
    assert not node.motion_calls
    assert not widget.start_button.isEnabled()
    for unused in range(3):
        widget.poll()
    assert len(node.calls) == 1
    node.answer(node.calls[0])
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True]
    node.answer(node.motion_calls[0], reason='started')
    widget.poll()
    assert len(node.calls) == 2
    assert not widget.start_button.isEnabled()


@pytest.mark.parametrize('stationary', ['pan', 'tilt'])
def test_zero_amplitude_and_phase_keep_one_axis_at_zero(panel, stationary):
    widget, node, unused = panel
    widget.amplitude[stationary].setValue(0.0)
    widget.phase[stationary].setValue(0.0)
    begin_start(widget, node)
    assert parameters(node.calls[0])[f'motion.sine.{stationary}'] == [0.0, 0.0, 0.0]


def test_independent_signed_phases_and_period_are_preserved(panel):
    widget, node, unused = panel
    widget.amplitude['pan'].setValue(12.5)
    widget.amplitude['tilt'].setValue(8.5)
    widget.phase['pan'].setValue(-90.0)
    widget.phase['tilt'].setValue(35.0)
    widget.period.setValue(40.0)
    begin_start(widget, node)
    assert parameters(node.calls[0]) == {
        'motion.sine.pan': [0.0, 12.5, -90.0],
        'motion.sine.tilt': [0.0, 8.5, 35.0],
        'motion.sine.period_sec': 40.0,
    }


def test_parameter_rejection_never_starts_and_shows_backend_reason(panel):
    widget, node, unused = panel
    widget.start_motion()
    node.answer(node.calls[0], successful=False, reason='speed limit exceeded')
    widget.poll()
    assert not node.motion_calls
    assert 'speed limit exceeded' in detail(widget)
    assert widget.start_button.isEnabled()


@pytest.mark.parametrize('service', [CONFIG_SERVICE, MOTION_SERVICE])
def test_missing_service_returns_without_blocking_or_writing(panel, service):
    widget, node, unused = panel
    node.unavailable.add(service)
    widget.start_motion()
    assert not node.calls
    assert widget.status_label.text()
    assert widget.start_button.isEnabled()


def test_explicit_start_rejection_does_not_retry_or_change_other_controls(panel):
    widget, node, unused = panel
    start = begin_start(widget, node)
    node.answer(start, successful=False, reason='motor output disabled')
    widget.poll()
    assert 'motor output disabled' in detail(widget)
    widget.poll()
    assert len(node.calls) == 2
    assert widget.start_button.isEnabled()


def test_repeated_start_click_is_ignored_while_operation_pending(panel):
    widget, node, unused = panel
    widget.start_motion()
    widget.start_motion()
    widget.start_motion()
    assert len(node.calls) == 1
    node.answer(node.calls[0])
    widget.poll()
    widget.start_motion()
    assert [call.request.data for call in node.motion_calls] == [True]


def test_stop_while_parameter_request_pending_cancels_late_start(panel):
    widget, node, unused = panel
    widget.start_motion()
    config = node.calls[0]
    widget.stop_motion()
    assert [call.request.data for call in node.motion_calls] == [False]
    node.answer(config)
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [False]
    node.answer(node.motion_calls[-1], reason='stopped')
    widget.poll()
    assert not any(call.request.data for call in node.motion_calls)


def test_stop_during_pending_start_reasserts_stop_after_late_start_ack(panel):
    widget, node, unused = panel
    start = begin_start(widget, node)
    widget.stop_button.click()
    immediate_stop = node.motion_calls[-1]
    assert [call.request.data for call in node.motion_calls] == [True, False]
    node.answer(immediate_stop, reason='stopped before start ACK')
    widget.poll()
    assert not widget.start_button.isEnabled()
    node.answer(start, reason='late started')
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True, False, False]
    node.answer(node.motion_calls[-1], reason='stopped after late start')
    widget.poll()
    assert widget.start_button.isEnabled()


def test_config_timeout_never_continues_to_start_when_late_ack_arrives(panel):
    widget, node, now = panel
    widget.start_motion()
    config = node.calls[0]
    now[0] += 3.1
    widget.poll()
    assert not node.motion_calls
    assert not widget.start_button.isEnabled()
    node.answer(config)
    widget.poll()
    assert not node.motion_calls
    # The old request has now settled with no Start sent; explicit retry is safe.
    assert widget.start_button.isEnabled()


def test_start_timeout_blocks_restart_and_keeps_late_start_stoppable(panel):
    widget, node, now = panel
    start = begin_start(widget, node)
    now[0] += 3.1
    widget.poll()
    assert not widget.start_button.isEnabled()
    # An uncertain Start result defensively requests Stop, never a repeat Start.
    assert [call.request.data for call in node.motion_calls] == [True, False]
    count = len(node.calls)
    widget.start_motion()
    assert len(node.calls) == count
    node.answer(node.motion_calls[-1], reason='stopped')
    widget.poll()
    node.answer(start, reason='late started')
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True, False, False]
    node.answer(node.motion_calls[-1], reason='confirmed stop')
    widget.poll()
    assert widget.start_button.isEnabled()


@pytest.mark.parametrize('phase', ['config', 'start'])
def test_service_future_exceptions_are_visible_and_require_stop(panel, phase):
    widget, node, unused = panel
    if phase == 'config':
        widget.start_motion()
        pending = node.calls[0]
    else:
        pending = begin_start(widget, node)
    pending.future.set_exception(RuntimeError('simulated transport error'))
    widget.poll()
    assert 'simulated transport error' in detail(widget)
    assert not widget.start_button.isEnabled()
    if phase == 'start':
        assert [call.request.data for call in node.motion_calls] == [True, False]
    else:
        assert not node.motion_calls


def test_status_subscription_retains_backend_state_without_direct_widget_updates(panel):
    widget, node, unused = panel
    callback, qos = node.subscriptions[STATUS_TOPIC]
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    original = widget.status_label.text()
    callback(String(data='RUNNING sine diagnostic'))
    assert widget.status_label.text() == original
    widget.poll()
    assert 'running sine diagnostic' in detail(widget)
    assert not node.calls


def test_stop_only_sends_false_never_zero_angles_or_motor_commands(panel):
    widget, node, unused = panel
    widget.stop_motion()
    assert len(node.calls) == 1
    assert node.calls[0].service == MOTION_SERVICE
    assert node.calls[0].request.data is False
    node.answer(node.calls[0], reason='stopped; hold last position')
    widget.poll()
    assert len(node.calls) == 1


def test_peak_speed_limit_requires_slower_period_without_changing_controller_limit(panel):
    widget, node, unused = panel
    widget.period.setValue(5.0)
    widget.start_motion()
    assert not node.calls
    assert 'speed' in detail(widget)
    assert widget.start_button.isEnabled()


def test_start_in_other_mode_sends_no_parameters_and_does_not_switch_mode(panel):
    widget, node, unused = panel
    widget._kinematics_selected = lambda: False
    widget.poll()
    assert not widget.start_button.isEnabled()
    widget.start_motion()
    assert not node.calls
    assert 'kinematics' in detail(widget)


def test_mode_change_during_configuration_prevents_start(panel):
    widget, node, unused = panel
    widget.start_motion()
    widget._kinematics_selected = lambda: False
    node.answer(node.calls[0])
    widget.poll()
    assert not node.motion_calls
    assert 'mode changed' in detail(widget)


def test_stop_is_available_even_outside_kinematics_mode(panel):
    widget, node, unused = panel
    widget._kinematics_selected = lambda: False
    widget.poll()
    assert widget.stop_button.isEnabled()
    widget.stop_button.click()
    assert [call.request.data for call in node.motion_calls] == [False]


@pytest.mark.parametrize('failure', ['unavailable', 'timeout', 'rejected', 'exception'])
def test_unconfirmed_stop_never_reenables_start(panel, failure):
    widget, node, now = panel
    if failure == 'unavailable':
        node.unavailable.add(MOTION_SERVICE)
    widget.stop_motion()
    if failure == 'timeout':
        now[0] += 3.1
    elif failure == 'rejected':
        node.answer(node.motion_calls[-1], successful=False, reason='stop rejected')
    elif failure == 'exception':
        node.motion_calls[-1].future.set_exception(RuntimeError('lost connection'))
    widget.poll()
    assert not widget.start_button.isEnabled()
    assert widget.stop_button.isEnabled()
    assert 'stop' in detail(widget)
    count = len(node.calls)
    widget.start_motion()
    assert len(node.calls) == count


def test_pending_stop_is_repeated_after_start_ack_even_if_first_stop_ack_is_late(panel):
    widget, node, unused = panel
    start = begin_start(widget, node)
    widget.stop_motion()
    first_stop = node.motion_calls[-1]
    node.answer(start, reason='started')
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True, False]
    node.answer(first_stop, reason='stopped')
    widget.poll()
    assert [call.request.data for call in node.motion_calls] == [True, False, False]
    assert not widget.start_button.isEnabled()
    node.answer(node.motion_calls[-1], reason='stop after start confirmed')
    widget.poll()
    assert widget.start_button.isEnabled()


def test_backend_stop_arriving_during_start_request_is_not_dropped(panel):
    widget, node, unused = panel
    start = begin_start(widget, node)
    callback, unused = node.subscriptions[STATUS_TOPIC]
    callback(String(data='STOPPED because feedback disappeared'))
    widget.poll()
    assert not widget.start_button.isEnabled()
    node.answer(start, reason='started before feedback disappeared')
    widget.poll()
    assert 'feedback disappeared' in detail(widget)
    assert widget.start_button.isEnabled()
    assert [call.request.data for call in node.motion_calls] == [True]


def test_initial_latched_stopped_message_does_not_override_new_start_ack(panel):
    widget, node, unused = panel
    callback, unused = node.subscriptions[STATUS_TOPIC]
    callback(String(data='STOPPED initial'))
    widget.start_motion()
    widget.poll()
    node.answer(node.calls[0])
    widget.poll()
    node.answer(node.motion_calls[0], reason='started now')
    widget.poll()
    assert not widget.start_button.isEnabled()
    assert 'started now' in detail(widget)


def test_config_timeout_does_not_allow_new_config_until_old_request_settles(panel):
    widget, node, now = panel
    widget.start_motion()
    old_config = node.calls[0]
    now[0] += 3.1
    widget.poll()
    widget.start_motion()
    assert len(node.calls) == 1
    widget.stop_motion()
    node.answer(node.motion_calls[-1], reason='stopped')
    widget.poll()
    assert not widget.start_button.isEnabled()
    widget.start_motion()
    assert len(node.calls) == 2
    node.answer(old_config)
    widget.poll()
    assert not any(call.request.data for call in node.motion_calls)
    # A late settings response is followed by another explicit stop confirmation.
    if not node.motion_calls[-1].future.done():
        node.answer(node.motion_calls[-1], reason='stopped after configuration')
        widget.poll()
    assert widget.start_button.isEnabled()
