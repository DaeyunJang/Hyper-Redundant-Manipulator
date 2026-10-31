"""Position gain controls use fake parameter services, never a live ROS node."""

from concurrent.futures import Future
from dataclasses import dataclass
import math
import os

from PyQt5.QtWidgets import QApplication
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from gui_py_pkg import position_pid


@dataclass
class Call:
    name: str
    service: object
    request: object
    future: Future


class Client:
    def __init__(self, node, service, name):
        self.node, self.service, self.name = node, service, name

    def service_is_ready(self):
        return self.node.ready[self.service]

    def wait_for_service(self, *_args, **_kwargs):
        raise AssertionError('GUI must not wait synchronously for ROS services.')

    def call_async(self, request):
        assert isinstance(request, self.service.Request)
        if self.node.call_error:
            raise self.node.call_error
        future = Future()
        self.node.calls.append(Call(self.name, self.service, request, future))
        return future


class FakeNode:
    def __init__(self):
        self.calls = []
        self.ready = {GetParameters: True, SetParametersAtomically: True}
        self.call_error = None
        self.values = dict(zip(position_pid.GAIN_NAMES, [6.0, 0.0, .06, 7.0, 0.0, .07]))

    def create_client(self, service, name):
        assert (service, name) in (
            (GetParameters, '/robot_control/get_parameters'),
            (SetParametersAtomically, '/robot_control/set_parameters_atomically'))
        return Client(self, service, name)

    @property
    def writes(self):
        return [call for call in self.calls if call.service is SetParametersAtomically]

    def answer(self, call=None, success=True, reason=''):
        call = call or self.calls[-1]
        if call.service is GetParameters:
            response = GetParameters.Response(values=[
                ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                               double_value=self.values[name])
                for name in call.request.names])
        else:
            response = SetParametersAtomically.Response(result=SetParametersResult(
                successful=success, reason=reason))
            if success:
                for parameter in call.request.parameters:
                    self.values[parameter.name] = parameter.value.double_value
        call.future.set_result(response)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    node = FakeNode()
    widget = position_pid.PositionPIDPanel(node)
    widget.timer.stop()  # Explicit deterministic polling, never a ROS executor.
    yield widget, node
    widget.shutdown()
    widget.close()


def read(widget, node):
    widget.read_settings()
    node.answer()
    widget.poll()


def details(widget):
    return (widget.status_label.text() + ' ' + widget.status_label.toolTip()).lower()


@pytest.mark.parametrize('axis', ['pan', 'tilt'])
def test_gain_tooltips_describe_rate_pd_units_and_single_dt_integration(panel, axis):
    widget, node = panel
    for term, units in [('p', 'P: rad/(m*s).'), ('d', 'D: rad/m.')]:
        tooltip = widget.gains[axis, term].toolTip()
        assert units in tooltip
        assert 'angular rate (rad/s)' in tooltip
        assert 'speed-limit the rate' in tooltip
        assert 'integrate once using feedback dt' in tooltip
        assert 'delta_angle = rate * dt' in tooltip
        assert 'angular increment per update' not in tooltip
        assert 'rad*s/m' not in tooltip
    assert 'requires I=0' in widget.gains[axis, 'i'].toolTip()
    assert not node.calls


def test_startup_is_placeholder_and_requires_successful_read(panel):
    widget, node = panel
    assert not node.calls
    assert widget.gains['pan', 'p'].value() == 5.0
    assert widget.gains['tilt', 'd'].value() == .05
    assert 'placeholder' in details(widget)
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    assert not node.calls
    read(widget, node)
    assert node.calls[0].request.names == list(position_pid.GAIN_NAMES)
    assert widget.gains['pan', 'p'].value() == 6.0
    assert widget.gains['tilt', 'p'].value() == 7.0
    assert widget.apply_button.isEnabled()
    assert all(not widget.gains[axis, 'i'].isEnabled() for axis in ('pan', 'tilt'))
    assert not node.writes


def test_discovery_reads_once_and_offline_requires_fresh_read(panel):
    widget, node = panel
    widget.update_nodes(set())
    assert 'offline' in details(widget)
    assert not widget.read_button.isEnabled()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 1
    node.answer()
    widget.poll()
    for _ in range(3):
        widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 1
    widget.update_nodes(set())
    assert not widget.apply_button.isEnabled()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 2
    assert not widget.apply_button.isEnabled()
    node.answer()
    widget.poll()
    assert widget.apply_button.isEnabled()
    assert not node.writes


def test_apply_sends_exactly_six_atomic_double_gains_and_waits_for_ack(panel):
    widget, node = panel
    read(widget, node)
    widget.gains['pan', 'p'].setValue(8.0)
    widget.gains['tilt', 'd'].setValue(.12)
    assert 'edited' in details(widget)
    assert not node.writes
    widget.apply_settings()
    assert len(node.writes) == 1
    call = node.writes[0]
    parameters = call.request.parameters
    assert [p.name for p in parameters] == list(position_pid.GAIN_NAMES)
    assert [p.value.double_value for p in parameters] == [8., 0., .06, 7., 0., .12]
    assert all(p.value.type == ParameterType.PARAMETER_DOUBLE for p in parameters)
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    widget.read_settings()
    assert len(node.calls) == 2
    assert 'applying' in details(widget)
    node.answer(call)
    widget.poll()
    assert 'applied' in details(widget)
    assert widget.apply_button.isEnabled()


def test_unedited_read_values_keep_full_precision_when_another_gain_changes(panel):
    widget, node = panel
    name = position_pid.GAIN_NAMES[0]
    node.values[name] = 5.123456789
    read(widget, node)
    widget.gains['tilt', 'p'].setValue(9.)
    widget.apply_settings()
    parameters = {p.name: p.value.double_value for p in node.writes[0].request.parameters}
    assert parameters[name] == 5.123456789


def test_atomic_rejection_reports_reason_without_claiming_applied(panel):
    widget, node = panel
    read(widget, node)
    widget.gains['pan', 'p'].setValue(9.)
    widget.apply_settings()
    node.answer(success=False, reason='Position I must be zero')
    widget.poll()
    assert 'rejected' in details(widget)
    assert 'position i must be zero' in details(widget)
    assert node.values[position_pid.GAIN_NAMES[0]] == 6.


@pytest.mark.parametrize('value', [-1., math.nan, math.inf])
def test_nonfinite_or_negative_gain_is_rejected_without_write(panel, value):
    widget, node = panel
    read(widget, node)
    widget._snapshot['pan', 'p'] = value
    widget.apply_settings()
    assert 'invalid' in details(widget)
    assert not node.writes


def test_nonzero_i_is_rejected_even_if_programmatically_changed(panel):
    widget, node = panel
    read(widget, node)
    widget.gains['pan', 'i'].setValue(1.)
    widget.apply_settings()
    assert 'i=0' in details(widget)
    assert not node.writes


def test_invalid_server_read_cannot_enable_apply(panel):
    widget, node = panel
    node.values[position_pid.GAIN_NAMES[1]] = 1.0
    read(widget, node)
    assert not widget.apply_button.isEnabled()
    assert 'i=0' in details(widget)
    assert not node.writes


def test_missing_service_is_nonblocking_bounded_wait_without_write(panel):
    widget, node = panel
    node.ready[GetParameters] = False
    widget.read_settings()
    assert widget.busy and not node.calls
    widget._deadline = 0.
    widget.poll()
    assert not widget.busy
    assert 'offline' in details(widget)
    assert not widget.apply_button.isEnabled()
    assert not node.calls


def test_service_can_appear_during_nonblocking_wait(panel):
    widget, node = panel
    node.ready[GetParameters] = False
    widget.read_settings()
    assert not node.calls
    node.ready[GetParameters] = True
    widget.poll()
    assert len(node.calls) == 1
    node.answer()
    widget.poll()
    assert widget.apply_button.isEnabled()


def test_unavailable_write_service_sends_nothing_and_requires_read(panel):
    widget, node = panel
    read(widget, node)
    node.ready[SetParametersAtomically] = False
    widget.apply_settings()
    assert not node.writes
    widget._deadline = 0.
    widget.poll()
    assert 'no write was sent' in details(widget)
    assert not widget.apply_button.isEnabled()


def test_read_timeout_does_not_allow_late_old_read_to_restore_loaded_state(panel):
    widget, node = panel
    widget.read_settings()
    old = node.calls[-1]
    widget._deadline = 0.
    widget.poll()
    assert not widget.busy and not widget.apply_button.isEnabled()
    node.answer(old)
    widget.poll()
    assert not widget.apply_button.isEnabled()
    read(widget, node)
    assert widget.apply_button.isEnabled()


def test_write_timeout_is_unknown_blocks_retries_and_late_ack_requires_read(panel):
    widget, node = panel
    read(widget, node)
    widget.gains['pan', 'p'].setValue(8.)
    widget.apply_settings()
    outstanding = node.writes[-1]
    widget._deadline = 0.
    widget.poll()
    assert 'unknown' in details(widget)
    assert not outstanding.future.done()
    assert widget.busy
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    widget.read_settings()
    for _ in range(3):
        widget.poll()
    assert len(node.writes) == 1 and len(node.calls) == 2
    node.answer(outstanding)
    widget.poll()
    assert not widget.busy
    assert 'read again' in details(widget)
    assert not widget.apply_button.isEnabled()
    read(widget, node)
    assert widget.apply_button.isEnabled()
    assert widget.gains['pan', 'p'].value() == 8.


def test_future_exception_during_write_is_unknown_and_requires_read(panel):
    widget, node = panel
    read(widget, node)
    widget.apply_settings()
    node.writes[-1].future.set_exception(RuntimeError('lost response'))
    widget.poll()
    assert 'unknown' in details(widget)
    assert 'lost response' in details(widget)
    assert not widget.apply_button.isEnabled()
    assert len(node.writes) == 1


def test_offline_during_pending_write_keeps_it_pending_until_late_ack(panel):
    widget, node = panel
    widget.update_nodes({'/robot_control'})
    node.answer()
    widget.poll()
    widget.apply_settings()
    outstanding = node.writes[-1]
    widget.update_nodes(set())
    assert widget.busy and not outstanding.future.done()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 2
    node.answer(outstanding)
    widget.poll()
    assert 'read again' in details(widget)
    assert not widget.apply_button.isEnabled()


def test_shutdown_never_sends_queued_apply_or_retries(panel):
    widget, node = panel
    read(widget, node)
    node.ready[SetParametersAtomically] = False
    widget.apply_settings()
    widget.shutdown()
    node.ready[SetParametersAtomically] = True
    widget.poll()
    widget.read_settings()
    widget.apply_settings()
    widget.update_nodes({'/robot_control'})
    assert not widget.timer.isActive()
    assert not node.writes


def test_call_exception_is_reported_without_automatic_retry(panel):
    widget, node = panel
    read(widget, node)
    node.call_error = RuntimeError('send unavailable')
    widget.apply_settings()
    assert 'unknown' in details(widget)
    assert not widget.apply_button.isEnabled()
    assert not node.writes
    node.call_error = None
    widget.poll()
    assert not node.writes
