"""Admittance target controls with fake parameter services and no live ROS."""

from concurrent.futures import Future
from dataclasses import dataclass
import math
import os

from PyQt5.QtWidgets import QApplication
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from gui_py_pkg import admittance_target


@dataclass
class Call:
    service: object
    request: object
    future: Future


class Client:
    def __init__(self, node, service):
        self.node, self.service = node, service

    def service_is_ready(self):
        return self.node.ready[self.service]

    def wait_for_service(self, *_args, **_kwargs):
        raise AssertionError('Parameter services must never block the Qt thread.')

    def call_async(self, request):
        assert isinstance(request, self.service.Request)
        if self.node.call_error:
            raise self.node.call_error
        call = Call(self.service, request, Future())
        self.node.calls.append(call)
        return call.future


class FakeNode:
    def __init__(self):
        self.calls = []
        self.ready = {GetParameters: True, SetParametersAtomically: True}
        self.call_error = None
        self.values = dict(zip(admittance_target.PARAMETER_NAMES, (0.1, -0.2, -1., 2.)))

    def create_client(self, service, name):
        assert (service, name) in (
            (GetParameters, '/robot_control/get_parameters'),
            (SetParametersAtomically, '/robot_control/set_parameters_atomically'))
        return Client(self, service)

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
    widget = admittance_target.AdmittanceTargetPanel(node)
    widget.timer.stop()
    yield widget, node
    widget.shutdown()
    widget.close()


def read(widget, node):
    widget.read_settings()
    node.answer()
    widget.poll()


def details(widget):
    return (widget.status_label.text() + ' ' + widget.status_label.toolTip()).lower()


def test_startup_requires_read_and_never_writes_or_exposes_placeholder_target(panel):
    widget, node = panel
    assert not node.calls
    assert widget.target_plot_mn() is None
    assert widget.confirmed_target_mn() is None
    assert widget.confirmed_force_sign is None
    assert 'placeholder' in details(widget)
    assert not widget.apply_button.isEnabled()
    assert all(not spin.isEnabled() for spin in widget.targets.values())
    widget.apply_settings()
    assert not node.calls
    read(widget, node)
    assert node.calls[0].request.names == list(admittance_target.PARAMETER_NAMES)
    assert widget.targets['y'].value() == 100.
    assert widget.targets['z'].value() == -200.
    assert widget.confirmed_target_mn() == (100., -200.)
    assert widget.target_plot_mn() == (-100., 200.)
    assert widget.confirmed_force_sign == -1.
    assert 'max 2000' in widget.limits_label.text()
    assert widget.apply_button.isEnabled()
    assert 'motion' in widget.apply_button.toolTip()
    assert not node.writes


@pytest.mark.parametrize('sign', [-1., 1.])
def test_plot_targets_divide_by_confirmed_force_sign_and_preserve_signed_zero(panel, sign):
    widget, node = panel
    node.values['admittance/force_sign'] = sign
    node.values[admittance_target.TARGET_NAMES[0]] = -0.25
    node.values[admittance_target.TARGET_NAMES[1]] = 0.
    read(widget, node)
    assert widget.confirmed_target_mn() == (-250., 0.)
    assert widget.target_plot_mn() == (-250. / sign, 0.)
    widget.targets['y'].setValue(700.)
    assert widget.target_plot_mn() == (-250. / sign, 0.)
    assert not node.writes


def test_discovery_reads_once_and_recovery_requires_fresh_confirmed_read(panel):
    widget, node = panel
    widget.update_nodes(set())
    assert not widget.read_button.isEnabled()
    assert widget.target_plot_mn() is None
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 1
    node.answer()
    widget.poll()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 1
    widget.update_nodes(set())
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 2
    assert widget.target_plot_mn() is None
    node.answer()
    widget.poll()
    assert widget.target_plot_mn() == (-100., 200.)
    assert not node.writes


def test_apply_sends_only_two_targets_atomically_in_newtons_and_verifies_ack(panel):
    widget, node = panel
    events = []
    widget.settings_changed.connect(lambda: events.append(widget.target_plot_mn()))
    read(widget, node)
    widget.targets['y'].setValue(-300.)
    widget.targets['z'].setValue(400.)
    assert widget.target_plot_mn() == (-100., 200.)
    widget.apply_settings()
    assert len(node.writes) == 1
    parameters = node.writes[0].request.parameters
    assert [parameter.name for parameter in parameters] == list(admittance_target.TARGET_NAMES)
    assert [parameter.value.double_value for parameter in parameters] == [-.3, .4]
    assert all(parameter.value.type == ParameterType.PARAMETER_DOUBLE for parameter in parameters)
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    widget.read_settings()
    assert len(node.calls) == 2
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Verifying'
    assert widget.target_plot_mn() is None
    assert len(node.calls) == 3
    assert node.calls[-1].service is GetParameters
    assert not widget.apply_button.isEnabled()
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Applied'
    assert widget.confirmed_target_mn() == (-300., 400.)
    assert widget.target_plot_mn() == (300., -400.)
    assert widget.apply_button.isEnabled()
    assert events[0] == (-100., 200.) and events[-1] == (300., -400.)
    assert None in events


def test_unedited_target_keeps_server_precision_on_apply(panel):
    widget, node = panel
    precise = 0.123456789123
    node.values[admittance_target.TARGET_NAMES[0]] = precise
    read(widget, node)
    assert widget.targets['y'].value() == 123.457
    widget.targets['z'].setValue(500.)
    widget.apply_settings()
    assert node.writes[-1].request.parameters[0].value.double_value == precise


def test_rejected_atomic_update_restores_previous_confirmed_target_and_keeps_edits(panel):
    widget, node = panel
    read(widget, node)
    widget.targets['y'].setValue(300.)
    widget.apply_settings()
    node.answer(success=False, reason='target force norm rejected')
    widget.poll()
    assert 'rejected' in details(widget)
    assert widget.target_plot_mn() == (-100., 200.)
    assert widget.targets['y'].value() == 300.
    assert widget.apply_button.isEnabled()
    assert len(node.calls) == 2


def test_readback_with_concurrent_external_change_never_claims_requested_target_applied(panel):
    widget, node = panel
    read(widget, node)
    widget.targets['y'].setValue(300.)
    widget.apply_settings()
    node.answer()
    widget.poll()
    node.values[admittance_target.TARGET_NAMES[0]] = .5
    node.values['admittance/force_sign'] = 1.
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Changed'
    assert widget.target_plot_mn() == (500., -200.)
    assert widget.targets['y'].value() == 500.
    assert len(node.writes) == 1


@pytest.mark.parametrize('name,value', [
    ('admittance/target_force_y_N', math.nan),
    ('admittance/target_force_z_N', math.inf),
    ('admittance/force_sign', 0.), ('admittance/force_sign', 2.),
    ('admittance/max_force_N', 0.), ('admittance/max_force_N', -1.),
    ('admittance/target_force_y_N', 3.),
])
def test_invalid_server_read_disables_apply_and_plot(panel, name, value):
    widget, node = panel
    node.values[name] = value
    read(widget, node)
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    assert not node.writes


def test_target_vector_norm_limit_rejects_without_write_and_preserves_confirmed_plot(panel):
    widget, node = panel
    read(widget, node)
    widget.targets['y'].setValue(1500.)
    widget.targets['z'].setValue(1500.)
    widget.apply_settings()
    assert 'vector norm' in details(widget)
    assert widget.target_plot_mn() == (-100., 200.)
    assert not node.writes


@pytest.mark.parametrize('count,kind', [
    (3, ParameterType.PARAMETER_DOUBLE), (4, ParameterType.PARAMETER_INTEGER),
])
def test_missing_or_wrong_parameter_types_are_not_confirmed(panel, count, kind):
    widget, node = panel
    widget.read_settings()
    node.calls[-1].future.set_result(GetParameters.Response(values=[
        ParameterValue(type=kind, double_value=1.) for _ in range(count)]))
    widget.poll()
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()


def test_periodic_read_refreshes_sign_and_targets_without_overwriting_pending_edits(panel):
    widget, node = panel
    widget.update_nodes({'/robot_control'})
    node.answer()
    widget.poll()
    widget.targets['y'].setValue(700.)
    node.values[admittance_target.TARGET_NAMES[0]] = .25
    node.values[admittance_target.TARGET_NAMES[1]] = -.5
    node.values['admittance/force_sign'] = 1.
    widget._next_refresh = 0.
    widget.poll()
    assert node.calls[-1].service is GetParameters
    assert widget.targets['y'].isEnabled()
    node.answer()
    widget.poll()
    assert widget.targets['y'].value() == 700.
    assert widget.targets['z'].value() == -500.
    assert widget.target_plot_mn() == (250., -500.)
    assert widget.status_label.text() == 'Edited'
    assert not node.writes
    widget.apply_settings()
    assert [p.value.double_value for p in node.writes[-1].request.parameters] == [.7, -.5]


def test_periodic_read_does_not_run_without_discovery_or_after_unknown_result(panel):
    widget, node = panel
    read(widget, node)
    widget._next_refresh = 0.
    widget.poll()
    assert len(node.calls) == 1
    widget.update_nodes({'/robot_control'})
    node.answer()
    widget.poll()
    widget.apply_settings()
    node.writes[-1].future.set_exception(RuntimeError('lost response'))
    widget.poll()
    count = len(node.calls)
    widget._next_refresh = 0.
    widget.poll()
    assert len(node.calls) == count
    assert 'unknown' in details(widget)
    assert widget.target_plot_mn() is None


def test_missing_service_is_bounded_nonblocking_and_sends_no_write(panel):
    widget, node = panel
    node.ready[GetParameters] = False
    widget.read_settings()
    assert widget.busy and not node.calls
    widget._deadline = 0.
    widget.poll()
    assert not widget.busy and not node.calls
    assert 'offline' in details(widget)
    assert widget.target_plot_mn() is None


def test_service_appears_during_nonblocking_read_wait(panel):
    widget, node = panel
    node.ready[GetParameters] = False
    widget.read_settings()
    node.ready[GetParameters] = True
    widget.poll()
    node.answer()
    widget.poll()
    assert widget.target_plot_mn() == (-100., 200.)


def test_unavailable_write_service_never_writes_and_requires_read(panel):
    widget, node = panel
    read(widget, node)
    node.ready[SetParametersAtomically] = False
    widget.apply_settings()
    widget._deadline = 0.
    widget.poll()
    assert not node.writes
    assert 'no write was sent' in details(widget)
    assert not widget.apply_button.isEnabled()
    assert widget.target_plot_mn() is None


@pytest.mark.parametrize('done_before_poll', [False, True])
def test_read_timeout_does_not_accept_late_old_snapshot(panel, done_before_poll):
    widget, node = panel
    widget.read_settings()
    outstanding = node.calls[-1]
    widget._deadline = 0.
    if done_before_poll:
        node.answer(outstanding)
    widget.poll()
    if not done_before_poll:
        node.answer(outstanding)
        widget.poll()
    assert widget.target_plot_mn() is None
    assert not widget.busy
    assert not widget.apply_button.isEnabled()
    read(widget, node)
    assert widget.target_plot_mn() == (-100., 200.)


@pytest.mark.parametrize('success', [False, True])
def test_write_timeout_blocks_retries_and_late_ack_requires_explicit_read(panel, success):
    widget, node = panel
    read(widget, node)
    widget.targets['y'].setValue(300.)
    widget.apply_settings()
    outstanding = node.writes[-1]
    widget._deadline = 0.
    widget.poll()
    assert widget.busy and not outstanding.future.done()
    assert widget.target_plot_mn() is None
    assert 'unknown' in details(widget)
    widget.read_settings()
    widget.apply_settings()
    assert len(node.calls) == 2
    node.answer(outstanding, success=success)
    widget.poll()
    assert widget.target_plot_mn() is None
    assert not widget.busy
    assert not widget.apply_button.isEnabled()
    assert 'late apply ack' in details(widget)
    assert len(node.calls) == 2
    read(widget, node)
    assert widget.target_plot_mn() == (-300. if success else -100., 200.)


def test_ack_arriving_after_deadline_before_next_poll_is_not_applied(panel):
    widget, node = panel
    read(widget, node)
    widget.apply_settings()
    widget._deadline = 0.
    node.answer()
    widget.poll()
    assert 'late apply ack' in details(widget)
    assert widget.target_plot_mn() is None
    assert len(node.calls) == 2


def test_verification_timeout_does_not_show_unconfirmed_successful_write(panel):
    widget, node = panel
    read(widget, node)
    widget.apply_settings()
    node.answer()
    widget.poll()
    verification = node.calls[-1]
    widget._deadline = 0.
    widget.poll()
    node.answer(verification)
    widget.poll()
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    assert widget.status_label.text() == 'Read again'
    assert len(node.writes) == 1


def test_controller_disappearance_during_write_keeps_pending_ack_and_hides_plot(panel):
    widget, node = panel
    widget.update_nodes({'/robot_control'})
    node.answer()
    widget.poll()
    widget.apply_settings()
    outstanding = node.writes[-1]
    widget.update_nodes(set())
    assert widget.busy and not outstanding.future.done()
    assert widget.target_plot_mn() is None
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 2
    node.answer(outstanding)
    widget.poll()
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    assert len(node.calls) == 2


def test_shutdown_does_not_dispatch_queued_apply_or_retry(panel):
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
    assert widget.target_plot_mn() is None
    assert not widget.timer.isActive()
    assert not node.writes


def test_dispatch_exception_is_unknown_and_never_retries_automatically(panel):
    widget, node = panel
    read(widget, node)
    node.call_error = RuntimeError('send unavailable')
    widget.apply_settings()
    assert 'unknown' in details(widget)
    assert widget.target_plot_mn() is None
    assert not widget.apply_button.isEnabled()
    node.call_error = None
    widget.poll()
    assert not node.writes
