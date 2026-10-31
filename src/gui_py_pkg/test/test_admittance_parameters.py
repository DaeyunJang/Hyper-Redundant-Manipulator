"""M/B/K panel checks using fake parameter services, never live robot ROS calls."""

from concurrent.futures import Future
from dataclasses import dataclass
import math
import os

from PyQt5.QtWidgets import QApplication, QDialog, QLineEdit, QWidget
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from gui_py_pkg import admittance_parameters as module


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
        raise AssertionError('The GUI must never block on parameter services.')

    def call_async(self, request):
        assert isinstance(request, self.service.Request)
        call = Call(self.service, request, Future())
        self.node.calls.append(call)
        return call.future


class FakeNode:
    def __init__(self):
        self.calls = []
        self.ready = {GetParameters: True, SetParametersAtomically: True}
        self.values = dict(zip(module.PARAMETER_NAMES, (0.05, 1.0, 20.0, 1)))

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
                ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                               integer_value=self.values[name]) if name == 'control_mode' else
                ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                               double_value=self.values[name]) for name in call.request.names])
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
    widget = module.AdmittanceParametersPanel(node)
    widget.timer.stop()
    yield widget, node
    widget.shutdown()
    widget.close()


def read(widget, node):
    widget.read_settings()
    node.answer()
    widget.poll()


def test_requires_read_and_only_reads_three_gains_and_integer_mode(panel):
    widget, node = panel
    assert not node.calls
    assert not widget.apply_button.isEnabled()
    assert not widget.tune_button.isEnabled()
    assert all(not spin.isEnabled() for spin in widget.parameters.values())
    widget.apply_settings()
    assert not node.calls
    read(widget, node)
    assert node.calls[-1].request.names == list(module.PARAMETER_NAMES)
    assert [spin.value() for spin in widget.parameters.values()] == [.05, 1., 20.]
    assert widget.apply_button.isEnabled()
    assert widget.tune_button.isEnabled()
    assert not node.writes


@pytest.mark.parametrize('accepted', [False, True])
def test_tuning_only_stages_candidate_and_never_calls_services(panel, monkeypatch, accepted):
    from gui_py_pkg.admittance_tuning import Candidate

    widget, node = panel
    read(widget, node)
    widget.parameters['m'].setValue(.3)
    old_values = tuple(spin.value() for spin in widget.parameters.values())
    calls_before = len(node.calls)

    class FakeDialog:
        def __init__(self, current, parent):
            assert current == (.05, 1., 20.)  # Confirmed, not pending .3.
            assert parent is widget
            self.candidate = Candidate(.1, 2., 0.)

        def exec_(self):
            return QDialog.Accepted if accepted else QDialog.Rejected

        def deleteLater(self):
            pass

    monkeypatch.setattr(module, 'AdmittanceTuningDialog', FakeDialog)
    widget.open_tuning()
    assert len(node.calls) == calls_before
    assert not node.writes
    values = tuple(spin.value() for spin in widget.parameters.values())
    assert values == ((.1, 2., 0.) if accepted else old_values)
    assert widget._edited_keys == (set(module.KEYS) if accepted else {'m'})
    if accepted:
        assert widget.status_label.text() == 'Candidate'
        widget.apply_settings()
        assert len(node.writes) == 1
        assert [parameter.value.double_value for parameter in
                node.writes[-1].request.parameters] == [.1, 2., 0.]


def test_tuning_can_inspect_admittance_but_still_cannot_apply(panel, monkeypatch):
    from gui_py_pkg.admittance_tuning import Candidate

    widget, node = panel
    node.values['control_mode'] = 4
    read(widget, node)
    assert widget.tune_button.isEnabled()

    class FakeDialog:
        candidate = Candidate(.1, 2., 0.)

        def __init__(self, *_unused):
            pass

        def exec_(self):
            return QDialog.Accepted

        def deleteLater(self):
            pass

    monkeypatch.setattr(module, 'AdmittanceTuningDialog', FakeDialog)
    widget.open_tuning()
    assert widget._edited_keys == set(module.KEYS)
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    assert not node.writes
    assert widget.status_label.text() == 'Leave Admit'


def test_tuning_needs_confirmed_snapshot_and_discards_result_if_controller_disappears(
        panel, monkeypatch):
    from gui_py_pkg.admittance_tuning import Candidate

    widget, node = panel
    widget.open_tuning()
    assert not node.calls
    read(widget, node)
    old_values = tuple(spin.value() for spin in widget.parameters.values())

    class FakeDialog:
        candidate = Candidate(.1, 2., 0.)

        def __init__(self, *_unused):
            pass

        def exec_(self):
            widget.update_nodes(set())
            return QDialog.Accepted

        def deleteLater(self):
            pass

    monkeypatch.setattr(module, 'AdmittanceTuningDialog', FakeDialog)
    widget.open_tuning()
    assert tuple(spin.value() for spin in widget.parameters.values()) == old_values
    assert not node.writes
    assert not widget.apply_button.isEnabled()
    assert not widget.tune_button.isEnabled()


def test_discovery_and_reappearance_reads_without_writes(panel):
    widget, node = panel
    widget.update_nodes(set())
    assert not widget.read_button.isEnabled()
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 1
    node.answer()
    widget.poll()
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


def test_explicit_apply_is_atomic_three_doubles_only_and_requires_readback(panel):
    widget, node = panel
    read(widget, node)
    for key, value in zip(module.KEYS, (.1, 3., 40.)):
        widget.parameters[key].setValue(value)
    assert not node.writes
    widget.apply_settings()
    assert len(node.writes) == 1
    request = node.writes[0].request
    assert [parameter.name for parameter in request.parameters] == list(module.GAIN_NAMES)
    assert [parameter.value.double_value for parameter in request.parameters] == [.1, 3., 40.]
    assert all(parameter.value.type == ParameterType.PARAMETER_DOUBLE
               for parameter in request.parameters)
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    widget.read_settings()
    assert len(node.calls) == 2
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Verifying'
    assert not widget.apply_button.isEnabled()
    assert len(node.calls) == 3
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Applied'
    assert widget.apply_button.isEnabled()


def test_unedited_values_preserve_server_precision(panel):
    widget, node = panel
    precise = .0123456789123
    node.values['admittance/M_d'] = precise
    read(widget, node)
    assert widget.parameters['m'].value() == .0123
    widget.parameters['b'].setValue(2.)
    widget.apply_settings()
    assert node.writes[-1].request.parameters[0].value.double_value == precise


def test_known_admittance_mode_disables_apply_but_allows_pending_edits(panel):
    widget, node = panel
    node.values['control_mode'] = 4
    read(widget, node)
    assert widget.status_label.text() == 'Leave Admit'
    assert not widget.apply_button.isEnabled()
    widget.parameters['m'].setValue(.2)
    widget.apply_settings()
    assert not node.writes
    assert widget.parameters['m'].value() == .2
    assert 'never changes mode' in widget.status_label.toolTip()


def test_refresh_updates_mode_and_unedited_fields_preserving_pending_values(panel):
    widget, node = panel
    widget.update_nodes({'/robot_control'})
    node.answer()
    widget.poll()
    widget.parameters['b'].setValue(2.)
    node.values['admittance/M_d'] = .1
    node.values['admittance/B_d'] = 3.
    node.values['control_mode'] = 4
    widget._next_refresh = 0.
    widget.poll()
    assert widget.parameters['b'].isEnabled()
    node.answer()
    widget.poll()
    assert widget.parameters['m'].value() == .1
    assert widget.parameters['b'].value() == 2.
    assert not widget.apply_button.isEnabled()
    node.values['control_mode'] = 3
    widget._next_refresh = 0.
    widget.poll()
    node.answer()
    widget.poll()
    assert widget.apply_button.isEnabled()
    assert widget.status_label.text() == 'Edited'
    assert widget.parameters['b'].value() == 2.
    assert not node.writes


def test_backend_race_rejection_never_switches_mode_or_retries(panel):
    widget, node = panel
    read(widget, node)
    widget.parameters['k'].setValue(0.)
    widget.apply_settings()
    node.values['control_mode'] = 4
    node.answer(success=False, reason='Leave admittance mode before editing gains.')
    widget.poll()
    assert widget.status_label.text() == 'Rejected'
    assert 'Leave admittance mode' in widget.status_label.toolTip()
    assert widget.parameters['k'].value() == 0.
    assert len(node.writes) == 1
    assert len(node.calls) == 2
    widget.update_nodes({'/robot_control'})
    # First discovered appearance reads mode without writing pending edits.
    node.answer()
    widget.poll()
    assert not widget.apply_button.isEnabled()
    assert len(node.writes) == 1


@pytest.mark.parametrize('values', [
    (0., 0., 0.), (-1., 1., 1.), (math.nan, 1., 1.),
    (1., math.inf, 1.), (1., 1., module.DISPLAY_LIMIT * 2),
])
def test_invalid_server_gains_disable_apply(panel, values):
    widget, node = panel
    node.values.update(zip(module.GAIN_NAMES, values))
    read(widget, node)
    assert not widget.apply_button.isEnabled()
    assert not node.writes


def test_all_zero_edit_is_rejected_without_ros_write(panel):
    widget, node = panel
    read(widget, node)
    for spin in widget.parameters.values():
        spin.setValue(0.)
    widget.apply_settings()
    assert widget.status_label.text() == 'Invalid'
    assert 'not all zero' in widget.status_label.toolTip()
    assert not node.writes


@pytest.mark.parametrize('values', [(0., 1., 0.), (0., 0., 20.), (.05, 0., 0.)])
def test_degenerate_valid_model_parameters_are_not_blocked(panel, values):
    widget, node = panel
    read(widget, node)
    for key, value in zip(module.KEYS, values):
        widget.parameters[key].setValue(value)
    widget.apply_settings()
    assert len(node.writes) == 1


@pytest.mark.parametrize('bad_mode', [0, 5])
def test_unknown_mode_disables_apply(panel, bad_mode):
    widget, node = panel
    node.values['control_mode'] = bad_mode
    read(widget, node)
    assert not widget.apply_button.isEnabled()
    assert not node.writes


@pytest.mark.parametrize('case', ['partial', 'gain_integer', 'mode_double'])
def test_incomplete_or_wrong_typed_read_never_enables_apply(panel, case):
    widget, node = panel
    widget.read_settings()
    values = [ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.)
              for _ in range(3)] + [ParameterValue(
                  type=ParameterType.PARAMETER_INTEGER, integer_value=1)]
    if case == 'partial':
        values.pop()
    elif case == 'gain_integer':
        values[0].type = ParameterType.PARAMETER_INTEGER
    else:
        values[-1].type = ParameterType.PARAMETER_DOUBLE
    node.calls[-1].future.set_result(GetParameters.Response(values=values))
    widget.poll()
    assert not widget.apply_button.isEnabled()
    assert not node.writes


def test_unknown_write_timeout_prevents_overlapping_writes_until_ack_and_read(panel):
    widget, node = panel
    read(widget, node)
    widget.apply_settings()
    widget._deadline = 0.
    widget.poll()
    assert widget.status_label.text() == 'Unknown'
    assert widget.busy
    widget.apply_settings()
    widget.read_settings()
    assert len(node.calls) == 2
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Read again'
    assert not widget.apply_button.isEnabled()
    assert not widget.busy
    read(widget, node)
    assert widget.apply_button.isEnabled()
    assert len(node.writes) == 1


def test_offline_during_write_preserves_outstanding_operation(panel):
    widget, node = panel
    read(widget, node)
    widget.apply_settings()
    widget.update_nodes(set())
    assert widget.busy
    assert widget.status_label.text() == 'Unknown'
    widget.update_nodes({'/robot_control'})
    assert len(node.calls) == 2
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Read again'
    assert not widget.apply_button.isEnabled()
    assert len(node.writes) == 1


def test_read_timeout_and_missing_service_never_write(panel):
    widget, node = panel
    node.ready[GetParameters] = False
    widget.read_settings()
    widget._deadline = 0.
    widget.poll()
    assert widget.status_label.text() == 'Offline'
    assert not widget.busy
    assert not node.calls
    node.ready[GetParameters] = True
    widget.read_settings()
    widget._deadline = 0.
    widget.poll()
    assert widget.status_label.text() == 'Read again'
    assert not widget.busy
    assert not widget.apply_button.isEnabled()
    assert not node.writes


def test_verified_readback_reports_external_changes(panel):
    widget, node = panel
    read(widget, node)
    widget.parameters['m'].setValue(.1)
    widget.apply_settings()
    node.answer()
    widget.poll()
    node.values['admittance/M_d'] = .2
    node.answer()
    widget.poll()
    assert widget.status_label.text() == 'Changed'
    assert widget.parameters['m'].value() == .2
    assert len(node.writes) == 1


def test_shutdown_and_poll_never_apply_or_restart_requests(panel):
    widget, node = panel
    read(widget, node)
    widget.shutdown()
    widget.update_nodes({'/robot_control'})
    widget.apply_settings()
    widget.read_settings()
    widget.poll()
    assert len(node.calls) == 1
    assert not node.writes
    assert not widget.timer.isActive()
    assert not widget.apply_button.isEnabled()


def test_populated_fields_and_status_fit_compact_panel_with_parent_font(app):
    parent = QWidget()
    parent.setStyleSheet('font-size: 10pt;')
    node = FakeNode()
    widget = module.AdmittanceParametersPanel(node, parent)
    widget.timer.stop()
    widget.resize(320, 48)
    parent.show()
    widget.show()
    try:
        app.processEvents()
        read(widget, node)
        app.processEvents()
        assert widget.minimumSizeHint().width() <= 320
        assert widget.minimumSizeHint().height() <= 48
        for spin in widget.parameters.values():
            edit = spin.findChild(QLineEdit)
            assert spin.fontMetrics().horizontalAdvance(spin.text()) <= edit.width() - 4
        for short in ('Read first', 'Leave Admit', 'Read again', 'Verifying', 'Rejected'):
            widget._status(short, 'Test status', error=True)
            app.processEvents()
            assert (widget.status_label.fontMetrics().horizontalAdvance(short)
                    <= widget.status_label.width())
        node.values['control_mode'] = 4
        read(widget, node)
        assert widget.status_label.text() == 'Leave Admit'
        assert not widget.apply_button.isEnabled()
        assert not node.writes
    finally:
        widget.shutdown()
        widget.close()
        parent.close()
