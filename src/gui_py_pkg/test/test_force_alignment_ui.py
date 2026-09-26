"""GUI force-axis selection is CSV-only and Record waits for atomic ACK."""

from concurrent.futures import Future
import os
import time
from types import SimpleNamespace
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
import pytest
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import SetBool

from gui_py_pkg.force_alignment import ForceAlignmentPanel, request_record_start
from gui_py_pkg.gui_node import MyGUI
from record_pkg.force_alignment import create_alignment


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    widget = ForceAlignmentPanel()
    yield widget
    widget.close()


def select_axes(panel, axes):
    for combo, axis in zip(panel.axis_combos, axes):
        index = combo.findData(axis)
        assert index >= 0
        combo.setCurrentIndex(index)


@pytest.mark.parametrize(('axes', 'matrix', 'equations'), [
    (['+x', '-y', '-z'], [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
     ('aligned_fx = +sensor_fx', 'aligned_fy = -sensor_fy', 'aligned_fz = -sensor_fz')),
    (['+x', '-z', '+y'], [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
     ('aligned_fx = +sensor_fx', 'aligned_fy = -sensor_fz', 'aligned_fz = +sensor_fy')),
])
def test_mapping_means_output_base_component_from_sensor(panel, axes, matrix, equations):
    panel.enabled_checkbox.setChecked(True)
    select_axes(panel, axes)
    config = panel.settings()
    assert config['enabled'] is True
    assert config['axes'] == axes
    assert config['matrix_base_from_sensor'] == matrix
    assert config['source_frame'] == 'fts_sensor' and config['target_frame'] == 'hrm_base'
    assert config['units'] == 'unchanged'
    assert all(equation in panel.summary.text() for equation in equations)
    assert 'Original columns/units unchanged' in panel.summary.text()
    assert 'Invalid' not in panel.summary.text()


@pytest.mark.parametrize('axes', [['+x', '+x', '+z'], ['+x', '+y', '-z']])
def test_duplicate_axis_and_reflection_are_visible_errors(panel, axes):
    panel.enabled_checkbox.setChecked(True)
    select_axes(panel, axes)
    with pytest.raises(ValueError):
        panel.settings()
    assert 'Invalid axis mapping' in panel.summary.text()
    assert '#d93025' in panel.summary.styleSheet()


def test_disabled_option_is_raw_only_even_during_invalid_edit(panel):
    panel.enabled_checkbox.setChecked(True)
    select_axes(panel, ['+x', '+x', '-z'])
    panel.enabled_checkbox.setChecked(False)
    config = panel.settings()
    assert config['enabled'] is False
    assert config['axes'] == ['+x', '+y', '+z']
    assert not any(combo.isEnabled() for combo in panel.axis_combos)
    assert 'OFF' in panel.summary.text() and 'original force columns only' in panel.summary.text()
    assert 'Invalid' not in panel.summary.text()


def test_session_display_reflects_recorders_frozen_mapping(panel):
    actual = create_alignment(True, ['+x', '-z', '+y'])
    panel.show_session(actual)
    assert panel.enabled_checkbox.isChecked()
    assert [combo.currentData() for combo in panel.axis_combos] == actual['axes']
    assert panel.settings() == actual
    panel.show_session(create_alignment(False))
    assert panel.settings()['enabled'] is False
    assert 'OFF' in panel.summary.text()


@pytest.mark.parametrize('bad', [None, {}, {'enabled': True, 'axes': ['+x', '+x', '+z']}])
def test_invalid_session_display_does_not_replace_last_valid_mapping(panel, bad):
    actual = create_alignment(True, ['+x', '-y', '-z'])
    panel.show_session(actual)
    panel.show_session(bad)
    assert panel.settings() == actual


class FakeClient:
    def __init__(self, ready=True, call_error=None):
        self.ready = ready
        self.call_error = call_error
        self.requests = []
        self.future = Future()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        if self.call_error is not None:
            raise self.call_error
        return self.future


def settings_ack(client, success=True, reason=''):
    client.future.set_result(SetParametersAtomically.Response(
        result=SetParametersResult(successful=success, reason=reason)))


@pytest.mark.parametrize('enabled', [False, True])
def test_exact_atomic_parameters_and_record_only_after_successful_ack(enabled):
    settings, recorder = FakeClient(), FakeClient()
    alignment = create_alignment(enabled, ['+x', '-z', '+y'])
    result = request_record_start(settings, recorder, alignment)
    assert isinstance(result, Future) and not result.done()
    assert len(settings.requests) == 1 and not recorder.requests
    request = settings.requests[0]
    assert isinstance(request, SetParametersAtomically.Request)
    assert [parameter.name for parameter in request.parameters] == [
        'force_alignment_enabled', 'force_alignment_axes', 'contact_segment_id']
    enabled_parameter, axes_parameter, contact_parameter = request.parameters
    assert contact_parameter.value.type == ParameterType.PARAMETER_INTEGER
    assert contact_parameter.value.integer_value == 0
    assert enabled_parameter.value.type == ParameterType.PARAMETER_BOOL
    assert enabled_parameter.value.bool_value is enabled
    assert axes_parameter.value.type == ParameterType.PARAMETER_STRING_ARRAY
    assert axes_parameter.value.string_array_value == ['+x', '-z', '+y']

    settings_ack(settings)
    assert len(recorder.requests) == 1
    assert isinstance(recorder.requests[0], SetBool.Request)
    assert recorder.requests[0].data is True
    assert not result.done()
    response = SetBool.Response(success=True, message='Start accepted')
    recorder.future.set_result(response)
    assert result.result() is response


@pytest.mark.parametrize(('settings_ready', 'record_ready'), [(False, True), (True, False)])
def test_missing_either_service_never_applies_settings_or_starts(settings_ready, record_ready):
    settings, recorder = FakeClient(settings_ready), FakeClient(record_ready)
    assert request_record_start(settings, recorder, create_alignment()) is None
    assert not settings.requests and not recorder.requests


def test_contact_id_validation_and_atomic_transfer(panel):
    from PyQt5.QtGui import QValidator
    spin = panel.contact_segment_id
    assert (spin.minimum(), spin.maximum(), spin.value()) == (0, 18, 0)
    for text in ('-1', '19', '1.5', 'abc'):
        assert spin.validate(text, len(text))[0] != QValidator.Acceptable
    for value in (0, 9, 18):
        spin.setValue(value)
        settings, recorder = FakeClient(), FakeClient()
        request_record_start(settings, recorder, panel.settings(), spin.value())
        assert settings.requests[0].parameters[2].value.integer_value == value
        assert not recorder.requests
    for bad in (-1, 19, 1.5, '9', True):
        settings, recorder = FakeClient(), FakeClient()
        with pytest.raises(ValueError):
            request_record_start(settings, recorder, panel.settings(), bad)
        assert not settings.requests and not recorder.requests


def test_rejected_atomic_settings_never_start_recording():
    settings, recorder = FakeClient(), FakeClient()
    result = request_record_start(settings, recorder, create_alignment(True))
    settings_ack(settings, success=False, reason='Session is locked')
    assert not result.result().success
    assert 'Session is locked' in result.result().message
    assert not recorder.requests


@pytest.mark.parametrize('stage', ['call', 'future'])
def test_settings_exceptions_never_start_recording(stage):
    error = RuntimeError('settings disconnected') if stage == 'call' else None
    settings = FakeClient(call_error=error)
    recorder = FakeClient()
    result = request_record_start(settings, recorder, create_alignment(True))
    if stage == 'future':
        settings.future.set_exception(RuntimeError('settings disconnected'))
    assert not result.result().success
    assert 'settings disconnected' in result.result().message
    assert not recorder.requests


def test_malformed_settings_response_never_starts_recording():
    settings, recorder = FakeClient(), FakeClient()
    result = request_record_start(settings, recorder, create_alignment())
    settings.future.set_result(None)
    assert not result.result().success
    assert not recorder.requests


def test_record_service_disappearing_after_ack_does_not_start():
    settings, recorder = FakeClient(), FakeClient()
    result = request_record_start(settings, recorder, create_alignment())
    recorder.ready = False
    settings_ack(settings)
    assert not result.result().success and 'disappeared' in result.result().message
    assert not recorder.requests


@pytest.mark.parametrize('stage', ['call', 'future'])
def test_record_service_exception_is_reported_without_false_success(stage):
    settings = FakeClient()
    error = RuntimeError('record disconnected') if stage == 'call' else None
    recorder = FakeClient(call_error=error)
    result = request_record_start(settings, recorder, create_alignment())
    settings_ack(settings)
    if stage == 'future':
        recorder.future.set_exception(RuntimeError('record disconnected'))
    assert not result.result().success
    assert 'record disconnected' in result.result().message
    assert len(recorder.requests) == 1


def test_record_service_negative_reply_is_preserved():
    settings, recorder = FakeClient(), FakeClient()
    result = request_record_start(settings, recorder, create_alignment())
    settings_ack(settings)
    response = SetBool.Response(success=False, message='Required input missing')
    recorder.future.set_result(response)
    assert result.result() is response


def test_abandoned_request_does_not_start_on_a_late_settings_ack():
    settings, recorder = FakeClient(), FakeClient()
    result = request_record_start(settings, recorder, create_alignment())
    assert result.cancel()
    settings_ack(settings)
    assert not recorder.requests


def test_request_uses_a_copy_of_mapping_not_later_caller_edits():
    settings, recorder = FakeClient(), FakeClient()
    alignment = create_alignment(True, ['+x', '-z', '+y'])
    request_record_start(settings, recorder, alignment)
    alignment['axes'][:] = ['+x', '+y', '+z']
    axes = settings.requests[0].parameters[1].value.string_array_value
    assert axes == ['+x', '-z', '+y']


@pytest.fixture
def ui(panel):
    return SimpleNamespace(
        node=SimpleNamespace(send_request_record_start=Mock(), record_status=None),
        force_alignment_panel=panel, record_button=QPushButton('Record'),
        record_label=QLabel(), record_future=None, record_error=None,
        record_request_time=0.0, is_recording=False,
    )


def publish_status(ui, state, **extra):
    ui.node.record_status = (
        dict(state=state, detail='test', directory='/tmp/test', **extra), time.monotonic())


def test_invalid_gui_mapping_blocks_request_before_any_service_call(ui):
    ui.force_alignment_panel.enabled_checkbox.setChecked(True)
    select_axes(ui.force_alignment_panel, ['+x', '+x', '+z'])
    MyGUI.start_recording(ui)
    ui.node.send_request_record_start.assert_not_called()
    assert 'Invalid force-axis mapping' in ui.record_label.text()
    assert ui.force_alignment_panel.isEnabled()


def test_gui_locks_mapping_while_ack_pending_and_until_fresh_status(ui):
    ui.node.send_request_record_start.return_value = Future()
    ui.force_alignment_panel.show_session(create_alignment(True, ['+x', '-z', '+y']))
    ui.force_alignment_panel.contact_segment_id.setValue(9)
    MyGUI.start_recording(ui)
    sent = ui.node.send_request_record_start.call_args.args[0]
    assert sent['axes'] == ['+x', '-z', '+y']
    assert ui.node.send_request_record_start.call_args.args[1] == 9
    assert not ui.force_alignment_panel.contact_segment_id.isEnabled()
    assert not ui.force_alignment_panel.isEnabled()
    assert not ui.record_button.isEnabled()
    ui.record_future.set_result(SetBool.Response(success=True, message='Accepted'))
    MyGUI.update_record_status(ui)
    assert not ui.force_alignment_panel.isEnabled()
    assert not ui.record_button.isEnabled()


def test_inflight_idle_heartbeat_does_not_unlock_after_successful_start_ack(ui):
    ui.node.send_request_record_start.return_value = Future()
    MyGUI.start_recording(ui)
    # A heartbeat produced while parameter/Record requests were still in flight
    # is newer than request_time but cannot prove that accepted capture is idle.
    publish_status(ui, 'idle', force_alignment_scope='next_session',
                   force_alignment=create_alignment(False))
    ui.record_future.set_result(SetBool.Response(success=True, message='Accepted'))
    MyGUI.update_record_status(ui)
    assert not ui.force_alignment_panel.isEnabled()
    assert not ui.record_button.isEnabled()
    publish_status(ui, 'starting', force_alignment_scope='session',
                   force_alignment=create_alignment(True))
    MyGUI.update_record_status(ui)
    assert not ui.force_alignment_panel.isEnabled()
    assert ui.is_recording


@pytest.mark.parametrize('state', ['starting', 'recording', 'stopping', 'exporting'])
def test_busy_status_locks_panel_and_displays_actual_session_settings(ui, state):
    actual = create_alignment(True, ['+x', '-y', '-z'])
    publish_status(ui, state, force_alignment_scope='session', force_alignment=actual,
                   contact_segment_id=9)
    MyGUI.update_record_status(ui)
    assert not ui.force_alignment_panel.isEnabled()
    assert ui.force_alignment_panel.settings() == actual
    assert ui.force_alignment_panel.contact_segment_id.value() == 9
    assert not ui.force_alignment_panel.contact_segment_id.isEnabled()


def test_idle_next_session_status_does_not_overwrite_user_edits(ui):
    edited = create_alignment(True, ['+x', '-z', '+y'])
    ui.force_alignment_panel.show_session(edited)
    publish_status(ui, 'stopped', force_alignment_scope='next_session',
                   force_alignment=create_alignment(False))
    MyGUI.update_record_status(ui)
    assert ui.force_alignment_panel.isEnabled()
    assert ui.force_alignment_panel.settings() == edited


def test_failed_start_unlocks_panel_without_waiting_for_status(ui):
    future = Future()
    ui.node.send_request_record_start.return_value = future
    MyGUI.start_recording(ui)
    future.set_result(SetBool.Response(success=False, message='Settings rejected'))
    MyGUI.update_record_status(ui)
    assert ui.force_alignment_panel.isEnabled() and ui.record_button.isEnabled()
    assert 'Settings rejected' in ui.record_label.text()


def test_stale_status_keeps_settings_locked(ui):
    publish_status(ui, 'recording')
    ui.node.record_status = (ui.node.record_status[0], time.monotonic() - 4.0)
    MyGUI.update_record_status(ui)
    assert not ui.force_alignment_panel.isEnabled()
    assert not ui.record_button.isEnabled()
    assert 'stale' in ui.record_label.text()
