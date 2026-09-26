"""Camera exposure controls use fake parameter services, never a live ROS graph."""

from concurrent.futures import Future
from dataclasses import dataclass
import os

from PyQt5.QtWidgets import QApplication
import pytest
from rcl_interfaces.msg import (
    IntegerRange, ParameterDescriptor, ParameterType, ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import (
    DescribeParameters, GetParameters, ListParameters, SetParameters,
)

from gui_py_pkg import camera_exposure


HRM = '/camera/camera'
TAG = '/tag_camera/tag_camera'


def parameter_value(value):
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)


@dataclass
class PendingCall:
    target: str
    service_type: object
    request: object
    future: Future


class FakeClient:
    def __init__(self, node, service_type, service_name):
        self.node = node
        self.service_type = service_type
        self.target = service_name.rsplit('/', 1)[0]

    def service_is_ready(self):
        return self.node.ready

    def wait_for_service(self, *args, **kwargs):
        raise AssertionError('The GUI must not block waiting for a parameter service.')

    def call_async(self, request):
        assert isinstance(request, self.service_type.Request)
        future = Future()
        self.node.calls.append(PendingCall(
            self.target, self.service_type, request, future))
        return future

    def remove_pending_request(self, future):
        pass


class FakeNode:
    """Deterministic service requests and ACKs without creating any ROS node."""

    def __init__(self):
        self.calls = []
        self.ready = True
        self.ignore_writes = False
        self.values = {
            TAG: {'rgb_camera.enable_auto_exposure': True,
                  'rgb_camera.exposure': 156},
            HRM: {'depth_module.enable_auto_exposure': True,
                  'depth_module.exposure': 10000,
                  'depth_module.color_profile': '640x480x30'},
        }
        self.ranges = {
            TAG: IntegerRange(from_value=1, to_value=10000, step=1),
            HRM: IntegerRange(from_value=100, to_value=200000, step=100),
        }
        self.read_only = set()

    def create_client(self, service_type, service_name, **kwargs):
        assert service_type in (
            ListParameters, DescribeParameters, GetParameters, SetParameters)
        assert service_name.rsplit('/', 1)[0] in (HRM, TAG)
        return FakeClient(self, service_type, service_name)

    def destroy_client(self, client):
        pass

    @property
    def writes(self):
        return [call for call in self.calls if call.service_type is SetParameters]

    def answer(self, call, successful=True, reason=''):
        assert not call.future.done()
        values = self.values[call.target]
        response = call.service_type.Response()
        if call.service_type is ListParameters:
            response.result.names = list(values)
        elif call.service_type is DescribeParameters:
            for name in call.request.names:
                descriptor = ParameterDescriptor(
                    name=name, read_only=name in self.read_only,
                    type=(ParameterType.PARAMETER_BOOL if isinstance(values[name], bool)
                          else ParameterType.PARAMETER_INTEGER))
                if name.endswith('.exposure'):
                    descriptor.integer_range = [self.ranges[call.target]]
                response.descriptors.append(descriptor)
        elif call.service_type is GetParameters:
            response.values = [parameter_value(values[name]) for name in call.request.names]
        else:
            response.results = [SetParametersResult(successful=successful, reason=reason)
                                for unused in call.request.parameters]
            if successful and not self.ignore_writes:
                for parameter in call.request.parameters:
                    value = parameter.value
                    values[parameter.name] = (
                        value.bool_value if value.type == ParameterType.PARAMETER_BOOL
                        else value.integer_value)
        call.future.set_result(response)

    def drain(self, panel, include_writes=True):
        """Respond until idle, optionally leaving the first Set ACK outstanding."""
        for unused in range(30):
            panel.poll()
            pending = [call for call in self.calls if not call.future.done()]
            if not pending:
                return
            assert len(pending) == 1, 'Only one parameter-service phase may be in flight.'
            if pending[0].service_type is SetParameters and not include_writes:
                return
            self.answer(pending[0])
        pytest.fail('Parameter operation did not settle after 30 service phases.')


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    node = FakeNode()
    widget = camera_exposure.CameraExposurePanel(node)
    widget.update_nodes({HRM, TAG})
    node.drain(widget)
    yield widget, node
    widget.close()


def read(panel, node, target=TAG):
    panel.target_combo.setCurrentIndex(panel.target_combo.findData(target))
    panel.read_settings()
    node.drain(panel)


def written(call):
    return {parameter.name: (
        parameter.value.bool_value if parameter.value.type == ParameterType.PARAMETER_BOOL
        else parameter.value.integer_value) for parameter in call.request.parameters}


def status_detail(widget):
    return (widget.status_label.text() + ' ' + widget.status_label.toolTip()).lower()


def test_startup_edit_and_read_never_write_camera_settings(panel):
    widget, node = panel
    assert widget.target_combo.currentData() == TAG
    assert {widget.target_combo.itemData(index)
            for index in range(widget.target_combo.count())} == {HRM, TAG}
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(5)
    node.drain(widget)
    read(widget, node)
    assert widget.exposure_spin.value() == pytest.approx(15.6)
    assert widget.auto_checkbox.isChecked()
    read(widget, node, HRM)
    assert widget.exposure_spin.value() == pytest.approx(10.0)
    assert not node.writes


@pytest.mark.parametrize('target, minimum, maximum, step, current', [
    (TAG, 0.1, 1000.0, 0.1, 15.6),
    (HRM, 0.1, 200.0, 0.1, 10.0),
])
def test_device_declared_range_and_step_are_converted_to_ms(
        panel, target, minimum, maximum, step, current):
    widget, node = panel
    read(widget, node, target)
    assert widget.exposure_spin.minimum() == pytest.approx(minimum)
    assert widget.exposure_spin.maximum() == pytest.approx(maximum)
    assert widget.exposure_spin.singleStep() == pytest.approx(step)
    assert widget.exposure_spin.value() == pytest.approx(current)
    assert not node.writes


def test_d405_microsecond_step_retains_three_decimal_places_in_ms(panel):
    widget, node = panel
    node.ranges[HRM].step = 1
    node.values[HRM]['depth_module.exposure'] = 12345
    read(widget, node, HRM)
    assert widget.exposure_spin.singleStep() == pytest.approx(0.001)
    assert widget.exposure_spin.value() == pytest.approx(12.345)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(12.346)
    widget.apply_settings()
    node.drain(widget)
    assert written(node.writes[-1]) == {'depth_module.exposure': 12346}


@pytest.mark.parametrize('target, prefix, raw_value', [
    (TAG, 'rgb_camera', 200),
    (HRM, 'depth_module', 20000),
])
def test_manual_apply_waits_for_auto_off_ack_then_writes_only_selected_target(
        panel, target, prefix, raw_value):
    widget, node = panel
    read(widget, node, target)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    assert len(node.writes) == 1
    assert written(node.writes[0]) == {f'{prefix}.enable_auto_exposure': False}
    assert node.writes[0].target == target
    for unused in range(3):
        widget.poll()
    assert len(node.writes) == 1

    node.answer(node.writes[0])
    node.drain(widget, include_writes=False)
    assert len(node.writes) == 2
    assert written(node.writes[1]) == {f'{prefix}.exposure': raw_value}
    assert node.writes[1].target == target
    assert node.writes[1].request.parameters[0].value.type == ParameterType.PARAMETER_INTEGER
    calls_before_ack = len(node.calls)
    node.answer(node.writes[1])
    node.drain(widget)
    assert any(call.service_type is GetParameters for call in node.calls[calls_before_ack:])
    assert widget.exposure_spin.value() == pytest.approx(20.0)
    assert not widget.auto_checkbox.isChecked()
    assert 'applied' in widget.status_label.text().lower()


def test_auto_on_apply_never_sends_an_exposure_value(panel):
    widget, node = panel
    node.values[TAG]['rgb_camera.enable_auto_exposure'] = False
    read(widget, node)
    widget.exposure_spin.setValue(50.0)
    widget.auto_checkbox.setChecked(True)
    widget.apply_settings()
    node.drain(widget)
    assert len(node.writes) == 1
    assert written(node.writes[0]) == {'rgb_camera.enable_auto_exposure': True}
    assert node.values[TAG]['rgb_camera.exposure'] == 156
    assert 'applied' in widget.status_label.text().lower()


def test_rejected_auto_off_ack_prevents_manual_exposure_write(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    node.answer(node.writes[0], successful=False, reason='Camera is busy')
    node.drain(widget)
    assert len(node.writes) == 1
    assert node.values[TAG]['rgb_camera.enable_auto_exposure'] is True
    assert 'busy' in status_detail(widget)
    assert 'applied' not in widget.status_label.text().lower()


def test_exposure_rejection_reports_partial_change_without_claiming_success(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    node.answer(node.writes[0])
    node.drain(widget, include_writes=False)
    node.answer(node.writes[1], successful=False, reason='Exposure rejected by device')
    node.drain(widget)
    assert len(node.writes) == 2
    assert node.values[TAG]['rgb_camera.enable_auto_exposure'] is False
    assert node.values[TAG]['rgb_camera.exposure'] == 156
    status = status_detail(widget)
    assert 'rejected' in status
    assert 'auto' in status or 'partial' in status
    assert 'applied' not in widget.status_label.text().lower()


def test_readback_mismatch_does_not_report_applied(panel):
    widget, node = panel
    read(widget, node)
    node.ignore_writes = True
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget)
    assert node.writes
    status = status_detail(widget)
    assert any(word in status for word in ('mismatch', 'verify', 'verification', 'readback'))
    assert 'applied' not in widget.status_label.text().lower()


def test_rgb_parameters_take_precedence_over_depth_parameters(panel):
    widget, node = panel
    node.values[TAG].update({
        'depth_module.enable_auto_exposure': False,
        'depth_module.exposure': 10000,
        'depth_module.color_profile': '640x480x30',
    })
    read(widget, node)
    assert widget.exposure_spin.value() == pytest.approx(15.6)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget)
    assert all(name.startswith('rgb_camera.') for call in node.writes for name in written(call))
    assert node.values[TAG]['depth_module.exposure'] == 10000


def test_depth_exposure_without_color_profile_is_not_used_for_rgb(panel):
    widget, node = panel
    del node.values[HRM]['depth_module.color_profile']
    read(widget, node, HRM)
    assert not widget.apply_button.isEnabled()
    widget.apply_settings()
    node.drain(widget)
    assert not node.writes


def test_read_only_exposure_parameter_cannot_be_applied(panel):
    widget, node = panel
    node.read_only.add('rgb_camera.exposure')
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.apply_settings()
    node.drain(widget)
    assert not node.writes


def test_parameter_service_unavailable_times_out_without_blocking_or_writes(panel, monkeypatch):
    widget, node = panel
    now = [100.0]
    monkeypatch.setattr(camera_exposure.time, 'monotonic', lambda: now[0])
    node.ready = False
    previous_calls = len(node.calls)
    widget.read_settings()
    for unused in range(3):
        widget.poll()
    assert len(node.calls) == previous_calls
    now[0] += 3.1
    widget.poll()
    assert not node.writes
    assert not widget.apply_button.isEnabled()
    assert 'time' in status_detail(widget)


def test_parameter_service_can_become_ready_during_async_discovery(panel, monkeypatch):
    widget, node = panel
    now = [100.0]
    monkeypatch.setattr(camera_exposure.time, 'monotonic', lambda: now[0])
    node.ready = False
    previous_calls = len(node.calls)
    widget.read_settings()
    widget.poll()
    assert len(node.calls) == previous_calls
    now[0] += 1.0
    node.ready = True
    node.drain(widget)
    assert len(node.calls) > previous_calls
    assert widget.apply_button.isEnabled()
    assert widget.exposure_spin.value() == pytest.approx(15.6)
    assert not node.writes


def test_target_change_cancels_followup_write_after_pending_auto_off(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    first = node.writes[0]
    assert not widget.target_combo.isEnabled()
    widget.target_combo.setCurrentIndex(widget.target_combo.findData(HRM))
    if not first.future.done():
        node.answer(first)
    node.drain(widget)
    assert len(node.writes) == 1
    assert first.target == TAG
    assert node.values[HRM]['depth_module.enable_auto_exposure'] is True
    assert node.values[HRM]['depth_module.exposure'] == 10000


def test_auto_off_timeout_cannot_trigger_a_late_exposure_write(panel, monkeypatch):
    widget, node = panel
    now = [100.0]
    monkeypatch.setattr(camera_exposure.time, 'monotonic', lambda: now[0])
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    pending = node.writes[0]
    # Running futures cannot be cancelled: model a service ACK that arrives late.
    assert pending.future.set_running_or_notify_cancel()
    now[0] += 3.1
    widget.poll()
    assert 'time' in status_detail(widget)
    assert 'applied' not in widget.status_label.text().lower()
    assert not pending.future.done()
    node.answer(pending)
    node.drain(widget)
    assert len(node.writes) == 1


def test_failed_future_stops_followup_writes_and_is_visible(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    widget.apply_settings()
    node.drain(widget, include_writes=False)
    node.writes[0].future.set_exception(RuntimeError('Connection lost'))
    node.drain(widget)
    assert len(node.writes) == 1
    assert 'lost' in status_detail(widget)
    assert 'applied' not in widget.status_label.text().lower()


def test_apply_rechecks_changed_device_range_before_any_write(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    node.ranges[TAG].to_value = 100
    widget.apply_settings()
    node.drain(widget)
    assert not node.writes
    assert 'range' in status_detail(widget)


def test_apply_rechecks_changed_sensor_module_before_any_write(panel):
    widget, node = panel
    read(widget, node)
    widget.auto_checkbox.setChecked(False)
    widget.exposure_spin.setValue(20.0)
    node.values[TAG] = {
        'depth_module.enable_auto_exposure': True,
        'depth_module.exposure': 10000,
        'depth_module.color_profile': '640x480x30',
    }
    widget.apply_settings()
    node.drain(widget)
    assert not node.writes
    assert 'changed' in status_detail(widget)
