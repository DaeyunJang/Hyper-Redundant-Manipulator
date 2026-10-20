"""GUI ROI changes apply only to the next stopped-estimator launch."""

from concurrent.futures import Future
import os
from pathlib import Path
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication, QCheckBox, QDialog, QLabel
import pytest

from gui_py_pkg.gui_node import MyGUI
from gui_py_pkg.system_manager import load_component_specs


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def ui(app, tmp_path, monkeypatch):
    node = SimpleNamespace(
        context=object(), record_status=None,
        discovered_node_names=Mock(return_value=set()),
        get_logger=Mock(return_value=Mock()), send_request_record_start=Mock())
    managed = SimpleNamespace(status=Mock(return_value='stopped'))
    manager = SimpleNamespace(processes={'estimation': managed}, start=Mock(return_value=True))
    interface = SimpleNamespace(
        node=node, system_manager=manager,
        estimation_roi={'x': 11, 'y': 22, 'w': 130, 'h': 140},
        roi_config_path=tmp_path / 'config_ROI_ref.json', roi_editor_active=False,
        roi_dialog=None,
        roi_label=QLabel(), record_label=QLabel(),
        record_future=None, is_recording=False,
        motor_output_checkbox=QCheckBox(),
        serial_port_selector=SimpleNamespace(selected_port=Mock(return_value='/dev/ttyUSB0')),
        camera_selector=SimpleNamespace(substitutions=Mock(return_value={
            'camera_device_type': 'D405$', 'camera_serial_no': "''"})),
        tag_camera_selector=SimpleNamespace(substitutions=Mock(return_value={
            'tag_camera_device_type': 'D435i$', 'tag_camera_serial_no': "''"})),
        camera_selectors={},
        auto_start_enabled={'estimation': True},
    )
    for name in ('process_substitutions', 'update_roi_label', 'roi_recording_block_reason',
                 'roi_edit_block_reason', 'start_component'):
        setattr(interface, name, MethodType(getattr(MyGUI, name), interface))
    interface.information = Mock()
    interface.warning = Mock()
    monkeypatch.setattr('gui_py_pkg.gui_node.QMessageBox.information', interface.information)
    monkeypatch.setattr('gui_py_pkg.gui_node.QMessageBox.warning', interface.warning)
    return interface


@pytest.fixture
def dialog(monkeypatch):
    instance = SimpleNamespace(
        selected_roi={'x': 30, 'y': 40, 'w': 250, 'h': 260},
        exec_=Mock(return_value=QDialog.Accepted), deleteLater=Mock())
    factory = Mock(return_value=instance)
    monkeypatch.setattr('gui_py_pkg.gui_node.RoiEditorDialog', factory)
    return factory, instance


@pytest.mark.parametrize('status', [
    'running', 'starting', 'stopping', 'graph_pending', 'unhealthy', 'external'])
def test_editor_requires_estimator_fully_stopped(ui, dialog, status):
    ui.system_manager.processes['estimation'].status.return_value = status
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    ui.information.assert_called_once()
    assert 'wait for STOPPED' in ui.information.call_args.args[-1]
    assert not ui.roi_editor_active


@pytest.mark.parametrize('status', ['stopped', 'failed'])
def test_successful_editor_sets_next_start_roi_without_launching(ui, dialog, status):
    ui.system_manager.processes['estimation'].status.return_value = status
    before = dict(ui.estimation_roi)

    def execute():
        assert ui.roi_editor_active
        assert ui.estimation_roi == before
        assert not dialog[0].call_args.kwargs['apply_guard']()
        return QDialog.Accepted

    dialog[1].exec_.side_effect = execute
    MyGUI.edit_estimation_roi(ui)
    assert ui.estimation_roi == dialog[1].selected_roi
    assert ui.estimation_roi is not dialog[1].selected_roi
    assert not ui.roi_editor_active
    assert ui.roi_dialog is None
    dialog[1].deleteLater.assert_called_once()
    ui.system_manager.start.assert_not_called()
    assert 'Next estimation ROI' in ui.roi_label.text()
    assert 'x=30' in ui.roi_label.text() and 'w=250' in ui.roi_label.text()
    assert 'AprilTag continues to use the full camera image' in ui.roi_label.toolTip()
    assert dialog[0].call_args.args[0] is ui.node.context
    assert dialog[0].call_args.args[1] == before
    assert dialog[0].call_args.args[2] == ui.roi_config_path


def test_cancel_preserves_roi_and_does_not_launch(ui, dialog):
    before = dict(ui.estimation_roi)
    dialog[1].exec_.return_value = QDialog.Rejected
    MyGUI.edit_estimation_roi(ui)
    assert ui.estimation_roi == before
    assert not ui.roi_editor_active
    ui.system_manager.start.assert_not_called()


@pytest.mark.parametrize('change', ['external_estimator', 'recording_started', 'motor_enabled'])
def test_apply_rechecks_guards_after_nested_dialog_loop(ui, dialog, change):
    before = dict(ui.estimation_roi)

    def execute():
        if change == 'external_estimator':
            ui.system_manager.processes['estimation'].status.return_value = 'external'
        elif change == 'recording_started':
            ui.node.record_status = ({'state': 'recording'}, time.monotonic())
        else:
            ui.motor_output_checkbox.setChecked(True)
        assert dialog[0].call_args.kwargs['apply_guard']()
        return QDialog.Accepted

    dialog[1].exec_.side_effect = execute
    MyGUI.edit_estimation_roi(ui)
    assert ui.estimation_roi == before
    assert not ui.roi_editor_active
    ui.warning.assert_called_once()
    ui.system_manager.start.assert_not_called()


def test_dialog_failure_releases_editor_guard_without_changing_roi(ui, dialog):
    before = dict(ui.estimation_roi)
    dialog[1].exec_.side_effect = RuntimeError('camera snapshot failed')
    MyGUI.edit_estimation_roi(ui)
    assert not ui.roi_editor_active
    assert ui.estimation_roi == before
    ui.warning.assert_called_once()
    assert 'camera snapshot failed' in ui.warning.call_args.args[-1]


def test_missing_installed_configuration_blocks_editor(ui, dialog):
    ui.roi_config_path = None
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    ui.warning.assert_called_once()
    assert 'not installed' in ui.warning.call_args.args[-1]


@pytest.mark.parametrize('state', ['starting', 'recording', 'stopping', 'exporting'])
def test_record_lifecycle_blocks_editing_and_estimator_restart(ui, dialog, state):
    ui.node.record_status = ({'state': state}, time.monotonic())
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    assert 'recording and CSV export' in ui.information.call_args.args[-1]
    MyGUI.start_component(ui, 'estimation')
    ui.system_manager.start.assert_not_called()
    assert 'recording and CSV export' in ui.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('flag', ['pending_request', 'recording_flag'])
def test_local_capture_flags_block_before_status_arrives(ui, dialog, flag):
    if flag == 'pending_request':
        ui.record_future = Future()
    else:
        ui.is_recording = True
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    assert ui.roi_recording_block_reason()


def test_enabled_motor_output_blocks_editor_without_commanding_anything(ui, dialog):
    ui.motor_output_checkbox.setChecked(True)
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    ui.system_manager.start.assert_not_called()
    assert 'disable physical motor output' in ui.information.call_args.args[-1]
    assert ui.motor_output_checkbox.isChecked()


def test_scheduled_estimator_start_is_skipped_while_editor_open(ui):
    ui.roi_editor_active = True
    MyGUI.start_component_if_selected(ui, 'estimation')
    ui.system_manager.start.assert_not_called()
    assert 'ROI editor is open' in ui.node.get_logger().error.call_args.args[0]

    ui.roi_editor_active = False
    ui.estimation_roi = {'x': 8, 'y': 9, 'w': 100, 'h': 120}
    MyGUI.start_component_if_selected(ui, 'estimation')
    ui.system_manager.start.assert_called_once()
    assert ui.system_manager.start.call_args.args[2]['roi_x'] == 8
    assert ui.system_manager.start.call_args.args[2]['roi_height'] == 120


def test_deselected_auto_start_stays_stopped(ui):
    ui.auto_start_enabled['estimation'] = False
    MyGUI.start_component_if_selected(ui, 'estimation')
    ui.system_manager.start.assert_not_called()


def test_missing_roi_prevents_estimator_start_but_not_apriltag(ui):
    ui.estimation_roi = None
    MyGUI.start_component(ui, 'estimation')
    ui.system_manager.start.assert_not_called()
    MyGUI.start_component(ui, 'apriltag')
    ui.system_manager.start.assert_called_once()
    assert ui.system_manager.start.call_args.args[0] == 'apriltag'


def test_record_start_is_blocked_while_editor_open(ui):
    ui.roi_editor_active = True
    MyGUI.start_recording(ui)
    ui.node.send_request_record_start.assert_not_called()
    assert 'Close the ROI editor' in ui.record_label.text()


def test_launch_substitutions_are_estimator_only_not_apriltag_crop(ui):
    specs = load_component_specs(
        Path(__file__).parents[1] / 'config' / 'system_components.json')
    specs = {spec.component_id: spec for spec in specs}
    substitutions = ui.process_substitutions()
    assert substitutions['roi_x'] == 11 and substitutions['roi_y'] == 22
    assert substitutions['roi_width'] == 130 and substitutions['roi_height'] == 140
    command = specs['estimation'].command(substitutions)
    assert command[:4] == ['ros2', 'launch', 'estimation_pkg', '_launch.py']
    assert set(command[4:]) == {'roi_x:=11', 'roi_y:=22', 'roi_width:=130', 'roi_height:=140'}
    assert specs['apriltag'].command(substitutions) == [
        'ros2', 'launch', 'launcher', 'apriltag.launch.py']
    camera_command = specs['camera'].command(substitutions)
    assert camera_command[:4] == ['ros2', 'launch', 'launcher', 'cuda_realsense.launch.py']
    assert set(camera_command[4:]) == {'device_type:=D405$', "serial_no:=''"}
    assert not any(argument.startswith('roi_') for argument in camera_command)


def test_absent_estimation_component_has_clear_edit_block(ui, dialog):
    ui.system_manager.processes = {}
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    assert 'not configured' in ui.information.call_args.args[-1]


def test_direct_run_estimator_alias_blocks_roi_edit(ui, dialog):
    ui.node.discovered_node_names.return_value = {'/segment_estimation_node'}
    assert ui.system_manager.processes['estimation'].status.return_value == 'stopped'
    MyGUI.edit_estimation_roi(ui)
    dialog[0].assert_not_called()
    ui.information.assert_called_once()
    assert 'externally started segment_estimation_node' in ui.information.call_args.args[-1]


def test_direct_run_estimator_alias_blocks_duplicate_gui_start(ui):
    ui.node.discovered_node_names.return_value = {'/segment_estimation_node'}
    MyGUI.start_component(ui, 'estimation')
    ui.system_manager.start.assert_not_called()
    error = ui.node.get_logger().error.call_args.args[0]
    assert 'externally started segment_estimation_node' in error


def test_no_roi_label_explains_next_start_requirement(ui):
    ui.estimation_roi = None
    MyGUI.update_roi_label(ui)
    assert 'choose an area before Start' in ui.roi_label.text()
