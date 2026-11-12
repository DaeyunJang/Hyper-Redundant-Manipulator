# AUTHOR: Daeyun -- Normal GUI component and motor opt-in regression checks.
"""Exercise the operator workflow with fake nodes; never initialize ROS."""

import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg import gui_node, image_preview


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app, monkeypatch):
    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    monkeypatch.setattr(gui_node.MyGUI, 'init_fts_plot', lambda self: None)
    source_config = Path(__file__).parents[1] / 'config' / 'system_components.json'
    monkeypatch.setenv('HRM_SYSTEM_COMPONENTS_CONFIG', str(source_config))
    popen = Mock(side_effect=AssertionError('No subprocess may launch in this test.'))
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False)
    node.discovered_node_names.return_value = set()
    widget = gui_node.MyGUI(node)
    yield widget
    widget.close()
    popen.assert_not_called()


def test_gui_keeps_motor_bridge_and_output_disabled_by_default(window):
    assert window.windowTitle() == 'Hyper-Redundant Manipulator Control'
    assert 'tcp' in window.component_widgets
    assert window.component_widgets['tcp']['button'].text() == 'Start'
    assert window.component_widgets['tcp']['button'].isEnabled()
    assert window.motor_output_checkbox.isEnabled()
    assert not window.motor_output_checkbox.isChecked()
    assert not window.process_substitutions()['motor_output_enabled']
    assert window.motor_output_note.text() == 'OFF — IK preview only; motor_command is blocked'
    window.node.set_motor_output_enabled.assert_not_called()


def test_rejected_motor_confirmation_keeps_output_disabled(window, monkeypatch):
    warning = Mock(return_value=gui_node.QMessageBox.No)
    monkeypatch.setattr(gui_node.QMessageBox, 'warning', warning)
    window.motor_output_checkbox.setChecked(True)
    warning.assert_called_once()
    assert warning.call_args.args[-1] == gui_node.QMessageBox.No
    assert not window.motor_output_checkbox.isChecked()
    assert not window.process_substitutions()['motor_output_enabled']
    window.node.set_motor_output_enabled.assert_not_called()


@pytest.mark.parametrize('controller_ready', [False, True])
def test_motor_enable_requires_confirmation_and_sets_selected_gate(
        window, monkeypatch, controller_ready):
    warning = Mock(return_value=gui_node.QMessageBox.Yes)
    monkeypatch.setattr(gui_node.QMessageBox, 'warning', warning)
    future = Mock() if controller_ready else None
    window.node.set_motor_output_enabled.return_value = future
    window.motor_output_checkbox.setChecked(True)
    warning.assert_called_once()
    assert window.motor_output_checkbox.isChecked()
    assert window.process_substitutions()['motor_output_enabled']
    window.node.set_motor_output_enabled.assert_called_once_with(True)
    if controller_ready:
        future.add_done_callback.assert_called_once()
        assert window.motor_gate_initialized_for_node
    else:
        assert 'applies when Robot control is next started' in window.motor_output_note.text()


def test_motor_disable_does_not_require_confirmation(window, monkeypatch):
    window.motor_output_checkbox.blockSignals(True)
    window.motor_output_checkbox.setChecked(True)
    window.motor_output_checkbox.blockSignals(False)
    warning = Mock(side_effect=AssertionError('Disabling must not ask for confirmation.'))
    monkeypatch.setattr(gui_node.QMessageBox, 'warning', warning)
    window.motor_output_checkbox.setChecked(False)
    assert not window.process_substitutions()['motor_output_enabled']
    window.node.set_motor_output_enabled.assert_called_once_with(False)
    warning.assert_not_called()


@pytest.mark.parametrize('component_id', ['tcp', 'robot_control'])
def test_component_button_preserves_normal_start_stop(window, monkeypatch, component_id):
    manager = window.system_manager
    managed = manager.processes[component_id]
    status = Mock(return_value='stopped')
    monkeypatch.setattr(managed, 'status', status)
    start = Mock(return_value=True)
    stop = Mock(return_value=True)
    monkeypatch.setattr(manager, 'start', start)
    monkeypatch.setattr(manager, 'stop', stop)
    window.component_widgets[component_id]['button'].click()
    start.assert_called_once_with(component_id, set(), window.process_substitutions())
    assert start.call_args.args[2]['motor_output_enabled'] is False
    status.return_value = 'running'
    window.component_widgets[component_id]['button'].click()
    stop.assert_called_once_with(component_id)
    window.node.set_motor_output_enabled.assert_not_called()


@pytest.mark.parametrize('enabled', [False, True])
def test_node_updates_requested_motor_gate_when_service_is_ready(enabled):
    node = Mock()
    client = node.robot_control_parameter_client
    client.service_is_ready.return_value = True
    assert gui_node.GUINode.set_motor_output_enabled(node, enabled) is client.call_async.return_value
    request = client.call_async.call_args.args[0]
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'motor_output_enabled'
    assert request.parameters[0].value.bool_value is enabled


@pytest.mark.parametrize('enabled', [False, True])
def test_node_defers_motor_gate_update_when_controller_is_unavailable(enabled):
    node = Mock()
    node.robot_control_parameter_client.service_is_ready.return_value = False
    assert gui_node.GUINode.set_motor_output_enabled(node, enabled) is None
    node.robot_control_parameter_client.call_async.assert_not_called()


@pytest.mark.parametrize('enabled, label', [(False, 'DISABLED'), (True, 'ENABLED')])
def test_motor_gate_feedback_reports_controller_acknowledgement(window, enabled, label):
    future = Mock()
    future.result.return_value = SimpleNamespace(results=[SimpleNamespace(successful=True)])
    window.motor_output_parameter_finished(future, enabled)
    assert window.motor_output_note.text() == f'{label} — Robot control acknowledged the setting'


def test_config_override_is_explicit_and_missing_path_does_not_fall_back(monkeypatch, tmp_path):
    config = tmp_path / 'container-components.json'
    monkeypatch.setenv('HRM_SYSTEM_COMPONENTS_CONFIG', str(config))
    selected = gui_node.MyGUI._system_config_path(object())
    assert selected == config
    with pytest.raises(FileNotFoundError):
        gui_node.load_component_specs(selected)
