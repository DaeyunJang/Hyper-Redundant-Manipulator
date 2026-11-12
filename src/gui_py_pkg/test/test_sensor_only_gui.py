# AUTHOR: Daeyun -- Sensor-only GUI workflow regression checks.
"""Exercise motor guards with fake nodes and processes; never initialize ROS."""

import os
from pathlib import Path
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg import gui_node, image_preview


@pytest.fixture(autouse=True)
def clean_environment(monkeypatch):
    monkeypatch.delenv('HRM_SENSOR_ONLY', raising=False)
    monkeypatch.delenv('HRM_SYSTEM_COMPONENTS_CONFIG', raising=False)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window_factory(app, monkeypatch):
    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    monkeypatch.setattr(gui_node.MyGUI, 'init_fts_plot', lambda self: None)
    source_config = Path(__file__).parents[1] / 'config' / 'system_components.json'
    monkeypatch.setenv('HRM_SYSTEM_COMPONENTS_CONFIG', str(source_config))
    popen = Mock(side_effect=AssertionError('No subprocess may launch in this test.'))
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    windows = []

    def create(sensor_only):
        if sensor_only:
            monkeypatch.setenv('HRM_SENSOR_ONLY', '1')
        node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False)
        node.discovered_node_names.return_value = set()
        window = gui_node.MyGUI(node)
        windows.append(window)
        return window

    yield create
    for window in windows:
        window.close()
    popen.assert_not_called()


def test_sensor_only_gui_is_visible_and_locks_enable(window_factory):
    window = window_factory(True)
    assert 'SENSOR ONLY' in window.windowTitle()
    assert 'SENSOR ONLY' in window.motor_output_note.text()
    assert 'tcp' not in window.component_widgets
    assert not window.motor_output_checkbox.isEnabled()
    assert not window.motor_output_checkbox.isChecked()
    assert not window.process_substitutions()['motor_output_enabled']
    window.node.set_motor_output_enabled.assert_not_called()


def test_programmatic_toggle_cannot_enable_output(window_factory, monkeypatch):
    window = window_factory(True)
    warning = Mock(side_effect=AssertionError('No enable confirmation should open.'))
    monkeypatch.setattr(gui_node.QMessageBox, 'warning', warning)
    window.motor_output_checkbox.setChecked(True)
    assert not window.motor_output_checkbox.isChecked()
    window.motor_output_toggled(True)
    assert not window.motor_output_checkbox.isChecked()
    assert not window.process_substitutions()['motor_output_enabled']
    window.node.set_motor_output_enabled.assert_not_called()
    warning.assert_not_called()


def test_native_gui_keeps_motor_enable_and_bridge(window_factory):
    window = window_factory(False)
    assert 'SENSOR ONLY' not in window.windowTitle()
    assert 'tcp' in window.component_widgets
    assert window.motor_output_checkbox.isEnabled()
    assert not window.motor_output_checkbox.isChecked()


def test_node_guard_rejects_enable_before_touching_service(monkeypatch):
    monkeypatch.setenv('HRM_SENSOR_ONLY', '1')
    node = Mock()
    assert gui_node.GUINode.set_motor_output_enabled(node, True) is None
    node.robot_control_parameter_client.service_is_ready.assert_not_called()
    node.robot_control_parameter_client.call_async.assert_not_called()
    node.get_logger().warning.assert_called_once()


@pytest.mark.parametrize('sensor_only, enabled', [(True, False), (False, True)])
def test_disabling_stays_available_and_native_enable_is_unchanged(
        monkeypatch, sensor_only, enabled):
    if sensor_only:
        monkeypatch.setenv('HRM_SENSOR_ONLY', '1')
    node = Mock()
    client = node.robot_control_parameter_client
    client.service_is_ready.return_value = True
    assert gui_node.GUINode.set_motor_output_enabled(node, enabled) is client.call_async.return_value
    request = client.call_async.call_args.args[0]
    assert len(request.parameters) == 1
    assert request.parameters[0].name == 'motor_output_enabled'
    assert request.parameters[0].value.bool_value is enabled


def test_sensor_only_blocks_direct_motor_publish(monkeypatch):
    monkeypatch.setenv('HRM_SENSOR_ONLY', '1')
    node = Mock()
    message = object()
    gui_node.GUINode.publish(node, message)
    node.motor_command_publisher_.publish.assert_not_called()
    monkeypatch.delenv('HRM_SENSOR_ONLY')
    gui_node.GUINode.publish(node, message)
    node.motor_command_publisher_.publish.assert_called_once_with(message)


def test_config_override_is_explicit_and_missing_path_does_not_fall_back(monkeypatch, tmp_path):
    config = tmp_path / 'container-components.json'
    monkeypatch.setenv('HRM_SYSTEM_COMPONENTS_CONFIG', str(config))
    selected = gui_node.MyGUI._system_config_path(object())
    assert selected == config
    with pytest.raises(FileNotFoundError):
        gui_node.load_component_specs(selected)
