"""Port discovery and GUI launch wiring, with no hardware/process launches."""

import os
from pathlib import Path
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg import serial_port_selector as ports


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def selector(app, monkeypatch):
    monkeypatch.setattr(ports, 'discover_serial_ports',
                        lambda: ['/dev/ttyUSB0', '/dev/ttyACM1'])
    monkeypatch.setattr(Path, 'is_char_device', lambda self: True)
    widget = ports.SerialPortSelector()
    yield widget
    widget.close()


def test_discovery_lists_both_families_in_numeric_order_without_open(tmp_path):
    for name in ('ttyUSB10', 'ttyUSB2', 'ttyACM1', 'ttyACM0', 'ttyS0', 'ttyUSB2bad'):
        (tmp_path / name).touch()
    assert ports.discover_serial_ports(tmp_path) == [
        str(tmp_path / name) for name in ('ttyUSB2', 'ttyUSB10', 'ttyACM0', 'ttyACM1')]


def test_single_port_selected_at_startup(app, monkeypatch):
    monkeypatch.setattr(ports, 'discover_serial_ports', lambda: ['/dev/ttyACM2'])
    widget = ports.SerialPortSelector()
    assert widget.selected_port() == '/dev/ttyACM2'
    widget.close()


def test_start_refreshes_reconnected_port_without_opening(selector, monkeypatch):
    selector.combo.setEditText('/dev/ttyUSB0')
    monkeypatch.setattr(ports, 'discover_serial_ports', lambda: ['/dev/ttyUSB1'])
    monkeypatch.setattr(Path, 'is_char_device', lambda self: str(self) == '/dev/ttyUSB1')
    assert selector.port_for_start() == '/dev/ttyUSB1'
    assert selector.launch_port is None


def test_refresh_preserves_unplugged_selection_and_offers_new_port(selector, monkeypatch):
    selector.combo.setEditText('/dev/ttyUSB0')
    monkeypatch.setattr(ports, 'discover_serial_ports', lambda: ['/dev/ttyUSB1'])
    selector.refresh_button.click()
    assert selector.selected_port() == '/dev/ttyUSB0'
    assert selector.combo.itemText(0) == '/dev/ttyUSB1'
    selector.combo.setCurrentIndex(0)
    assert selector.port_for_start() == '/dev/ttyUSB1'


def test_editing_or_refresh_never_changes_current_launch_port(selector):
    selector.mark_started('/dev/ttyUSB0')
    selector.set_process_status('running')
    selector.combo.setEditText('/dev/ttyACM1')
    selector.refresh_ports()
    assert selector.launch_port == '/dev/ttyUSB0'
    assert selector.selected_port() == '/dev/ttyACM1'
    assert 'Launch port: /dev/ttyUSB0' in selector.note.text()
    assert 'Next Start: /dev/ttyACM1' in selector.note.text()
    selector.set_process_status('stopped')
    assert 'Launch port:' not in selector.note.text()


def test_by_id_keeps_case_and_survives_refresh(selector):
    selector.combo.setEditText('/dev/serial/by-id/usb-ESP32_AbCd-if00')
    selector.refresh_ports()
    assert selector.port_for_start() == '/dev/serial/by-id/usb-ESP32_AbCd-if00'


@pytest.mark.parametrize('path', ['', 'USB0', '/dev/ttyusb0', '/tmp/file', '/dev/null'])
def test_invalid_path_is_rejected_and_visible(selector, path):
    selector.combo.setEditText(path)
    with pytest.raises(ValueError, match='Select'):
        selector.port_for_start()
    assert '#d93025' in selector.note.styleSheet()


def test_disconnected_path_error_remains_until_edit_or_refresh(selector, monkeypatch):
    selector.combo.setEditText('/dev/ttyUSB0')
    monkeypatch.setattr(Path, 'is_char_device', lambda self: False)
    with pytest.raises(ValueError, match='not present'):
        selector.port_for_start()
    selector.set_process_status('stopped')
    assert 'Connect the ESP32' in selector.note.text()
    selector.refresh_ports()
    assert selector.error is None


def test_external_reader_is_not_claimed_by_selected_port(selector):
    selector.set_process_status('external')
    assert 'External reader' in selector.note.text()
    assert 'Launch port:' not in selector.note.text()


@pytest.fixture
def window(app, monkeypatch):
    from gui_py_pkg import gui_node, image_preview

    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    monkeypatch.setattr(gui_node.MyGUI, 'init_fts_plot', lambda self: None)
    monkeypatch.setattr(ports, 'discover_serial_ports',
                        lambda: ['/dev/ttyUSB1', '/dev/ttyACM0'])
    monkeypatch.setattr(Path, 'is_char_device', lambda self: True)
    # Exercise the real process manager's command construction, not Popen.
    popen = Mock(return_value=Mock(pid=12345))
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    monkeypatch.setattr('gui_py_pkg.system_manager.os.killpg', Mock())
    node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False)
    node.discovered_node_names.return_value = set()
    widget = gui_node.MyGUI(node)
    yield widget, popen
    widget.close()


@pytest.mark.parametrize('port', ['/dev/ttyUSB1', '/dev/ttyACM0'])
def test_gui_start_button_passes_selected_port_to_launch(window, port):
    widget, popen = window
    assert widget.system_tab.isAncestorOf(widget.serial_port_selector)
    widget.serial_port_selector.combo.setEditText(port)
    widget.component_widgets['serial']['button'].click()
    assert popen.call_args.args[0] == [
        'ros2', 'launch', 'serial_pkg', '_launch.py', f'serial_port:={port}']
    assert widget.serial_port_selector.launch_port == port
    assert not widget.motor_output_checkbox.isChecked()
    widget.serial_port_selector.combo.setEditText('/dev/ttyUSB7')
    widget.serial_port_selector.refresh_button.click()
    assert popen.call_count == 1
    popen.return_value.send_signal.assert_not_called()


def test_missing_port_blocks_only_serial_start(window, monkeypatch):
    widget, popen = window
    widget.serial_port_selector.combo.setEditText('/dev/ttyUSB1')
    monkeypatch.setattr(Path, 'is_char_device', lambda self: False)
    widget.start_component_if_selected('serial')
    popen.assert_not_called()
    assert 'not present' in widget.serial_port_selector.note.text()
    widget.start_component('robot_control')
    assert popen.call_args.args[0][-1] == 'motor_output_enabled:=false'


def test_startup_with_auto_schedule_cannot_open_an_unselected_device(window):
    widget, popen = window
    assert widget.serial_port_selector.selected_port() == ''
    widget.start_component_if_selected('serial')
    popen.assert_not_called()
    assert 'Select /dev/ttyUSB' in widget.serial_port_selector.note.text()
