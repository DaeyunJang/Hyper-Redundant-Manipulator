"""Record button follows service acknowledgement and recorder lifecycle."""

from concurrent.futures import Future
import os
import time
from types import SimpleNamespace
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
import pytest

from gui_py_pkg.gui_node import MyGUI
from gui_py_pkg.force_alignment import ForceAlignmentPanel


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def ui(app):
    return SimpleNamespace(
        node=SimpleNamespace(send_request_record_start=Mock(),
                             send_request_record_stop=Mock(), record_status=None),
        record_button=QPushButton('Record'), record_label=QLabel(),
        force_alignment_panel=ForceAlignmentPanel(),
        is_recording=False, record_future=None, record_error=None,
        record_request_time=0.0)


def status(ui, state):
    ui.node.record_status = ({'state': state, 'detail': 'test', 'directory': '/tmp/test'}, time.monotonic())


def test_start_does_not_pretend_success(ui):
    future = Future()
    ui.node.send_request_record_start.return_value = future
    MyGUI.start_recording(ui)
    assert not ui.is_recording
    assert not ui.record_button.isEnabled()
    future.set_result(SimpleNamespace(success=True, message='Start accepted'))
    status(ui, 'starting')
    MyGUI.update_record_status(ui)
    # The ACK alone does not unlock controls; use a subsequent fresh heartbeat.
    assert not ui.record_button.isEnabled()
    status(ui, 'starting')
    MyGUI.update_record_status(ui)
    assert ui.is_recording and ui.record_button.text() == 'Stop'


def test_failed_start_stays_inactive_and_visible(ui):
    future = Future()
    ui.node.send_request_record_start.return_value = future
    MyGUI.start_recording(ui)
    future.set_result(SimpleNamespace(success=False, message='Missing required publishers'))
    status(ui, 'idle')
    MyGUI.update_record_status(ui)
    assert not ui.is_recording
    assert 'Missing required' in ui.record_label.text()
    status(ui, 'idle')
    MyGUI.update_record_status(ui)
    assert 'Missing required' in ui.record_label.text()


def test_stop_button_waits_for_flush(ui):
    status(ui, 'recording')
    MyGUI.update_record_status(ui)
    future = Future()
    ui.node.send_request_record_stop.return_value = future
    MyGUI.stop_recording(ui)
    future.set_result(SimpleNamespace(success=True, message='Flushing'))
    status(ui, 'stopping')
    MyGUI.update_record_status(ui)
    assert ui.is_recording and not ui.record_button.isEnabled()
    status(ui, 'stopped')
    MyGUI.update_record_status(ui)
    assert not ui.is_recording and ui.record_button.isEnabled()


def test_stale_recorder_is_not_shown_as_healthy(ui):
    status(ui, 'recording')
    ui.node.record_status = (ui.node.record_status[0], time.monotonic() - 4)
    MyGUI.update_record_status(ui)
    assert 'stale' in ui.record_label.text()
    assert not ui.record_button.isEnabled()


def test_service_unavailable_does_not_toggle_recording(ui):
    ui.node.send_request_record_start.return_value = None
    MyGUI.start_recording(ui)
    assert not ui.is_recording
    assert 'unavailable' in ui.record_label.text()


def test_exporting_blocks_new_record_until_finished(ui):
    status(ui, 'exporting')
    MyGUI.update_record_status(ui)
    assert not ui.is_recording
    assert not ui.record_button.isEnabled()
    assert ui.record_button.text() == 'Exporting…'
    status(ui, 'stopped')
    MyGUI.update_record_status(ui)
    assert ui.record_button.isEnabled()


def test_incomplete_is_warning_not_green_success(ui):
    status(ui, 'stopped')
    ui.node.record_status[0]['data_quality'] = 'incomplete'
    MyGUI.update_record_status(ui)
    assert '#b9770e' in ui.record_button.styleSheet()

