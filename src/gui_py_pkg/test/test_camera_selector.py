"""Compact camera startup selection; no SDK enumeration, ROS nodes or hardware."""

import os
from pathlib import Path
import re
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication, QGroupBox, QScrollArea
import pytest

from gui_py_pkg.camera_selector import CameraSelector


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def selector(app):
    widget = CameraSelector()
    yield widget
    widget.close()


def test_default_and_model_list_are_explicit_not_enumerated(selector):
    assert [selector.combo.itemText(index) for index in range(selector.combo.count())] == [
        'D405', 'D435i', 'D435', 'D455', 'D415']
    assert selector.combo.currentText() == 'D405'
    assert selector.selected_device_type() == 'D405$'
    assert selector.serial_edit.text() == ''
    assert selector.for_start() == {'camera_device_type': 'D405$', 'camera_serial_no': "''"}
    assert selector.launch_selection is None


def test_model_regex_keeps_d435_distinct_from_d435i(selector):
    selector.combo.setCurrentText('D435')
    expression = selector.selected_device_type()
    assert expression == 'D435$'
    assert re.search(expression, 'Intel RealSense D435')
    assert not re.search(expression, 'Intel RealSense D435i')
    selector.combo.setCurrentText('D435i')
    assert selector.selected_device_type() == 'D435i$'
    assert re.search(selector.selected_device_type(), 'Intel RealSense D435i')


@pytest.mark.parametrize('serial', ['001234567890', '_001234567890'])
def test_serial_stays_a_string_with_leading_zeros(selector, serial):
    selector.serial_edit.setText(serial)
    expected = {'camera_device_type': 'D405$', 'camera_serial_no': '_001234567890'}
    assert selector.for_start() == expected
    assert selector.substitutions() == expected


@pytest.mark.parametrize('serial', [
    'abc', '12a34', '__1234', '__', '_', '-1234', '12 34', '１２３４', '١٢٣٤', '1.5',
])
def test_bad_serial_is_rejected_only_when_camera_is_started(selector, serial):
    selector.serial_edit.setText(serial)
    # Generic substitutions also serve other components and must not validate.
    assert set(selector.substitutions()) == {'camera_device_type', 'camera_serial_no'}
    with pytest.raises(ValueError):
        selector.for_start()
    assert selector.launch_selection is None


@pytest.mark.parametrize('status', [
    'starting', 'running', 'stopping', 'graph_pending', 'unhealthy', 'external'])
def test_running_or_external_camera_selection_is_locked(selector, status):
    selector.set_process_status(status)
    assert not selector.combo.isEnabled()
    assert not selector.serial_edit.isEnabled()
    if status == 'external':
        assert 'external' in selector.toolTip().lower()
        assert selector.launch_selection is None


@pytest.mark.parametrize('status', ['stopped', 'failed'])
def test_stopped_or_failed_camera_can_be_reconfigured(selector, status):
    selector.set_process_status('running')
    selector.set_process_status(status)
    assert selector.combo.isEnabled() and selector.serial_edit.isEnabled()


def test_started_selection_is_a_frozen_request_not_a_live_device_query(selector):
    selector.combo.setCurrentText('D435i')
    selector.serial_edit.setText('00123')
    selected = selector.for_start()
    selector.mark_started(selected)
    assert selector.launch_selection == selected
    selected['camera_device_type'] = 'D455$'
    assert selector.launch_selection['camera_device_type'] == 'D435i$'
    assert not selector.combo.isEnabled()
    assert not selector.serial_edit.isEnabled()


def test_tag_selector_defaults_and_serial_have_their_own_substitution_keys(app):
    widget = CameraSelector(default_model='D435i', substitution_prefix='tag_camera')
    try:
        assert widget.combo.currentText() == 'D435i'
        assert widget.for_start() == {
            'tag_camera_device_type': 'D435i$', 'tag_camera_serial_no': "''"}
        widget.serial_edit.setText('_000123')
        assert widget.for_start()['tag_camera_serial_no'] == '_000123'
        selection = dict(widget.for_start(), camera_device_type='D405$')
        widget.mark_started(selection)
        assert widget.launch_selection == {
            'tag_camera_device_type': 'D435i$', 'tag_camera_serial_no': '_000123'}
        assert not widget.combo.isEnabled()
        assert 'camera_device_type' not in widget.launch_selection
    finally:
        widget.close()


@pytest.fixture
def window(app, monkeypatch):
    from gui_py_pkg import gui_node, image_preview

    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    monkeypatch.setattr(gui_node.MyGUI, 'init_fts_plot', lambda self: None)
    source_config = Path(__file__).parents[1] / 'config' / 'system_components.json'
    monkeypatch.setattr(gui_node.MyGUI, '_system_config_path', lambda self: source_config)
    # A regression must not launch real processes during a GUI unit test.
    popen = Mock(side_effect=AssertionError('No subprocess may launch in this test.'))
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False,
                last_motor_state_time=None, last_loadcell_data_time=None,
                last_fts_data_time=None)
    node.discovered_node_names.return_value = set()
    widget = gui_node.MyGUI(node)
    widget.system_manager.start = Mock(return_value=True)
    yield widget
    widget.close()
    popen.assert_not_called()
    receiver.close.assert_called_once()


def test_gui_launch_keeps_cuda_default_and_passes_only_camera_arguments(window):
    widget = window
    selector = widget.camera_selector
    selector.combo.setCurrentText('D435i')
    selector.serial_edit.setText('001234')
    widget.component_widgets['camera']['button'].click()
    widget.system_manager.start.assert_called_once()
    name, discovered, substitutions = widget.system_manager.start.call_args.args
    assert name == 'camera' and discovered == set()
    command = widget.system_manager.processes['camera'].spec.command(substitutions)
    assert command == ['ros2', 'launch', 'launcher', 'cuda_realsense.launch.py',
                       'device_type:=D435i$', 'serial_no:=_001234']
    assert selector.launch_selection == {
        'camera_device_type': 'D435i$', 'camera_serial_no': '_001234'}
    assert not selector.combo.isEnabled()
    assert not widget.motor_output_checkbox.isChecked()


def test_default_gui_camera_start_uses_d405_cuda_with_empty_serial(window):
    window.start_component('camera')
    substitutions = window.system_manager.start.call_args.args[2]
    command = window.system_manager.processes['camera'].spec.command(substitutions)
    assert command[:4] == ['ros2', 'launch', 'launcher', 'cuda_realsense.launch.py']
    assert command[4:] == [
        'device_type:=D405$',
        "serial_no:=''",
    ]


def test_default_tag_start_uses_second_cuda_launch_without_changing_hrm_camera(window):
    window.start_component('tag_camera')
    name, discovered, substitutions = window.system_manager.start.call_args.args
    assert name == 'tag_camera' and discovered == set()
    command = window.system_manager.processes[name].spec.command(substitutions)
    assert command == [
        'ros2', 'launch', 'launcher', 'cuda_apriltag_camera.launch.py',
        'device_type:=D455$', "serial_no:=''",
    ]
    assert window.tag_camera_selector.launch_selection == {
        'tag_camera_device_type': 'D455$', 'tag_camera_serial_no': "''"}
    assert window.camera_selector.launch_selection is None
    assert window.camera_selector.combo.isEnabled()
    assert window.camera_selector.selected_device_type() == 'D405$'
    assert not window.motor_output_checkbox.isChecked()


def test_independent_model_serial_commands_only_use_the_selected_camera(window):
    window.camera_selector.serial_edit.setText('001111')
    before = window.camera_selector.substitutions()
    window.tag_camera_selector.combo.setCurrentText('D455')
    window.tag_camera_selector.serial_edit.setText('_002222')
    assert window.camera_selector.substitutions() == before
    window.component_widgets['tag_camera']['button'].click()
    substitutions = window.system_manager.start.call_args.args[2]
    command = window.system_manager.processes['tag_camera'].spec.command(substitutions)
    assert command[-2:] == ['device_type:=D455$', 'serial_no:=_002222']
    frozen = dict(window.tag_camera_selector.launch_selection)

    window.camera_selector.combo.setCurrentText('D435')
    window.camera_selector.serial_edit.setText('003333')
    assert window.tag_camera_selector.launch_selection == frozen
    assert window.tag_camera_selector.substitutions() == frozen
    window.start_component('camera')
    substitutions = window.system_manager.start.call_args.args[2]
    command = window.system_manager.processes['camera'].spec.command(substitutions)
    assert command[-2:] == ['device_type:=D435$', 'serial_no:=_003333']
    assert window.tag_camera_selector.launch_selection == frozen


def test_invalid_tag_serial_blocks_only_tag_camera(window):
    window.tag_camera_selector.serial_edit.setText('__')
    window.start_component('tag_camera')
    window.system_manager.start.assert_not_called()
    assert window.tag_camera_selector.launch_selection is None
    assert window.tag_camera_selector.combo.isEnabled()
    window.node.get_logger().error.assert_called()

    window.start_component('camera')
    assert window.system_manager.start.call_args.args[0] == 'camera'
    assert window.camera_selector.launch_selection['camera_device_type'] == 'D405$'
    window.start_component('robot_control')
    assert window.system_manager.start.call_args.args[0] == 'robot_control'
    assert window.tag_camera_selector.launch_selection is None
    assert not window.motor_output_checkbox.isChecked()


@pytest.mark.parametrize('component_id', ['camera', 'tag_camera'])
@pytest.mark.parametrize('status', ['running', 'external'])
def test_status_update_locks_only_its_own_camera_selector(window, component_id, status):
    window.system_manager.processes[component_id].status = Mock(return_value=status)
    window.update_system_status()
    selected = window.camera_selectors[component_id]
    other = window.camera_selectors['tag_camera' if component_id == 'camera' else 'camera']
    assert not selected.combo.isEnabled() and not selected.serial_edit.isEnabled()
    assert other.combo.isEnabled() and other.serial_edit.isEnabled()
    if status == 'external':
        assert selected.launch_selection is None
        assert 'external' in selected.toolTip().lower()
    window.system_manager.processes[component_id].status.return_value = 'stopped'
    window.update_system_status()
    assert selected.combo.isEnabled() and selected.serial_edit.isEnabled()


def test_invalid_pending_camera_serial_does_not_block_other_components(window):
    widget = window
    widget.camera_selector.serial_edit.setText('not-a-serial')
    widget.start_component('camera')
    widget.system_manager.start.assert_not_called()
    assert widget.camera_selector.launch_selection is None
    widget.node.get_logger().error.assert_called()
    widget.start_component('robot_control')
    assert widget.system_manager.start.call_args.args[0] == 'robot_control'
    command = widget.system_manager.processes['robot_control'].spec.command(
        widget.system_manager.start.call_args.args[2])
    assert command[-1] == 'motor_output_enabled:=false'


@pytest.mark.parametrize('component_id', ['camera', 'tag_camera'])
def test_camera_selector_is_inside_existing_camera_row_without_scroll_area(window, component_id):
    selector = window.camera_selectors[component_id]
    assert window.system_tab.isAncestorOf(selector)
    parent, packages = selector.parentWidget(), None
    while parent is not None:
        assert not isinstance(parent, QScrollArea)
        if isinstance(parent, QGroupBox) and parent.title() == 'Packages':
            packages = parent
        parent = parent.parentWidget()
    assert packages is not None
    assert packages.layout().count() == len(window.system_manager.specs) + 1
    assert packages.layout().itemAt(packages.layout().count() - 1).widget() is (
        window.camera_exposure)

    def contains_widget(layout):
        for index in range(layout.count()):
            item = layout.itemAt(index)
            if item.widget() is selector or (item.layout() and contains_widget(item.layout())):
                return True
        return False
    camera_index = next(index for index, spec in enumerate(window.system_manager.specs)
                        if spec.component_id == component_id)
    assert contains_widget(packages.layout().itemAt(camera_index).layout())


def test_camera_selector_renders_as_one_compact_row_at_normal_window_size(app, window):
    window.resize(1500, 900)
    window.show()
    app.processEvents()
    for selector in window.camera_selectors.values():
        assert selector.width() > 150
        assert 0 < selector.height() <= 40
        assert selector.combo.geometry().top() == selector.serial_edit.geometry().top()


def test_tag_camera_adds_half_second_start_without_changing_existing_schedule(window, monkeypatch):
    specs = {spec.component_id: spec for spec in window.system_manager.specs}
    expected = {
        'camera': (True, 0.0), 'tag_camera': (True, 0.5), 'serial': (True, 1.0),
        'tcp': (True, 2.0), 'robot_control': (True, 3.0), 'estimation': (True, 4.0),
        'record': (False, 5.0), 'apriltag': (True, 6.0),
    }
    assert {name: (spec.auto_start, spec.delay_sec) for name, spec in specs.items()} == expected
    assert specs['camera'].expected_nodes == ('/camera/camera',)
    assert specs['tag_camera'].expected_nodes == ('/tag_camera/tag_camera',)
    assert specs['tag_camera'].label == 'Tag camera (CUDA)'
    assert window.auto_start_enabled == {name: enabled for name, (enabled, _) in expected.items()}
    timers = []

    def new_timer(parent):
        assert parent is window
        timer = Mock()
        timers.append(timer)
        return timer

    with monkeypatch.context() as timer_patch:
        timer_patch.setattr('gui_py_pkg.gui_node.QTimer', new_timer)
        window.schedule_auto_start()
    assert [timer.start.call_args.args[0] for timer in timers] == [
        0, 500, 1000, 2000, 3000, 4000, 6000]
    window.system_manager.start.assert_not_called()
    # Trigger only the two camera callbacks: no real timers/processes or ROS nodes.
    for timer, expected_id in zip(timers[:2], ('camera', 'tag_camera')):
        timer.setSingleShot.assert_called_once_with(True)
        timer.timeout.connect.call_args.args[0]()
        assert window.system_manager.start.call_args.args[0] == expected_id
    window.cancel_scheduled_starts()
    assert not window.auto_start_timers
    for timer in timers:
        timer.stop.assert_called_once()
        timer.deleteLater.assert_called_once()
