"""All motor/record controls and three previews stay on screen, without ROS."""

import os
from pathlib import Path
from unittest.mock import Mock

from PyQt5.QtCore import QPoint, QRect
from PyQt5.QtWidgets import QApplication, QLabel, QScrollArea
import pytest


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app, monkeypatch):
    from gui_py_pkg import gui_node, image_preview

    receiver = Mock()
    receiver.take_latest.return_value = ({}, {})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    monkeypatch.setattr(gui_node.MyGUI, 'init_timer', lambda self: None)
    # Render the real Matplotlib canvas, but never animate or read fake sensors.
    monkeypatch.setattr(gui_node, 'FuncAnimation', Mock())
    source_config = Path(__file__).parents[1] / 'config' / 'system_components.json'
    monkeypatch.setattr(gui_node.MyGUI, '_system_config_path', lambda self: source_config)
    popen = Mock(side_effect=AssertionError('Layout tests cannot start processes.'))
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    node = Mock(numofmotors=4, num_loadcells=4, auto_start_components=False,
                last_motor_state_time=None, last_loadcell_data_time=None,
                last_fts_data_time=None, record_status=None)
    widget = gui_node.MyGUI(node)
    widget.camera_exposure.timer.stop()
    widget.sine_motion_panel.timer.stop()
    widget.record_timer.stop()
    yield widget
    widget.close()
    if hasattr(widget, 'figure'):
        gui_node.plt.close(widget.figure)
    popen.assert_not_called()
    receiver.close.assert_called_once()


def visible_inside(child, parent):
    bounds = QRect(child.mapTo(parent, QPoint(0, 0)), child.size())
    assert child.isVisible(), child.objectName()
    assert parent.rect().contains(bounds), (child, bounds, parent.rect())


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
@pytest.mark.parametrize('aligned', [False, True])
def test_motor_record_and_previews_fit_without_scroll(app, window, size, aligned):
    window.force_alignment_panel.enabled_checkbox.setChecked(aligned)
    window.image_preview.timer.stop()
    window.image_preview.status_labels['tag'].setText('Rectified: ID 0, 1\nMatched detections')
    window.image_preview.status_labels['depth'].setText('● Receiving')
    window.image_preview.status_labels['skeleton'].setText('● Receiving')
    window.resize(*size)
    window.show()
    app.processEvents()
    # Check actual dimensions too: silently growing the window is not a fit.
    assert (window.width(), window.height()) == size
    assert window.main_splitter.widget(1) is window.control_tab
    assert not window.control_tab.findChildren(QScrollArea)
    assert set(window.image_preview.canvases) == {'depth', 'skeleton', 'tag'}
    controls = [
        window.motor_output_checkbox, window.motor_output_note,
        window.motor_data_status_label, window.record_button, window.record_label,
        window.force_alignment_panel.enabled_checkbox,
        window.force_alignment_panel.summary, window.motor_kinematics_button,
        window.force_alignment_panel.contact_segment_id,
        window.LPF_parameter, window.MAF_parameter,
        window.image_preview.near, window.image_preview.far,
        *window.checkbox_mode_list, *window.checkbox_amode_list,
        *window.motor_state_line_edit_list, *window.motor_pub_line_edit_list,
        *window.motor_pub_button_list, *window.target_wire_length_line_edit_list,
        *window.motor_kinematics_line_edit_list,
        *window.sine_motion_panel.amplitude.values(),
        *window.sine_motion_panel.phase.values(),
        window.sine_motion_panel.period,
        window.sine_motion_panel.start_button,
        window.sine_motion_panel.stop_button,
        window.sine_motion_panel.status_label,
        *window.force_alignment_panel.axis_combos,
        *window.image_preview.canvases.values(),
        *window.image_preview.status_labels.values(),
    ]
    for control in controls:
        visible_inside(control, window.control_tab)
    for canvas in window.image_preview.canvases.values():
        assert canvas.width() >= 150 and canvas.height() >= 100
    exposure = window.camera_exposure
    for control in (exposure, exposure.target_combo, exposure.auto_checkbox,
                    exposure.exposure_spin, exposure.read_button, exposure.apply_button,
                    exposure.status_label):
        visible_inside(control, window.system_tab)
    assert not window.motor_output_checkbox.isChecked()
    assert all(not button.isEnabled() for button in window.motor_pub_button_list)


def test_existing_left_right_and_safety_locations_are_preserved(window):
    assert window.main_splitter.widget(0) is window.left_splitter
    assert window.left_splitter.widget(0) is window.system_tab
    assert window.left_splitter.widget(1) is window.sensor_tab
    assert window.control_tab.isAncestorOf(window.motor_output_checkbox)
    assert window.sensor_tab.isAncestorOf(window.zero_button)
    assert window.control_layout.indexOf(window.image_preview) == (
        window.control_layout.count() - 1)
    move_tip_index = next(
        index for index in range(window.control_layout.count())
        if window.control_layout.itemAt(index).layout()
        is window.motor_kinematics_layout_fin)
    assert window.control_layout.indexOf(window.sine_motion_panel) == move_tip_index + 1


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_sine_controls_have_visible_labels_and_follow_move_tip(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    panel = window.sine_motion_panel
    labels = panel.findChildren(QLabel)
    assert [label.text() for label in labels] == [
        'IK sine', 'Pan A', 'φ', 'Tilt A', 'φ', 'T', 'Idle']
    for label in labels:
        visible_inside(label, window.control_tab)
        assert label.width() >= label.sizeHint().width()
    assert panel.y() >= (window.motor_kinematics_layout_fin.geometry().bottom() + 1)
    assert panel.height() >= panel.minimumSizeHint().height()


@pytest.mark.parametrize('group,expected_labels', [
    (0, ['fx', 'fy', 'fz']), (1, ['tx', 'ty', 'tz'])])
def test_sensor_legend_stays_upper_right_as_traces_and_limits_change(
        window, group, expected_labels):
    axis = window.fts_axes[group]
    legend = axis.get_legend()
    assert [line.get_label() for line in axis.lines] == expected_labels
    assert [text.get_text() for text in legend.get_texts()] == expected_labels
    assert legend._loc == 1  # Matplotlib's fixed 'upper right', not automatic 'best'.
    positions = []
    for scale in (1.0, -1000.0, 5000.0):
        for index, line in enumerate(window.lines):
            line.set_data([0, 1, 2], [scale * index, scale, -scale])
        axis.set_xlim(0, 2)
        axis.set_ylim(-abs(scale) * 7, abs(scale) * 7)
        window.canvas.draw()
        bounds = legend.get_window_extent(window.canvas.get_renderer())
        positions.append(bounds.transformed(axis.transAxes.inverted()).bounds)
    for position in positions:
        assert position == pytest.approx(positions[0])


def test_torque_display_and_independent_graph_scales(window, monkeypatch):
    from geometry_msgs.msg import WrenchStamped
    from gui_py_pkg import gui_node
    window.node.fts_data = WrenchStamped()
    wrench = window.node.fts_data.wrench
    wrench.force.x = 1000.1234
    wrench.torque.x, wrench.torque.y, wrench.torque.z = 0.30000000004, -1.27, 2.0
    window.update_fts()
    assert [edit.text() for edit in window.fts_sub_line_edit_list[3:]] == [
        '0.3', '-1.3', '2.0']
    assert wrench.torque.x == 0.30000000004
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    window.update_fts_plot(0)
    assert window.fts_axes[0].get_ylim()[1] > 1000
    assert window.fts_axes[1].get_ylim()[1] < 3


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_force_axis_pairs_keep_labels_close_and_groups_apart(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    panel = window.force_alignment_panel
    labels = {label.text(): label for label in panel.findChildren(QLabel)}
    previous_combo = None
    for axis, combo in zip('xyz', panel.axis_combos):
        label = labels[f'Base F{axis} =']
        assert label.width() == label.sizeHint().width()
        assert combo.width() == combo.sizeHint().width()
        assert combo.x() - (label.x() + label.width()) == 4
        if previous_combo is not None:
            assert label.x() - (previous_combo.x() + previous_combo.width()) == 16
        visible_inside(label, window.control_tab)
        visible_inside(combo, window.control_tab)
        previous_combo = combo
    assert not window.control_tab.findChildren(QScrollArea)


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_mode_choices_stay_grouped_at_natural_widths(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    for layout, label, checkboxes in (
            (window.layout_mode, window.label_mode, window.checkbox_mode_list),
            (window.layout_amode, window.label_amode, window.checkbox_amode_list)):
        controls = [label, *checkboxes]
        assert label.x() == layout.geometry().x()
        for control in controls:
            assert control.width() == control.sizeHint().width()
            visible_inside(control, window.control_tab)
        for previous, following in zip(controls, controls[1:]):
            assert following.x() - (previous.x() + previous.width()) == 12
        assert layout.itemAt(layout.count() - 1).spacerItem() is not None
