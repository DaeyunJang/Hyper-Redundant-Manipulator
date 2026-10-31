"""All motor/record controls and three previews stay on screen, without ROS."""

import os
from pathlib import Path
from unittest.mock import Mock

from PyQt5.QtCore import QPoint, QRect, Qt
from PyQt5.QtWidgets import QApplication, QFrame, QLabel, QScrollArea
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
    widget.position_sine_panel.timer.stop()
    widget.position_pid_panel.timer.stop()
    widget.admittance_parameters_panel.timer.stop()
    widget.admittance_target_panel.timer.stop()
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


def read_admittance_targets(window, y=0.25, z=-0.5, force_sign=-1.0, future=None):
    """Confirm settings through a fake service response, without a ROS executor."""
    from concurrent.futures import Future
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters

    values = {
        'admittance/target_force_y_N': y,
        'admittance/target_force_z_N': z,
        'admittance/force_sign': force_sign,
        'admittance/max_force_N': 2.0,
    }
    client = window.node.create_client.return_value
    client.service_is_ready.return_value = True
    if future is None:
        future = Future()
        client.call_async.return_value = future
        window.admittance_target_panel.read_settings()
    request = client.call_async.call_args.args[0]
    future.set_result(GetParameters.Response(values=[
        ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=values[name])
        for name in request.names]))
    window.admittance_target_panel.poll()


def receive_unified_force(window, now):
    from custom_interfaces.msg import ForceIdEstimate
    from geometry_msgs.msg import Vector3
    from gui_py_pkg.force_id_preview import ForceIdPreview

    window.node.force_id_preview = ForceIdPreview()
    message = ForceIdEstimate()
    message.header.frame_id = 'hrm_base'
    message.valid = True
    message.status = 'ready'
    message.mode = 'class0'
    message.force_scope = 'tip'
    message.external_force = Vector3(x=0.011, y=0.021, z=0.031)
    message.probabilities = [1.0] + [0.0] * 18
    message.contact_id = 0
    message.control_contact_id = 18
    message.load_state = message.UNLOADED
    message.heatmap_center_id = 1
    window.node.force_id_preview.receive(message, now=now)


@pytest.mark.parametrize('value,color', [
    (0.0, 'black'), (999.9, 'black'), (1000.0, 'black'),
    (1000.1, '#e67e00'), (1499.9, '#e67e00'), (1500.0, 'red'),
    (2000.0, 'red'), (-1000.0, 'black'), (-1000.1, '#e67e00'),
    (-1500.0, 'red'), (float('nan'), 'black'), (float('inf'), 'black'),
])
def test_sensor_value_colors_and_original_formatting(window, value, color):
    from geometry_msgs.msg import WrenchStamped
    from custom_interfaces.msg import LoadcellState

    message = WrenchStamped()
    for vector in (message.wrench.force, message.wrench.torque):
        vector.x = vector.y = vector.z = value
    window.node.fts_data = message
    window.node.loadcell_data = LoadcellState(stress=[value] * 4)
    window.update_fts()
    window.update_loadcell()
    for field in window.fts_sub_line_edit_list + window.lc_sub_line_edit_list:
        assert field.styleSheet() == f'QLineEdit {{ color: {color}; }}'
    assert window.fts_sub_line_edit_list[0].text() == str(value)
    assert window.fts_sub_line_edit_list[3].text() == f'{value:.1f}'
    assert window.lc_sub_line_edit_list[0].text() == str(value)


def test_sensor_colors_reset_and_missing_loadcells_clear(window):
    from geometry_msgs.msg import WrenchStamped
    from custom_interfaces.msg import LoadcellState

    window.node.fts_data = WrenchStamped()
    window.node.fts_data.wrench.force.x = 1500.0
    window.node.loadcell_data = LoadcellState(stress=[1500.0] * 4)
    window.update_fts()
    window.update_loadcell()
    assert 'red' in window.fts_sub_line_edit_list[0].styleSheet()
    window.node.fts_data.wrench.force.x = 0.0
    window.node.loadcell_data = LoadcellState(stress=[1001.0])
    window.update_fts()
    window.update_loadcell()
    assert 'black' in window.fts_sub_line_edit_list[0].styleSheet()
    assert '#e67e00' in window.lc_sub_line_edit_list[0].styleSheet()
    for field in window.lc_sub_line_edit_list[1:]:
        assert field.text() == '—'
        assert 'black' in field.styleSheet()


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
        window.force_alignment_panel.save_images_checkbox,
        window.LPF_parameter, window.MAF_parameter,
        window.image_preview.near, window.image_preview.far,
        window.image_preview.contact_checkbox,
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
        *window.position_pid_panel.gains.values(),
        window.position_pid_panel.read_button,
        window.position_pid_panel.apply_button,
        window.position_pid_panel.status_label,
        *window.admittance_parameters_panel.parameters.values(),
        window.admittance_parameters_panel.read_button,
        window.admittance_parameters_panel.apply_button,
        window.admittance_parameters_panel.status_label,
        *window.admittance_target_panel.targets.values(),
        window.admittance_target_panel.read_button,
        window.admittance_target_panel.apply_button,
        window.admittance_target_panel.status_label,
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


def test_contact_image_and_numeric_heatmap_share_the_existing_snapshot(window):
    import time
    from gui_py_pkg.force_id_preview import ForceIdPreview, ForceIdSnapshot

    preview = ForceIdPreview()
    preview.received_at = time.monotonic()
    probabilities = tuple(1.0 if index == 9 else 0.0 for index in range(19))
    preview.value = ForceIdSnapshot(
        valid=True, force_mn=(10.0, -20.0, 0.0), probabilities=probabilities,
        mode='masked', contact_id=9, heatmap_center_id=9,
        received_at=preview.received_at, source_stamp=1_000_000_000)
    window.node.force_id_preview = preview
    window.update_fts()
    assert window.image_preview.force_id_snapshot is preview.value
    assert window.force_id_monitor.last_snapshot is preview.value

    # Stale/absent sources cannot leave an independent old image prediction.
    preview.received_at -= 1.0
    window.update_fts()
    assert not window.image_preview.force_id_snapshot.valid
    window.node.force_id_preview = None
    window.update_fts()
    assert window.image_preview.force_id_snapshot is None
    assert window.control_layout.indexOf(window.image_preview) == (
        window.control_layout.count() - 1)
    move_tip_index = next(
        index for index in range(window.control_layout.count())
        if window.control_layout.itemAt(index).layout()
        is window.motor_kinematics_layout_fin)
    assert window.control_layout.indexOf(window.sine_motion_stack) == move_tip_index + 1
    assert window.control_layout.indexOf(window.controller_settings_panel) == move_tip_index + 2
    assert window.controller_settings_panel.isAncestorOf(window.position_pid_panel)
    assert window.controller_settings_panel.isAncestorOf(window.admittance_parameters_panel)
    assert window.control_layout.indexOf(window.admittance_target_panel) > move_tip_index + 2


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
    assert window.sine_motion_stack.y() >= (
        window.motor_kinematics_layout_fin.geometry().bottom() + 1)
    assert panel.height() >= panel.minimumSizeHint().height()


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_admittance_targets_form_one_visible_row_below_controller_settings(app, window, size):
    read_admittance_targets(window)
    window.resize(*size)
    window.show()
    app.processEvents()
    panel = window.admittance_target_panel
    assert panel.y() >= window.controller_settings_panel.geometry().bottom() + 1
    assert panel.height() <= 36
    assert set(panel.targets) == {'y', 'z'}
    for widget in (*panel.findChildren(QLabel), *panel.targets.values(),
                   panel.read_button, panel.apply_button):
        visible_inside(widget, window.control_tab)
    # Both target inputs and actions occupy the same compact row.
    centers = [widget.mapTo(panel, widget.rect().center()).y()
               for widget in (*panel.targets.values(), panel.read_button, panel.apply_button)]
    assert max(centers) - min(centers) <= 1
    assert not window.control_tab.findChildren(QScrollArea)


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
@pytest.mark.parametrize('aligned', [False, True])
def test_pid_and_admittance_parameters_have_compact_horizontal_groups(
        app, window, size, aligned):
    window.force_alignment_panel.enabled_checkbox.setChecked(aligned)
    window.resize(*size)
    window.show()
    app.processEvents()
    container = window.controller_settings_panel
    pid, admittance = window.position_pid_panel, window.admittance_parameters_panel
    assert (window.width(), window.height()) == size
    assert set(admittance.parameters) == {'m', 'b', 'k'}
    assert pid.geometry().right() < admittance.geometry().left()
    assert abs(pid.y() - admittance.y()) <= 2
    for panel in (pid, admittance):
        visible_inside(panel, window.control_tab)
        assert panel.height() <= 48
        for caption in panel.findChildren(QLabel):
            visible_inside(caption, window.control_tab)
            # Avoid a nominally visible label whose text is clipped horizontally.
            assert caption.width() >= caption.minimumSizeHint().width()
    separators = [frame for frame in container.findChildren(QFrame)
                  if frame.frameShape() == QFrame.VLine]
    assert separators
    assert any(pid.geometry().right() < frame.geometry().left()
               and frame.geometry().right() < admittance.geometry().left()
               for frame in separators)
    horizontal = [frame for frame in window.control_tab.findChildren(QFrame)
                  if frame.frameShape() == QFrame.HLine]
    assert any(container.geometry().bottom() < frame.mapTo(window.control_tab, QPoint()).y()
               < window.admittance_target_panel.y() for frame in horizontal)
    assert not window.control_tab.findChildren(QScrollArea)


def test_controller_settings_start_read_first_without_parameter_writes(window):
    from rcl_interfaces.srv import SetParametersAtomically

    panel = window.admittance_parameters_panel
    assert panel.read_button.isEnabled()
    assert not panel.apply_button.isEnabled()
    assert not any(spin.isEnabled() for spin in panel.parameters.values())
    assert panel.status_label.text() == 'Read first'
    # Constructing the new controls must not write gains, target forces, or modes.
    requests = window.node.create_client.return_value.call_async.call_args_list
    assert all(not isinstance(call.args[0], SetParametersAtomically.Request)
               for call in requests)
    assert not window.motor_output_checkbox.isChecked()


@pytest.mark.parametrize('mode_index', [3, 4])
@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_cartesian_sine_replaces_ik_row_without_extra_scroll(app, window, size, mode_index):
    # Explicit user mode selection is mocked; the waveform itself never changes mode.
    window.checkbox_mode_list[mode_index].click()
    window.resize(*size)
    window.show()
    app.processEvents()
    panel = window.position_sine_panel
    assert (window.width(), window.height()) == size
    assert window.sine_motion_stack.currentWidget() is panel
    assert panel.isVisible() and not window.sine_motion_panel.isVisible()
    assert not window.control_tab.findChildren(QScrollArea)
    for widget in (*panel.amplitude.values(), *panel.phase.values(), panel.period,
                   panel.start_button, panel.stop_button, panel.status_label,
                   *panel.findChildren(QLabel), *window.image_preview.canvases.values()):
        visible_inside(widget, window.control_tab)
    assert [label.text() for label in panel.findChildren(QLabel)] == [
        'Y/Z sine', 'Y A', 'φ', 'Z A', 'φ', 'T', 'Idle']
    assert [label.text() for label in window.motor_kinematics_label_list] == [
        'Move Tip | Y (m) / sine centre', 'Move Tip | Z (m) / sine centre']
    assert window.controller_settings_panel.y() > window.sine_motion_stack.y()
    assert all(canvas.width() >= 150 and canvas.height() >= 100
               for canvas in window.image_preview.canvases.values())
    assert not window.motor_output_checkbox.isChecked()


def test_ik_and_cartesian_settings_centres_survive_mode_changes(window):
    window.motor_kinematics_line_edit_list[0].setText('30')
    window.motor_kinematics_line_edit_list[1].setText('-15')
    window.sine_motion_panel.amplitude['pan'].setValue(12.)
    window.sine_motion_panel.phase['tilt'].setValue(-35.)
    window.checkbox_mode_list[3].click()
    assert [edit.text() for edit in window.motor_kinematics_line_edit_list] == ['0', '0']
    window.motor_kinematics_line_edit_list[0].setText('-0.003')
    window.motor_kinematics_line_edit_list[1].setText('0.005')
    window.position_sine_panel.amplitude['y'].setValue(.004)
    window.position_sine_panel.phase['z'].setValue(45.)
    window.checkbox_amode_list[1].click()  # Relative does not change sine-centre meaning.
    params = window.position_sine_panel.motion_parameters()
    assert list(params[0].value.double_array_value) == [-.003, .004, 0.]
    assert list(params[1].value.double_array_value) == [.005, .010, 45.]
    window.checkbox_mode_list[4].click()
    assert [edit.text() for edit in window.motor_kinematics_line_edit_list] == ['-0.003', '0.005']
    window.checkbox_mode_list[1].click()
    assert [edit.text() for edit in window.motor_kinematics_line_edit_list] == ['30', '-15']
    assert window.sine_motion_panel.amplitude['pan'].value() == 12.
    assert window.sine_motion_panel.phase['tilt'].value() == -35.
    window.checkbox_mode_list[3].click()
    assert [edit.text() for edit in window.motor_kinematics_line_edit_list] == ['-0.003', '0.005']
    assert window.position_sine_panel.amplitude['y'].value() == .004
    assert window.position_sine_panel.phase['z'].value() == 45.


@pytest.mark.parametrize('group,expected_labels', [
    (0, ['fx', 'fy', 'fz', 'fx_pred', 'fy_pred', 'fz_pred']),
    (1, ['tx', 'ty', 'tz'])])
def test_sensor_legend_stays_upper_right_as_traces_and_limits_change(
        window, group, expected_labels):
    axis = window.fts_axes[group]
    legend = axis.get_legend()
    assert [line.get_label() for line in axis.lines if line in window.lines] == expected_labels
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
    lower, upper = window.torque_axes[0].get_ylim()
    assert all(lower < value < upper for value in (0.3, -1.27, 2.0))
    assert upper - lower < 5


def test_force_predictions_expand_force_scale_but_not_torque_scale(window, monkeypatch):
    import numpy as np
    from geometry_msgs.msg import Vector3, WrenchStamped
    from gui_py_pkg import force_prediction, gui_node

    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    window.node.fts_data = WrenchStamped()
    wrench = window.node.fts_data.wrench
    wrench.force.x, wrench.force.y, wrench.force.z = 10., 20., 30.
    wrench.torque.x, wrench.torque.y, wrench.torque.z = 0.1, 0.2, 0.3
    window.node.force_prediction = force_prediction.ForcePredictionPreview()
    window.node.force_prediction.receive(Vector3(x=11., y=21., z=3000.))
    assert window.update_fts_plot(0) is window.lines
    assert window.data_y.shape == (9, 50)
    assert np.allclose(
        window.data_y[:, -1], [10., 20., 30., 0.1, 0.2, 0.3, 11., 21., 3000.])
    for index, line in enumerate(window.lines):
        assert np.allclose(line.get_ydata(), window.data_y[index], equal_nan=True)
        assert list(line.get_xdata()) == list(range(50))
    lower, upper = window.force_axes[0].get_ylim()
    assert all(lower < value < upper for value in (10., 20., 30., 11., 21., 3000.))
    assert upper > 3000
    assert all(axis.get_ylim()[1] < 1 for axis in window.torque_axes)
    assert all(axis.get_xlim() == (0., 49.) for axis in window.fts_axes)


def test_plot_refresh_is_limited_to_ten_hz_without_changing_sensor_buffers(window):
    from gui_py_pkg import gui_node

    args, kwargs = gui_node.FuncAnimation.call_args
    assert args == (window.figure, window.update_fts_plot)
    assert kwargs['interval'] == 100
    assert kwargs['cache_frame_data'] is False
    assert window.data_y.shape == (9, 50)


def test_unchanged_plot_limits_are_not_reassigned(window, monkeypatch):
    from geometry_msgs.msg import WrenchStamped
    from gui_py_pkg import gui_node

    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    window.node.fts_data = WrenchStamped()
    window.update_fts_plot(0)
    for axis in window.fts_axes:
        monkeypatch.setattr(axis, 'set_xlim', Mock(wraps=axis.set_xlim))
        monkeypatch.setattr(axis, 'set_ylim', Mock(wraps=axis.set_ylim))
    window.update_fts_plot(1)
    for axis in window.fts_axes:
        axis.set_xlim.assert_not_called()
        axis.set_ylim.assert_not_called()


def test_sensor_graphs_have_horizontal_tick_grids_and_subtle_predictions(window):
    from matplotlib.colors import rgb_to_hsv, to_rgb
    from matplotlib.lines import Line2D

    window.canvas.draw()
    for axis in window.fts_axes:
        grids = [tick.gridline for tick in axis.yaxis.get_major_ticks()]
        assert grids and all(grid.get_visible() for grid in grids)
        assert not any(grid.get_visible() for grid in axis.get_xgridlines())
        for tick, grid in zip(axis.yaxis.get_major_ticks(), grids):
            red, green, blue = to_rgb(grid.get_color())
            assert (green, blue) == pytest.approx((red, red))
            assert 0.65 <= red < 1.0
            assert grid.get_ydata()[0] == tick.get_loc()
        assert axis.get_axisbelow()

    def dash_lengths(line):
        # Matplotlib 3.5 stores the same unscaled lengths under the older name.
        if hasattr(line, '_unscaled_dash_pattern'):
            return line._unscaled_dash_pattern[1]
        return line._us_dashSeq

    default_dash = dash_lengths(Line2D([], [], linestyle='--'))
    for measured, predicted in zip(window.lines[:3], window.prediction_lines):
        measured_rgb = to_rgb(measured.get_color())
        predicted_rgb = to_rgb(predicted.get_color())
        assert measured.get_linestyle() == '-'
        assert predicted.get_linestyle() == '--'
        assert predicted.get_linewidth() < measured.get_linewidth()
        assert rgb_to_hsv(predicted_rgb)[0] == pytest.approx(
            rgb_to_hsv(measured_rgb)[0], abs=0.01)
        assert rgb_to_hsv(predicted_rgb)[2] < rgb_to_hsv(measured_rgb)[2]
        # Matplotlib exposes custom dash lengths through this stored pattern.
        dash = dash_lengths(predicted)
        assert dash and sum(dash) < sum(default_dash)


def test_admittance_target_lines_are_thin_red_references_on_yz_only(window):
    from matplotlib.colors import to_rgb

    assert set(window.admittance_target_lines) == {'y', 'z'}
    assert len(window.lines) == 9
    axis = window.force_axes[0]
    for component in ('y', 'z'):
        target = window.admittance_target_lines[component]
        assert target.axes is axis and target in axis.lines
        assert target not in window.lines
        assert target not in window.prediction_lines
        assert not target.get_visible()
        assert target.get_linestyle() == '-'
        assert 0 < target.get_linewidth() < window.lines[1].get_linewidth()
        red, green, blue = to_rgb(target.get_color())
        assert red > 0.75 and green < 0.3 and blue < 0.3
        assert list(target.get_xdata()) == [0.0, 1.0]
    assert [line for line in axis.lines if line in window.lines] == [
        *window.lines[:3], *window.prediction_lines]
    assert list(window.torque_axes[0].lines) == window.lines[3:6]


@pytest.mark.parametrize('force_sign', [-1.0, 1.0])
def test_confirmed_admittance_targets_use_base_prediction_sign_and_matching_limits(
        window, monkeypatch, force_sign):
    import numpy as np
    from geometry_msgs.msg import WrenchStamped
    from gui_py_pkg import force_id_preview, gui_node

    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(force_id_preview.time, 'monotonic', lambda: 10.0)
    window.node.fts_data = WrenchStamped()
    wrench = window.node.fts_data.wrench
    wrench.force.x, wrench.force.y, wrench.force.z = 10., 20., 30.
    wrench.torque.x, wrench.torque.y, wrench.torque.z = 0.1, 0.2, 0.3
    receive_unified_force(window, now=10.0)
    # A known base frame alone does not confirm the controller's target values.
    window.update_fts_plot(0)
    assert not any(line.get_visible() for line in window.admittance_target_lines.values())
    read_admittance_targets(window, force_sign=force_sign)
    expected_targets = (250.0 / force_sign, -500.0 / force_sign)
    assert window.admittance_target_panel.target_plot_mn() == expected_targets
    window.node.reset_mock()
    assert window.update_fts_plot(1) is window.lines
    axis = window.force_axes[0]
    for component, value in zip(('y', 'z'), expected_targets):
        target = window.admittance_target_lines[component]
        assert target.get_visible()
        assert list(target.get_ydata()) == pytest.approx([value, value])
        lower, upper = axis.get_ylim()
        assert lower < value < upper
    assert all(axis.get_ylim()[1] < 1 for axis in window.torque_axes)
    assert window.data_y.shape == (9, 50)
    # Raw measured and predicted values are never negated to match control sign.
    assert np.allclose(window.data_y[:, -1], [10., 20., 30., 0.1, 0.2, 0.3, 11., 21., 31.])
    for index, line in enumerate(window.lines):
        assert np.allclose(line.get_ydata(), window.data_y[index], equal_nan=True)
    assert not window.node.mock_calls
    assert not window.motor_output_checkbox.isChecked()


def test_admittance_target_edits_wait_for_readback_and_plotting_has_no_service_calls(
        window, monkeypatch):
    from concurrent.futures import Future
    from geometry_msgs.msg import WrenchStamped
    from rcl_interfaces.msg import SetParametersResult
    from rcl_interfaces.srv import SetParametersAtomically
    from gui_py_pkg import force_id_preview, gui_node

    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(force_id_preview.time, 'monotonic', lambda: 10.0)
    window.node.fts_data = WrenchStamped()
    receive_unified_force(window, now=10.0)
    read_admittance_targets(window)
    panel = window.admittance_target_panel
    window.update_fts_plot(0)
    original = [list(line.get_ydata()) for line in window.admittance_target_lines.values()]
    window.node.reset_mock()
    panel.targets['y'].setValue(750.0)
    panel.targets['z'].setValue(-100.0)
    window.update_fts_plot(1)
    assert panel.target_plot_mn() == (-250.0, 500.0)
    assert [list(line.get_ydata()) for line in window.admittance_target_lines.values()] == original
    assert not window.node.mock_calls

    future = Future()
    window.node.create_client.return_value.call_async.return_value = future
    panel.apply_settings()
    window.update_fts_plot(2)
    assert [list(line.get_ydata()) for line in window.admittance_target_lines.values()] == original
    future.set_result(SetParametersAtomically.Response(
        result=SetParametersResult(successful=True, reason='')))
    readback = Future()
    window.node.create_client.return_value.call_async.return_value = readback
    panel.poll()
    assert panel.target_plot_mn() is None
    window.update_fts_plot(3)
    assert not any(line.get_visible() for line in window.admittance_target_lines.values())
    read_admittance_targets(window, y=0.75, z=-0.1, future=readback)
    assert panel.target_plot_mn() == (-750.0, 100.0)
    # Controller readback emits the refresh signal without waiting for animation.
    for component, value in (('y', -750.0), ('z', 100.0)):
        assert list(window.admittance_target_lines[component].get_ydata()) == [value, value]
    window.node.reset_mock()
    window.update_fts_plot(4)
    for component, value in (('y', -750.0), ('z', 100.0)):
        assert list(window.admittance_target_lines[component].get_ydata()) == [value, value]
    assert not window.node.mock_calls
    assert not window.motor_output_checkbox.isChecked()


def test_admittance_targets_need_base_frame_and_survive_stale_prediction(
        window, monkeypatch):
    import numpy as np
    from geometry_msgs.msg import Vector3, WrenchStamped
    from gui_py_pkg import force_id_preview, force_prediction, gui_node

    clock = {'now': 10.0}
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(force_id_preview.time, 'monotonic', lambda: clock['now'])
    window.node.fts_data = WrenchStamped()
    window.node.force_prediction = force_prediction.ForcePredictionPreview()
    window.node.force_prediction.receive(Vector3(x=1.0, y=2.0, z=3.0))
    read_admittance_targets(window)
    window.update_fts_plot(0)
    # Legacy sensor-frame predictions cannot be compared with a base-frame target.
    assert not any(line.get_visible() for line in window.admittance_target_lines.values())
    receive_unified_force(window, now=clock['now'])
    window.update_fts_plot(1)
    assert all(line.get_visible() for line in window.admittance_target_lines.values())
    clock['now'] += 0.6
    window.node.force_prediction.receive(Vector3(x=1.0, y=2.0, z=3.0))
    window.update_fts_plot(2)
    assert np.isnan(window.data_y[6:9, -1]).all()
    # The known reference remains useful while fresh prediction samples are absent.
    for component, value in (('y', -250.0), ('z', 500.0)):
        line = window.admittance_target_lines[component]
        assert line.get_visible()
        assert list(line.get_ydata()) == [value, value]
    window.node.force_id_preview = None
    window.update_fts_plot(3)
    assert not any(line.get_visible() for line in window.admittance_target_lines.values())
    assert np.allclose(window.data_y[6:9, -1], [1.0, 2.0, 3.0])
    assert window.force_axes[0].get_ylim()[0] > -100
    assert window.force_axes[0].get_ylim()[1] < 100
    receive_unified_force(window, now=clock['now'])
    window.update_fts_plot(4)
    assert all(line.get_visible() for line in window.admittance_target_lines.values())
    window.node.reset_mock()
    window.admittance_target_panel.update_nodes(set())
    window.update_fts_plot(5)
    assert not any(line.get_visible() for line in window.admittance_target_lines.values())
    assert not window.node.mock_calls
    assert not window.motor_output_checkbox.isChecked()


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_two_grouped_graphs_keep_sensor_readouts_paired_and_predictions_below(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    assert len(window.fts_axes) == len(window.figure.axes) == 2
    assert len(window.lines) == 9
    assert list(window.force_axes) == [window.fts_axes[0]]
    assert list(window.torque_axes) == [window.fts_axes[1]]
    force_axis, torque_axis = window.fts_axes
    for axis, row in ((force_axis, 0), (torque_axis, 1)):
        subplot = axis.get_subplotspec()
        assert subplot.get_gridspec().get_geometry() == (2, 1)
        assert (subplot.rowspan.start, subplot.colspan.start) == (row, 0)
    assert [line for line in force_axis.lines if line in window.lines] == [
        *window.lines[:3], *window.prediction_lines]
    assert list(torque_axis.lines) == window.lines[3:6]
    assert force_axis.get_position().x0 == pytest.approx(torque_axis.get_position().x0)
    assert torque_axis.get_position().y1 < force_axis.get_position().y0
    assert 'mN' in force_axis.get_title() + force_axis.get_ylabel()
    assert 'mN m' in torque_axis.get_title() + torque_axis.get_ylabel()
    for i in range(3):
        force = window.fts_sub_line_edit_list[i]
        torque = window.fts_sub_line_edit_list[i + 3]
        assert window.fts_grid.itemAtPosition(i, 1).widget() is force
        assert window.fts_grid.itemAtPosition(i, 3).widget() is torque
        assert force.y() == torque.y()
        assert torque.x() > force.x() + force.width()
        assert abs(force.width() - torque.width()) <= 1
        assert force.width() >= 40
    bottom = max(e.y() + e.height() for e in window.fts_sub_line_edit_list)
    for field in window.predicted_force_line_edits:
        assert field.y() >= bottom
        assert field.isReadOnly() and field.text() == '—'


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_sensor_graphs_stay_legible_with_only_vertical_graph_scrolling(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    graph_scroll = window.sensor_graph_scroll
    assert graph_scroll.widget() is window.canvas
    assert window.canvas.height() >= 160
    assert graph_scroll.horizontalScrollBarPolicy() == Qt.ScrollBarAlwaysOff
    assert graph_scroll.horizontalScrollBar().maximum() == 0
    assert window.canvas.width() == graph_scroll.viewport().width()
    needs_scroll = graph_scroll.viewport().height() < window.canvas.minimumHeight()
    assert graph_scroll.verticalScrollBar().isVisible() == needs_scroll
    assert (graph_scroll.verticalScrollBar().maximum() > 0) == needs_scroll
    if size == (1366, 768):
        assert needs_scroll
    else:
        assert not needs_scroll
    assert not window.control_tab.findChildren(QScrollArea)
    assert not window.image_preview.findChildren(QScrollArea)

    window.canvas.draw()
    renderer = window.canvas.get_renderer()
    figure_bounds = window.figure.bbox
    # All axes, tick labels, titles and legends remain inside the full canvas,
    # including when its lower rows need scrolling into view.
    for artist in [*window.fts_axes, window.fts_frame_note]:
        bounds = artist.get_tightbbox(renderer)
        assert bounds.x0 >= figure_bounds.x0
        assert bounds.y0 >= figure_bounds.y0
        assert bounds.x1 <= figure_bounds.x1
        assert bounds.y1 <= figure_bounds.y1


def test_prediction_values_lines_stale_and_recovery(app, window, monkeypatch, tmp_path):
    import numpy as np
    from geometry_msgs.msg import Vector3, WrenchStamped
    from gui_py_pkg import force_prediction, gui_node
    clock = {'now': 10.0}
    monkeypatch.setattr(force_prediction.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(gui_node.rclpy, 'shutdown', lambda **kwargs: None)
    window.node.fts_data = WrenchStamped()
    window.node.force_prediction = force_prediction.ForcePredictionPreview()
    window.update_fts()
    window.update_fts_plot(0)
    assert np.isnan(window.data_y[6:9]).all()
    preview = window.node.force_prediction
    preview.receive(Vector3(x=0.0, y=-12.5, z=1200.0))
    window.update_fts()
    window.update_fts_plot(1)
    assert [f.text() for f in window.predicted_force_line_edits] == ['0.0', '-12.5', '1200.0']
    assert np.allclose(window.data_y[6:9, -1], [0.0, -12.5, 1200.0])
    assert window.force_axes[0].get_ylim()[1] > 1200
    assert all(axis.get_ylim()[1] < 1 for axis in window.torque_axes)
    window.resize(1500, 900)
    window.show()
    app.processEvents()
    window.canvas.draw()
    assert window.grab().save(str(tmp_path / 'prediction_gui.png'))
    clock['now'] = 10.6
    window.update_fts()
    window.update_fts_plot(2)
    assert all(f.text() == 'stale' for f in window.predicted_force_line_edits)
    assert np.isnan(window.data_y[6:9, -1]).all()
    assert all(np.isnan(line.get_ydata()[-1]) for line in window.prediction_lines)
    preview.receive(Vector3(x=float('nan'), y=0.0, z=0.0))
    window.update_fts()
    assert all(f.text() == 'invalid' for f in window.predicted_force_line_edits)
    preview.receive(Vector3())
    window.update_fts()
    window.update_fts_plot(3)
    assert all(f.text() == '0.0' for f in window.predicted_force_line_edits)
    assert np.isnan(window.data_y[6:9, -2]).all()
    assert np.equal(window.data_y[6:9, -1], 0.).all()


def test_unified_force_source_has_priority_and_does_not_fall_back_on_stale(window, monkeypatch):
    from geometry_msgs.msg import Vector3, WrenchStamped
    from custom_interfaces.msg import ForceIdEstimate
    from gui_py_pkg import force_id_preview, force_prediction

    clock = {'now': 10.0}
    monkeypatch.setattr(force_id_preview.time, 'monotonic', lambda: clock['now'])
    window.node.fts_data = WrenchStamped()
    window.node.force_prediction = force_prediction.ForcePredictionPreview()
    window.node.force_prediction.receive(Vector3(x=123.0, y=234.0, z=345.0))
    window.node.force_id_preview = force_id_preview.ForceIdPreview()
    message = ForceIdEstimate()
    message.header.frame_id = 'hrm_base'
    message.valid = True
    message.status = 'ready'
    message.mode = 'class0'
    message.force_scope = 'tip'
    message.external_force = Vector3(x=0.1, y=-0.2, z=0.0)
    message.probabilities = [1.0] + [0.0] * 18
    message.contact_id = 0
    message.control_contact_id = 18
    message.load_state = message.UNLOADED
    message.heatmap_center_id = 1
    message.id_tol = 2
    window.node.force_id_preview.receive(message)
    window.update_fts()
    assert [field.text() for field in window.predicted_force_line_edits] == ['100.0', '-200.0', '0.0']
    assert all('[base]' in label.text() for label in window.predicted_force_labels)
    assert all('hrm_base' in field.toolTip() for field in window.predicted_force_line_edits)
    assert window.fts_frame_note.get_text() == 'Measured: sensor | predicted: hrm_base'
    assert 'Unloaded' in window.force_id_monitor.status_label.text()
    assert 'control ID 18' in window.force_id_monitor.status_label.text()
    clock['now'] += 0.6
    # Even a freshly received legacy vector cannot overwrite stale unified input.
    window.node.force_prediction.receive(Vector3(x=1000.0, y=2000.0, z=3000.0))
    window.update_fts()
    assert all(field.text() == 'stale' for field in window.predicted_force_line_edits)
    assert not window.motor_output_checkbox.isChecked()
    # Removing the unified source restores the legacy sensor-frame annotation.
    window.node.force_id_preview = None
    window.update_fts()
    assert window.fts_frame_note.get_text() == 'Measured / predicted: sensor'
    assert all('[base]' not in label.text() for label in window.predicted_force_labels)


@pytest.mark.parametrize('size', [(1366, 768), (1500, 900), (1920, 1080)])
def test_force_id_controls_fit_without_enabling_motors(app, window, size):
    window.resize(*size)
    window.show()
    app.processEvents()
    assert (window.width(), window.height()) == size
    spec = next(s for s in window.system_manager.specs if s.component_id == 'force_id')
    assert window.auto_start_enabled['force_id'] is spec.auto_start
    assert not window.force_id_monitor.body_checkbox.isChecked()
    assert not window.motor_output_checkbox.isChecked()
    for control in (window.force_model_selector, window.force_model_selector.combo,
                    window.force_model_selector.refresh_button):
        visible_inside(control, window.system_tab)
    for control in (window.force_id_monitor, window.force_id_monitor.body_checkbox,
                    window.force_id_monitor.tolerance, *window.force_id_monitor.cells,
                    window.force_id_monitor.status_label):
        visible_inside(control, window.sensor_tab)
    for cell in window.force_id_monitor.cells:
        assert cell.width() >= cell.fontMetrics().horizontalAdvance(cell.text())
    assert window.sensor_plot_widget.findChildren(QScrollArea) == [window.sensor_graph_scroll]


def test_force_id_start_uses_selected_bundle_and_has_no_motor_side_effect(window, tmp_path):
    from gui_py_pkg.force_model_selector import ForceModelSelector

    bundle = tmp_path / 'test model' / 'hrm_bundle.pt'
    bundle.parent.mkdir()
    bundle.touch()
    window.force_model_selector = ForceModelSelector(roots=[tmp_path])
    window.force_model_selector.combo.setCurrentIndex(1)
    window.node.discovered_node_names.return_value = set()
    window.system_manager.start = Mock(return_value=True)
    window.start_component('force_id')
    name, discovered, substitutions = window.system_manager.start.call_args.args
    assert name == 'force_id' and discovered == set()
    assert substitutions['force_model_path'] == str(bundle)
    assert substitutions['body_contact_enabled'] is False
    assert substitutions['force_id_tol'] == 2
    assert substitutions['motor_output_enabled'] is False
    spec = next(s for s in window.system_manager.specs if s.component_id == 'force_id')
    command = spec.command(substitutions)
    assert command[:4] == ['ros2', 'launch', 'estimation_pkg', 'force_id_estimation.launch.py']
    assert f'model_path:={bundle}' in command  # Spaces remain inside one argv element.
    assert 'device:=cpu' in command
    window.node.set_motor_output_enabled.assert_not_called()
    window.node.send_request_move_tool_angle.assert_not_called()


def test_body_toggle_only_changes_inference_parameters(app, window):
    from concurrent.futures import Future
    from types import SimpleNamespace
    from rcl_interfaces.msg import ParameterType

    client = window.node.force_id_parameter_client
    client.service_is_ready.return_value = False
    window.force_id_monitor.body_checkbox.setChecked(True)
    client.call_async.assert_not_called()
    assert 'next inference Start' in window.force_id_monitor.settings_label.text()
    client.service_is_ready.return_value = True
    future = Future()
    client.call_async.return_value = future
    window.force_id_monitor.tolerance.setValue(3)
    params = {p.name: p.value for p in client.call_async.call_args.args[0].parameters}
    assert set(params) == {'body_contact_enabled', 'id_tol'}
    assert params['body_contact_enabled'].type == ParameterType.PARAMETER_BOOL
    assert params['body_contact_enabled'].bool_value is True
    assert params['id_tol'].integer_value == 3
    future.set_result(SimpleNamespace(result=SimpleNamespace(successful=True, reason='')))
    app.processEvents()
    assert 'Applied' in window.force_id_monitor.settings_label.text()
    window.node.set_motor_output_enabled.assert_not_called()
    window.node.send_request_set_control_mode.assert_not_called()


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
