"""Recording-reference producer contracts; no ROS nodes or motors are started."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src/control_node.cpp').read_text()
INTERFACES = PACKAGE.parent / 'custom_interfaces'


def _block_after(text, anchor):
    start = text.index('{', text.index(anchor))
    depth = 1
    cursor = start + 1
    while depth:
        if text[cursor] == '{':
            depth += 1
        elif text[cursor] == '}':
            depth -= 1
        cursor += 1
    return text[start + 1:cursor - 1]


def test_reference_schema_distinguishes_clocks_points_and_output_gate():
    schema = (INTERFACES / 'msg/ControlReference.msg').read_text()
    fields = [line for line in schema.splitlines()
              if line.strip() and not line.lstrip().startswith('#')]
    assert fields == [
        'std_msgs/Header header',
        'builtin_interfaces/Time feedback_stamp',
        'string control_mode',
        'float64 pan_rad',
        'float64 tilt_rad',
        'bool cartesian_reference_valid',
        'geometry_msgs/Point nominal_position',
        'geometry_msgs/Point target_position',
        'bool motor_output_enabled',
    ]
    assert '"msg/ControlReference.msg"' in (
        INTERFACES / 'CMakeLists.txt').read_text()
    assert 'other guards may still prevent motor publication' in schema


def test_reference_has_two_generation_sites_and_reliable_latest_qos():
    assert '"/control/reference", qos_reliable_latest' in SOURCE
    assert SOURCE.count('control_reference_publisher_->publish(reference);') == 2


def test_kinematics_records_requested_angles_after_motor_target_conversion():
    ik = _block_after(SOURCE, 'void ControlNode::cal_inverse_kinematics(')
    reference = ik[ik.index('ControlReference reference;'):]
    assert 'reference.header.stamp = now();' in reference
    assert 'reference.header.frame_id = "hrm_base";' in reference
    assert 'reference.control_mode = "kinematics";' in reference
    assert 'reference.pan_rad = pAngle * M_PI / 180.0;' in reference
    assert 'reference.tilt_rad = tAngle * M_PI / 180.0;' in reference
    assert 'reference.cartesian_reference_valid = false;' in reference
    # Generated ROS messages zero-initialize the absent feedback stamp and points.
    assert 'reference.feedback_stamp' not in reference
    assert 'reference.nominal_position' not in reference
    assert 'reference.target_position' not in reference
    assert ik.index('ControlReference reference;') > ik.rindex('#endif')
    assert reference.rstrip().endswith(
        'control_reference_publisher_->publish(reference);')
    # The configured gate is telemetry, not a new condition on publishing it.
    assert 'reference.motor_output_enabled = motor_output_enabled_.load();' in reference
    assert 'if (' not in reference


def test_feedback_modes_record_source_time_and_current_step_references():
    diagnostics = _block_after(
        SOURCE, 'void ControlNode::publish_position_diagnostics(')
    reference = diagnostics[diagnostics.index('ControlReference reference;'):]
    assert 'reference.header.stamp = now();' in reference
    assert 'reference.header.frame_id = "hrm_base";' in reference
    assert 'reference.feedback_stamp = sample.header.stamp;' in reference
    assert 'reference.control_mode = ControlModeToString(mode);' in reference
    assert 'reference.pan_rad = HRM_position_controller_.surgical_tool_.pAngle_;' in reference
    assert 'reference.tilt_rad = HRM_position_controller_.surgical_tool_.tAngle_;' in reference
    assert 'reference.cartesian_reference_valid = true;' in reference
    for index, axis in enumerate('xyz'):
        assert f'reference.nominal_position.{axis} = x_desired_({index});' in reference
        assert (
            f'reference.target_position.{axis} = '
            f'HRM_position_controller_.x_desired_({index});') in reference
    assert 'reference.motor_output_enabled = motor_output_enabled_.load();' in reference
    assert 'if (' not in reference
    assert 'get_IK_result' not in reference
    assert 'computeEndEffectorPosition' not in reference


def test_feedback_reference_is_published_after_the_current_controller_update():
    worker = _block_after(
        SOURCE, 'void ControlNode::run_position_with_admittance_control_thread()')
    assert worker.count('publish_position_diagnostics(sample, dt, mode);') == 1
    assert worker.index('HRM_position_controller_.update(x_t_, x_actual_, dt)') < (
        worker.index('publish_position_diagnostics(sample, dt, mode);'))
