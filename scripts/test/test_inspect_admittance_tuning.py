"""Read-only diagnostic tests using synthetic messages; never import or start ROS."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'inspect_admittance_tuning.py'
SPEC = importlib.util.spec_from_file_location('inspect_admittance_tuning_test', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def ros_stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def vector(y=0.0, z=0.0, x=0.0):
    return NS(x=x, y=y, z=z)


def pose(y=0.0, z=0.0):
    return NS(position=vector(y, z))


def diagonal(value):
    result = [0.0] * 36
    result[7] = result[14] = value
    return result


def messages(source_ns=1_000_000_000):
    def header(ns=source_ns):
        return NS(stamp=ros_stamp(ns), frame_id='hrm_base')

    admittance = NS(
        header=header(), m_matrix=diagonal(.15), b_matrix=diagonal(20.0),
        k_matrix=diagonal(30.0), dt=.05,
        desired_force=NS(force=vector(.2, 0.0)),
        env_force=NS(force=vector(.112, .01)),
        delta_force=NS(force=vector(-.088, .01)),
        x=pose(-.0029, .001), x_dot=pose(0.0, 0.0), x_ddot=pose(0.0, 0.0),
    )
    position = NS(
        header=header(), x_actual=pose(-.002828, .0048),
        x_desired=pose(-.0029, .005), x_error=pose(-.000072, .0002),
        p_gain=20.55, i_gain=0.0, d_gain=5.0, dt=.05, sampling_time=.05,
        del_theta_pan=0.0, del_theta_tilt=0.0,
    )
    reference = NS(
        header=header(source_ns + 20_000_000), feedback_stamp=ros_stamp(source_ns),
        control_mode='admittance', cartesian_reference_valid=True,
        motor_output_enabled=True, nominal_position=vector(0.0, .004),
        target_position=vector(-.0029, .005), pan_rad=math.radians(20),
        tilt_rad=math.radians(-30),
    )
    return admittance, position, reference


def series(count=31, *, axis='y', modify=None):
    rows = []
    for index in range(count):
        data = messages(1_000_000_000 + index * 50_000_000)
        if modify:
            modify(index, *data)
        rows.append(module.sample_row(*data, axis))
    return rows


def hints(report):
    return ' '.join(report['hints'])


def test_stamp_uses_exact_integer_nanoseconds():
    assert module.stamp(NS(sec=1793000000, nanosec=123456789)) == 1793000000123456789


def test_sample_pairs_source_stamp_not_publication_stamp_and_converts_si_units():
    row = module.sample_row(*messages(), 'y')
    assert row['source_time_ns'] == 1_000_000_000
    assert row['target_mn'] == pytest.approx(200.0)
    assert row['environment_mn'] == pytest.approx(112.0)
    assert row['force_error_mn'] == pytest.approx(-88.0)
    assert row['force_error_mn'] == pytest.approx(row['environment_mn'] - row['target_mn'])
    assert row['offset_mm'] == pytest.approx(-2.9)
    assert row['spring_mn'] == pytest.approx(-87.0)
    assert row['effective_mm'] == pytest.approx(-2.9)
    assert row['actual_mm'] == pytest.approx(-2.828)
    assert row['position_error_mm'] == pytest.approx(-.072)
    assert row['command_angle_deg'] == pytest.approx(-30.0)
    assert row['dt_s'] == .05
    assert (row['m'], row['b'], row['k']) == (.15, 20.0, 30.0)


def test_z_axis_uses_pan_command_and_selected_force_components():
    row = module.sample_row(*messages(), 'z')
    assert row['target_mn'] == 0.0
    assert row['environment_mn'] == pytest.approx(10.0)
    assert row['force_error_mn'] == pytest.approx(10.0)
    assert row['force_error_mn'] == pytest.approx(row['environment_mn'] - row['target_mn'])
    assert row['offset_mm'] == pytest.approx(1.0)
    assert row['nominal_mm'] == pytest.approx(4.0)
    assert row['effective_mm'] == pytest.approx(5.0)
    assert row['command_angle_deg'] == pytest.approx(20.0)


def test_signed_velocity_and_acceleration_terms_keep_units_and_sign():
    adm, pos, ref = messages()
    adm.x_dot.position.y = -.002
    adm.x_ddot.position.y = .1
    row = module.sample_row(adm, pos, ref, 'y')
    assert row['offset_velocity_mm_s'] == pytest.approx(-2.0)
    assert row['damping_mn'] == pytest.approx(-40.0)
    assert row['inertia_mn'] == pytest.approx(15.0)


@pytest.mark.parametrize('which', ('position', 'reference'))
def test_mismatched_source_stamp_is_rejected(which):
    adm, pos, ref = messages()
    if which == 'position':
        pos.header.stamp.nanosec += 1
    else:
        ref.feedback_stamp.nanosec += 1
    with pytest.raises(ValueError, match='timestamp'):
        module.sample_row(adm, pos, ref, 'y')


def test_zero_source_stamp_rejected():
    with pytest.raises(ValueError, match='timestamp'):
        module.sample_row(*messages(0), 'y')


@pytest.mark.parametrize('index', (0, 1, 2))
def test_non_base_frame_rejected(index):
    data = messages()
    data[index].header.frame_id = 'camera_color_optical_frame'
    with pytest.raises(ValueError, match='hrm_base'):
        module.sample_row(*data, 'y')


@pytest.mark.parametrize('mode', ('position', 'kinematics', ''))
def test_non_admittance_reference_rejected(mode):
    data = messages()
    data[2].control_mode = mode
    with pytest.raises(ValueError, match='Admittance'):
        module.sample_row(*data, 'y')


def test_invalid_cartesian_reference_and_axis_rejected():
    data = messages()
    data[2].cartesian_reference_valid = False
    with pytest.raises(ValueError):
        module.sample_row(*data, 'y')
    with pytest.raises(ValueError):
        module.sample_row(*messages(), 'x')


@pytest.mark.parametrize('matrix', ('m_matrix', 'b_matrix', 'k_matrix'))
def test_requires_exactly_36_matrix_entries(matrix):
    data = messages()
    setattr(data[0], matrix, [0.0] * 35)
    with pytest.raises(ValueError, match='6x6'):
        module.sample_row(*data, 'y')


@pytest.mark.parametrize('matrix', ('m_matrix', 'b_matrix', 'k_matrix'))
@pytest.mark.parametrize('axis,index', (('y', 13), ('z', 8)))
def test_rejects_cross_axis_matrix_coupling(matrix, axis, index):
    data = messages()
    getattr(data[0], matrix)[index] = .1
    with pytest.raises(ValueError, match='uncoupled'):
        module.sample_row(*data, axis)


@pytest.mark.parametrize('matrix', ('m_matrix', 'b_matrix', 'k_matrix'))
@pytest.mark.parametrize('index', (7, 13, 35))
def test_nonfinite_matrix_entries_rejected_even_off_diagonal_or_inactive(matrix, index):
    data = messages()
    getattr(data[0], matrix)[index] = math.nan
    with pytest.raises(ValueError):
        module.sample_row(*data, 'y')


@pytest.mark.parametrize('field', ('force', 'offset', 'angle', 'dt'))
def test_nonfinite_sample_rejected(field):
    adm, pos, ref = messages()
    if field == 'force':
        adm.env_force.force.y = math.nan
    elif field == 'offset':
        adm.x.position.y = math.inf
    elif field == 'angle':
        ref.tilt_rad = math.nan
    else:
        adm.dt = math.nan
    with pytest.raises(ValueError):
        module.sample_row(adm, pos, ref, 'y')


def test_reference_offset_mismatch_is_not_silently_paired():
    data = messages()
    data[2].target_position.y += .001
    with pytest.raises(ValueError, match='mismatch'):
        module.sample_row(*data, 'y')


def test_negative_gain_or_time_interval_rejected():
    data = messages()
    data[0].dt = -.01
    with pytest.raises(ValueError):
        module.sample_row(*data, 'y')
    data = messages()
    data[0].k_matrix[7] = -1.0
    with pytest.raises(ValueError):
        module.sample_row(*data, 'y')


def test_long_control_interval_rejected_instead_of_claiming_valid_update():
    data = messages()
    data[0].dt = .3
    with pytest.raises(ValueError):
        module.sample_row(*data, 'y')


@pytest.mark.parametrize('field', ('x_desired', 'x_error'))
def test_position_diagnostics_must_match_reference(field):
    data = messages()
    getattr(data[1], field).position.y += .001
    with pytest.raises(ValueError, match='mismatch'):
        module.sample_row(*data, 'y')


@pytest.mark.parametrize('sign', (-1.0, 1.0))
def test_spring_balance_uses_signed_environment_minus_target_with_small_inner_error(sign):
    def signed_sample(index, adm, pos, ref):
        for force in (adm.desired_force, adm.env_force, adm.delta_force):
            force.force.y *= sign
        for value in (adm.x, pos.x_actual, pos.x_desired, pos.x_error):
            value.position.y *= sign
        ref.target_position.y *= sign

    report, = module.summarize(series(modify=signed_sample))
    assert report['samples'] == 31
    assert report['duration_s'] == pytest.approx(1.5)
    assert report['environment_mean_mn'] == pytest.approx(112.0 * sign)
    assert report['force_error_mean_mn'] == pytest.approx(-88.0 * sign)
    assert report['spring_mean_mn'] == pytest.approx(-87.0 * sign)
    assert report['position_error_rmse_mm'] == pytest.approx(.072)
    assert 'SPRING_BALANCE' in hints(report)
    assert 'IN_BAND' not in hints(report)


def test_opposite_spring_sign_does_not_claim_static_force_balance():
    rows = series()
    for row in rows:
        row['spring_mn'] *= -1.0
    report, = module.summarize(rows)
    assert 'SPRING_BALANCE' not in hints(report)


def test_inner_position_lag_is_distinguished_from_spring_balance():
    rows = series()
    for row in rows:
        row['position_error_mm'] = 2.0
    report, = module.summarize(rows)
    assert 'POSITION_LAG' in hints(report)
    assert 'SPRING_BALANCE' not in hints(report)


def test_k_zero_residual_force_warns_about_continuing_displacement():
    rows = series()
    for row in rows:
        row['k'] = 0.0
        row['spring_mn'] = 0.0
    report, = module.summarize(rows)
    assert 'K_ZERO_ERROR' in hints(report)


def test_zero_force_target_does_not_mean_axis_is_disabled():
    report, = module.summarize(series(axis='z'))
    assert 'ZERO_TARGET_IS_ACTIVE' in hints(report)


def test_entire_tail_must_be_in_band_not_just_its_mean():
    rows = series()
    for index, row in enumerate(rows):
        row['force_error_mn'] = 30.0 if index % 2 else -30.0
    report, = module.summarize(rows)
    assert abs(report['force_error_mean_mn']) < 20.0
    assert 'IN_BAND' not in hints(report)
    for row in rows:
        row['force_error_mn'] = 10.0
    report, = module.summarize(rows)
    assert 'IN_BAND' in hints(report)
    assert 'not proof of stability' in hints(report)


def test_short_window_never_claims_convergence_or_spring_cause():
    rows = series(count=5)
    for row in rows:
        row['force_error_mn'] = 0.0
    report, = module.summarize(rows)
    assert 'SHORT_WINDOW' in hints(report)
    assert 'IN_BAND' not in hints(report)
    assert 'SPRING_BALANCE' not in hints(report)


@pytest.mark.parametrize('key,new_value', (
    ('target_mn', 300.0), ('m', .3), ('b', 30.0), ('k', 10.0),
    ('nominal_mm', 2.0), ('enabled', 0),
    ('other_target_mn', 150.0), ('other_nominal_mm', -3.0),
    ('pan_pd_p', 10.0), ('pan_pd_d', 2.0),
))
def test_setting_changes_split_intervals_before_statistics(key, new_value):
    rows = series(count=40)
    for row in rows[20:]:
        row[key] = new_value
    first, second = module.summarize(rows)
    assert first['samples'] == second['samples'] == 20
    assert first['source_end_ns'] < second['source_start_ns']
    if key == 'enabled':
        assert 'OUTPUT_DISABLED' in hints(second)


def test_long_source_gap_splits_intervals():
    rows = series(count=40)
    for row in rows[20:]:
        row['source_time_ns'] += 1_000_000_000
    reports = module.summarize(rows)
    assert [report['samples'] for report in reports] == [20, 20]


def test_zero_dt_frame_marks_boundary_and_is_not_a_control_update():
    rows = series(count=40)
    rows[20]['dt_s'] = 0.0
    reports = module.summarize(rows)
    assert [report['samples'] for report in reports] == [20, 19]


def test_axes_are_summarized_separately_and_input_order_is_irrelevant():
    rows = series() + series(axis='z')
    reports = module.summarize(list(reversed(rows)))
    assert [report['axis'] for report in reports] == ['y', 'z']
    assert [report['samples'] for report in reports] == [31, 31]


def test_statistics_use_only_last_two_seconds_of_each_interval():
    rows = series(count=100)
    for index, row in enumerate(rows):
        row['force_error_mn'] = 100.0 if index < 59 else 10.0
    report, = module.summarize(rows)
    assert report['samples'] == 100
    assert report['tail_duration_s'] == pytest.approx(2.0)
    assert report['force_error_mean_mn'] == pytest.approx(10.0)
    assert 'IN_BAND' in hints(report)


@pytest.mark.parametrize('keyword', ('force_tolerance_mn', 'position_tolerance_mm'))
@pytest.mark.parametrize('value', (0.0, -1.0, math.nan, math.inf))
def test_invalid_diagnostic_tolerances_rejected(keyword, value):
    with pytest.raises(ValueError):
        module.summarize([], **{keyword: value})


def test_empty_capture_has_no_convergence_claim():
    assert module.summarize([]) == []


def test_output_disabled_samples_do_not_claim_in_band_physical_tracking():
    rows = series()
    for row in rows:
        row['enabled'] = 0
        row['force_error_mn'] = 0.0
    report, = module.summarize(rows)
    assert 'OUTPUT_DISABLED' in hints(report)
    assert 'IN_BAND' not in hints(report)
