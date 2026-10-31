"""Offline unit and edge-case checks; no ROS services or hardware access."""

from dataclasses import FrozenInstanceError
import math

import pytest

from gui_py_pkg.admittance_tuning import Candidate, analyze_model, propose_gains


def candidate(**changes):
    arguments = dict(
        mode='force', force_error_mn=100.0, correction_speed_mm_s=5.0,
        velocity_time_constant_s=0.1, spring_error_budget_mn=10.0,
        correction_distance_mm=5.0,
    )
    arguments.update(changes)
    return propose_gains(**arguments)


def test_force_candidate_uses_si_units_and_velocity_time_constant():
    gains = candidate()
    assert gains == Candidate(mass=2.0, damping=20.0, stiffness=0.0)
    report = analyze_model(gains.mass, gains.damping, gains.stiffness)
    assert report['velocity_time_constant_s'] == pytest.approx(.1)
    assert report['nominal_velocity_mm_s'] == pytest.approx(5.0)
    assert report['bias_drift_mm_s'] == pytest.approx(.5)
    assert report['steady_offset_mm'] is None
    assert report['natural_frequency_rad_s'] is None
    assert report['damping_ratio'] is None
    assert any('unbounded' in warning for warning in report['warnings'])


def test_compliance_candidate_is_budget_per_offset_not_error_guarantee():
    gains = candidate(mode='compliance')
    assert gains == Candidate(mass=2.0, damping=20.0, stiffness=2.0)
    report = analyze_model(gains.mass, gains.damping, gains.stiffness)
    assert report['spring_error_mn'] == pytest.approx(10.0)
    assert report['steady_offset_mm'] == pytest.approx(50.0)
    assert report['bias_drift_mm_s'] is None
    assert report['natural_frequency_rad_s'] == pytest.approx(1.0)
    assert report['damping_ratio'] == pytest.approx(5.0)


def test_candidate_is_immutable():
    gains = candidate()
    with pytest.raises(FrozenInstanceError):
        gains.stiffness = 5.0


def test_force_mode_does_not_use_spring_budget_to_add_stiffness():
    assert candidate(spring_error_budget_mn=0.0).stiffness == 0.0
    assert candidate(spring_error_budget_mn=1000.0).stiffness == 0.0


@pytest.mark.parametrize('field', (
    'force_error_mn', 'correction_speed_mm_s', 'velocity_time_constant_s',
    'correction_distance_mm',
))
@pytest.mark.parametrize('value', (0.0, -1.0, float('nan'), float('inf'), -float('inf')))
def test_candidate_requires_positive_finite_design_inputs(field, value):
    with pytest.raises(ValueError):
        candidate(**{field: value})


@pytest.mark.parametrize('value', (-1.0, float('nan'), float('inf')))
def test_spring_budget_is_validated_even_if_force_mode_does_not_use_it(value):
    with pytest.raises(ValueError):
        candidate(spring_error_budget_mn=value)


def test_invalid_mode_and_zero_compliance_budget_rejected():
    with pytest.raises(ValueError):
        candidate(mode='auto')
    with pytest.raises(ValueError):
        candidate(mode='compliance', spring_error_budget_mn=0.0)


def test_force_velocity_unit_conversion_matches_si_calculation():
    gains = candidate(force_error_mn=250.0, correction_speed_mm_s=12.5,
                      velocity_time_constant_s=.075)
    assert gains.damping == pytest.approx((250.0 * 1e-3) / (12.5 * 1e-3))
    assert gains.mass == pytest.approx(1.5)


def test_spring_residual_matches_measured_example():
    report = analyze_model(.15, 20.0, 30.0, displacement_mm=-2.897)
    assert report['spring_error_mn'] == pytest.approx(-86.91)


def test_small_stiffness_large_static_offset_is_not_hidden_or_clamped():
    report = analyze_model(.15, 20.0, .01, force_error_mn=10.0)
    assert report['steady_offset_mm'] == pytest.approx(1000.0)
    assert any('unreachable' in warning for warning in report['warnings'])


def test_signed_input_values_keep_signs():
    report = analyze_model(.15, 20.0, 5.0, force_error_mn=-200.0,
                           displacement_mm=-2.0, bias_mn=-10.0)
    assert report['nominal_velocity_mm_s'] == pytest.approx(-10.0)
    assert report['spring_error_mn'] == pytest.approx(-10.0)
    assert report['steady_offset_mm'] == pytest.approx(-40.0)
    drift = analyze_model(.15, 20.0, 0.0, bias_mn=-10.0)
    assert drift['bias_drift_mm_s'] == pytest.approx(-.5)


def test_first_order_damper_and_spring_are_valid():
    report = analyze_model(0.0, 10.0, 2.0)
    assert report['velocity_time_constant_s'] == 0.0
    assert report['nominal_velocity_mm_s'] == 10.0
    assert report['steady_offset_mm'] == 50.0
    assert report['natural_frequency_rad_s'] is None
    assert any('first-order' in warning for warning in report['warnings'])


def test_pure_damping_has_drift_and_no_second_order_metrics():
    report = analyze_model(0.0, 10.0, 0.0)
    assert report['bias_drift_mm_s'] == 1.0
    assert report['natural_frequency_rad_s'] is None
    assert report['damping_ratio'] is None


def test_pure_stiffness_has_no_velocity_or_time_constant():
    report = analyze_model(0.0, 0.0, 20.0)
    assert report['velocity_time_constant_s'] is None
    assert report['nominal_velocity_mm_s'] is None
    assert report['steady_offset_mm'] == 5.0
    assert report['bias_drift_mm_s'] is None
    assert any('algebraic' in warning for warning in report['warnings'])


def test_undamped_mass_spring_is_not_presented_as_convergent():
    report = analyze_model(2.0, 0.0, 8.0)
    assert report['natural_frequency_rad_s'] == pytest.approx(2.0)
    assert report['damping_ratio'] == 0.0
    assert report['nominal_velocity_mm_s'] is None
    assert any('without decay' in warning for warning in report['warnings'])


def test_pure_mass_has_no_steady_offset_or_velocity():
    report = analyze_model(2.0, 0.0, 0.0)
    assert report['nominal_velocity_mm_s'] is None
    assert report['steady_offset_mm'] is None
    assert any('accelerates' in warning for warning in report['warnings'])


@pytest.mark.parametrize('values', (
    (0.0, 0.0, 0.0), (-1.0, 2.0, 3.0), (1.0, -2.0, 3.0),
    (1.0, 2.0, -3.0), (math.nan, 2.0, 3.0), (1.0, math.inf, 3.0),
))
def test_invalid_models_rejected(values):
    with pytest.raises(ValueError):
        analyze_model(*values)


@pytest.mark.parametrize('field', ('force_error_mn', 'displacement_mm', 'bias_mn'))
def test_nonfinite_analysis_inputs_rejected(field):
    with pytest.raises(ValueError):
        analyze_model(.15, 20.0, 30.0, **{field: math.nan})


def test_derived_overflow_is_rejected_instead_of_showing_infinite_candidate():
    with pytest.raises(ValueError):
        candidate(force_error_mn=1e308, correction_speed_mm_s=1e-308)
    with pytest.raises(ValueError):
        analyze_model(.15, 1e-308, 0.0, force_error_mn=1e308)


def test_warning_contract_and_no_closed_loop_guarantee():
    report = analyze_model(.15, 20.0, 30.0)
    assert isinstance(report['warnings'], tuple)
    assert all(isinstance(warning, str) for warning in report['warnings'])
    assert any('inner position-loop' in warning for warning in report['warnings'])
    assert any('not a motion limit' in warning for warning in report['warnings'])


def test_tiny_mass_warns_that_runtime_branch_differs_from_continuous_model():
    report = analyze_model(.0001, 20.0, 30.0)
    assert report['velocity_time_constant_s'] == pytest.approx(.000005)
    assert any('omit inertia' in warning for warning in report['warnings'])


def test_tiny_damping_and_spring_with_no_inertia_warn_about_runtime_branch():
    report = analyze_model(0.0, .0001, .0001)
    assert any('omit damping' in warning for warning in report['warnings'])
    assert any('omit the spring' in warning for warning in report['warnings'])


def test_small_damping_in_active_mass_branch_is_not_reported_as_omitted():
    report = analyze_model(.1, .0001, .0001)
    assert not any('omit damping' in warning for warning in report['warnings'])
    assert not any('omit the spring' in warning for warning in report['warnings'])


def test_overdamped_relaxation_can_be_slow_despite_short_velocity_time_constant():
    report = analyze_model(.15, 30.0, 5.0)
    assert report['velocity_time_constant_s'] == pytest.approx(.005)
    assert report['relaxation_time_s'] == pytest.approx(5.994995826, rel=1e-8)
    assert any('not the force-settling time' in warning for warning in report['warnings'])


def test_critical_damping_relaxation_is_inverse_natural_frequency():
    report = analyze_model(1.0, 4.0, 4.0)
    assert report['damping_ratio'] == pytest.approx(1.0)
    assert report['relaxation_time_s'] == pytest.approx(.5)


def test_underdamped_relaxation_is_envelope_not_oscillation_period():
    report = analyze_model(1.0, 2.0, 4.0)
    assert report['damping_ratio'] == pytest.approx(.5)
    assert report['relaxation_time_s'] == pytest.approx(1.0)


def test_first_order_spring_damper_relaxation_is_b_over_k():
    assert analyze_model(0.0, 30.0, 5.0)['relaxation_time_s'] == pytest.approx(6.0)


@pytest.mark.parametrize('gains', ((.15, 20.0, 0.0), (1.0, 0.0, 4.0), (0.0, 0.0, 4.0)))
def test_no_finite_dynamic_relaxation_for_no_spring_no_damping_or_pure_stiffness(gains):
    assert analyze_model(*gains)['relaxation_time_s'] is None


def test_overdamped_slow_root_avoids_catastrophic_cancellation():
    report = analyze_model(.001, 1e8, 1.0)
    assert report['relaxation_time_s'] == pytest.approx(1e8)
