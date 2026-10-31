"""Unit-aware, offline calculations for inspecting virtual admittance gains.

The scalar model is M*x_ddot + B*x_dot + K*x = force_error, where
force_error = environment - target (after force_sign).
These calculations are only a candidate-selection aid: they do not include the
inner position controller, contact stiffness, delays, estimator errors, or cross-axis coupling.
They cannot establish closed-loop stability or tune a connected robot.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Candidate:
    """Candidate gains in kg, N*s/m, and N/m, respectively."""

    mass: float
    damping: float
    stiffness: float


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be a finite number.') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number.')
    return number


def _nonnegative(value, name):
    number = _finite(value, name)
    if number < 0.0:
        raise ValueError(f'{name} must be nonnegative.')
    return number


def _positive(value, name):
    number = _finite(value, name)
    if number <= 0.0:
        raise ValueError(f'{name} must be positive.')
    return number


def propose_gains(*, mode, force_error_mn, correction_speed_mm_s,
                  velocity_time_constant_s, spring_error_budget_mn,
                  correction_distance_mm):
    """Calculate a candidate; never apply it or promise a convergence rate.

    ``B = force_error / speed`` uses mN and mm/s, giving N*s/m directly.
    It assumes negligible inertia and spring force, and is NOT a speed limit.
    ``M = B*tau`` uses the *velocity* time constant M/B, not force-settling time.

    ``force`` sets K=0, so a persistent force bias can cause unbounded drift.
    ``compliance`` chooses K from an allowed spring force at a chosen offset:
    ``K = spring_error_budget_mn / correction_distance_mm``. This is a static
    design allowance, not a bound on actual tracking error or displacement.
    """
    if mode not in ('force', 'compliance'):
        raise ValueError("mode must be 'force' or 'compliance'.")
    force = _positive(force_error_mn, 'force_error_mn')
    speed = _positive(correction_speed_mm_s, 'correction_speed_mm_s')
    tau = _positive(velocity_time_constant_s, 'velocity_time_constant_s')
    budget = _nonnegative(spring_error_budget_mn, 'spring_error_budget_mn')
    distance = _positive(correction_distance_mm, 'correction_distance_mm')
    if mode == 'compliance' and budget == 0.0:
        raise ValueError('Compliance mode requires a positive spring error budget.')

    damping = _positive(force / speed, 'derived damping')
    mass = _positive(damping * tau, 'derived mass')
    stiffness = (0.0 if mode == 'force' else
                 _positive(budget / distance, 'derived stiffness'))
    return Candidate(mass, damping, stiffness)


def analyze_model(mass, damping, stiffness, *, force_error_mn=100.0,
                  displacement_mm=5.0, bias_mn=10.0):
    """Return model-only metrics; absent/undefined metrics are ``None``.

    Signed force error, displacement, and bias retain their signs in the result.
    ``steady_offset_mm`` is a formal static balance, not a guarantee that an
    equilibrium is reachable or stable. ``nominal_velocity_mm_s`` ignores the
    spring and inertia terms. No metric is a software or hardware motion limit.
    ``relaxation_time_s`` is the slow-pole (or underdamped-envelope) time constant
    of this free scalar virtual model, NOT settling time during robot contact.
    """
    mass = _nonnegative(mass, 'mass')
    damping = _nonnegative(damping, 'damping')
    stiffness = _nonnegative(stiffness, 'stiffness')
    if not any(value > 0.0 for value in (mass, damping, stiffness)):
        raise ValueError('M/B/K must not all be zero.')
    force = _finite(force_error_mn, 'force_error_mn')
    displacement = _finite(displacement_mm, 'displacement_mm')
    bias = _finite(bias_mn, 'bias_mn')

    natural_frequency = damping_ratio = None
    if mass > 0.0 and stiffness > 0.0:
        # Separate square roots avoid overflow in the intermediate M*K product.
        natural_frequency = math.sqrt(stiffness) / math.sqrt(mass)
        damping_ratio = (damping / math.sqrt(mass)) / math.sqrt(stiffness) / 2.0

    relaxation_time = None
    if stiffness > 0.0 and damping > 0.0:
        if mass == 0.0:
            relaxation_time = damping / stiffness
        elif damping_ratio >= 1.0:
            # Slow root is -2*K/(B + sqrt(B**2 - 4*M*K)). Factoring B
            # avoids catastrophic cancellation of the two nearly equal roots,
            # and avoids forming either B**2 or M*K as an intermediate.
            inverse_ratio = 1.0 / damping_ratio
            relaxation_time = ((damping / stiffness) * .5 *
                               (1.0 + math.sqrt(1.0 - inverse_ratio * inverse_ratio)))
        else:
            relaxation_time = (mass / damping) * 2.0

    result = {
        'velocity_time_constant_s': mass / damping if damping > 0.0 else None,
        'nominal_velocity_mm_s': force / damping if damping > 0.0 else None,
        'spring_error_mn': stiffness * displacement,
        'steady_offset_mm': force / stiffness if stiffness > 0.0 else None,
        'bias_drift_mm_s': bias / damping if stiffness == 0.0 and damping > 0.0 else None,
        'natural_frequency_rad_s': natural_frequency,
        'damping_ratio': damping_ratio,
        'relaxation_time_s': relaxation_time,
    }
    for name, value in result.items():
        if value is not None:
            _finite(value, name)

    warnings = [
        'Model-only estimates: inner position-loop dynamics, contact stiffness, '
        'delay, force bias, and axis coupling can prevent stable force convergence.',
        'Force error / B is an inertia- and spring-neglecting velocity estimate, '
        'not a motion limit or a guaranteed convergence speed.',
        'Relaxation time describes only the free virtual scalar model, not '
        'the force-settling time of the robot in contact.',
    ]
    # The GUI sets equal Y/Z diagonal entries in otherwise zero 6x6 matrices.
    # This is only a diagnostic for that configuration, not an emulation of the
    # matrix solver, its branch selection, or its adaptive integration substeps.
    branch_threshold = 1e-3 / math.sqrt(2.0)
    if 0.0 < mass <= branch_threshold:
        warnings.append(
            'This M is below the controller matrix-norm branch threshold '
            '(1e-3 for equal Y/Z diagonals). Runtime may omit inertia; '
            'the continuous-M metrics above do not describe that branch.')
    if mass <= branch_threshold and 0.0 < damping <= branch_threshold:
        warnings.append(
            'This B is below the controller matrix-norm branch threshold '
            '(1e-3 for equal Y/Z diagonals). With inactive inertia, runtime '
            'may omit damping; ideal B-based metrics then do not apply.')
    if mass <= branch_threshold and 0.0 < stiffness <= branch_threshold:
        warnings.append(
            'This K is below the controller matrix-norm branch threshold '
            '(1e-3 for equal Y/Z diagonals). With inactive inertia, runtime '
            'may omit the spring; the ideal static offset then does not apply.')
    if stiffness > 0.0:
        warnings.append(
            'K > 0 permits residual force error K*offset at static balance. '
            'A very small K can demand an unreachable offset; equilibrium is not guaranteed.')
    else:
        warnings.append(
            'K = 0 has no positional restoring spring. Persistent target-force '
            'error or sensor bias can cause unbounded displacement without contact.')
    if damping == 0.0 and mass > 0.0:
        warnings.append(
            'B = 0 provides no virtual damping. Pure inertia accelerates under '
            'constant force; a mass-spring system can oscillate without decay.')
    if mass == 0.0 and damping == 0.0:
        warnings.append(
            'Pure stiffness is an algebraic offset force_error/K, not a dynamic '
            'filter; it provides no velocity or acceleration smoothing.')
    elif mass == 0.0:
        warnings.append(
            'M = 0 uses a first-order damping model; M/B = 0 is not a force-settling time.')
    result['warnings'] = tuple(warnings)
    return result
