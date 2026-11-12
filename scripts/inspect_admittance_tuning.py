#!/usr/bin/env python3
# AUTHOR: Daeyun - HRM source-time-paired control diagnostics.
"""Read-only, source-stamp-paired admittance tuning measurements.

No publishers, services, parameter writes or robot motion commands are created.
Force values are controller-side force ON the robot, after force_sign conversion.
The force error is environment - target, as published in delta_force;
the virtual model balances that signed error against M*x_ddot + B*x_dot + K*x.
This measures estimator-based tracking, not independent force-sensor accuracy.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time


def stamp(value):
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def sample_row(admittance, position, reference, axis):
    if axis not in ('y', 'z'):
        raise ValueError('Expected y or z')
    source = stamp(admittance.header.stamp)
    if source <= 0 or source != stamp(position.header.stamp) or source != stamp(reference.feedback_stamp):
        raise ValueError('Diagnostics must share the same positive source-image timestamp')
    if reference.control_mode.lower() != 'admittance' or not reference.cartesian_reference_valid:
        raise ValueError('Not an Admittance Cartesian reference')
    if any(msg.header.frame_id != 'hrm_base' for msg in (admittance, position, reference)):
        raise ValueError('Expected hrm_base diagnostics')
    i = 1 if axis == 'y' else 2
    get = lambda value: getattr(value, axis)
    if any(len(matrix) != 36 for matrix in (
            admittance.m_matrix, admittance.b_matrix, admittance.k_matrix)):
        raise ValueError('Expected 6x6 M/B/K')
    if any(not math.isfinite(value) for matrix in (
            admittance.m_matrix, admittance.b_matrix, admittance.k_matrix) for value in matrix):
        raise ValueError('Non-finite M/B/K matrix')
    # The live node configures diagonal Y/Z matrices; reject cross coupling
    # rather than misreport a diagonal term as the full matrix product.
    if any(abs(matrix[i + 6*j]) > 1.e-12 for matrix in (
            admittance.m_matrix, admittance.b_matrix, admittance.k_matrix)
           for j in range(6) if j != i):
        raise ValueError('This diagnostic summary requires uncoupled matrix rows')
    m, b, k = (matrix[i + 6*i] for matrix in (
        admittance.m_matrix, admittance.b_matrix, admittance.k_matrix))
    row = dict(source_time_ns=source, axis=axis, enabled=int(reference.motor_output_enabled),
               dt_s=admittance.dt, m=m, b=b, k=k,
               target_mn=get(admittance.desired_force.force)*1000,
               environment_mn=get(admittance.env_force.force)*1000,
               # Keep the published filter RHS; force_sign is already applied.
               force_error_mn=get(admittance.delta_force.force)*1000,
               offset_mm=get(admittance.x.position)*1000,
               offset_velocity_mm_s=get(admittance.x_dot.position)*1000,
               spring_mn=k*get(admittance.x.position)*1000,
               damping_mn=b*get(admittance.x_dot.position)*1000,
               inertia_mn=m*get(admittance.x_ddot.position)*1000,
               nominal_mm=get(reference.nominal_position)*1000,
               other_target_mn=getattr(admittance.desired_force.force, 'z' if axis == 'y' else 'y')*1000,
               other_nominal_mm=getattr(reference.nominal_position, 'z' if axis == 'y' else 'y')*1000,
               pan_pd_p=position.p_gain, pan_pd_d=position.d_gain,
               effective_mm=get(reference.target_position)*1000,
               actual_mm=get(position.x_actual.position)*1000,
               position_error_mm=get(position.x_error.position)*1000,
               command_angle_deg=math.degrees(
                   reference.tilt_rad if axis == 'y' else reference.pan_rad))
    if not all(math.isfinite(value) for key, value in row.items() if key != 'axis'):
        raise ValueError('Non-finite diagnostic sample')
    if not 0 <= row['dt_s'] <= .25 or min(m, b, k) < 0:
        raise ValueError('Invalid control interval or M/B/K')
    if (abs(row['effective_mm'] - get(position.x_desired.position)*1000) > 1.e-5
            or abs(row['effective_mm'] - row['actual_mm'] - row['position_error_mm']) > 1.e-5):
        raise ValueError('Position/reference mismatch')
    if abs(row['effective_mm'] - row['offset_mm'] - row['nominal_mm']) > 1.e-5:
        raise ValueError('Reference/offset mismatch')
    return row


def summarize(rows, force_tolerance_mn=20., position_tolerance_mm=.5):
    """Separate target/gain/nominal/gate changes and gaps; never pool different trials."""
    if any(not math.isfinite(v) or v <= 0 for v in (force_tolerance_mn, position_tolerance_mm)):
        raise ValueError('Positive finite diagnostic tolerances required')
    reports = []
    signature_keys = ('target_mn', 'other_target_mn', 'm', 'b', 'k', 'nominal_mm',
                      'other_nominal_mm', 'pan_pd_p', 'pan_pd_d', 'enabled')
    for axis in ('y', 'z'):
        groups = []
        previous = None
        for row in sorted((r for r in rows if r['axis'] == axis), key=lambda r: r['source_time_ns']):
            signature = tuple(row[key] for key in signature_keys)
            gap = (row['source_time_ns'] - previous['source_time_ns']) / 1.e9 if previous else 0
            if (previous is None or signature != tuple(previous[key] for key in signature_keys)
                    or row['dt_s'] == 0 or gap > .25 or gap <= 0):
                groups.append([])
            if row['dt_s'] > 0:
                groups[-1].append(row)
            previous = row
        for group in groups:
            if not group:
                continue
            last = group[-1]
            tail = [r for r in group if last['source_time_ns'] - r['source_time_ns'] <= 2.e9]
            mean = lambda key: statistics.mean(r[key] for r in tail)
            rms = lambda key: math.sqrt(statistics.mean(r[key]**2 for r in tail))
            duration = (last['source_time_ns'] - group[0]['source_time_ns']) / 1.e9
            tail_duration = (last['source_time_ns'] - tail[0]['source_time_ns']) / 1.e9
            enough = tail_duration >= 1.0 and len(tail) >= 10
            hints = []
            if not last['enabled']:
                hints.append('OUTPUT_DISABLED: not a physical tracking trial')
            if not enough:
                hints.append('SHORT_WINDOW: keep conditions fixed longer; no convergence conclusion')
            else:
                if last['enabled'] and all(abs(r['force_error_mn']) <= force_tolerance_mn for r in tail):
                    hints.append('IN_BAND: entire observed tail within force tolerance (not proof of stability)')
                if rms('position_error_mm') > position_tolerance_mm:
                    hints.append('POSITION_LAG: inner loop is not following effective target closely')
                elif (last['k'] > 0 and abs(mean('force_error_mn')) > force_tolerance_mn
                      and rms('offset_velocity_mm_s') < .5
                      and abs(mean('force_error_mn') - mean('spring_mn'))
                      <= max(force_tolerance_mn, .2*abs(mean('force_error_mn')))):
                    hints.append('SPRING_BALANCE: consistent with K*offset retaining force error')
                if last['k'] == 0 and abs(mean('force_error_mn')) > force_tolerance_mn:
                    hints.append('K_ZERO_ERROR: residual force can continue displacement; check bias/contact/lag')
            if last['target_mn'] == 0 and rms('offset_mm') > position_tolerance_mm:
                hints.append('ZERO_TARGET_IS_ACTIVE: zero force target does not disable this axis')
            reports.append(dict(axis=axis, source_start_ns=group[0]['source_time_ns'],
                source_end_ns=last['source_time_ns'], duration_s=duration, samples=len(group),
                tail_duration_s=tail_duration, target_mn=last['target_mn'],
                m=last['m'], b=last['b'], k=last['k'], enabled=bool(last['enabled']),
                environment_mean_mn=mean('environment_mn'), force_error_mean_mn=mean('force_error_mn'),
                force_error_rmse_mn=rms('force_error_mn'), spring_mean_mn=mean('spring_mn'),
                damping_mean_mn=mean('damping_mn'), inertia_mean_mn=mean('inertia_mn'),
                offset_mean_mm=mean('offset_mm'), offset_velocity_rms_mm_s=rms('offset_velocity_mm_s'),
                position_error_rmse_mm=rms('position_error_mm'),
                command_angle_peak_deg=max(abs(r['command_angle_deg']) for r in group), hints=hints))
    return reports


def collect(duration):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from custom_interfaces.msg import AdmittanceControl, PositionControl, ControlReference

    rclpy.init(args=[])
    node = rclpy.create_node('admittance_tuning_readonly')
    streams = {'admittance': {}, 'position': {}, 'reference': {}}
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = []
    try:
        for key, topic, message_type in (
                ('admittance', '/admittance_controller', AdmittanceControl),
                ('position', '/position_controller', PositionControl),
                ('reference', '/control/reference', ControlReference)):
            def receive(message, key=key):
                value = message.feedback_stamp if key == 'reference' else message.header.stamp
                streams[key][stamp(value)] = message
            subscriptions.append(node.create_subscription(message_type, topic, receive, qos))
        until = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass  # Partial report only; no cleanup command or parameter changes.
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    common = set.intersection(*(set(values) for values in streams.values()))
    rows, invalid = [], 0
    for source in sorted(common):
        for axis in ('y', 'z'):
            try:
                rows.append(sample_row(*(streams[key][source] for key in streams), axis))
            except (ValueError, AttributeError, TypeError, OverflowError):
                invalid += 1
    return rows, dict(received_unique={key: len(values) for key, values in streams.items()},
                      paired_frames=len(common), skipped_axis_rows=invalid,
                      unmatched_unique={key: len(values)-len(common) for key, values in streams.items()})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15., help='Read-only capture seconds (0..300).')
    parser.add_argument('--force-tolerance-mn', type=float, default=20., help='Analysis threshold only.')
    parser.add_argument('--position-tolerance-mm', type=float, default=.5, help='Analysis threshold only.')
    parser.add_argument('--csv', type=Path, help='Optional new sample CSV; never overwrites an existing file.')
    args = parser.parse_args(argv)
    for value in (args.duration, args.force_tolerance_mn, args.position_tolerance_mm):
        if not math.isfinite(value) or value <= 0:
            parser.error('Duration/tolerances must be finite and positive')
    if args.duration > 300:
        parser.error('Capture at most 300 seconds at a time')
    if args.csv and (args.csv.exists() or not args.csv.parent.is_dir()):
        parser.error('CSV must be a NEW file in an existing directory')
    print('Read-only capture: do not change gains/targets just for this tool. '
          'Unexpected motion: stop via GUI/hardware, not this diagnostic.', flush=True)
    rows, counts = collect(args.duration)
    print(json.dumps(counts, ensure_ascii=False))
    reports = summarize(rows, args.force_tolerance_mn, args.position_tolerance_mm)
    if not reports:
        print('No valid active Admittance samples; no convergence claim. Do not enable motors just to fill this report.')
        return 1
    if args.csv:
        with args.csv.open('x', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f'Samples saved: {args.csv}')
    print('Per fixed-setting interval; statistics use last <=2s. '
          'Estimator feedback only, NOT ground-truth force or full settling-time validation.')
    print(json.dumps(reports, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
