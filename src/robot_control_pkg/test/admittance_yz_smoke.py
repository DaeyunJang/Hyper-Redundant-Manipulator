#!/usr/bin/env python3
"""Fake-feedback integration: domain 79, motor gate OFF, actuator topic remapped.

No TCP/camera/driver launches. Run after sourcing install/setup.bash with
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 and HRM_CONTROL_TEST_EXECUTABLE pointing
to the freshly built isolated binary. Never run on the hardware DDS domain.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from custom_interfaces.msg import (
    AdmittanceControl, ForceIdEstimate, LoadcellState, MotorCommand,
    MotorState, PositionControl, SegmentAngle,
)
from custom_interfaces.srv import SetGoalPosition
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '79', 'Use isolated domain 79'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'
    executable_setting = os.environ.get('HRM_CONTROL_TEST_EXECUTABLE')
    assert executable_setting, 'Explicit isolated-build HRM_CONTROL_TEST_EXECUTABLE required'
    executable = Path(executable_setting).expanduser().resolve(strict=True)
    assert executable.is_file() and os.access(executable, os.X_OK)
    started = time.monotonic()
    rclpy.init()
    node = Node('admittance_yz_smoke')
    child = None
    log_dir = Path(tempfile.mkdtemp(prefix='hrm_admittance_smoke_'))
    log = (log_dir / 'control.log').open('w')
    adm, pos, previews, commands, statuses = [], [], [], [], []
    qos = QoSProfile(depth=100)
    node.create_subscription(AdmittanceControl, '/admittance_controller', adm.append, qos)
    node.create_subscription(PositionControl, '/position_controller', pos.append, qos)
    node.create_subscription(MotorCommand, '/position/target_motor_command', previews.append, qos)
    node.create_subscription(MotorCommand, '/smoke/blocked_motor_command', commands.append, qos)
    node.create_subscription(String, '/position/control_status', lambda msg: statuses.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    motor_pub = node.create_publisher(MotorState, '/motor_state', 10)
    load_pub = node.create_publisher(LoadcellState, '/loadcell_state', 10)
    angle_pub = node.create_publisher(SegmentAngle, '/estimated_segment_angle', 10)
    force_pub = node.create_publisher(ForceIdEstimate, '/force_id_estimation', 10)
    legacy_pub = node.create_publisher(Vector3, '/estimated_external_force', 10)
    client = node.create_client(SetParametersAtomically, '/admittance_smoke_control/set_parameters_atomically')
    get_client = node.create_client(GetParameters, '/admittance_smoke_control/get_parameters')
    goal_client = node.create_client(SetGoalPosition, '/position/set_goal_position')
    origin = [10000, 20000, 30000, 40000]
    feed = dict(force=False, angle=True, motor=True, load=True, kind='valid',
                xyz=[0., 0., 0.], tension=10., body=False, contact_id=8)
    angle = SegmentAngle()
    angle.header.frame_id = 'hrm_base'
    for field in angle.get_fields_and_field_types():
        if field != 'header':
            setattr(angle, field, [0.0] * 18)
    next_angle = 0.0

    def send():
        nonlocal next_angle
        stamp = node.get_clock().now().to_msg()
        if feed['motor']:
            motor = MotorState()
            motor.header.stamp = stamp
            motor.actual_position = origin
            motor.actual_velocity = [0]*4
            motor.actual_acceleration = [0]*4
            motor.actual_torque = [0]*4
            motor_pub.publish(motor)
        if feed['load']:
            load = LoadcellState()
            load.header.stamp = stamp
            load.stress = [feed['tension']]*4
            load_pub.publish(load)
        if feed['angle'] and time.monotonic() >= next_angle:
            next_angle = time.monotonic() + 1/30
            angle.header.stamp = stamp
            angle_pub.publish(angle)
        legacy_pub.publish(Vector3(x=999., y=999., z=999.))  # Must never authorize/adulterate input.
        if feed['force']:
            force = ForceIdEstimate()
            force.header.stamp = stamp
            force.published_stamp = stamp
            force.header.frame_id = 'hrm_base'
            force.valid = True
            force.status = 'ready'
            force.model_name = 'synthetic_tip_model'
            force.mode = 'class0'
            force.force_scope = 'all_single_contact'
            force.load_state = force.LOADED
            force.contact_id = force.control_contact_id = 18
            force.tip_force_valid = True
            force.heatmap_center_id = 18
            force.window_samples = force.samples_in_window = 30
            force.body_contact_enabled = feed['body']
            force.probabilities[18] = 1.
            force.external_force.x, force.external_force.y, force.external_force.z = feed['xyz']
            kind = feed['kind']
            if kind == 'body':
                force.contact_id = force.control_contact_id = feed['contact_id']
                force.heatmap_center_id = feed['contact_id']
                force.probabilities[18] = 0.
                force.probabilities[feed['contact_id']] = 1.
                force.body_contact_detected = True
                force.tip_force_valid = False
            elif kind == 'unloaded':
                force.contact_id = 0
                force.load_state = force.UNLOADED
                force.probabilities[18] = 0.
                force.probabilities[0] = 1.
            elif kind == 'masked':
                force.mode = 'masked'
                force.status = 'ready_load_unknown'
                force.contact_id = force.heatmap_center_id = feed['contact_id']
                force.control_contact_id = -1
                force.load_state = force.LOAD_UNKNOWN
                force.tip_force_valid = False
                force.probabilities[18] = 0.
                force.probabilities[feed['contact_id']] = 1.
            elif kind == 'range_body':
                force.status = 'ready_force_ranges'
                force.contact_id = 0
                force.control_contact_id = force.heatmap_center_id = feed['contact_id']
                force.probabilities[18] = 0.
                force.probabilities[0] = .6
                force.probabilities[feed['contact_id']] = .4
                force.body_contact_detected = True
                force.tip_force_valid = False
            elif kind == 'range_unloaded':
                force.status = 'ready_force_ranges'
                force.mode = 'masked'
                force.contact_id = force.heatmap_center_id = feed['contact_id']
                force.load_state = force.UNLOADED
                force.probabilities[18] = 0.
                force.probabilities[feed['contact_id']] = 1.
            elif kind == 'invalid':
                force.valid = False
            elif kind in ('warming_up', 'input_time_skew',
                          'stale_input:/estimated_segment_angle',
                          'missing_or_invalid_input:/estimated_segment_angle',
                          'inference_failed:non-finite force'):
                force.valid = False
                force.status = kind
                force.header.stamp.sec = force.header.stamp.nanosec = 0
                force.samples_in_window = 0
                force.external_force.x = force.external_force.y = force.external_force.z = float('nan')
            elif kind == 'nan':
                force.external_force.z = float('nan')
            elif kind == 'stale_nan':
                force.header.stamp.sec -= 2
                force.external_force.z = float('nan')
            elif kind == 'frame':
                force.header.frame_id = 'camera'
            elif kind == 'stale':
                force.header.stamp.sec -= 2
            elif kind == 'changed_model':
                force.model_name = 'other_tip_model'
            elif kind == 'changed_model_warming_up':
                force.model_name = 'other_tip_model'
                force.valid = False
                force.status = 'warming_up'
            elif kind == 'unsupported_scope':
                force.force_scope = 'body_only'
            force_pub.publish(force)

    timer = node.create_timer(0.01, send)
    timer.cancel()  # Discover/check the private domain before sending fake feedback.

    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
            if child is not None:
                assert child.poll() is None, f'Controller exited; inspect {log_dir}'

    def params(**values):
        assert values.get('motor_output_enabled', False) is False, 'Never enable physical motor output'
        assert client.wait_for_service(timeout_sec=5.)
        req = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        future = client.call_async(req)
        until = time.monotonic()+3.
        while not future.done() and time.monotonic()<until:
            spin(0.01)
        assert future.done(), 'Parameter response timeout'
        return future.result().result

    def get_params(*names):
        assert get_client.wait_for_service(timeout_sec=5.)
        future = get_client.call_async(GetParameters.Request(names=list(names)))
        until = time.monotonic() + 3.
        while not future.done() and time.monotonic() < until:
            spin(.01)
        assert future.done() and future.exception() is None
        return future.result().values

    def reenter():
        assert params(control_mode=1).successful
        spin(0.08)
        feed.update(kind='valid', force=True, angle=True, motor=True, load=True, tension=10.)
        start = len(adm)
        assert params(control_mode=4).successful
        spin(0.25)
        assert adm[start].dt == 0.0
        assert adm[start].x.position.y == adm[start].x.position.z == 0.0
        assert statuses[-1].startswith('READY'), statuses[-1]

    def goal(y, z):
        assert goal_client.wait_for_service(timeout_sec=5.)
        request = SetGoalPosition.Request()
        request.reference_type = 'absolute'
        request.goal_position.position.y = y
        request.goal_position.position.z = z
        future = goal_client.call_async(request)
        until = time.monotonic() + 3.
        while not future.done() and time.monotonic() < until:
            spin(.01)
        assert future.done() and future.result().success, 'Fresh control must accept the nominal goal'

    def check_nominal_goal(first, y, z):
        # DDS may deliver the two diagnostics in either order. Freeze the
        # cohort before waiting, so ongoing feedback cannot move the target;
        # require every paired sample rather than ignoring an unmatched stamp.
        samples = list(adm[first:])
        assert samples, 'Expected admittance samples to pair with the nominal goal'
        required_stamps = {(message.header.stamp.sec, message.header.stamp.nanosec)
                           for message in samples}
        deadline = time.monotonic() + 3.0
        while True:
            by_stamp = {(message.header.stamp.sec, message.header.stamp.nanosec): message
                        for message in pos}
            missing = required_stamps - by_stamp.keys()
            if not missing:
                break
            assert time.monotonic() < deadline, f'Missing paired Position diagnostics: {missing}'
            spin(.01)
        for message in samples:
            position = by_stamp[(message.header.stamp.sec, message.header.stamp.nanosec)]
            assert abs(position.x_desired.position.y - message.x.position.y - y) < 1e-12
            assert abs(position.x_desired.position.z - message.x.position.z - z) < 1e-12

    def check_recovery(first, saved_offset, y, z):
        assert len(adm) > first and statuses[-1].startswith('READY'), statuses[-1]
        assert all(0. < message.dt <= .25 for message in adm[first:]), 'No re-entry or gap catch-up dt'
        recovered = adm[first]
        # Resume the retained offset from zero velocity using the restored
        # legacy update and its numerical substeps, NOT the old 2 mm/s bound.
        # Runtime matrices here are diagonal Y/Z; Frobenius norms of M^+ B
        # and M^+ K are the norms of these two nonzero diagonal entries.
        diagonal_indices = (7, 14)
        damping_rate = math.hypot(*(
            recovered.b_matrix[index] / recovered.m_matrix[index]
            for index in diagonal_indices))
        spring_rate = math.sqrt(math.hypot(*(
            recovered.k_matrix[index] / recovered.m_matrix[index]
            for index in diagonal_indices)))
        steps = max(1, math.ceil(recovered.dt * (damping_rate + spring_rate)))
        step_dt = recovered.dt / steps
        for axis, saved in zip(('y', 'z'), saved_offset):
            diagonal = {'y': 7, 'z': 14}[axis]  # column-major 6x6 matrix
            mass = recovered.m_matrix[diagonal]
            damping = recovered.b_matrix[diagonal]
            stiffness = recovered.k_matrix[diagonal]
            force_error = (getattr(recovered.env_force.force, axis) -
                           getattr(recovered.desired_force.force, axis))
            assert abs(getattr(recovered.delta_force.force, axis) - force_error) < 1e-12
            expected, velocity = saved, 0.0
            for _ in range(steps):
                acceleration = (force_error - damping * velocity - stiffness * expected) / mass
                velocity += acceleration * step_dt
                expected += velocity * step_dt
            assert abs(getattr(recovered.x.position, axis) - expected) < 1e-10
            assert abs(getattr(recovered.x_dot.position, axis) - velocity) < 1e-10
            assert abs(getattr(recovered.x_ddot.position, axis) - acceleration) < 1e-10
        check_nominal_goal(first, y, z)
        assert not commands, 'Dry-run leaked a motor command'

    try:
        spin(0.4)
        assert set(node.get_node_names()) <= {'admittance_yz_smoke'}, 'Domain 79 already in use'
        timer.reset()
        child = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:=admittance_smoke_control',
            '-r', 'motor_command:=/smoke/blocked_motor_command',
            '-p', 'control_mode:=4', '-p', 'motor_output_enabled:=false',
            '-p', 'position_control/feedback_timeout_sec:=1.0',
            # Stable, deterministic fake-data settings, independent of the
            # operator's source defaults. Never applied to the hardware node.
            '-p', 'admittance/M_d:=0.05', '-p', 'admittance/B_d:=1.0',
            '-p', 'admittance/K_d:=20.0',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        spin(1.)
        assert not adm and not previews, 'Legacy force cannot start admittance'
        assert '/force_id_estimation' in statuses[-1]
        gate, motor_output, force_sign = get_params(
            'admittance/use_contact_id_gate', 'motor_output_enabled', 'admittance/force_sign')
        assert not gate.bool_value and not motor_output.bool_value
        assert force_sign.double_value == -1.0
        feed['force']=True
        spin(0.7)
        assert statuses[-1].startswith('READY (dry-run)'), statuses[-1]
        assert 'ID display-only' in statuses[-1]
        assert list(previews[-1].target_position) == origin
        assert adm[-1].desired_force.force.y == adm[-1].desired_force.force.z == 0.
        feed['xyz']=[-1., 0., 0.]
        spin(0.3)
        assert adm[-1].env_force.force.x == 1.0  # N, sign once, no scale/axis swap.
        assert adm[-1].x.position.x == adm[-1].x.position.y == adm[-1].x.position.z == 0.
        assert list(previews[-1].target_position) == origin
        print('PASS: legacy ignored, initial hold, X observable but passive, N/sign mapping', flush=True)

        # Zero force and zero filter state leave a nonzero nominal target
        # unchanged through the actual Admittance -> Position path.
        feed['xyz'] = [0., 0., 0.]
        goal(-.01, 0.)
        spin(.1)
        first = len(adm)
        spin(.2)
        for message in adm[first:]:
            assert message.desired_force.force.y == message.desired_force.force.z == 0.
            assert message.env_force.force.y == message.env_force.force.z == 0.
            assert message.delta_force.force.y == message.delta_force.force.z == 0.
            assert message.x.position.y == message.x.position.z == 0.
            assert message.x_dot.position.y == message.x_dot.position.z == 0.
        check_nominal_goal(first, -.01, 0.)
        goal(0., 0.)
        spin(.1)
        print('PASS: zero target/input force and zero state preserve nominal Y=-0.01 m, Z=0', flush=True)

        # Each active axis sees both polarities. The node applies -1 once;
        # the wrapper maps the legacy convention to physical env-desired.
        for raw_force in ([0., .04, -.02], [0., -.04, .02]):
            feed['xyz'] = raw_force
            first = len(adm)
            spin(.5)
            for axis, incoming in zip(('y', 'z'), raw_force[1:]):
                assert getattr(adm[-1].desired_force.force, axis) == 0.
                assert abs(getattr(adm[-1].env_force.force, axis) + incoming) < 1e-12
                assert abs(getattr(adm[-1].delta_force.force, axis) + incoming) < 1e-12
                assert getattr(adm[-1].x.position, axis) * incoming < 0.
                assert (getattr(pos[-1].x_desired.position, axis) -
                        getattr(pos[-1].x_actual.position, axis)) * incoming < 0.
            check_nominal_goal(first, 0., 0.)
            assert list(previews[-1].target_position) != origin
        assert params(**{'admittance/target_force_y_N':0.04, 'admittance/target_force_z_N':-0.02}).successful
        spin(1.)
        assert adm[-1].delta_force.force.y == adm[-1].delta_force.force.z == 0.
        assert abs(adm[-1].x.position.y)<0.00005
        assert not params(**{'admittance/target_force_y_N':0.01,
                             'admittance/target_force_z_N':5.}).successful
        spin(0.1)
        assert adm[-1].desired_force.force.y == 0.04  # Atomic rejection keeps both old targets.
        assert not params(**{'admittance/M_d':0.1}).successful
        assert params(**{'admittance/target_force_y_N':0., 'admittance/target_force_z_N':0.}).successful
        print('PASS: +/-Y/Z, one input sign, env-desired delta, nominal+offset, cable previews, targets', flush=True)

        # The restored filter must not silently clip offsets at 5 mm or their
        # velocities at 2 mm/s. Only synthetic inputs, blocked motor publisher.
        first = len(adm)
        feed['xyz'] = [0., -.16, .12]
        spin(.8)
        samples = adm[first:]
        assert adm[-1].x.position.y > .005
        assert adm[-1].x.position.z < -.005
        assert abs(adm[-1].x.position.y - .008) < .0001  # (env-desired)/K
        assert abs(adm[-1].x.position.z + .006) < .0001
        assert max(abs(message.x_dot.position.y) for message in samples) > .002
        assert max(abs(message.x_dot.position.z) for message in samples) > .002
        assert 'offset/velocity limited' not in statuses[-1]
        assert not commands, 'Unclipped admittance test leaked a motor command'
        print('PASS: offsets exceed 5 mm and velocities exceed 2 mm/s without old clipping', flush=True)

        # Releasing the force keeps the existing spring/damper state; it must
        # decay dynamically instead of resetting the offset to zero.
        first = len(adm)
        feed['xyz'] = [0., 0., 0.]
        spin(.12)
        released = [message for message in adm[first:]
                    if message.env_force.force.y == message.env_force.force.z == 0.]
        assert released, 'Expected a fresh zero-force sample after release'
        assert released[0].desired_force.force.y == released[0].desired_force.force.z == 0.
        assert released[0].delta_force.force.y == released[0].delta_force.force.z == 0.
        assert released[0].x.position.y > .001 and released[0].x.position.z < -.001
        print('PASS: zero forces retain and evolve a nonzero admittance state', flush=True)

        # A slow/missing image must not integrate a long catch-up interval,
        # including gaps shorter than the configured one-second freshness limit.
        feed['xyz']=[0., 0., 0.]
        goal(.001, -.001)
        spin(0.4)
        for gap in (.4, 1.15):
            feed['angle']=False
            spin(gap)
            count=len(adm)
            saved_offset=(adm[-1].x.position.y, adm[-1].x.position.z)
            feed['angle']=True
            spin(.2)
            check_recovery(count, saved_offset, .001, -.001)
        print('PASS: camera gap freezes velocity, auto-resumes without catch-up', flush=True)

        # Invalid producer messages carry NaNs and zero source stamps. They
        # pause the whole controller; recovery consumes a fresh valid sample.
        feed['xyz']=[0., -.01, .005]
        spin(.25)
        for kind in ('warming_up', 'input_time_skew',
                     'stale_input:/estimated_segment_angle',
                     'missing_or_invalid_input:/estimated_segment_angle', 'stale', 'outage'):
            feed.update(kind=kind, force=kind != 'outage')
            spin(.38)
            assert statuses[-1].startswith('WAITING') and 'auto-resume' in statuses[-1], (kind, statuses[-1])
            count=len(adm)
            preview_count=len(previews)
            saved_offset=(adm[-1].x.position.y, adm[-1].x.position.z)
            spin(.15)
            assert len(adm) == count and len(previews) == preview_count, 'Paused force must not drive control'
            feed.update(kind='valid', force=True)
            spin(.2)
            check_recovery(count, saved_offset, .001, -.001)
            assert adm[count].env_force.force.y == .01
            assert adm[count].env_force.force.z == -.005
        print('PASS: force warm-up/stale/outage pauses retain goal and offset, then resume with fresh force', flush=True)

        # Default: localization is a diagnostic only, not a force-control gate.
        # This includes every body ID and raw class0 + positive operating ID.
        feed['xyz'] = [0., -.01, .005]
        for contact_id in range(1, 18):
            feed.update(kind='body', contact_id=contact_id)
            count = len(adm)
            spin(.08)
            assert len(adm) > count and statuses[-1].startswith('READY'), (contact_id, statuses[-1])
            assert adm[-1].env_force.force.y == .01
            assert adm[-1].env_force.force.z == -.005
        for kind in ('masked', 'unloaded', 'range_body', 'range_unloaded'):
            feed.update(kind=kind, body=True)
            count = len(adm)
            spin(.12)
            assert len(adm) > count and statuses[-1].startswith('READY'), (kind, statuses[-1])
            assert adm[-1].env_force.force.y == .01, 'Do not replace unloaded force with zero'
        rejected = params(**{'admittance/use_contact_id_gate': True})
        assert not rejected.successful, 'Do not change the contact policy while Admittance is active'
        assert not get_params('admittance/use_contact_id_gate')[0].bool_value
        print('PASS: default ID-independent control for body IDs 1..17, unloaded, unknown and range policy', flush=True)

        # Optional old ID gate remains available, but only changes outside mode4.
        assert params(control_mode=1).successful
        assert params(**{'admittance/use_contact_id_gate': True}).successful
        reenter()
        for kind in ('body', 'masked', 'range_body'):
            feed['kind'] = kind
            spin(.18)
            assert statuses[-1].startswith('FAULT'), (kind, statuses[-1])
            count = len(adm)
            feed['kind'] = 'valid'
            spin(.15)
            assert len(adm) == count, 'An enabled ID gate still requires manual mode re-entry'
            reenter()
        feed['kind'] = 'range_unloaded'
        spin(.15)
        assert statuses[-1].startswith('READY')
        assert not params(**{'admittance/use_contact_id_gate': False}).successful
        assert params(control_mode=1).successful
        assert params(**{'admittance/use_contact_id_gate': False}).successful
        reenter()
        print('PASS: optional ID gate protects tip-only operation; live policy changes rejected', flush=True)

        # ID-independent operation does not weaken measurement/safety guards.
        for kind in ('invalid', 'inference_failed:non-finite force', 'nan', 'stale_nan',
                     'frame', 'changed_model', 'changed_model_warming_up', 'unsupported_scope'):
            feed['kind']=kind
            spin(0.18)
            assert statuses[-1].startswith('FAULT'), (kind,statuses[-1])
            assert list(previews[-1].target_position) == origin
            count=len(adm)
            feed['kind']='valid'
            spin(0.15)
            assert len(adm)==count, 'Do not auto-resume a force validity fault'
            reenter()
        feed['kind']='unloaded'
        feed['xyz']=[0., -0.01, 0.]
        spin(0.2)
        assert adm[-1].env_force.force.y==0.01  # No artificial 0 N for unloaded class.
        feed['xyz']=[0., 3., 0.]
        spin(0.2)
        assert 'max_force_N' in statuses[-1]
        feed['xyz']=[0., 0., 0.]
        reenter()
        feed['tension']=2500.
        spin(0.2)
        assert 'tension' in statuses[-1]
        reenter()
        for source in ('motor', 'load'):
            feed[source]=False
            spin(1.15)
            assert statuses[-1].startswith('FAULT'), (source, statuses[-1])
            count=len(adm)
            feed[source]=True
            spin(.2)
            assert len(adm) == count, 'Hardware feedback failure must remain latched'
            reenter()
        assert not commands, 'Dry-run leaked a motor command'
        gate, motor_output = get_params('admittance/use_contact_id_gate', 'motor_output_enabled')
        assert not gate.bool_value and not motor_output.bool_value
        print('PASS: invalid/NaN/frame/model/scope/overforce/tension and hardware feedback faults remain latched', flush=True)
        print(json.dumps(dict(
            result='PASS', duration_sec=round(time.monotonic()-started, 2),
            admittance_messages=len(adm), position_messages=len(pos),
            actuator_messages=len(commands), motor_output_enabled=False,
            default_contact_id_gate=False, log_dir=str(log_dir), executable=str(executable),
        )), flush=True)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3.)
        log.close()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
