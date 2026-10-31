#!/usr/bin/env python3
"""Exercise recording telemetry with synthetic feedback, never live hardware.

Require ROS_DOMAIN_ID=84, ROS_LOCALHOST_ONLY=1 and an explicit
HRM_CONTROL_TEST_EXECUTABLE from an isolated build outside the workspace.
No driver, TCP, camera or GUI is launched. The motor gate remains false and
the actuator topic is also remapped to a private test-only topic.
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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from custom_interfaces.msg import (
    AdmittanceControl, ControlReference, ForceIdEstimate, LoadcellState,
    MotorCommand, MotorState, PositionControl, SegmentAngle,
)
from custom_interfaces.srv import MoveToolAngle, SetGoalPosition
from geometry_msgs.msg import PointStamped
from rcl_interfaces.srv import GetParameters, SetParametersAtomically


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def xyz(point):
    return point.x, point.y, point.z


def close_xyz(left, right):
    return all(abs(a - b) < 1e-9 for a, b in zip(left, right))


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '84', 'Use private DDS domain 84'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'
    configured_executable = os.environ.get('HRM_CONTROL_TEST_EXECUTABLE')
    assert configured_executable, 'Explicit isolated-build executable required'
    executable = Path(configured_executable).expanduser().resolve(strict=True)
    workspace = Path(__file__).resolve().parents[3]
    assert not executable.is_relative_to(workspace), 'Do not run workspace binaries'
    assert executable.is_file() and os.access(executable, os.X_OK)

    rclpy.init()
    node = Node('control_reference_smoke')
    log_dir = Path(tempfile.mkdtemp(prefix='hrm_control_reference_smoke_'))
    log = (log_dir / 'control.log').open('w')
    child = None
    references, positions, admittance, fk_positions, actuator_commands = [], [], [], [], []
    node.create_subscription(ControlReference, '/control/reference', references.append, 100)
    node.create_subscription(PositionControl, '/position_controller', positions.append, 100)
    node.create_subscription(AdmittanceControl, '/admittance_controller', admittance.append, 100)
    node.create_subscription(PointStamped, '/kinematics/fk_tip_position', fk_positions.append, 100)
    node.create_subscription(MotorCommand, '/reference_smoke/blocked_motor_command',
                             actuator_commands.append, 100)
    publishers = {
        'motor': node.create_publisher(MotorState, '/motor_state', 10),
        'load': node.create_publisher(LoadcellState, '/loadcell_state', 10),
        'angle': node.create_publisher(SegmentAngle, '/estimated_segment_angle', 10),
        'force': node.create_publisher(ForceIdEstimate, '/force_id_estimation', 10),
    }
    params_client = node.create_client(
        SetParametersAtomically, '/reference_smoke_control/set_parameters_atomically')
    read_client = node.create_client(GetParameters, '/reference_smoke_control/get_parameters')
    ik_client = node.create_client(MoveToolAngle, '/kinematics/move_tool_angle')
    goal_client = node.create_client(SetGoalPosition, '/position/set_goal_position')
    feed = dict(enabled=False, angle=True, force=False)
    image_stamps = set()
    angle = SegmentAngle()
    angle.header.frame_id = 'hrm_base'
    for field in angle.get_fields_and_field_types():
        if field != 'header':
            setattr(angle, field, [0.0] * 18)
    angle.pan_relative = [-0.01 if i % 2 else 0.0 for i in range(18)]
    angle.tilt_relative = [0.02 if i % 2 == 0 else 0.0 for i in range(18)]
    next_image = 0.0

    def send_feedback():
        nonlocal next_image
        if not feed['enabled']:
            return
        now = node.get_clock().now()
        motor = MotorState()
        motor.header.stamp = now.to_msg()
        motor.actual_position = [10000, 20000, 30000, 40000]
        motor.actual_velocity = motor.actual_acceleration = motor.actual_torque = [0] * 4
        publishers['motor'].publish(motor)
        load = LoadcellState()
        load.header.stamp = now.to_msg()
        load.stress = [10.0] * 4
        publishers['load'].publish(load)
        if feed['angle'] and time.monotonic() >= next_image:
            next_image = time.monotonic() + 0.05
            # A deliberate 30 ms source latency proves the two clocks differ.
            angle.header.stamp = (now - Duration(seconds=0.03)).to_msg()
            image_stamps.add(ns(angle.header.stamp))
            publishers['angle'].publish(angle)
        if feed['force']:
            force = ForceIdEstimate()
            force.header.stamp = force.published_stamp = now.to_msg()
            force.header.frame_id = 'hrm_base'
            force.valid = force.tip_force_valid = True
            force.status = 'ready'
            force.model_name = 'synthetic_reference_test'
            force.mode = 'class0'
            force.force_scope = 'tip'
            force.load_state = force.LOADED
            force.contact_id = force.control_contact_id = 18
            force.window_samples = force.samples_in_window = 30
            force.probabilities[18] = 1.0
            force.external_force.y = -0.02
            force.external_force.z = 0.01
            publishers['force'].publish(force)

    timer = node.create_timer(0.01, send_feedback)
    timer.cancel()

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if child is not None:
                assert child.poll() is None, f'Controller exited; inspect {log_dir}'
            assert not actuator_commands, 'Motor-disabled smoke emitted an actuator command'

    def until(predicate, detail, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin(0.02)
        assert predicate(), f'{detail}; inspect {log_dir}'

    def call(client, request):
        until(client.service_is_ready, 'Service unavailable')
        future = client.call_async(request)
        until(future.done, 'Service response timed out')
        return future.result()

    def mode(value):
        result = call(params_client, SetParametersAtomically.Request(parameters=[
            Parameter('control_mode', value=value).to_parameter_msg()])).result
        assert result.successful, result.reason

    def assert_gate_disabled():
        result = call(read_client, GetParameters.Request(names=['motor_output_enabled']))
        assert len(result.values) == 1 and result.values[0].bool_value is False

    def check_common(reference):
        assert reference.header.frame_id == 'hrm_base'
        assert ns(reference.header.stamp) > 0
        assert not reference.motor_output_enabled
        assert math.isfinite(reference.pan_rad) and math.isfinite(reference.tilt_rad)

    def verify_feedback_references(mode_name):
        diagnostics = {ns(msg.header.stamp): msg for msg in positions}
        forces = {ns(msg.header.stamp): msg for msg in admittance}
        tips = {ns(msg.header.stamp): msg for msg in fk_positions}
        matching = [msg for msg in references if msg.control_mode == mode_name]
        assert len(matching) >= 5
        stamps = [ns(msg.feedback_stamp) for msg in matching]
        assert len(stamps) == len(set(stamps)), 'Repeated reference for one accepted image'
        for reference in matching:
            check_common(reference)
            source_ns = ns(reference.feedback_stamp)
            assert source_ns in image_stamps
            latency = (ns(reference.header.stamp) - source_ns) * 1e-9
            assert 0.02 <= latency < 0.5, latency
            assert reference.cartesian_reference_valid
            assert reference.pan_rad < 0.0 < reference.tilt_rad
            assert abs(reference.pan_rad) < 1.0 and abs(reference.tilt_rad) < 1.0
            diagnostic, tip = diagnostics[source_ns], tips[source_ns]
            assert tip.header.frame_id == 'hrm_base'
            assert close_xyz(xyz(tip.point), xyz(diagnostic.x_actual.position))
            assert 0.01 < math.sqrt(sum(v*v for v in xyz(tip.point))) < 0.2
            assert close_xyz(xyz(reference.target_position), xyz(diagnostic.x_desired.position))
            if mode_name == 'position':
                assert close_xyz(xyz(reference.nominal_position), xyz(reference.target_position))
            else:
                expected = tuple(a + b for a, b in zip(
                    xyz(reference.nominal_position), xyz(forces[source_ns].x.position)))
                assert close_xyz(xyz(reference.target_position), expected)
        for previous, current in zip(matching, matching[1:]):
            diagnostic = diagnostics[ns(current.feedback_stamp)]
            assert abs(current.pan_rad - previous.pan_rad - diagnostic.del_theta_pan) < 1e-8
            assert abs(current.tilt_rad - previous.tilt_rad - diagnostic.del_theta_tilt) < 1e-8
        return matching

    try:
        spin(0.7)
        assert set(node.get_node_names()) <= {'control_reference_smoke'}, 'Domain 84 is occupied'
        child = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:=reference_smoke_control',
            '-r', 'motor_command:=/reference_smoke/blocked_motor_command',
            '-p', 'control_mode:=1', '-p', 'motor_output_enabled:=false',
            '-p', 'position_control/feedback_timeout_sec:=0.5',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, 'ROS_LOG_DIR': str(log_dir / 'ros')})
        assert_gate_disabled()
        # Discover both ends before publishing an event on a volatile topic.
        until(lambda: node.count_publishers('/control/reference') == 1,
              'Reference publisher unavailable')
        spin(0.2)
        assert not references and not fk_positions
        for request_mode, pan, tilt, expected_pan, expected_tilt in (
                (0, -12.0, 7.0, -12.0, 7.0),
                (1, 3.0, -2.0, -9.0, 5.0)):
            before = len(references)
            requested_at = node.get_clock().now().nanoseconds
            response = call(ik_client, MoveToolAngle.Request(
                mode=request_mode, panangle=pan, tiltangle=tilt, gripangle=0.0))
            assert response.success
            until(lambda: len(references) > before, 'Missing IK reference')
            spin(0.1)
            assert len(references) == before + 1, 'IK must publish once per generated command'
            reference = references[-1]
            check_common(reference)
            assert requested_at <= ns(reference.header.stamp) <= node.get_clock().now().nanoseconds
            assert reference.control_mode == 'kinematics'
            assert abs(reference.pan_rad - math.radians(expected_pan)) < 1e-10
            assert abs(reference.tilt_rad - math.radians(expected_tilt)) < 1e-10
            assert ns(reference.feedback_stamp) == 0
            assert not reference.cartesian_reference_valid
            assert (xyz(reference.nominal_position) == xyz(reference.target_position)
                    == (0.0, 0.0, 0.0))
        print('PASS: absolute/relative IK, signed radians, generation time, '
              'zero feedback/Cartesian fields', flush=True)

        feed['enabled'] = True
        timer.reset()
        mode(3)
        until(lambda: sum(msg.control_mode == 'position' for msg in references) >= 5,
              'Position references unavailable')
        initial = next(msg for msg in references if msg.control_mode == 'position')
        assert abs(initial.pan_rad - sum(angle.pan_relative)) < 1e-7
        assert abs(initial.tilt_rad - sum(angle.tilt_relative)) < 1e-7
        request = SetGoalPosition.Request()
        request.reference_type = 'relative'
        request.goal_position.position.y = 0.001
        request.goal_position.position.z = -0.001
        assert call(goal_client, request).success
        spin(0.4)
        position_refs = verify_feedback_references('position')
        assert position_refs[-1].pan_rad < initial.pan_rad
        assert position_refs[-1].tilt_rad > initial.tilt_rad
        print('PASS: Position current-step targets/angles, source-image clock, '
              'matching metric FK', flush=True)

        feed['force'] = True
        spin(0.1)
        mode(4)
        until(lambda: sum(msg.control_mode == 'admittance' for msg in references) >= 8,
              'Admittance references unavailable')
        # Stop generating new image messages and drain pending callbacks before pairing.
        feed['angle'] = False
        spin(0.15)
        admittance_refs = verify_feedback_references('admittance')
        assert any(msg.target_position.y - msg.nominal_position.y > 1e-5 and
                   msg.target_position.z - msg.nominal_position.z < -1e-5
                   for msg in admittance_refs)
        held_counts = len(references), len(fk_positions)
        spin(0.15)
        assert held_counts == (len(references), len(fk_positions)), 'Held feedback was republished'
        assert_gate_disabled()
        assert set(node.get_node_names()) <= {'control_reference_smoke', 'reference_smoke_control'}
        print('PASS: Admittance nominal plus effective offset; no held-frame '
              'republish; motor gate OFF', flush=True)
        print(json.dumps(dict(result='PASS', domain=84, reference_messages=len(references),
                              position_references=len(position_refs),
                              admittance_references=len(admittance_refs),
                              fk_messages=len(fk_positions),
                              actuator_messages=len(actuator_commands),
                              executable=str(executable), log_directory=str(log_dir)),
                         sort_keys=True), flush=True)
    finally:
        timer.cancel()
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3.0)
        log.close()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.try_shutdown()
        print(f'Test logs: {log_dir}', flush=True)


if __name__ == '__main__':
    main()
