#!/usr/bin/env python3
"""Synthetic Cartesian-sine integration test; never connect to the hardware domain.

Require ROS_DOMAIN_ID=82, ROS_LOCALHOST_ONLY=1 and HRM_CONTROL_TEST_EXECUTABLE
pointing to the freshly built, isolated robot_control executable. No driver,
TCP, camera or GUI is launched. Motor output stays false for the entire test
and the actuator topic is additionally remapped to a private test-only name.
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from custom_interfaces.msg import (
    AdmittanceControl, ForceIdEstimate, LoadcellState, MotorCommand, MotorState,
    PositionControl, SegmentAngle,
)
from custom_interfaces.srv import SetGoalPosition
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def xyz(point):
    return point.x, point.y, point.z


def close_xyz(left, right, tolerance=1e-10):
    return all(abs(a - b) <= tolerance for a, b in zip(left, right))


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '82', 'Use isolated DDS domain 82'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'
    executable_setting = os.environ.get('HRM_CONTROL_TEST_EXECUTABLE')
    assert executable_setting, 'Explicit isolated-build HRM_CONTROL_TEST_EXECUTABLE required'
    executable = Path(executable_setting).expanduser().resolve(strict=True)
    assert executable.is_file() and os.access(executable, os.X_OK)
    started = time.monotonic()
    rclpy.init()
    node = Node('position_sine_smoke')
    child = None
    log_dir = Path(tempfile.mkdtemp(prefix='hrm_position_sine_smoke_'))
    log = (log_dir / 'control.log').open('w')
    positions, admittance, references, previews, actuator_commands = [], [], [], [], []
    control_status, sine_status = [], []
    qos = QoSProfile(depth=100)
    node.create_subscription(PositionControl, '/position_controller', positions.append, qos)
    node.create_subscription(AdmittanceControl, '/admittance_controller', admittance.append, qos)
    node.create_subscription(PointStamped, '/position/sine_reference', references.append, qos)
    node.create_subscription(MotorCommand, '/position/target_motor_command', previews.append, qos)
    node.create_subscription(MotorCommand, '/smoke/blocked_motor_command', actuator_commands.append, qos)
    for topic, target in (('/position/control_status', control_status),
                          ('/position/sine_status', sine_status)):
        node.create_subscription(
            String, topic, lambda msg, target=target: target.append(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publishers = {
        'motor': node.create_publisher(MotorState, '/motor_state', 10),
        'load': node.create_publisher(LoadcellState, '/loadcell_state', 10),
        'angle': node.create_publisher(SegmentAngle, '/estimated_segment_angle', 10),
        'force': node.create_publisher(ForceIdEstimate, '/force_id_estimation', 10),
    }
    params_client = node.create_client(
        SetParametersAtomically, '/position_sine_smoke_control/set_parameters_atomically')
    read_client = node.create_client(GetParameters, '/position_sine_smoke_control/get_parameters')
    sine_client = node.create_client(SetBool, '/position/sine_motion')
    goal_client = node.create_client(SetGoalPosition, '/position/set_goal_position')
    motor_origin = [10000, 20000, 30000, 40000]
    feed = dict(motor=False, load=False, angle=False, force=False,
                force_xyz=(0.0, 0.0, 0.0), image_period=0.05)
    angle = SegmentAngle()
    angle.header.frame_id = 'hrm_base'
    for field in angle.get_fields_and_field_types():
        if field != 'header':
            setattr(angle, field, [0.0] * 18)
    image_stamps = set()
    next_image = 0.0

    def send_feedback():
        nonlocal next_image
        stamp = node.get_clock().now().to_msg()
        if feed['motor']:
            message = MotorState()
            message.header.stamp = stamp
            message.actual_position = motor_origin
            message.actual_velocity = message.actual_acceleration = message.actual_torque = [0] * 4
            publishers['motor'].publish(message)
        if feed['load']:
            message = LoadcellState()
            message.header.stamp = stamp
            message.stress = [10.0] * 4
            publishers['load'].publish(message)
        if feed['angle'] and time.monotonic() >= next_image:
            next_image = time.monotonic() + feed['image_period']
            angle.header.stamp = stamp
            image_stamps.add(stamp_ns(angle))
            publishers['angle'].publish(angle)
        if feed['force']:
            message = ForceIdEstimate()
            message.header.stamp = message.published_stamp = stamp
            message.header.frame_id = 'hrm_base'
            message.valid = True
            message.status = 'ready'
            message.model_name = 'synthetic_eligible_tip_force'
            message.mode = 'class0'
            message.force_scope = 'tip'
            message.load_state = message.LOADED
            message.contact_id = message.control_contact_id = 18
            message.tip_force_valid = True
            message.window_samples = message.samples_in_window = 30
            message.probabilities[18] = 1.0
            message.external_force.x, message.external_force.y, message.external_force.z = feed['force_xyz']
            publishers['force'].publish(message)

    timer = node.create_timer(0.01, send_feedback)
    timer.cancel()  # Check private-domain occupancy before sending any fake data.

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if child is not None:
                assert child.poll() is None, f'Controller exited; inspect {log_dir}'
            assert not actuator_commands, 'Dry-run leaked an actuator command'

    def until(predicate, detail, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin(0.02)
        assert predicate(), f'{detail}; inspect {log_dir}'

    def call(client, request):
        until(client.service_is_ready, 'Test service unavailable', timeout=5.0)
        future = client.call_async(request)
        until(future.done, 'Test service response timeout')
        return future.result()

    def params(**values):
        # This test must never enable motor output, even on its private domain.
        assert values.get('motor_output_enabled', False) is False
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        return call(params_client, request).result

    def start_sine(enabled=True):
        return call(sine_client, SetBool.Request(data=enabled))

    def assert_output_disabled():
        result = call(read_client, GetParameters.Request(names=['motor_output_enabled']))
        assert len(result.values) == 1 and result.values[0].bool_value is False

    def reenter(mode=3):
        assert params(control_mode=1).successful
        spin(0.08)
        before = len(positions)
        assert params(control_mode=mode).successful
        until(lambda: len(positions) > before, 'No fresh-feedback mode entry')
        assert positions[before].dt == 0.0
        assert close_xyz(xyz(positions[before].x_desired.position),
                         xyz(positions[before].x_actual.position))
        until(lambda: control_status and control_status[-1].startswith('READY'), 'Mode not ready')

    def paired_references(start_index):
        diagnostics = {stamp_ns(message): message for message in positions}
        return [(reference, diagnostics[stamp_ns(reference)])
                for reference in references[start_index:] if stamp_ns(reference) in diagnostics]

    try:
        spin(0.5)
        assert set(node.get_node_names()) <= {'position_sine_smoke'}, 'DDS domain 82 already occupied'
        timer.reset()
        child = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:=position_sine_smoke_control',
            '-r', 'motor_command:=/smoke/blocked_motor_command',
            '-p', 'control_mode:=1', '-p', 'motor_output_enabled:=false',
            '-p', 'position_control/feedback_timeout_sec:=0.25',
            '-p', 'position_control/pid_controller_pan/d_gain:=0.0',
            '-p', 'position_control/pid_controller_tilt/d_gain:=0.0',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        assert not start_sine().success, 'Kinematics mode must reject Cartesian sine'
        assert params(control_mode=3).successful
        assert not start_sine().success, 'Position mode without fresh feedback must reject Start'
        assert_output_disabled()
        assert not references
        feed.update(motor=True, load=True, angle=True)
        until(lambda: positions and control_status[-1].startswith('READY'), 'Initial entry not ready')
        initial_x = positions[-1].x_actual.position.x
        assert initial_x > 0.0

        for updates in ({'motion.position_sine.period_sec': 0.0},
                        {'motion.position_sine.max_speed_m_s': 0.0},
                        {'motion.position_sine.y': [0.0, 0.001]},
                        {'motion.position_sine.y': [0.0, -0.001, 0.0]}):
            assert not params(**updates).successful
        waveform = {'motion.position_sine.y': [0.0, 0.002, 0.0],
                    'motion.position_sine.z': [0.0, 0.002, 90.0],
                    'motion.position_sine.period_sec': 4.0,
                    'motion.position_sine.max_speed_m_s': 0.005}
        assert params(**waveform).successful
        print('PASS: wrong-mode/readiness rejection, parameter validation, motor gate OFF', flush=True)

        begin = len(references)
        assert start_sine().success
        assert not start_sine().success, 'Duplicate Start must not reset the running phase'
        assert not params(**{'motion.position_sine.period_sec': 8.0}).successful
        manual = SetGoalPosition.Request()
        manual.reference_type = 'relative'
        manual.goal_position.position.y = 0.001
        assert not call(goal_client, manual).success
        spin(2.5)
        pairs = paired_references(begin)
        assert len(pairs) >= 15, 'Reference must follow accepted image frames'
        assert any('DRY_RUN' in status for status in sine_status)
        for reference, diagnostic in pairs:
            assert reference.header.frame_id == 'hrm_base'
            assert stamp_ns(reference) in image_stamps
            assert abs(reference.point.x - initial_x) < 1e-10
            assert close_xyz(xyz(reference.point), xyz(diagnostic.x_desired.position))
        for previous, current in zip(references[begin:], references[begin + 1:]):
            interval = (stamp_ns(current) - stamp_ns(previous)) * 1e-9
            assert 0.0 < interval < 0.25
            for axis in ('y', 'z'):
                assert abs(getattr(current.point, axis) - getattr(previous.point, axis)) <= 0.005 * interval + 1e-9
        for axis in ('y', 'z'):
            values = [getattr(reference.point, axis) for reference, _ in pairs]
            assert max(values) - min(values) > 0.001
            assert max(abs(value) for value in values) <= 0.002 + 1e-10
        for reference, _ in pairs[-10:]:
            assert abs(math.hypot(reference.point.y, reference.point.z) - 0.002) < 1e-9
        # The first z target is +2 mm; the start must ramp to it, not jump there.
        first_reference, first_diagnostic = pairs[0]
        assert 0.0 < first_reference.point.z <= 0.005 * first_diagnostic.dt + 1e-9
        assert first_reference.point.z < 0.002
        print('PASS: image-paced Y/Z sine, X unchanged, bounded startup ramp, manual goal blocked', flush=True)

        assert start_sine(False).success
        spin(0.15)
        stopped_count = len(references)
        stopped_goal = xyz(positions[-1].x_desired.position)
        assert math.hypot(stopped_goal[1], stopped_goal[2]) > 0.0002
        before = len(positions)
        spin(0.3)
        assert len(positions) > before and len(references) == stopped_count
        assert all(close_xyz(xyz(message.x_desired.position), stopped_goal)
                   for message in positions[before:])
        assert sine_status[-1].startswith('STOPPED')

        assert start_sine().success
        spin(0.6)
        feed['angle'] = False
        spin(0.12)  # Drain a possible in-flight image before measuring the gap.
        held_reference = xyz(references[-1].point)
        gap_reference_count, gap_position_count = len(references), len(positions)
        spin(0.5)
        assert len(references) == gap_reference_count and len(positions) == gap_position_count
        assert sine_status[-1].startswith('PAUSED')
        feed['angle'] = True
        until(lambda: len(positions) > gap_position_count, 'Image gap did not auto-resume')
        resumed = positions[gap_position_count]
        # Recovery image only re-anchors time; the next regular frame advances
        # the held phase by its own dt, never by the missing interval.
        assert 0.0 < resumed.dt <= 0.25
        assert all(abs(actual - held) <= 0.005 * resumed.dt + 1e-8
                   for actual, held in zip(xyz(resumed.x_desired.position), held_reference))
        spin(0.2)
        assert len(references) > gap_reference_count
        assert any(stamp_ns(reference) == stamp_ns(resumed)
                   for reference in references[gap_reference_count:])
        print('PASS: Stop holds nominal goal; camera gaps freeze phase with no catch-up', flush=True)

        assert params(control_mode=1).successful
        spin(0.12)
        mode_stopped_count = len(references)
        assert sine_status[-1].startswith('STOPPED')
        spin(0.2)
        assert len(references) == mode_stopped_count
        reenter()
        spin(0.15)
        assert len(references) == mode_stopped_count, 'Mode re-entry must not restart the sine'
        assert start_sine().success
        spin(0.2)
        feed['load'] = False
        until(lambda: control_status[-1].startswith('FAULT'), 'Loadcell loss must stop the motion')
        until(lambda: sine_status[-1].startswith('STOPPED'), 'Sensor fault did not stop sine')
        assert not start_sine().success
        feed['load'] = True
        spin(0.15)
        reenter()
        print('PASS: mode changes and sensor faults stop sine; recovery needs explicit Start', flush=True)

        feed.update(force=True, force_xyz=(0.0, 0.0, 0.0))
        spin(0.1)
        reenter(mode=4)
        begin = len(references)
        assert start_sine().success
        spin(0.25)
        feed['force_xyz'] = (0.0, -0.02, 0.01)
        spin(0.5)
        force_by_stamp = {stamp_ns(message): message for message in admittance}
        triples = [(reference, position, force_by_stamp[stamp_ns(reference)])
                   for reference, position in paired_references(begin)
                   if stamp_ns(reference) in force_by_stamp]
        assert len(triples) >= 5
        assert any(message.x.position.y > 1e-5 and message.x.position.z < -1e-5
                   for _, _, message in triples)
        for reference, position, message in triples:
            expected = tuple(base + offset for base, offset in
                             zip(xyz(reference.point), xyz(message.x.position)))
            assert close_xyz(xyz(position.x_desired.position), expected)
            assert abs(position.x_desired.position.x - initial_x) < 1e-10
        assert start_sine(False).success
        spin(0.12)
        nominal = xyz(references[-1].point)
        before = len(positions)
        feed['force_xyz'] = (0.0, -0.04, 0.02)
        spin(0.25)
        force_by_stamp = {stamp_ns(message): message for message in admittance}
        checked = 0
        for position in positions[before:]:
            message = force_by_stamp.get(stamp_ns(position))
            if message is not None:
                reconstructed_nominal = tuple(target - offset for target, offset in
                    zip(xyz(position.x_desired.position), xyz(message.x.position)))
                assert close_xyz(reconstructed_nominal, nominal)
                checked += 1
        assert checked >= 2, 'Admittance must continue around the stopped nominal target'
        assert_output_disabled()
        assert not actuator_commands
        print('PASS: admittance offset adds to sine nominal; Stop preserves offset control; zero motor commands', flush=True)
        print(json.dumps(dict(result='PASS', elapsed_seconds=round(time.monotonic() - started, 2),
                              accepted_image_stamps=len(image_stamps),
                              sine_reference_messages=len(references),
                              position_messages=len(positions), actuator_messages=0,
                              executable=str(executable), log_directory=str(log_dir)), sort_keys=True), flush=True)
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


if __name__ == '__main__':
    main()
