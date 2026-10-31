#!/usr/bin/env python3
"""Exercise motor-profile parameters using isolated, dry-run fake feedback.

Requires ROS_DOMAIN_ID=79, ROS_LOCALHOST_ONLY=1, and
HRM_CONTROL_TEST_EXECUTABLE pointing to an isolated-build controller binary.
No TCP/camera nodes are launched; actuator output is disabled and remapped.
"""
import os
from pathlib import Path
import resource
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from custom_interfaces.msg import LoadcellState, MotorCommand, MotorState, SegmentAngle


PROFILE = 'position_control/motor_velocity_profile_percent'
GAIN = 'position_control/pid_controller_pan/p_gain'
NS = '/motor_profile_smoke'


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '79', 'Use isolated domain 79'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'
    executable = Path(os.environ['HRM_CONTROL_TEST_EXECUTABLE']).resolve(strict=True)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    rclpy.init()
    node = Node('motor_profile_smoke_test', namespace=NS)
    previews, commands = [], []
    subscriptions = [
        node.create_subscription(MotorCommand, 'position/target_motor_command', previews.append, 100),
        node.create_subscription(MotorCommand, 'blocked_motor_command', commands.append, 100),
    ]
    motor_pub = node.create_publisher(MotorState, 'motor_state', 10)
    load_pub = node.create_publisher(LoadcellState, 'loadcell_state', 10)
    angle_pub = node.create_publisher(SegmentAngle, 'estimated_segment_angle', 10)
    setter = node.create_client(SetParametersAtomically, NS + '/control/set_parameters_atomically')
    getter = node.create_client(GetParameters, NS + '/control/get_parameters')
    log_dir = Path(tempfile.mkdtemp(prefix='hrm_motor_profile_smoke_'))
    child = None
    log = None

    def send_feedback():
        stamp = node.get_clock().now().to_msg()
        motor = MotorState()
        motor.header.stamp = stamp
        motor.actual_position = [10000] * 4
        motor.actual_velocity = motor.actual_acceleration = motor.actual_torque = [0] * 4
        motor_pub.publish(motor)
        load = LoadcellState()
        load.header.stamp = stamp
        load.stress = [10.0] * 4
        load_pub.publish(load)
        angle = SegmentAngle()
        angle.header.stamp = stamp
        angle.header.frame_id = 'hrm_base'
        for field in angle.get_fields_and_field_types():
            if field != 'header':
                setattr(angle, field, [0.0] * 18)
        angle_pub.publish(angle)

    def spin_until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if child is not None:
                assert child.poll() is None, f'Controller exited; see {log_dir}'
            if predicate():
                return
        raise AssertionError(f'Timed out; see {log_dir}')

    def call(client, request):
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(request)
        spin_until(future.done)
        return future.result()

    def set_values(**values):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        return call(setter, request).result

    def assert_profile(expected):
        start = len(previews)
        spin_until(lambda: len(previews) >= start + 4)
        assert all(list(message.target_velocity_profile) == [expected] * 4
                   for message in previews[-3:])
        assert not commands, 'Dry-run leaked an actuator command'

    def stop_child():
        nonlocal child, log
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3.0)
        child = None
        if log is not None:
            log.close()
            log = None

    def start_child(label, override=None):
        nonlocal child, log
        path = log_dir / (label + '.log')
        log = path.open('w')
        args = [str(executable), '--ros-args', '-r', '__node:=control',
                '-r', '__ns:=' + NS, '-r', 'motor_command:=blocked_motor_command',
                '-p', 'control_mode:=3', '-p', 'motor_output_enabled:=false']
        if override is not None:
            args.extend(['-p', PROFILE + ':=' + override])
        child = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        return path

    timer = None
    try:
        discovery_end = time.monotonic() + 0.5
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert set(node.get_node_names()) <= {'motor_profile_smoke_test'}, 'Domain 79 already in use'
        timer = node.create_timer(0.05, send_feedback)
        start_child('default')
        assert_profile(50)
        for value in (1, 100, 37):
            assert set_values(**{PROFILE: value}).successful
            assert_profile(value)
        for value in (0, -1, 101, 4294967297, 50.0, True):
            assert not set_values(**{PROFILE: value}).successful, value
            assert_profile(37)

        # Validate the entire batch before mutating the active profile or gain.
        assert not set_values(**{PROFILE: 20, GAIN: -1.0}).successful
        assert_profile(37)
        assert not set_values(**{GAIN: 7.0, PROFILE: 101}).successful
        values = call(getter, GetParameters.Request(names=[PROFILE, GAIN])).values
        assert values[0].integer_value == 37 and values[1].double_value == 5.0
        print('PASS: default, runtime bounds/types and atomic batch rejection', flush=True)
        stop_child()

        previews.clear()
        start_child('custom_startup', '37')
        assert_profile(37)
        stop_child()

        # Constructor rejects invalid startup overrides before any motor output.
        for index, value in enumerate(('0', '-1', '101', '4294967297', '50.0')):
            path = start_child(f'invalid_startup_{index}', value)
            code = child.wait(timeout=5.0)
            assert code != 0, value
            stop_child()
            message = path.read_text()
            assert 'motor_velocity_profile_percent' in message or 'Position motor velocity profile' in message
        assert not commands
        print(f'PASS: startup validation and zero actuator messages; logs={log_dir}', flush=True)
    finally:
        stop_child()
        if timer is not None:
            node.destroy_timer(timer)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
