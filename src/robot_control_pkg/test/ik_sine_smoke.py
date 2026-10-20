#!/usr/bin/env python3
"""Isolated service test. Motor output remapped, no TCP or hardware launched."""

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
from std_srvs.srv import SetBool
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from custom_interfaces.msg import MotorCommand, MotorState, LoadcellState


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '79'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = Node('ik_sine_smoke')
    directory = Path(tempfile.mkdtemp(prefix='hrm_ik_sine_'))
    log = (directory / 'control.log').open('w')
    child = None
    poses, wires, motors, statuses = [], [], [], []
    subscriptions = [
        node.create_subscription(Twist, '/surgical_tool_pose', poses.append, 100),
        node.create_subscription(Float64MultiArray, '/kinematics/target_wire_length', wires.append, 100),
        node.create_subscription(MotorCommand, '/test_only/motor_command', motors.append, 100),
        node.create_subscription(String, '/kinematics/sine_status',
                                 lambda m: statuses.append(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    params = node.create_client(SetParametersAtomically, '/ik_sine_test/set_parameters_atomically')
    get_params = node.create_client(GetParameters, '/ik_sine_test/get_parameters')
    service = node.create_client(SetBool, '/kinematics/sine_motion')
    legacy = node.create_client(SetBool, '/motion/move_sine_wave')
    motor_pub = node.create_publisher(MotorState, '/motor_state', 10)
    load_pub = node.create_publisher(LoadcellState, '/loadcell_state', 10)
    feedback = {'enabled': False, 'position': 10000, 'tension': 10.}

    def send_feedback():
        if not feedback['enabled']:
            return
        motor, load = MotorState(), LoadcellState()
        motor.header.stamp = load.header.stamp = node.get_clock().now().to_msg()
        motor.actual_position = [feedback['position']] * 4
        motor.actual_velocity = [0] * 4
        motor.actual_acceleration = [0] * 4
        motor.actual_torque = [0] * 4
        load.stress = [feedback['tension']] * 4
        motor_pub.publish(motor)
        load_pub.publish(load)

    timer = node.create_timer(.02, send_feedback)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            assert child.poll() is None, str(directory)

    def call(client, request):
        assert client.wait_for_service(timeout_sec=5.)
        future = client.call_async(request)
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            spin(.01)
        assert future.done(), 'Service timeout'
        return future.result()

    def settings(**values):
        return call(params, SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()
        ])).result

    def start():
        return call(service, SetBool.Request(data=True))

    def stop():
        assert call(service, SetBool.Request(data=False)).success
        spin(.05)  # Drain in-flight DDS messages.
        before = len(poses)
        spin(.12)
        assert len(poses) == before

    try:
        child = subprocess.Popen([
            'ros2', 'run', 'robot_control_pkg', 'robot_control', '--ros-args',
            '-r', '__node:=ik_sine_test', '-r', 'motor_command:=/test_only/motor_command',
            '-p', 'motor_output_enabled:=false',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        assert service.wait_for_service(timeout_sec=10.)
        defaults = call(get_params, GetParameters.Request(names=[
            'motion.sine.tilt', 'motion.sine.pan', 'motion.sine.period_sec',
            'motion.sine.max_speed_deg_s', 'motor_output_enabled'])).values
        assert list(defaults[0].double_array_value) == [0., 45., 0.]
        assert list(defaults[1].double_array_value) == [0., 45., 90.]
        assert defaults[2].double_value == 20.
        assert defaults[3].double_value == 15.
        assert not defaults[4].bool_value
        spin(.1)
        assert not poses and not wires and not motors  # Never auto-start.
        assert settings(**{'motion.sine.tilt': [22.5, 22.5, -90.],
                           'motion.sine.pan': [0., 0., 0.],
                           'motion.sine.period_sec': 30.,
                           'motion.sine.max_speed_deg_s': 5.}).successful
        assert not settings(**{'motion.sine.tilt': [30., 30., 0.]}).successful
        assert not settings(**{'motion.sine.period_sec': 1.}).successful
        assert not settings(**{'motion.sine.pan': [0., 1.]}).successful
        spin(.1)
        assert start().success
        assert not start().success  # No reset/phase jump on duplicate Start.
        assert not settings(**{'motion.sine.pan': [0., 1., 0.]}).successful
        assert not call(legacy, SetBool.Request(data=True)).success
        spin(.4)
        assert len(poses) >= 5 and len(wires) >= 5
        assert all(len(message.data) == 4 for message in wires)
        assert all(message.angular.y == 0. for message in poses)
        assert poses[-1].angular.z > 0.
        assert not motors
        stop()
        poses.clear()
        assert settings(**{'motion.sine.tilt': [0., 0., 0.],
                           'motion.sine.pan': [0., 5., 0.]}).successful
        assert start().success
        spin(.4)
        assert poses[-1].angular.y > 0.
        assert abs(poses[-1].angular.z) < 1e-10
        stop()
        assert settings(**{'motion.sine.tilt': [0., 5., 0.],
                           'motion.sine.pan': [0., 5., 90.]}).successful
        assert start().success
        spin(.1)
        # Gate changes cancel even a dry-run; enabling never starts motion.
        assert settings(motor_output_enabled=True).successful
        spin(.1)
        assert not start().success  # No motor/loadcell feedback, even in test domain.
        assert not motors
        assert settings(motor_output_enabled=False).successful
        assert start().success
        assert settings(control_mode=3).successful
        spin(.1)
        assert any('control mode changed' in text for text in statuses)
        assert not start().success
        assert not motors
        # Exercise physical-output guards with simulated feedback and a REMAPPED
        # output topic only. No TCP/drive process exists in this isolated domain.
        assert settings(control_mode=1).successful
        assert settings(**{'motion.sine.tilt': [0., 0., 0.],
                           'motion.sine.pan': [0., 0., 0.]}).successful
        assert start().success
        spin(.4)
        stop()
        feedback['enabled'] = True
        spin(.15)
        assert settings(motor_output_enabled=True).successful
        assert '1000 counts' in start().message
        feedback.update(position=0, tension=3000.)
        spin(.1)
        assert not start().success
        assert not motors
        feedback['tension'] = 10.
        spin(.1)
        assert start().success
        spin(.15)
        assert motors and all(list(m.target_position) == [0]*4 for m in motors)
        feedback['tension'] = 3000.
        spin(.1)
        assert any('safety check failed' in text for text in statuses)
        before = len(motors)
        spin(.1)
        assert len(motors) == before
        feedback['tension'] = 10.
        spin(.1)
        assert start().success
        feedback['enabled'] = False
        spin(.4)
        before = len(motors)
        spin(.1)
        assert len(motors) == before
        assert not start().success  # Stale data never automatically resumes.
        print('IK sine smoke PASS: tilt/pan, stop, gates, mismatch, tension/stale guards; no hardware.')
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        print(f'Test logs: {directory}')


if __name__ == '__main__':
    main()
