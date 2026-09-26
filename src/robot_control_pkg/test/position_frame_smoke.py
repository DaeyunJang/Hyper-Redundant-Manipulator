#!/usr/bin/env python3
"""Isolated fake-feedback test: no camera/TCP and motor output always disabled.

Source install/setup.bash, then run with ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1.
The actuator topic is additionally remapped to a test-only name.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from custom_interfaces.msg import LoadcellState, MotorCommand, MotorState, PositionControl, SegmentAngle
from custom_interfaces.srv import SetGoalPosition
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '78', 'Use isolated domain 78'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost only'
    rclpy.init()
    node = Node('position_frame_smoke')
    child = None
    log_dir = Path(tempfile.mkdtemp(prefix='hrm_position_smoke_'))
    log = (log_dir / 'control.log').open('w')
    diagnostics, previews, actual_commands, statuses = [], [], [], []
    qos = QoSProfile(depth=100)
    subscriptions = [
        node.create_subscription(PositionControl, '/position_controller', diagnostics.append, qos),
        node.create_subscription(MotorCommand, '/position/target_motor_command', previews.append, qos),
        node.create_subscription(MotorCommand, '/smoke/blocked_motor_command', actual_commands.append, qos),
        node.create_subscription(String, '/position/control_status',
                                 lambda msg: statuses.append(msg.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    motor_pub = node.create_publisher(MotorState, '/motor_state', 10)
    load_pub = node.create_publisher(LoadcellState, '/loadcell_state', 10)
    angle_pub = node.create_publisher(SegmentAngle, '/estimated_segment_angle', 10)
    parameter_client = node.create_client(SetParameters, '/position_pd_smoke_control/set_parameters')
    goal_client = node.create_client(SetGoalPosition, '/position/set_goal_position')
    motor_origin = [10000, 20000, 30000, 40000]
    feed = {'motor': False, 'load': False, 'angle': False, 'bad_load': False,
            'angle_kind': 'valid', 'period': 0.1, 'tension': 10.0}
    angle = SegmentAngle()
    angle.header.frame_id = 'hrm_base'
    for field in angle.get_fields_and_field_types():
        if field != 'header':
            setattr(angle, field, [0.0] * 18)
    angle.tilt_relative = [0.02 if i % 2 == 0 else 0.0 for i in range(18)]
    angle.pan_relative = [0.01 if i % 2 else 0.0 for i in range(18)]
    next_angle = 0.0

    def send_sensors():
        nonlocal next_angle
        stamp = node.get_clock().now().to_msg()
        if feed['motor']:
            motor = MotorState()
            motor.header.stamp = stamp
            motor.actual_position = motor_origin
            motor.actual_velocity = [0] * 4
            motor.actual_acceleration = [0] * 4
            motor.actual_torque = [0] * 4
            motor_pub.publish(motor)
        if feed['load']:
            load = LoadcellState()
            load.header.stamp = stamp
            load.stress = [] if feed['bad_load'] else [feed['tension']] * 4
            load_pub.publish(load)
        if feed['angle'] and time.monotonic() >= next_angle:
            next_angle = time.monotonic() + feed['period']
            if feed['angle_kind'] != 'duplicate':
                angle.header.stamp = stamp
            angle.header.frame_id = 'camera' if feed['angle_kind'] == 'wrong_frame' else 'hrm_base'
            angle.tilt_relative[0] = float('nan') if feed['angle_kind'] == 'nan' else 0.02
            if feed['angle_kind'] == 'stale':
                angle.header.stamp.sec -= 5
            angle_pub.publish(angle)

    timer = node.create_timer(0.01, send_sensors)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
            if child is not None:
                assert child.poll() is None, f'Control exited: {child.returncode}; see {log_dir}'

    def call(client, request):
        assert client.wait_for_service(timeout_sec=3.0)
        future = client.call_async(request)
        until = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < until:
            spin(0.01)
        assert future.done(), 'Service timeout'
        return future.result()

    def parameter(name, value):
        typed = (ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
                 if isinstance(value, int) else
                 ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        return call(parameter_client, SetParameters.Request(
            parameters=[Parameter(name=name, value=typed)])).results[0]

    def goal(y=0.0, z=0.0):
        req = SetGoalPosition.Request()
        req.reference_type = 'relative'
        req.goal_position.position.x = 999.0  # X must remain passive.
        req.goal_position.position.y = y
        req.goal_position.position.z = z
        return call(goal_client, req)

    def reenter():
        assert parameter('control_mode', 1).successful
        spin(0.1)
        start = len(diagnostics)
        # The first frame can arrive while the parameter service is returning.
        assert parameter('control_mode', 3).successful
        spin(0.45)
        fresh = diagnostics[start:]
        assert fresh and fresh[0].dt == 0.0
        assert fresh[0].x_desired == fresh[0].x_actual
        assert list(previews[-1].target_position) == motor_origin

    try:
        spin(0.4)
        assert set(node.get_node_names()) <= {'position_frame_smoke'}, 'Domain 78 already in use'
        executable = Path(__file__).resolve().parents[3] / 'install/robot_control_pkg/lib/robot_control_pkg/robot_control'
        child = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:=position_pd_smoke_control',
            '-r', 'motor_command:=/smoke/blocked_motor_command',
            '-p', 'control_mode:=3', '-p', 'motor_output_enabled:=false',
            '-p', 'position_control/pid_controller_pan/p_gain:=3.0',
            '-p', 'position_control/pid_controller_tilt/p_gain:=4.0',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        spin(1.0)
        assert not diagnostics and not previews, 'No-input startup must wait without indexing arrays'
        assert not goal(0.001).success
        print('PASS: no-feedback startup and premature goal rejection', flush=True)

        feed.update(motor=True, load=True, angle=True)
        spin(1.2)
        assert diagnostics and statuses[-1] == 'READY (dry-run)'
        assert diagnostics[0].dt == 0.0
        assert diagnostics[0].x_desired == diagnostics[0].x_actual
        assert list(previews[-1].target_position) == motor_origin
        assert all(msg.header.frame_id == 'hrm_base' for msg in diagnostics)
        assert all(0.08 < msg.dt < 0.14 for msg in diagnostics[1:])
        print('PASS: nonzero motor/angle entry, hrm_base diagnostics, source-frame dt ~0.1s', flush=True)

        before_goal = len(diagnostics)
        assert goal(y=0.001, z=-0.001).success
        spin(1.2)
        first_step = next(msg for msg in diagnostics[before_goal:]
                          if abs(msg.x_error.position.y) > 0.0009)
        assert abs(first_step.del_theta_tilt - 0.004) < 1e-7
        assert abs(first_step.del_theta_pan + 0.003) < 1e-7
        assert list(previews[-1].target_position) != motor_origin
        assert all(msg.target_position == previews[-1].target_position for msg in previews[-5:])
        assert diagnostics[-1].x_desired.position.x == diagnostics[-1].x_actual.position.x
        assert abs(diagnostics[-1].x_error.position.y - 0.001) < 1e-10
        assert abs(diagnostics[-1].x_error.position.z + 0.001) < 1e-10
        assert abs(diagnostics[-1].del_theta_pan) < 1e-10
        assert abs(diagnostics[-1].del_theta_tilt) < 1e-10
        assert parameter('control_mode', 3).successful
        spin(0.25)
        assert diagnostics[-1].x_desired != diagnostics[-1].x_actual
        assert not parameter('position_control/pid_controller_tilt/i_gain', 1.0).successful
        assert not parameter('position_control/pid_controller_pan/p_gain', -1.0).successful
        print('PASS: constant-error no drift, X ignored, same-mode no reset, bad gain rejection', flush=True)

        spin(0.05)
        before = len(diagnostics)
        preview_before = len(previews)
        saved_goal = diagnostics[-1].x_desired
        saved_target = list(previews[-1].target_position)
        feed['angle_kind'] = 'duplicate'
        spin(0.7)
        assert len(diagnostics) <= before + 1  # Allow one in-flight sample.
        assert len(previews) <= preview_before + 1, 'Camera gaps must not send a hold/extra command'
        assert statuses[-1].startswith('WAITING: fresh image')
        assert list(previews[-1].target_position) == saved_target
        feed['angle_kind'] = 'valid'
        before = len(diagnostics)
        spin(0.4)
        assert len(diagnostics) > before, 'Camera-only gaps must resume automatically'
        assert diagnostics[before].dt > 0.25, 'Preserve the true source-frame gap in diagnostics'
        assert all(msg.dt > 0.0 for msg in diagnostics[before:]), 'Not a new mode entry'
        assert diagnostics[-1].x_desired == saved_goal
        assert list(previews[-1].target_position) == saved_target
        assert statuses[-1] == 'READY (dry-run)'
        feed['angle'] = False
        spin(0.7)
        assert statuses[-1].startswith('WAITING: fresh image')
        feed['angle'] = True
        before = len(diagnostics)
        spin(0.35)
        assert len(diagnostics) > before and diagnostics[-1].x_desired == saved_goal

        # Even a continuous low image rate must not repeatedly latch or reset.
        feed['period'] = 0.35
        before = len(diagnostics)
        spin(1.2)
        assert len(diagnostics) >= before + 2
        assert diagnostics[-1].dt > 0.25
        assert diagnostics[-1].x_desired == saved_goal
        assert not any(status.startswith('FAULT') for status in statuses)
        feed['period'] = 0.1
        reenter()
        print('PASS: duplicate/missing/slow camera frames auto-resume, goal/target preserved, no hold or mode reset', flush=True)

        feed['period'] = 0.05
        spin(0.6)
        assert all(0.035 < msg.dt < 0.085 for msg in diagnostics[-5:])
        for invalid in ['nan', 'wrong_frame', 'stale']:
            saved_goal = diagnostics[-1].x_desired
            feed['angle_kind'] = invalid
            spin(0.55)
            assert statuses[-1].startswith('WAITING: fresh image'), invalid
            feed['angle_kind'] = 'valid'
            before = len(diagnostics)
            spin(0.25)
            assert len(diagnostics) > before and diagnostics[-1].x_desired == saved_goal
        feed['bad_load'] = True
        spin(0.35)
        assert statuses[-1].startswith('FAULT')
        feed['bad_load'] = False
        reenter()
        print('PASS: variable frame rate, NaN/frame/stamp rejection, empty loadcell recovery', flush=True)

        for sensor in ['motor', 'load']:
            # Real sensor faults still latch, including while the camera is absent.
            feed['angle'] = False
            feed[sensor] = False
            spin(0.6)
            assert statuses[-1].startswith('FAULT'), sensor
            feed[sensor] = True
            feed['angle'] = True
            before = len(diagnostics)
            spin(0.3)
            assert len(diagnostics) == before, 'Motor/loadcell faults must not auto-resume'
            reenter()
        feed['tension'] = 2500.0
        spin(0.2)
        assert statuses[-1].startswith('FAULT')
        feed['tension'] = 10.0
        reenter()
        assert parameter('position_control/pid_controller_pan/p_gain', 6.0).successful
        assert parameter('position_control/pid_controller_tilt/p_gain', 7.0).successful
        before_goal = len(diagnostics)
        assert goal(y=0.0001, z=-0.0001).success
        spin(0.3)
        first_step = next(msg for msg in diagnostics[before_goal:]
                          if abs(msg.x_error.position.y) > 0.00009)
        assert abs(first_step.del_theta_tilt - 0.0007) < 1e-7
        assert abs(first_step.del_theta_pan + 0.0006) < 1e-7
        print('PASS: motor/loadcell timeout, tension trip, both-axis startup/runtime gains applied', flush=True)
        assert not actual_commands, 'Dry-run leaked an actuator command'
        print(f'PASS: zero actuator messages; diagnostics={len(diagnostics)}; logs={log_dir}', flush=True)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3.0)
        log.close()
        node.destroy_timer(timer)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
