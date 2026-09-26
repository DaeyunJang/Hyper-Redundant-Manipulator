"""Isolated real recorder/controller metadata test. No camera/TCP/motor motion.

ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 python3 src/record_pkg/test/metadata_roundtrip.py
Artifacts are retained in /tmp; no bag replay is performed.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from custom_interfaces.msg import MotorCommand
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '76' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated domain 76 and ROS_LOCALHOST_ONLY=1.')
    root = Path(tempfile.mkdtemp(prefix='hrm_metadata_roundtrip_'))
    print(f'Artifacts: {root}', flush=True)
    rclpy.init()
    node = rclpy.create_node('metadata_test_source')
    node.declare_parameter('experiment_label', 'first')
    node.declare_parameter('byte_setting', [b'\x00', b'\xff'])
    node.declare_parameter('channel_order', ['East', 'West', 'South', 'North'])
    pub = node.create_publisher(String, '/metadata_test_tick', 10)
    node.create_timer(0.05, lambda: pub.publish(String(data='test')))
    commands, statuses, processes, logs = [], [], [], []
    node.create_subscription(MotorCommand, '/metadata_test/blocked_motor_command', commands.append, 10)
    node.create_subscription(String, '/data/record_status', lambda msg: statuses.append(json.loads(msg.data)),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    recorder = node.create_client(SetBool, '/data/record')
    control_params = node.create_client(SetParameters, '/robot_control/set_parameters')

    def spin_until(condition, timeout=10):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Timed out; status={statuses[-2:]}')
            rclpy.spin_once(node, timeout_sec=0.02)

    def spin_for(duration):
        deadline = time.monotonic() + duration
        spin_until(lambda: time.monotonic() >= deadline, duration + 1)

    def start_process(package, executable, args):
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        log = (root / f'{executable}.log').open('w')
        logs.append(log)
        processes.append(subprocess.Popen([str(path), '--ros-args', *args],
                                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def record(start):
        future = recorder.call_async(SetBool.Request(data=start))
        spin_until(future.done)
        assert future.result().success, future.result().message

    try:
        spin_for(1)
        assert node.get_node_names() == [node.get_name()], 'Test domain is occupied; do not touch other nodes.'
        config = json.loads((Path(get_package_share_directory('record_pkg')) / 'config/recording.json').read_text())
        config.update(topic_groups={'test': ['/metadata_test_tick', '/parameter_events']},
                      image_archive={'enabled': False}, depth_archive={'enabled': False},
                      enabled_groups=['test'], extra_topics=[], required_topics=['/metadata_test_tick'],
                      metadata_nodes=['/robot_control', '/record', '/metadata_test_source', '/not_running'])
        config_path = root / 'test_recording.json'
        config_path.write_text(json.dumps(config))
        start_process('robot_control_pkg', 'robot_control', [
            '-r', '__node:=robot_control', '-p', 'control_mode:=1', '-p', 'motor_output_enabled:=false',
            '-r', 'motor_command:=/metadata_test/blocked_motor_command',
            '-p', 'hardware.OP_MODE:=9', '-p', 'position_control/pid_controller_pan/p_gain:=4.0'])
        start_process('record_pkg', 'record', [
            '-p', f'config_file:={config_path}', '-p', f'output_root:={root / "record"}'])
        spin_until(lambda: recorder.service_is_ready() and control_params.service_is_ready())
        spin_for(1.0)  # Let the recorder observe small required telemetry before Start.
        future = control_params.call_async(SetParameters.Request(parameters=[
            Parameter('hardware.OP_MODE', value=9).to_parameter_msg()]))
        spin_until(future.done)
        assert not future.result().results[0].successful, 'Compiled metadata must be read-only.'

        directories = []
        for label in ('first', 'second'):
            node.set_parameters([Parameter('experiment_label', value=label)])
            statuses.clear()
            record(True)
            spin_until(lambda: statuses and statuses[-1]['state'] == 'recording')
            directory = Path(statuses[-1]['directory'])
            directories.append(directory)
            data = json.loads((directory / 'session.json').read_text())
            snapshot = data['snapshot']
            runtime = snapshot['runtime_parameters']['nodes']
            hardware = snapshot['hardware_constants']['sources']['/robot_control']
            assert hardware['OP_MODE'] == 8 and hardware['OP_MODE_label'] == 'CSP'
            assert hardware['NUM_OF_SEGMENTS'] == 19 and hardware['NUM_OF_BENDING_JOINTS'] == 18
            assert abs(hardware['TOTAL_LENGTH'] - 82.27) < 1e-9
            parameters = runtime['/robot_control']['parameters']
            assert parameters['position_control/pid_controller_pan/p_gain']['value'] == 4.0
            assert parameters['motor_output_enabled']['value'] is False
            assert runtime['/metadata_test_source']['parameters']['experiment_label']['value'] == label
            assert runtime['/metadata_test_source']['parameters']['byte_setting']['value'] == [0, 255]
            assert runtime['/not_running']['status'] == 'unavailable'
            assert runtime['/record']['parameters']['config_file']['value'] == str(config_path)
            assert data['session_id'] == directory.name and data['start_requested_local']
            for header in ('hw_definition.hpp', 'control_parameters.hpp'):
                reference = snapshot['reference_files']['robot_control_pkg/config/' + header]
                assert reference['content'] and len(reference['sha256']) == 64
            assert snapshot['package_configs']['estimation_pkg/config.json']
            spin_for(0.3)
            record(False)
            spin_until(lambda: statuses[-1]['state'] == 'stopped')
            data = json.loads((directory / 'session.json').read_text())
            assert data['message_counts']['/metadata_test_tick'] > 0
            assert (directory / 'bag/metadata.yaml').is_file()
            spin_until(lambda: not any(name.startswith('rosbag2_recorder') for name in node.get_node_names()))
        assert directories[0] != directories[1]

        # Immediate stop retains explicitly partial metadata, without pending jobs leaking.
        statuses.clear()
        record(True)
        record(False)
        spin_until(lambda: statuses and statuses[-1]['state'] in ('stopped', 'failed'))
        data = json.loads((Path(statuses[-1]['directory']) / 'session.json').read_text())
        assert data['snapshot']['runtime_parameters']['status'] == 'partial'
        assert any(entry['status'] == 'cancelled' for entry in data['snapshot']['runtime_parameters']['nodes'].values())
        assert not commands, f'Unexpected actuator messages: {len(commands)}'
        assert all(process.poll() is None for process in processes)
        print('PASS: two bags, refreshed parameters, compiled/read-only hardware, missing-node timeout, '
              'cancelled snapshot, fixed file copies, zero actuator messages.', flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
