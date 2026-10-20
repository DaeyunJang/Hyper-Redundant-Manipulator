"""Manual isolated-domain integration test; never start camera/TCP/control nodes.

ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 python3 src/record_pkg/test/rosbag_roundtrip.py
Do not run in the hardware domain. Test bags/logs are retained under /tmp.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from custom_interfaces.msg import LoadcellState, MotorState, SegmentAngle
from geometry_msgs.msg import TransformStamped, WrenchStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '76' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Test requires isolated ROS_DOMAIN_ID=76 and ROS_LOCALHOST_ONLY=1.')
    root = Path(tempfile.mkdtemp(prefix='hrm_record_roundtrip_'))
    print(f'Test artifacts: {root}', flush=True)
    rclpy.init()
    node = rclpy.create_node('record_test_sources')
    if node.get_node_names() != [node.get_name()]:
        # Discovery may not yet have completed; repeat after spinning below.
        print('Checking test domain for external nodes…', flush=True)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    types = {}
    publishers = {}

    def publisher(topic, msg_type, profile=10):
        types[topic] = msg_type
        publishers[topic] = node.create_publisher(msg_type, topic, profile)
        return publishers[topic]
    color_topic = '/camera/camera/color/image_rect_raw'
    depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
    color_pub = publisher(color_topic, Image, qos)
    depth_pub = publisher(depth_topic, Image, qos)
    info_pub = publisher('/camera/camera/color/camera_info', CameraInfo, qos)
    motor_pub = publisher('/motor_state', MotorState)
    loadcell_pub = publisher('/loadcell_state', LoadcellState)
    force_pub = publisher('/fts_data', WrenchStamped)
    kalman_pub = publisher('/fts_data_kalman_filter', WrenchStamped)
    angle_pub = publisher('/estimated_segment_angle', SegmentAngle)
    static_pub = publisher('/tf_static', TFMessage, QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    statuses = []
    node.create_subscription(String, '/data/record_status',
                             lambda m: statuses.append(json.loads(m.data)),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    width, height = 848, 480
    color = Image(width=width, height=height, encoding='rgb8', step=width * 3,
                  data=bytes([17, 23, 41]) * width * height)
    depth = Image(width=width, height=height, encoding='16UC1', step=width * 2,
                  data=bytes([120, 0]) * width * height)
    info = CameraInfo(width=width, height=height)
    info.k = [500.0, 0.0, 424.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    angles = SegmentAngle()
    for field in angles.get_fields_and_field_types():
        if field != 'header':
            setattr(angles, field, [0.0] * 18)
    angles.tilt_relative[0] = 0.01
    angles.pan_relative[1] = 0.02
    sent_stamps = []

    def camera_tick():
        stamp = node.get_clock().now().to_msg()
        for msg in (color, depth, info, angles):
            msg.header.stamp = stamp
            msg.header.frame_id = ('hrm_base' if msg is angles else 'camera_color_optical_frame')
        color_pub.publish(color)
        depth_pub.publish(depth)
        info_pub.publish(info)
        angle_pub.publish(angles)
        sent_stamps.append(stamp.sec * 10**9 + stamp.nanosec)

    def sensor_tick():
        motor = MotorState()
        motor.actual_position = [1000, 2000, 3000, 4000]
        loadcell = LoadcellState(stress=[10.0, 20.0, 30.0, 40.0])
        force = WrenchStamped()
        force.wrench.force.x = 1234.5
        filtered = WrenchStamped()
        filtered.wrench.force.x = 321.0
        stamp = node.get_clock().now().to_msg()
        for msg in (motor, loadcell, force, filtered):
            msg.header.stamp = stamp
        motor_pub.publish(motor)
        loadcell_pub.publish(loadcell)
        force_pub.publish(force)
        kalman_pub.publish(filtered)
    node.create_timer(1.0 / 30, camera_tick)
    node.create_timer(0.01, sensor_tick)
    client = node.create_client(SetBool, '/data/record')
    process = None

    def spin_until(condition, timeout=10):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Condition timed out; recent status: {statuses[-2:]}')
            rclpy.spin_once(node, timeout_sec=0.01)

    def service(value):
        future = client.call_async(SetBool.Request(data=value))
        spin_until(future.done)
        return future.result()
    try:
        deadline = time.monotonic() + 1.0
        spin_until(lambda: time.monotonic() > deadline)
        assert all(name == 'record_test_sources' for name in node.get_node_names()), node.get_node_names()
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_color_optical_frame'
        transform.child_frame_id = 'hrm_base'
        transform.transform.rotation.w = 1.0
        static_pub.publish(TFMessage(transforms=[transform]))  # Before recorder starts.
        config = json.loads((Path(get_package_share_directory('record_pkg')) / 'config/recording.json').read_text())
        # Explicit legacy full-image bag fixture, independent of crop-only defaults.
        config['image_archive'] = {'enabled': False}
        config['depth_archive'] = {'enabled': False}
        config['topic_groups'] = {'test': list(publishers)}
        config['enabled_groups'] = ['test']
        config['extra_topics'] = []
        # This fixture covers acquisition, not physical tags/control; declare
        # exactly the continuous fake sources as required (static TF is latched).
        config['required_topics'] = [topic for topic in publishers if topic != '/tf_static']
        config_path = root / 'test_recording.json'
        config_path.write_text(json.dumps(config))
        with (root / 'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'record_pkg', '_launch.py', f'output_root:={root}',
                 f'config_file:={config_path}'],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        spin_until(client.service_is_ready)
        deadline = time.monotonic() + 1.0
        spin_until(lambda: time.monotonic() > deadline)  # lightweight health subscriptions
        response = service(True)
        assert response.success, response.message
        spin_until(lambda: statuses and statuses[-1]['state'] == 'recording')
        assert not service(True).success  # Duplicate start must not overwrite.
        deadline = time.monotonic() + 5.0
        spin_until(lambda: time.monotonic() > deadline)
        assert service(False).success
        assert service(False).success  # Duplicate stop must not double-signal.
        spin_until(lambda: statuses[-1]['state'] in ('stopped', 'failed'), timeout=30)
        assert statuses[-1]['state'] == 'stopped', statuses[-1]
        directory = Path(statuses[-1]['directory'])
        session = json.loads((directory / 'session.json').read_text())
        assert not session['required_topics_without_messages']
        assert session['data_quality'] == 'complete', session['postprocess']
        assert (directory / 'csv/manifest.json').is_file()
        manifest = json.loads((directory / 'csv/manifest.json').read_text())
        assert manifest['status'] == 'complete'
        assert manifest['topics']['/motor_state']['rows_written'] > 0
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(directory / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        bag_types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        stamps = {color_topic: [], depth_topic: []}
        counts = {}
        static_found = False
        while reader.has_next():
            topic, serialized, received_ns = reader.read_next()
            msg = deserialize_message(serialized, bag_types[topic])
            counts[topic] = counts.get(topic, 0) + 1
            assert received_ns > 0
            if topic in stamps:
                stamps[topic].append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
                expected = color if topic == color_topic else depth
                assert msg.data == expected.data
                assert msg.encoding == expected.encoding and msg.step == expected.step
            if topic == '/motor_state':
                assert list(msg.actual_position) == [1000, 2000, 3000, 4000]
            if topic == '/loadcell_state':
                assert list(msg.stress) == [10.0, 20.0, 30.0, 40.0]
            if topic == '/fts_data':
                assert msg.wrench.force.x == 1234.5
            if topic == '/fts_data_kalman_filter':
                assert msg.wrench.force.x == 321.0
            if topic == '/estimated_segment_angle':
                assert len(msg.pan_relative) == len(msg.tilt_relative) == 18
                assert msg.pan_relative[1] == 0.02 and msg.tilt_relative[0] == 0.01
            if topic == '/tf_static':
                static_found = any(t.child_frame_id == 'hrm_base' for t in msg.transforms)
        assert static_found
        for topic, saved in stamps.items():
            expected = [s for s in sent_stamps if saved[0] <= s <= saved[-1]]
            assert saved == expected, (topic, len(saved), len(expected))
            assert len(saved) >= 140, (topic, len(saved))
        print(json.dumps({'counts': counts, 'source_stamps_and_payloads': 'preserved',
                          'camera_interior_drops': 0, 'static_tf_late_join': True,
                          'bag': str(directory / 'bag')}, indent=2), flush=True)
        # A second session must also flush when its owning launch is stopped.
        spin_until(lambda: not any(name.startswith('rosbag2_recorder')
                                   for name in node.get_node_names()))
        assert service(True).success
        spin_until(lambda: statuses[-1]['state'] == 'recording')
        second_directory = Path(statuses[-1]['directory'])
        assert second_directory != directory
        deadline = time.monotonic() + 1.0
        spin_until(lambda: time.monotonic() > deadline)
        process.send_signal(signal.SIGINT)
        spin_until(lambda: process.poll() is not None, timeout=15)
        closed = json.loads((second_directory / 'session.json').read_text())
        assert closed['state'] == 'stopped', closed
        assert not closed['required_topics_without_messages']
        assert closed['postprocess']['status'] == 'pending_or_interrupted'
        print('Active recording flushed on launch shutdown: PASS', flush=True)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
