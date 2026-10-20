#!/usr/bin/env python3
"""Synthetic images -> installed AprilTag detector -> stamped poses. No hardware.

Run after sourcing ROS, with PYTHONPATH="src/record_pkg:${PYTHONPATH}". This
script uses isolated ROS domain 79, launches only apriltag_node, and rejects an
occupied test domain. Set ROS_LOG_DIR to a writable directory in a sandbox.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '79'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


def main():
    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    from record_pkg.apriltag_pose import AprilTagPoseNode

    rclpy.init()
    probe = Node('apriltag_pose_test')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    pose_node = None
    detector = None

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_for(predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), description

    try:
        spin_for(0.3)
        assert set(probe.get_node_names()) <= {'apriltag_pose_test'}, 'Test domain is occupied.'
        pose_node = AprilTagPoseNode()
        executor.add_node(pose_node)
        results = {key: [] for key in ('tag0/pose', 'tag1/pose', 'tag1_in_tag0/pose')}
        subscriptions = [probe.create_subscription(
            PoseStamped, '/apriltag/' + key,
            lambda message, key=key: results[key].append(message), 10)
            for key in results]
        source_tfs = []
        subscriptions.append(probe.create_subscription(
            TFMessage, '/tf', lambda message: source_tfs.extend(message.transforms), 100))
        image_publisher = probe.create_publisher(
            Image, '/apriltag_test/image_rect', qos_profile_sensor_data)
        info_publisher = probe.create_publisher(
            CameraInfo, '/apriltag_test/camera_info', qos_profile_sensor_data)
        tf_publisher = probe.create_publisher(TFMessage, '/tf', 100)
        config = Path(__file__).resolve().parents[2] / 'launcher/config/apriltag.yaml'
        detector_command = [
            '/opt/ros/humble/lib/apriltag_ros/apriltag_node', '--ros-args',
            '--params-file', str(config),
            '-r', '__node:=apriltag_node',
            '-r', 'image_rect:=/apriltag_test/image_rect',
            '-r', 'camera_info:=/apriltag_test/camera_info',
        ]
        detector = subprocess.Popen(detector_command, start_new_session=True)
        wait_for(lambda: probe.count_subscribers('/apriltag_test/image_rect') > 0,
                 'Detector image subscription was not created.')
        image_endpoints = probe.get_subscriptions_info_by_topic('/apriltag_test/image_rect')
        print('Detector image subscription QoS:',
              [(endpoint.qos_profile.reliability.name, endpoint.qos_profile.depth)
               for endpoint in image_endpoints])
        # Actual camera sensor-data QoS must work, not only reliable test input.
        assert all(endpoint.qos_profile.reliability.name == 'BEST_EFFORT'
                   for endpoint in image_endpoints), 'Detector cannot accept sensor-data QoS.'
        spin_for(0.5)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        pixels = np.full((480, 848), 255, dtype=np.uint8)
        pixels[192:288, 176:272] = cv2.aruco.generateImageMarker(dictionary, 0, 96)
        pixels[192:288, 576:672] = cv2.aruco.generateImageMarker(dictionary, 1, 96)
        image = Image()
        image.header.frame_id = 'synthetic_color_optical_frame'
        image.height, image.width = pixels.shape
        image.encoding, image.step = 'mono8', image.width
        image.data = pixels.tobytes()
        info = CameraInfo()
        info.header.frame_id = image.header.frame_id
        info.width, info.height = image.width, image.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.] * 5
        info.k = [600., 0., 424., 0., 600., 240., 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [600., 0., 424., 0., 0., 600., 240., 0., 0., 0., 1., 0.]
        for index in range(8):
            image.header.stamp.sec = info.header.stamp.sec = 100 + index
            info_publisher.publish(info)
            image_publisher.publish(image)
            spin_for(0.2)
            if results['tag1_in_tag0/pose']:
                break
        assert results['tag1_in_tag0/pose'], 'Synthetic tags were not detected.'
        assert all(results[key] for key in results)
        relative = results['tag1_in_tag0/pose'][-1]
        assert relative.header.frame_id == 'ID0'
        assert relative.header.stamp == image.header.stamp
        for key in ('tag0/pose', 'tag1/pose'):
            value = results[key][-1]
            assert value.header.frame_id == image.header.frame_id
            assert value.header.stamp == image.header.stamp
        distance = math.sqrt(sum(getattr(relative.pose.position, axis)**2 for axis in 'xyz'))
        assert abs(distance - 0.020 * 400 / 96) < 0.001, distance
        last_tf = [transform for transform in source_tfs
                   if transform.header.stamp == image.header.stamp
                   and transform.child_frame_id in ('ID0', 'ID1')]
        assert {transform.child_frame_id for transform in last_tf} == {'ID0', 'ID1'}
        before = {key: len(value) for key, value in results.items()}
        # Re-run the image at the same stamp through the SAME DDS detector
        # publisher; a replacement publisher intentionally starts a new epoch.
        info_publisher.publish(info)
        image_publisher.publish(image)
        spin_for(0.2)
        assert before == {key: len(value) for key, value in results.items()}, 'Cached TF repeated.'
        # A missing tag or different image stamps must not produce relative data.
        for child, stamp in [('ID0', 200), ('ID1', 201)]:
            transform = TransformStamped()
            transform.header.frame_id = image.header.frame_id
            transform.header.stamp.sec = stamp
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            tf_publisher.publish(TFMessage(transforms=[transform]))
            spin_for(0.1)
        assert len(results['tag1_in_tag0/pose']) == before['tag1_in_tag0/pose']
        # A replacement detector has a different DDS endpoint. Only that event
        # permits a lower image-clock epoch; ordinary out-of-order TFs do not.
        old_source = pose_node.detector_source_id
        os.killpg(detector.pid, signal.SIGINT)
        detector.wait(timeout=3)
        detector = subprocess.Popen(detector_command, start_new_session=True)
        wait_for(lambda: pose_node.detector_source_id not in (None, old_source),
                 'Detector publisher replacement was not observed.')
        count_before_restart = len(results['tag1_in_tag0/pose'])
        for index in range(8):
            image.header.stamp.sec = info.header.stamp.sec = 50 + index
            info_publisher.publish(info)
            image_publisher.publish(image)
            spin_for(0.2)
            if len(results['tag1_in_tag0/pose']) > count_before_restart:
                break
        assert len(results['tag1_in_tag0/pose']) > count_before_restart
        assert results['tag1_in_tag0/pose'][-1].header.stamp.sec < 100
        assert pose_node.pairs.source_changes == 1
        print('PASS: synthetic IDs 0/1, 20 mm scale, camera/source stamps, relative pose, '
              'duplicate suppression, no cross-image pairing, and detector restart epoch.')
    finally:
        if detector is not None and detector.poll() is None:
            os.killpg(detector.pid, signal.SIGINT)
            try:
                detector.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(detector.pid, signal.SIGTERM)
                detector.wait(timeout=3)
        if pose_node is not None:
            executor.remove_node(pose_node)
            pose_node.destroy_node()
        executor.remove_node(probe)
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
