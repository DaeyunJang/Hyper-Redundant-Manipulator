"""Geometry publication remains optional, snapshot-based and source-stamped."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from estimation_pkg.segment_angle_estimation import SegmentEstimationNode


def publisher(count):
    return SimpleNamespace(get_subscription_count=lambda: count, publish=Mock())


def node_for_snapshot(geometry_subscribers=1):
    node = SegmentEstimationNode.__new__(SegmentEstimationNode)
    node.result_lock = threading.Lock()
    node.event = threading.Event()
    node.get_logger = Mock(return_value=Mock())
    node.segment_skeleton_image_publisher = publisher(1)
    node.segment_image_geometry_publisher = publisher(geometry_subscribers)
    node.segment_body_binary_image_publisher = publisher(0)
    node.centerline_points_publisher = publisher(0)
    node.reconstruction_markers_publisher = publisher(0)
    raw_s = np.linspace(0, 1, 400)
    boundaries = np.linspace(0, 1, 20)
    node.rbsc = SimpleNamespace(
        base_frame_id='hrm_base', image=np.zeros((480, 640, 3), dtype=np.uint8),
        extended_yx_coords=np.column_stack((450-400*raw_s, 200+100*raw_s)),
        raw_s_3d_source_indices=np.arange(400), raw_s_3d=raw_s,
        s_segment=boundaries, s_segment_center=(boundaries[:-1]+boundaries[1:])/2)
    return node


def test_snapshot_geometry_is_independent_of_next_result():
    node = node_for_snapshot()
    stamp = Time(sec=123, nanosec=456)
    node.queue_visualization(stamp, 'camera_color_optical_frame', 'rgb8')
    first = node.latest_visualization
    before = first['image_geometry'].pixels.copy()
    node.rbsc.extended_yx_coords[:, 1] += 10
    node.queue_visualization(Time(sec=124), 'camera_color_optical_frame', 'rgb8')
    np.testing.assert_array_equal(first['image_geometry'].pixels, before)
    np.testing.assert_allclose(node.latest_visualization['image_geometry'].pixels[:, 0],
                               before[:, 0]+10)
    assert first['stamp'] == stamp


def test_no_subscribers_does_not_build_optional_geometry():
    node = node_for_snapshot(0)
    del node.rbsc.raw_s_3d_source_indices
    node.queue_visualization(Time(sec=1), 'camera', 'rgb8')
    assert 'image_geometry' not in node.latest_visualization
    node.get_logger().warning.assert_not_called()


def test_bad_geometry_keeps_skeleton_and_regular_result():
    node = node_for_snapshot()
    node.rbsc.raw_s_3d[:] = 0
    node.queue_visualization(Time(sec=1), 'camera', 'rgb8')
    assert 'image_geometry' not in node.latest_visualization
    assert 'overlay' in node.latest_visualization
    assert node.event.is_set()
    node.get_logger().warning.assert_called_once()


def test_publication_keeps_source_header_crop_dimensions_and_pixels():
    node = node_for_snapshot()
    stamp = Time(sec=123, nanosec=456)
    node.queue_visualization(stamp, 'camera_color_optical_frame', 'rgb8')
    geometry = node.latest_visualization['image_geometry']
    header = Header(stamp=stamp, frame_id='camera_color_optical_frame')
    node.publish_segment_image_geometry(header, geometry)
    msg = node.segment_image_geometry_publisher.publish.call_args.args[0]
    assert msg.header == header
    assert (msg.width, msg.height, msg.samples_per_segment) == (640, 480, 9)
    assert len(msg.pixels) == 18*9
    assert len(msg.centers) == 18
    np.testing.assert_allclose([[p.x, p.y] for p in msg.pixels], geometry.pixels)
    assert all(p.z == 0 for p in msg.pixels)
    assert all(p.z == 0 for p in msg.centers)


def test_optional_publication_error_does_not_escape():
    node = node_for_snapshot()
    node.queue_visualization(Time(sec=1), 'camera', 'rgb8')
    node.segment_image_geometry_publisher.publish.side_effect = RuntimeError('DDS')
    node.publish_segment_image_geometry(Header(), node.latest_visualization['image_geometry'])
    node.get_logger().warning.assert_called_once()


def test_worker_publishes_geometry_without_other_visualization_subscribers(monkeypatch):
    node = node_for_snapshot()
    node.segment_skeleton_image_publisher = publisher(0)
    node.queue_visualization(Time(sec=123, nanosec=456), 'camera_color_optical_frame', 'rgb8')
    node.latest_crop = None
    node.latest_crop_depth = None
    node.shutdown_event = threading.Event()
    node.debug_timing_enabled = False
    node.visualized_frame_count = 0
    node.segment_image_geometry_publisher.publish.side_effect = lambda message: node.shutdown_event.set()
    monkeypatch.setattr('estimation_pkg.segment_angle_estimation.rclpy.ok', lambda: True)
    node.realtime_show()
    msg = node.segment_image_geometry_publisher.publish.call_args.args[0]
    assert msg.header.stamp == Time(sec=123, nanosec=456)
    assert msg.header.frame_id == 'camera_color_optical_frame'
    node.segment_skeleton_image_publisher.publish.assert_not_called()


def test_worker_continues_to_cloud_after_optional_geometry_publish_failure(monkeypatch):
    node = node_for_snapshot()
    node.segment_skeleton_image_publisher = publisher(0)
    node.centerline_points_publisher = publisher(1)
    node.rbsc.points_xyz = np.array([[0.0, 0.0, 0.0], [0.08, 0.0, 0.0]])
    node.queue_visualization(Time(sec=123), 'camera_color_optical_frame', 'rgb8')
    node.latest_crop = None
    node.latest_crop_depth = None
    node.shutdown_event = threading.Event()
    node.debug_timing_enabled = False
    node.visualized_frame_count = 0
    node.segment_image_geometry_publisher.publish.side_effect = RuntimeError('DDS')
    node.centerline_points_publisher.publish.side_effect = lambda message: node.shutdown_event.set()
    monkeypatch.setattr('estimation_pkg.segment_angle_estimation.rclpy.ok', lambda: True)
    node.realtime_show()
    node.segment_image_geometry_publisher.publish.assert_called_once()
    node.centerline_points_publisher.publish.assert_called_once()
    node.get_logger().warning.assert_called_once()
