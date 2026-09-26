"""Metric depth-fit endpoints and source-stamped telemetry, without hardware."""

import json
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from estimation_pkg.postprocess import RBSC
from estimation_pkg.segment_angle_estimation import SegmentEstimationNode


def reconstructor():
    rbsc = RBSC.__new__(RBSC)
    config_file = Path(__file__).parents[1] / 'estimation_pkg/config.json'
    rbsc.config = json.loads(config_file.read_text())
    rbsc.joint_kalman_states = None
    rbsc.joint_kalman_covariances = None
    rbsc.joint_kalman_timestamp_sec = None
    return rbsc


@pytest.mark.parametrize('length_factor', [0.96, 1.0, 1.04])
def test_metric_fit_endpoint_is_preserved_before_hardware_scaling(length_factor):
    rbsc = reconstructor()
    hardware_length = rbsc.config['num_of_segments'] * rbsc.config['length_of_segment'] * 1e-3
    measured_length = hardware_length * length_factor
    points = np.column_stack((np.linspace(0, measured_length, 200), np.zeros((200, 2))))
    original = points.copy()
    rbsc.reconstruct_3d_segments(points, timestamp_sec=None)
    np.testing.assert_allclose(rbsc.tip_position_unscaled_xyz, [measured_length, 0, 0], atol=1e-10)
    np.testing.assert_allclose(rbsc.segment_points_xyz[-1], [hardware_length, 0, 0], atol=1e-10)
    np.testing.assert_array_equal(points, original)
    assert not np.shares_memory(rbsc.tip_position_unscaled_xyz, rbsc.curve_dense_xyz)
    assert rbsc.segment_points_xyz.shape == (20, 3)


def test_curved_xyz_tips_share_frame_but_keep_distinct_length_scales():
    rbsc = reconstructor()
    t = np.linspace(0, 1, 200)
    rbsc.reconstruct_3d_segments(np.column_stack((0.09*t, 0.01*t*t, -0.006*t*t*t)))
    assert not np.isclose(rbsc.hardware_length_scale, 1.0)
    assert rbsc.tip_position_unscaled_xyz[1] > 0
    assert rbsc.tip_position_unscaled_xyz[2] < 0
    np.testing.assert_allclose(
        rbsc.segment_points_xyz[-1],
        rbsc.tip_position_unscaled_xyz * rbsc.hardware_length_scale, atol=1e-12)
    assert not np.allclose(rbsc.segment_points_xyz[-1], rbsc.segment_center_points_xyz[-1])


def test_tip_messages_keep_meter_values_frame_and_original_source_stamp():
    node = SimpleNamespace(tip_position_publisher=Mock(), tip_position_unscaled_publisher=Mock())
    stamp = Time(sec=123, nanosec=456789)
    boundary = np.array([0.06, 0.015, -0.01])
    unscaled = boundary * 1.04
    SegmentEstimationNode.publish_tip_positions(node, stamp, 'hrm_base', boundary, unscaled)
    for publisher, expected in ((node.tip_position_publisher, boundary),
                                (node.tip_position_unscaled_publisher, unscaled)):
        message = publisher.publish.call_args.args[0]
        assert message.header.stamp == stamp
        assert message.header.frame_id == 'hrm_base'
        np.testing.assert_allclose([message.point.x, message.point.y, message.point.z], expected)
    boundary[:] = 99
    assert node.tip_position_publisher.publish.call_args.args[0].point.x == 0.06


@pytest.mark.parametrize('invalid', [[1, 2], [1, 2, float('nan')], [1, 2, float('inf')]])
def test_invalid_tip_does_not_publish_either_endpoint(invalid):
    node = SimpleNamespace(tip_position_publisher=Mock(), tip_position_unscaled_publisher=Mock())
    with pytest.raises(ValueError, match='finite XYZ'):
        SegmentEstimationNode.publish_tip_positions(
            node, Time(sec=1), 'hrm_base', [0, 0, 0], invalid)
    node.tip_position_publisher.publish.assert_not_called()
    node.tip_position_unscaled_publisher.publish.assert_not_called()


def test_failed_reconstruction_does_not_publish_previous_tip(monkeypatch):
    monkeypatch.setattr('estimation_pkg.segment_angle_estimation.rclpy.ok', lambda: True)
    shutdown = threading.Event()
    ready = threading.Event()
    ready.set()

    def failed_frame(*args, **kwargs):
        shutdown.set()
        return None

    node = SimpleNamespace(
        shutdown_event=shutdown, frame_event=ready, frame_lock=threading.Lock(),
        current_frame_flag=True, current_frame_ROI=np.zeros((2, 2, 3)),
        current_frame_encoding='rgb8', current_depth=np.ones((2, 2)),
        camera_intrinsics={}, camera_frame_id='camera_color_optical_frame',
        color_stamp=Time(sec=10), depth_stamp=Time(sec=10), depth_scale=0.001,
        max_depth_age_sec=0.1, roi_x=0, roi_y=0, failed_frame_count=0,
        stamp_to_seconds=SegmentEstimationNode.stamp_to_seconds,
        rbsc=SimpleNamespace(
            postprocess=Mock(side_effect=failed_frame), last_error='Invalid depth',
            segment_points_xyz=np.ones((20, 3)), tip_position_unscaled_xyz=np.ones(3)),
        get_logger=Mock(return_value=Mock()), publish_tip_positions=Mock(),
        publish_segment_angles=Mock(), publish_base_transform=Mock(),
    )
    SegmentEstimationNode.process(node)
    assert node.failed_frame_count == 1
    node.publish_tip_positions.assert_not_called()
    node.publish_segment_angles.assert_not_called()
    node.publish_base_transform.assert_not_called()
