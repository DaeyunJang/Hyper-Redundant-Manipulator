"""Crop depth is subscriber gated and serialized off the DDS image callback."""

import json
import threading
from unittest.mock import Mock

from cv_bridge import CvBridge
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from estimation_pkg.segment_angle_estimation import SegmentEstimationNode


def node_and_image(subscribers=1):
    node = SegmentEstimationNode.__new__(SegmentEstimationNode)
    node.roi_x, node.roi_y, node.roi_w, node.roi_h = 1, 2, 3, 2
    node.br_rgb = CvBridge()
    node.frame_lock, node.result_lock = threading.Lock(), threading.Lock()
    node.event = threading.Event()
    node.depth_received_count = 0
    node.received_first_depth = True
    node.depth_scale_override = 0.
    node.latest_crop_depth = None
    node.crop_depth_publisher = Mock()
    node.crop_depth_publisher.get_subscription_count.return_value = subscribers
    node.crop_depth_info_publisher = Mock()
    node.crop_depth_metadata_publisher = Mock()
    node.get_logger = Mock(return_value=Mock())
    data = np.arange(30, dtype=np.uint16).reshape(5, 6)
    message = node.br_rgb.cv2_to_imgmsg(data, '16UC1')
    message.header.stamp.sec = 12
    message.header.frame_id = 'camera'
    info = CameraInfo()
    info.header = message.header
    info.width, info.height = 6, 5
    info.k = [200., 0., 3., 0., 200., 2.5, 0., 0., 1.]
    info.p = [200., 0., 3., 0., 0., 200., 2.5, 0., 0., 0., 1., 0.]
    node.depth_archive_camera_info = info
    return node, message, data


def test_no_depth_archive_work_without_subscribers():
    node, message, _ = node_and_image(0)
    node.aligned_depth_callback(message)
    assert node.latest_crop_depth is None
    assert not node.event.is_set()
    node.crop_depth_publisher.publish.assert_not_called()


@pytest.mark.parametrize('override,scale', [(0., .001), (.0001, .0001)])
def test_callback_only_queues_worker_publishes_raw_crop_and_exact_calibration(override, scale):
    node, message, full = node_and_image()
    node.depth_scale_override = override
    node.aligned_depth_callback(message)
    assert node.event.is_set()
    node.crop_depth_publisher.publish.assert_not_called()
    node.crop_depth_metadata_publisher.publish.assert_not_called()
    node.publish_crop_depth(node.latest_crop_depth)
    depth = node.crop_depth_publisher.publish.call_args.args[0]
    assert depth.header == message.header
    assert depth.encoding == '16UC1'
    np.testing.assert_array_equal(node.br_rgb.imgmsg_to_cv2(depth), full[2:4, 1:4])
    info = node.crop_depth_info_publisher.publish.call_args.args[0]
    assert info.width == 3 and info.height == 2
    assert info.k[2] == 2. and info.k[5] == .5
    metadata = json.loads(node.crop_depth_metadata_publisher.publish.call_args.args[0].data)
    assert metadata['depth_scale_m_per_unit'] == scale
    assert metadata['source_time_ns'] == 12_000_000_000
    assert metadata['roi'] == dict(x=1, y=2, width=3, height=2)


def test_missing_camera_info_cannot_publish_uncalibrated_depth():
    node, message, _ = node_and_image()
    node.depth_archive_camera_info = None
    node.aligned_depth_callback(message)
    node.publish_crop_depth(node.latest_crop_depth)
    node.crop_depth_publisher.publish.assert_not_called()
    node.get_logger.return_value.warning.assert_called_once()
