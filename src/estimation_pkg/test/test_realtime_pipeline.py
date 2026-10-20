"""Preserve geometry while avoiding work and shared-state races in the GUI path."""
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from skimage.morphology import skeletonize
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from estimation_pkg.postprocess import RBSC
from estimation_pkg.segment_angle_estimation import SegmentEstimationNode
from estimation_pkg.runtime import configure_image_transport


@pytest.mark.parametrize('edge', [False, True])
@pytest.mark.parametrize('width', [7, 24, 60])
def test_bounded_lee_skeleton_preserves_all_pixels(edge, width):
    body = np.zeros((480, 640), dtype=np.uint8)
    y = np.arange(0 if edge else 35, 480 if edge else 445)
    x = (200 + 90 * np.sin(y / 200)).astype(np.int32)
    cv2.polylines(body, [np.column_stack((x, y))], False, 255, width)
    # Include an irregular side branch: don't silently change the thinning.
    cv2.line(body, (260, 250), (320, 270), 255, width)
    np.testing.assert_array_equal(
        RBSC.skeletonize_body(body), skeletonize(body, method='lee'))


def test_empty_roi_fails_without_per_frame_stdout(capsys):
    rbsc = RBSC.__new__(RBSC)
    assert rbsc.postprocess(np.zeros((480, 640, 3), np.uint8)) is None
    assert 'No HRM body' in rbsc.last_error
    assert capsys.readouterr().out == ''


def publisher(count=1):
    return SimpleNamespace(get_subscription_count=lambda: count)


def test_visualization_snapshot_is_independent_of_next_reconstruction():
    node = SegmentEstimationNode.__new__(SegmentEstimationNode)
    node.result_lock = threading.Lock()
    node.event = threading.Event()
    node.segment_skeleton_image_publisher = publisher()
    node.segment_body_binary_image_publisher = publisher()
    node.centerline_points_publisher = publisher()
    node.reconstruction_markers_publisher = publisher()
    rbsc = SimpleNamespace(base_frame_id='hrm_base', image=np.zeros((8, 8, 3)),
                           extended_yx_coords=np.ones((5, 2)), body_image=np.ones((8, 8)))
    names = ('points_xyz', 'curve_dense_xyz', 'segment_points_xyz', 'segment_tangents_xyz',
             'segment_center_points_xyz', 'segment_center_tangents_xyz',
             'segment_directions_xyz', 'segment_directions_projected_xyz',
             'segment_directions_filtered_xyz')
    for name in names:
        setattr(rbsc, name, np.ones((5, 3)))
    node.rbsc = rbsc
    first_stamp = Header().stamp
    first_stamp.sec = 1
    node.queue_visualization(first_stamp, 'camera', 'rgb8')
    packet = node.latest_visualization
    for name in names:
        getattr(rbsc, name)[:] = 2
    second_stamp = Header().stamp
    second_stamp.sec = 2
    node.queue_visualization(second_stamp, 'camera', 'rgb8')
    assert node.event.is_set()
    assert packet['stamp'].sec == 1
    assert node.latest_visualization['stamp'].sec == 2
    for name in names:
        np.testing.assert_array_equal(packet[name], np.ones((5, 3)))
        np.testing.assert_array_equal(node.latest_visualization[name], np.full((5, 3), 2))


def test_arrow_ids_update_without_deleteall_and_removed_arrows_are_deleted():
    node = SegmentEstimationNode.__new__(SegmentEstimationNode)
    node.previous_marker_keys = None
    node.visualization_curve_max_points = 251
    boundary = np.zeros((20, 3))
    centers = np.zeros((19, 3))
    def make(b, c):
        return node.make_reconstruction_markers(
            Header(), b, b, b, c, c, c, c, c).markers
    first = make(boundary, centers)
    second = make(boundary, centers)
    assert sum(m.action == Marker.DELETEALL for m in first) == 1
    assert all(m.action == Marker.ADD for m in second)
    assert sum(m.type == Marker.ARROW for m in second) == 96
    reduced = make(boundary[:-1], centers[:-1])
    assert sum(m.action == Marker.DELETE for m in reduced) == 5
    assert not any(m.action == Marker.DELETEALL for m in reduced)


def test_explicit_dds_profile_is_not_overwritten(monkeypatch):
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/operator/custom.xml')
    assert configure_image_transport() == '/operator/custom.xml'


def test_other_rmw_is_not_forced_to_fastdds(monkeypatch):
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    assert configure_image_transport() is None
