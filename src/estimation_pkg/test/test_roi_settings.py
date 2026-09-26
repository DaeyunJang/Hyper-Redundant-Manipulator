"""Startup-only ROI settings and synthetic image cropping; no camera or motors."""

import importlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
CameraInfo = pytest.importorskip('sensor_msgs.msg').CameraInfo
module = importlib.import_module('estimation_pkg.segment_angle_estimation')


@pytest.mark.parametrize('values', [
    (-1, 0, 10, 10), (0, -1, 10, 10), (0, 0, 0, 10), (0, 0, 10, 0),
    (True, 0, 10, 10), (0, 0.0, 10, 10), (0, 0, '10', 10), (0, 0, 10, None),
])
def test_roi_requires_nonnegative_integer_origin_and_positive_integer_size(values):
    with pytest.raises(ValueError, match='must be an integer'):
        module.validate_roi(*values)


@pytest.fixture
def make_node(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '88')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    defaults = {'x': 2, 'y': 3, 'w': 4, 'h': 5}
    monkeypatch.setattr(
        module.SegmentEstimationNode, 'load_config', lambda self, path: dict(defaults))
    monkeypatch.setattr(module, 'RBSC', lambda: SimpleNamespace(config={}, last_error=''))
    monkeypatch.setattr(module, 'threadpool_limits', Mock())
    monkeypatch.setattr(module.cv2, 'setNumThreads', Mock())
    actual_process = module.SegmentEstimationNode.process
    monkeypatch.setattr(module.SegmentEstimationNode, 'process', lambda self: None)
    monkeypatch.setattr(module.SegmentEstimationNode, 'realtime_show', lambda self: None)
    node = None
    initialized = False

    def create(overrides=None):
        nonlocal node, initialized
        args = ['--ros-args']
        for key, value in (overrides or {}).items():
            args.extend(['-p', f'{key}:={value}'])
        rclpy.init(args=args)
        initialized = True
        node = module.SegmentEstimationNode()
        node._actual_process_for_test = actual_process
        return node

    yield create
    if node is not None:
        node.stop_workers()
        node.destroy_node()
    if initialized:
        rclpy.shutdown()


@pytest.mark.parametrize('overrides, expected', [
    ({}, (2, 3, 4, 5)),
    ({'roi_x': 1, 'roi_y': 2, 'roi_width': 6, 'roi_height': 7}, (1, 2, 6, 7)),
])
def test_node_uses_config_defaults_or_startup_overrides_and_refuses_live_changes(
        make_node, overrides, expected):
    node = make_node(overrides)
    names = ('roi_x', 'roi_y', 'roi_width', 'roi_height')
    assert (node.roi_x, node.roi_y, node.roi_w, node.roi_h) == expected
    for name, value in zip(names, expected):
        assert node.get_parameter(name).value == value
        assert node.describe_parameter(name).read_only
        response = node.set_parameters_atomically([Parameter(name, value=value + 1)])
        assert not response.successful
    assert (node.roi_x, node.roi_y, node.roi_w, node.roi_h) == expected


def image(node, values, encoding):
    message = node.br_rgb.cv2_to_imgmsg(values, encoding)
    message.header.stamp.sec = 10
    message.header.frame_id = 'synthetic_color_optical_frame'
    return message


def test_color_and_aligned_depth_use_same_rectangle_and_correct_full_image_offset(make_node):
    node = make_node()
    color = np.arange(12 * 14 * 3, dtype=np.uint8).reshape(12, 14, 3)
    depth = np.arange(12 * 14, dtype=np.uint16).reshape(12, 14)
    node.segment_crop_image_publisher = Mock()
    node.segment_crop_image_publisher.get_subscription_count.return_value = 1
    color_message = image(node, color, 'rgb8')
    depth_message = image(node, depth, '16UC1')
    node.color_image_rect_raw_callback(color_message)
    node.aligned_depth_callback(depth_message)
    np.testing.assert_array_equal(node.current_frame_ROI, color[3:8, 2:6])
    np.testing.assert_array_equal(node.current_depth, depth[3:8, 2:6])
    np.testing.assert_array_equal(node.latest_crop[0], color[3:8, 2:6])
    assert node.latest_crop[1] == color_message.header
    assert np.shares_memory(node.crop_to_roi(color, 'test'), color)
    calibration = CameraInfo()
    calibration.header = color_message.header
    calibration.k = [100., 0., 7., 0., 100., 6., 0., 0., 1.]
    node.camera_info_callback(calibration)

    def process_once(*args, **kwargs):
        node.shutdown_event.set()  # Stop before any synthetic TF/angle publication.
        return True

    node.rbsc.postprocess = Mock(side_effect=process_once)
    node._actual_process_for_test(node)
    node.rbsc.postprocess.assert_called_once()
    args, kwargs = node.rbsc.postprocess.call_args
    np.testing.assert_array_equal(args[0], color[3:8, 2:6])
    np.testing.assert_array_equal(kwargs['depth_image'], depth[3:8, 2:6])
    assert kwargs['roi_offset'] == (2, 3)
    assert kwargs['camera_intrinsics'] == {'fx': 100., 'fy': 100., 'cx': 7., 'cy': 6.}
    assert kwargs['depth_scale'] == 0.001


def test_out_of_bounds_frames_are_rejected_instead_of_silently_truncated(make_node):
    node = make_node()
    color = np.zeros((12, 14, 3), dtype=np.uint8)
    depth = np.ones((12, 14), dtype=np.uint16)
    node.color_image_rect_raw_callback(image(node, color, 'rgb8'))
    node.aligned_depth_callback(image(node, depth, '16UC1'))
    assert node.current_frame_flag and node.current_depth is not None
    # Configured y=3,h=5 requires at least height 8; NumPy alone would truncate.
    node.color_image_rect_raw_callback(image(node, color[:7], 'rgb8'))
    assert not node.current_frame_flag and node.current_frame_ROI is None
    assert node.color_stamp is None
    node.aligned_depth_callback(image(node, depth[:, :5], '16UC1'))
    assert node.current_depth is None and node.depth_stamp is None
    assert node.depth_scale is None
    assert node.failed_frame_count == 2


def test_roi_exactly_on_image_boundary_is_valid(make_node):
    node = make_node({'roi_x': 0, 'roi_y': 0, 'roi_width': 14, 'roi_height': 12})
    frame = np.zeros((12, 14, 3), dtype=np.uint8)
    cropped = node.crop_to_roi(frame, 'color')
    assert cropped.shape == frame.shape
    assert np.shares_memory(cropped, frame)


@pytest.mark.parametrize('overrides, expected', [
    ({}, {'roi_x': 20, 'roi_y': 30, 'roi_width': 40, 'roi_height': 50}),
    ({'roi_x': '1', 'roi_y': '2', 'roi_width': '3', 'roi_height': '4'},
     {'roi_x': 1, 'roi_y': 2, 'roi_width': 3, 'roi_height': 4}),
])
def test_launch_reads_roi_defaults_and_forwards_typed_integer_overrides(
        monkeypatch, tmp_path, overrides, expected):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch_log'))
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, LogInfo
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    (tmp_path / 'config_ROI_ref.json').write_text(json.dumps({'x': 20, 'y': 30, 'w': 40, 'h': 50}))
    path = Path(__file__).parents[1] / 'launch/_launch.py'
    spec = importlib.util.spec_from_file_location('estimation_roi_launch_test', path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda package: str(tmp_path))
    captured = {}

    def fake_node(**kwargs):
        captured.update(kwargs)
        return LogInfo(msg='No estimator or camera is launched by this test.')

    monkeypatch.setattr(launch_module, 'Node', fake_node)
    description = launch_module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    parameters = evaluate_parameters(context, normalize_parameters(captured['parameters']))
    assert list(parameters) == [expected]
    assert all(type(value) is int for value in parameters[0].values())
    assert captured['executable'] == 'segment_angle_estimator'
    assert captured['additional_env']['OPENBLAS_NUM_THREADS'] == '1'
