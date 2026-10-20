"""Synthetic capture tests; no ROS initialization, camera, GPU, or device access."""

import json
from types import SimpleNamespace as Struct
from unittest.mock import Mock

import numpy as np
import pytest

import capture_apriltag_comparison as capture


def messages(stamp_ns, *, width=3, height=2, focal=100., frame='camera_optical', encoding='rgb8'):
    stamp = Struct(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    header = Struct(stamp=stamp, frame_id=frame)
    image = Struct(header=header, width=width, height=height, encoding=encoding,
                   pixels=np.full((height, width, 3), [1, 2, 3], dtype=np.uint8))
    info = Struct(header=header, width=width, height=height,
                  k=[focal, 0., 1., 0., focal, 1., 0., 0., 1.])
    return image, info


def info_dict(info):
    return dict(header=dict(frame_id=info.header.frame_id,
                            stamp=vars(info.header.stamp)),
                width=info.width, height=info.height, k=info.k,
                distortion_model='plumb_bob', d=[0.] * 5, r=[0.] * 9, p=[0.] * 12)


def samples(frames=90, sample_hz=10):
    result = capture.CameraSamples(frames, sample_hz)
    result.to_dict = info_dict
    result.to_rgb = Mock(side_effect=lambda msg: (
        msg.pixels[:, :, ::-1] if msg.encoding == 'bgr8' else msg.pixels))
    return result


def pair(result, stamp_ns, **kwargs):
    image, info = messages(stamp_ns, **kwargs)
    result.push(0, image)
    result.push(1, info)
    return image, info


@pytest.mark.parametrize('values', [
    (0, 10, 30), (301, 10, 30), (True, 10, 30), (90, 0, 30), (90, float('nan'), 30),
    (90, 1e-320, 30), (90, 10, float('inf')), (90, 10, -1), (90, False, 30)])
def test_invalid_capture_options(values):
    with pytest.raises(ValueError):
        capture.validate_options(*values)


def test_exact_pairs_in_either_arrival_order_and_bounded_queues():
    result = samples()
    for index in range(30):
        result.push(0, messages(1_000_000_000 + index)[0])
    assert len(result.pending[0]) == 12 and result.counts['evicted'] == 18
    result.push(1, messages(1_000_000_001)[1])  # Evicted image cannot form a pair.
    assert not result.images
    image, info = messages(2_000_000_000)
    result.push(1, info)
    assert not result.images
    result.push(0, image)
    assert result.stamps == [2_000_000_000]
    for index in range(30):
        result.push(1, messages(3_000_000_000 + index)[1])
    assert len(result.pending[1]) == 12


def test_sampling_uses_unique_increasing_source_stamps_and_copies_rgb():
    result = samples(frames=3)
    first, _ = pair(result, 1_000_000_000, encoding='bgr8')
    pair(result, 1_000_000_000)
    pair(result, 900_000_000)
    pair(result, 0)
    for key in (1_033_333_333, 1_066_666_666, 1_099_999_999, 1_133_333_332,
                1_166_666_665, 1_199_999_998, 1_233_333_331, 1_300_000_000):
        pair(result, key)
    assert result.stamps == [1_000_000_000, 1_133_333_332, 1_233_333_331]
    assert result.to_rgb.call_count == 3
    first.pixels[:] = 0
    assert result.images[0][0, 0].tolist() == [3, 2, 1]
    assert json.loads(result.info_json)['header']['stamp']['sec'] == 1
    assert result.counts['invalid_stamp'] == 2


@pytest.mark.parametrize('kwargs', [
    {'width': 4}, {'height': 4}, {'focal': 101.}, {'frame': 'other'},
    {'focal': float('nan')}, {'focal': 0.}])
def test_calibration_changes_rejected_even_between_sampled_frames(kwargs):
    result = samples()
    pair(result, 1_000_000_000)
    with pytest.raises(ValueError):
        pair(result, 1_000_000_001, **kwargs)
    assert len(result.images) == 1


@pytest.mark.parametrize('field,value', [('width', 8), ('header', Struct(frame_id='other'))])
def test_image_info_mismatch_rejected(field, value):
    result = samples()
    image, info = messages(1_000_000_000)
    if field == 'header':
        value.stamp = image.header.stamp
    setattr(image, field, value)
    result.push(0, image)
    with pytest.raises(ValueError):
        result.push(1, info)


@pytest.mark.parametrize('status,expected_code', [
    ('complete', 0), ('timeout', 2), ('interrupted', 130), ('failed', 2)])
def test_cli_roundtrip_preserves_partial_samples_and_reports_failure(
        tmp_path, monkeypatch, status, expected_code):
    directory = tmp_path / status

    def fake_capture(args, result):
        result.to_dict, result.to_rgb = info_dict, lambda msg: msg.pixels
        pair(result, 1_000_000_000)
        pair(result, 1_100_000_000)
        if status == 'timeout':
            raise TimeoutError('Only 2/3 samples arrived.')
        if status == 'interrupted':
            raise KeyboardInterrupt()
        if status == 'failed':
            raise ValueError('Invalid calibration.')
    monkeypatch.setattr(capture, 'capture_live', fake_capture)
    frame_count = '2' if status == 'complete' else '3'
    code = capture.main(['--output', str(directory), '--frames', frame_count])
    assert code == expected_code
    metadata = json.loads((directory / 'capture_metadata.json').read_text())
    assert metadata['status'] == status and metadata['saved_frames'] == 2
    assert metadata['actual_sample_rate_hz'] == 10.
    assert metadata['first_source_stamp_ns'] == 1_000_000_000
    assert metadata['last_source_stamp_ns'] == 1_100_000_000
    assert metadata['topics']['image'] == '/camera/camera/color/image_rect_raw'
    with np.load(directory / 'capture.npz', allow_pickle=False) as data:
        assert data['images'].shape == (2, 2, 3, 3) and data['images'].dtype == np.uint8
        assert data['stamps_ns'].dtype == np.int64
        assert json.loads(data['camera_info_json'].item())['width'] == 3


def test_empty_timeout_has_metadata_but_no_npz(tmp_path, monkeypatch):
    monkeypatch.setattr(capture, 'capture_live', Mock(side_effect=TimeoutError('No camera.')))
    directory = tmp_path / 'empty'
    assert capture.main(['--output', str(directory)]) == 2
    assert not (directory / 'capture.npz').exists()
    metadata = json.loads((directory / 'capture_metadata.json').read_text())
    assert metadata['status'] == 'timeout' and metadata['saved_frames'] == 0
    assert metadata['actual_sample_rate_hz'] is None


def test_existing_directory_and_dangling_symlink_never_overwritten(tmp_path, monkeypatch):
    live = Mock()
    monkeypatch.setattr(capture, 'capture_live', live)
    link = tmp_path / 'link'
    link.symlink_to(tmp_path / 'missing_target', target_is_directory=True)
    for path in (tmp_path, link):
        with pytest.raises(SystemExit) as exc:
            capture.main(['--output', str(path)])
        assert exc.value.code == 2
    assert not (tmp_path / 'missing_target').exists()
    live.assert_not_called()


def test_dds_profile_respects_explicit_configuration_and_non_fastdds(monkeypatch):
    for name in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'RMW_IMPLEMENTATION'):
        monkeypatch.delenv(name, raising=False)
    profile = capture.configure_dds()
    assert profile.endswith('/src/estimation_pkg/config/fastdds_images.xml')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/explicit/profile.xml')
    assert capture.configure_dds() == '/explicit/profile.xml'
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    assert capture.configure_dds() is None


def test_actual_cvbridge_bgr8_conversion_without_ros_initialization():
    cv_bridge = pytest.importorskip('cv_bridge')
    bridge = cv_bridge.CvBridge()
    message = bridge.cv2_to_imgmsg(np.full((2, 3, 3), [1, 2, 3], np.uint8), encoding='bgr8')
    rgb = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
    assert rgb[0, 0].tolist() == [3, 2, 1] and rgb.dtype == np.uint8
