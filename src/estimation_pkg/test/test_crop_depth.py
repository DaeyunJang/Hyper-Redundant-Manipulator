"""Cropped depth intrinsics/units remain explicit and source time is preserved."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from estimation_pkg.crop_depth import crop_depth_calibration


def camera():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id='camera'),
        width=100, height=80, k=[200., 0., 50., 0., 201., 40., 0., 0., 1.],
        p=[200., 0., 50., 0., 0., 201., 40., 0., 0., 0., 1., 0.],
        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], d=[0.] * 5,
        roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
        binning_x=0, binning_y=0, distortion_model='plumb_bob')


def test_intrinsics_use_crop_pixels_and_do_not_modify_source():
    original = camera()
    before = deepcopy(original)
    header = deepcopy(original.header)
    header.stamp.sec = 12
    info, metadata = crop_depth_calibration(
        original, header, '16UC1', .001, (80, 100), (10, 20, 30, 40))
    assert original == before
    assert info.width == 30 and info.height == 40
    assert info.k[2] == info.p[2] == 40
    assert info.k[5] == info.p[6] == 20
    assert info.header == header
    assert info.roi.width == 0 and info.roi.x_offset == 0
    assert metadata['roi'] == dict(x=10, y=20, width=30, height=40)
    assert metadata['source_time_ns'] == 12_000_000_002
    assert metadata['depth_scale_m_per_unit'] == .001
    assert metadata['source_width'] == 100


@pytest.mark.parametrize('invalid', ['frame', 'dimensions', 'roi', 'scale', 'encoding'])
def test_invalid_depth_calibration_rejected(invalid):
    original = camera()
    header = deepcopy(original.header)
    shape, roi, scale, encoding = (80, 100), (10, 20, 30, 40), .001, '16UC1'
    if invalid == 'frame':
        header.frame_id = 'other'
    elif invalid == 'dimensions':
        shape = (81, 100)
    elif invalid == 'roi':
        roi = (90, 20, 30, 40)
    elif invalid == 'scale':
        scale = 0
    else:
        encoding = 'rgb8'
    with pytest.raises(ValueError):
        crop_depth_calibration(original, header, encoding, scale, shape, roi)
