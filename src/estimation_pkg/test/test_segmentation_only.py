"""Offline mask export stops at the unchanged production pre-skeleton stage."""

import cv2
import numpy as np
import pytest

from estimation_pkg.postprocess import RBSC


def synthetic_bgr():
    image = np.zeros((180, 240, 3), dtype=np.uint8)
    # Main bright body, with a narrow gap and a hole that smoothing closes.
    cv2.rectangle(image, (75, 20), (110, 160), (240, 240, 240), -1)
    cv2.rectangle(image, (110, 110), (155, 145), (180, 180, 180), -1)
    cv2.rectangle(image, (89, 70), (95, 76), (0, 0, 0), -1)
    cv2.rectangle(image, (105, 114), (110, 119), (0, 0, 0), -1)
    # Smaller white components must not become part of the HRM body.
    cv2.rectangle(image, (10, 20), (28, 40), (255, 255, 255), -1)
    cv2.rectangle(image, (195, 110), (220, 130), (130, 130, 130), -1)
    # Both exclusion masks see bright-enough pixels; these are not merely
    # dark colors that would have failed the grayscale threshold anyway.
    image[25:65, 125:165] = (80, 170, 200)
    blue = cv2.cvtColor(np.array([[[110, 100, 255]]], np.uint8), cv2.COLOR_HSV2BGR)
    image[15:65, 180:225] = blue[0, 0]
    return image


def reference_stage(image, color_encoding, binary_thresh=120):
    """Independent copy of the pre-existing stage, including all constants."""
    hsv_code, gray_code = (
        (cv2.COLOR_RGB2HSV, cv2.COLOR_RGB2GRAY)
        if color_encoding == 'rgb8'
        else (cv2.COLOR_BGR2HSV, cv2.COLOR_BGR2GRAY)
    )
    hsv = cv2.cvtColor(image, hsv_code)
    excluded = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 50, 50]), np.array([255, 220, 220])),
        cv2.inRange(hsv, np.array([100, 70, 50]), np.array([130, 255, 255])),
    )
    gray = cv2.cvtColor(image, gray_code)
    gray[excluded != 0] = 0
    _, binary = cv2.threshold(gray, binary_thresh, 255, cv2.THRESH_BINARY)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    assert count > 1
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    body_origin = (labels == largest).astype(np.uint8) * 255
    closed = cv2.morphologyEx(
        body_origin, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8)
    )
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros_like(closed)
    cv2.drawContours(filled, contours, -1, 255, thickness=cv2.FILLED)
    blurred = cv2.GaussianBlur(filled, (15, 15), 0)
    _, smoothed = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
    return gray, binary, body_origin, smoothed


@pytest.mark.parametrize('color_encoding', ['bgr8', 'rgb8'])
@pytest.mark.parametrize('binary_thresh', [100, 120, 200])
def test_segmentation_only_matches_existing_stage_exactly(color_encoding, binary_thresh):
    image = synthetic_bgr()
    if color_encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    before = image.copy()
    expected = reference_stage(image, color_encoding, binary_thresh)
    rbsc = RBSC.__new__(RBSC)

    mask = rbsc.postprocess(
        image, color_encoding=color_encoding, binary_thresh=binary_thresh,
        segmentation_only=True,
    )

    assert rbsc.last_error is None
    assert mask is rbsc.body_image
    assert mask.dtype == np.uint8
    assert mask.shape == image.shape[:2]
    assert set(np.unique(mask)) == {0, 255}
    for actual, reference in zip(
            (rbsc.gray_image, rbsc.binary_image, rbsc.body_image_origin, mask), expected):
        np.testing.assert_array_equal(actual, reference)
    np.testing.assert_array_equal(image, before)
    # The exported image is the smoothed body, not threshold-only output.
    assert not np.array_equal(mask, rbsc.binary_image)
    assert not np.array_equal(mask, rbsc.body_image_origin)


def test_rgb_and_bgr_archives_produce_identical_masks():
    bgr = synthetic_bgr()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    a, b = RBSC.__new__(RBSC), RBSC.__new__(RBSC)
    np.testing.assert_array_equal(
        a.postprocess(bgr, color_encoding='bgr8', segmentation_only=True),
        b.postprocess(rgb, color_encoding='rgb8', segmentation_only=True),
    )


def test_segmentation_only_never_enters_skeleton_or_3d(monkeypatch):
    rbsc = RBSC.__new__(RBSC)
    calls = []

    def forbidden(*args, **kwargs):
        calls.append(True)
        raise RuntimeError('later stage must not run')

    monkeypatch.setattr(rbsc, 'skeletonize_body', forbidden)
    monkeypatch.setattr(rbsc, 'deproject_skeleton_to_3d', forbidden)
    monkeypatch.setattr(rbsc, 'reconstruct_3d_segments', forbidden)
    result = rbsc.postprocess(synthetic_bgr(), segmentation_only=True)
    assert isinstance(result, np.ndarray)
    assert not calls
    assert rbsc.last_error is None


@pytest.mark.parametrize('kwargs', [{}, {'segmentation_only': False}])
def test_default_postprocess_still_enters_skeleton(monkeypatch, kwargs):
    rbsc = RBSC.__new__(RBSC)
    calls = []

    def skeleton_sentinel(body):
        calls.append(body)
        raise RuntimeError('skeleton sentinel reached')

    monkeypatch.setattr(rbsc, 'skeletonize_body', skeleton_sentinel)
    assert rbsc.postprocess(synthetic_bgr(), **kwargs) is None
    assert len(calls) == 1
    assert calls[0] is rbsc.body_image
    assert 'skeleton sentinel reached' in rbsc.last_error


@pytest.mark.parametrize('bad_image, message', [
    (None, 'color ROI is empty'),
    (np.zeros((0, 0, 3), np.uint8), 'color ROI is empty'),
    (np.zeros((180, 240, 3), np.uint8), 'No HRM body detected'),
])
def test_failed_frame_returns_none_instead_of_previous_mask(bad_image, message):
    rbsc = RBSC.__new__(RBSC)
    previous = rbsc.postprocess(synthetic_bgr(), segmentation_only=True)
    assert isinstance(previous, np.ndarray)
    result = rbsc.postprocess(bad_image, segmentation_only=True)
    assert result is None
    assert message in rbsc.last_error
    # A later valid image succeeds normally and clears the failure status.
    recovered = rbsc.postprocess(synthetic_bgr(), segmentation_only=True)
    np.testing.assert_array_equal(recovered, previous)
    assert rbsc.last_error is None
