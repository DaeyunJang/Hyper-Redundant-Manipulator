"""Contact overlay is a bounded, display-only image layer, not a control signal."""

from dataclasses import replace
import os
from types import SimpleNamespace
from unittest.mock import Mock

from cv_bridge import CvBridge
from custom_interfaces.msg import SegmentImageGeometry
from geometry_msgs.msg import Point32
import numpy as np
from PyQt5.QtCore import QRect
from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg.contact_overlay import contact_overlay
from gui_py_pkg.force_id_preview import ForceIdPreview, ForceIdSnapshot
from gui_py_pkg.image_preview import ImageCanvas
from gui_py_pkg.skeleton_preview import SkeletonFrameBuffer, SkeletonPreviewFrame, TIP_CACHE_SIZE


def image(sequence=10):
    msg = CvBridge().cv2_to_imgmsg(np.zeros((180, 360, 3), np.uint8), 'rgb8')
    msg.header.frame_id = 'camera_color_optical_frame'
    msg.header.stamp.sec = sequence
    return msg


def geometry(sequence=10):
    msg = SegmentImageGeometry()
    msg.header = image(sequence).header
    msg.width, msg.height, msg.samples_per_segment = 360, 180, 9
    for index in range(18):
        msg.pixels.extend(Point32(x=float(index * 18 + j), y=90., z=0.) for j in range(9))
    msg.centers = [Point32(x=float(index * 18 + 4), y=90., z=0.) for index in range(18)]
    return msg


def prediction(contact_id=1, probability=1., **changes):
    probabilities = [0.] * 19
    probabilities[contact_id] = probability
    probabilities[18 if contact_id != 18 else 1] += 1. - probability
    values = dict(valid=True, probabilities=tuple(probabilities), mode='masked',
                  contact_id=contact_id, heatmap_center_id=contact_id, load_state=0,
                  id_tol=0, source_stamp=10_000_000_000, received_at=100.)
    values.update(changes)
    return ForceIdSnapshot(**values)


def frame():
    return SkeletonPreviewFrame(image(), 100., geometry=geometry(), geometry_received_at=100.)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('segment_id', [1, 18])
def test_contact_ids_map_directly_to_geometry_without_fixed_base_offset(segment_id):
    overlay = contact_overlay(frame(), prediction(segment_id), 100.)
    assert len(overlay.segments) == 1
    actual_id, probability, points, center = overlay.segments[0]
    assert actual_id == segment_id and probability == 1.
    assert center == ((segment_id - 1) * 18 + 4, 90.)
    assert points[0][0] == (segment_id - 1) * 18


def test_probabilities_keep_raw_scale_and_tolerance_not_control_id():
    result = prediction(1, 0.4, id_tol=18, control_contact_id=18)
    overlay = contact_overlay(frame(), result, 100.)
    assert [(segment_id, p) for segment_id, p, _, _ in overlay.segments] == [(1, .4), (18, .6)]
    assert overlay.center_id == 1


@pytest.mark.parametrize('changes', [
    {'valid': False}, {'source_stamp': None}, {'received_at': None},
    {'source_stamp': float('nan')}, {'probabilities': ()},
    {'source_stamp': 9_499_999_999}, {'source_stamp': 10_500_000_001},
    {'received_at': 99.49}, {'received_at': 100.1},
    {'mode': 'class0', 'load_state': 1}, {'contact_id': 0, 'control_contact_id': 18},
])
def test_invalid_stale_unloaded_or_source_mismatched_inference_not_drawn(changes):
    assert contact_overlay(frame(), prediction(**changes), 100.) is None


def test_nearby_inference_need_not_have_identical_camera_timestamp():
    assert contact_overlay(frame(), prediction(source_stamp=10_400_000_000), 100.)


def test_masked_unloaded_hides_contact_without_replacing_raw_prediction():
    result = prediction(9, load_state=1, control_contact_id=18,
                        force_mn=(50., -30., 10.))
    assert contact_overlay(frame(), result, 100.) is None
    assert result.contact_id == 9 and result.probabilities[9] == 1.
    assert result.probabilities[18] == 0.
    assert result.force_mn == (50., -30., 10.)


def test_loaded_class0_zero_argmax_uses_positive_candidate_with_original_probability():
    probabilities = (0.8,) + (0.,) * 17 + (0.2,)
    result = prediction(18, mode='class0', load_state=2, control_contact_id=18,
                        probabilities=probabilities)
    result = replace(result, contact_id=0)
    overlay = contact_overlay(frame(), result, 100.)
    assert overlay is not None and overlay.center_id == 18
    assert len(overlay.segments) == 1
    assert overlay.segments[0][:2] == (18, .2)
    assert result.contact_id == 0 and result.probabilities == probabilities


@pytest.mark.parametrize('change', [
    lambda msg: setattr(msg, 'width', 359),
    lambda msg: setattr(msg, 'height', 181),
    lambda msg: setattr(msg, 'samples_per_segment', 0),
    lambda msg: setattr(msg, 'samples_per_segment', 65),
    lambda msg: setattr(msg, 'pixels', msg.pixels[:-1]),
    lambda msg: setattr(msg.pixels[0], 'x', float('nan')),
    lambda msg: setattr(msg.centers[0], 'y', float('inf')),
    lambda msg: setattr(msg.pixels[0], 'x', 1e6),
    lambda msg: setattr(msg.header, 'frame_id', 'another_camera'),
    lambda msg: setattr(msg.header.stamp, 'sec', 11),
])
def test_malformed_geometry_is_not_drawn(change):
    value = frame()
    change(value.geometry)
    assert contact_overlay(value, prediction(), 100.) is None


def test_stale_image_geometry_and_no_message_are_not_drawn():
    for value in (None, replace(frame(), geometry=None),
                  replace(frame(), received_at=99.79),
                  replace(frame(), geometry_received_at=99.79)):
        assert contact_overlay(value, prediction(), 100.) is None
    assert contact_overlay(frame(), None, 100.) is None


@pytest.mark.parametrize('geometry_first', [False, True])
def test_buffer_exact_image_geometry_stamp_and_frame_matching(geometry_first):
    cache = SkeletonFrameBuffer()
    im, geo = image(), geometry()
    events = [(cache.receive_image, im), (cache.receive_geometry, geo)]
    for receive, message in reversed(events) if geometry_first else events:
        receive(message, now=100.)
    assert cache.snapshot(100.).geometry is geo
    cache.receive_image(image(11), now=100.02)
    assert cache.snapshot(100.02).geometry is None
    wrong = geometry(11)
    wrong.header.frame_id = 'another_camera'
    cache.receive_geometry(wrong, now=100.02)
    assert cache.snapshot(100.02).geometry is None
    right = geometry(11)
    cache.receive_geometry(right, now=100.03)
    assert cache.snapshot(100.03).geometry is right
    assert cache.snapshot(100.24).geometry is None


def test_geometry_cache_bounded_and_existing_exact_tip_pairing_preserved():
    cache = SkeletonFrameBuffer()
    for sequence in range(1, 30):
        cache.receive_geometry(geometry(sequence), now=100.)
    assert len(cache.geometry) == TIP_CACHE_SIZE
    im = image(29)
    cache.receive_image(im, now=100.)
    tip = SimpleNamespace(header=im.header)
    cache.receive_tip('estimated_tip', tip, now=100.)
    cache.receive_tip('fk_tip', tip, now=100.)
    cache.receive_image(image(30), now=100.01)
    complete = cache.snapshot(100.01)
    assert complete.image is im and complete.estimated_tip is tip and complete.fk_tip is tip
    assert complete.geometry is not None


def test_source_stamp_is_retained_without_making_numeric_preview_require_one():
    shared = dict(header=SimpleNamespace(frame_id='hrm_base',
                                        stamp=SimpleNamespace(sec=10, nanosec=123)),
                  valid=True, status='ready', model_name='demo', mode='masked',
                  force_scope='all_single_contact', body_contact_enabled=False,
                  id_tol=0, samples_in_window=30, window_samples=30,
                  external_force=SimpleNamespace(x=0., y=0., z=0.),
                  probabilities=[0.] + [1.] + [0.] * 17,
                  contact_id=1, control_contact_id=-1, heatmap_center_id=1,
                  load_state=0, tip_force_valid=False, body_contact_detected=False)
    preview = ForceIdPreview()
    preview.receive(SimpleNamespace(**shared), now=100.)
    assert preview.snapshot(100.).source_stamp == 10_000_000_123
    assert preview.snapshot(100.).received_at == 100.
    shared['header'] = SimpleNamespace(frame_id='hrm_base')
    preview.receive(SimpleNamespace(**shared), now=100.)
    assert preview.snapshot(100.).valid and preview.snapshot(100.).source_stamp is None


def test_canvas_pixel_transform_accounts_for_source_size_and_letterbox(app):
    canvas = ImageCanvas()
    canvas.resize(300, 220)
    # Image converted to half-size; projection coordinates stay in original crop.
    pixels = np.zeros((90, 180, 3), np.uint8)
    canvas.set_rgb(pixels)
    overlay = contact_overlay(frame(), prediction(), 100.)
    area = QRect(20, 60, 260, 130)
    point = canvas.contact_point((179.5, 89.5), overlay, area)
    assert point.x() == 150. and point.y() == 125.
    canvas.set_overlay(('hrm_base | XYZ [mm]', 'Est: +1.0 +2.0 +3.0', 'FK: +1.0 +2.0 +3.0'))
    original = canvas.image.copy()
    canvas.set_contact_overlay(overlay)
    canvas.show()
    app.processEvents()
    assert canvas.image == original
    assert not pixels.any()
    canvas.close()


def test_force_only_updates_checkbox_and_expiry_clear_without_rgb_conversion(app, monkeypatch):
    from gui_py_pkg import image_preview
    clock = [100.]
    monkeypatch.setattr(image_preview.time, 'monotonic', lambda: clock[0])
    receiver = Mock()
    initial = frame()
    receiver.take_latest.return_value = ({'skeleton': initial}, {'skeleton': 100.})
    monkeypatch.setattr(image_preview, 'LatestImages', lambda context: receiver)
    panel = image_preview.ImagePreviewPanel(None)
    panel.timer.stop()
    panel.show()
    app.processEvents()
    try:
        panel.refresh()
        canvas = panel.canvases['skeleton']
        original = canvas.image.copy()
        convert = Mock(side_effect=AssertionError('Force changes must not reconvert images'))
        monkeypatch.setattr(image_preview, 'preview_rgb', convert)
        panel.set_force_id_snapshot(prediction(1))
        assert canvas.contact_overlay.center_id == 1
        panel.set_force_id_snapshot(prediction(18))
        assert canvas.contact_overlay.center_id == 18
        panel.contact_checkbox.setChecked(False)
        assert canvas.contact_overlay is None
        panel.contact_checkbox.setChecked(True)
        assert canvas.contact_overlay is not None
        receiver.take_latest.return_value = ({}, {'skeleton': 100.})
        clock[0] = 100.21
        panel.refresh()
        assert canvas.contact_overlay is None
        assert canvas.image == original
        assert not any(initial.image.data)
        convert.assert_not_called()
        panel.set_force_id_snapshot(None)
        assert canvas.contact_overlay is None
    finally:
        panel.stop()
        panel.close()
