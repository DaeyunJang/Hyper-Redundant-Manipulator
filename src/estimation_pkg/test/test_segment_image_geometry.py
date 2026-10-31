"""Crop-pixel correspondence is display-only and excludes the fixed base link."""

import json
from pathlib import Path

import numpy as np
import pytest

from estimation_pkg.segment_image_geometry import segment_image_pixels


def inputs():
    s = np.linspace(0.0, 1.0, 400)
    boundaries = np.linspace(0.0, 1.0, 20)
    return dict(
        extended_yx_coords=np.column_stack((450.0 - 400.0 * s, 200.0 + 100.0 * s)),
        source_indices=np.arange(len(s)), raw_s_3d=s,
        s_segment=boundaries,
        s_segment_center=(boundaries[:-1] + boundaries[1:]) / 2,
        image_shape=(480, 640, 3),
    )


def test_id_one_skips_fixed_base_and_id_eighteen_reaches_tip():
    values = inputs()
    result = segment_image_pixels(**values)
    segments = result.pixels.reshape(18, 9, 2)
    assert result.width == 640
    assert result.height == 480
    assert result.samples_per_segment == 9
    assert result.centers.shape == (18, 2)
    np.testing.assert_allclose(segments[0, 0], [200 + 100 / 19, 450 - 400 / 19])
    np.testing.assert_allclose(segments[-1, -1], [300, 50])
    np.testing.assert_allclose(segments[:-1, -1], segments[1:, 0])
    np.testing.assert_allclose(result.centers[0], [200 + 150 / 19, 450 - 600 / 19])
    np.testing.assert_allclose(result.centers[-1], [200 + 1850 / 19, 450 - 7400 / 19])


def test_curved_nonuniform_parameter_uses_existing_reconstruction_boundaries():
    values = inputs()
    s = values['raw_s_3d']
    values['extended_yx_coords'] = np.column_stack((450 - 380*s, 300 + 100*np.sin(3*s)))
    boundaries = np.linspace(0, 1, 20)**1.5
    values['s_segment'] = boundaries
    values['s_segment_center'] = (boundaries[:-1] + boundaries[1:]) / 2
    result = segment_image_pixels(**values)
    first_s = boundaries[1]
    np.testing.assert_allclose(result.pixels[0], [
        np.interp(first_s, s, values['extended_yx_coords'][:, 1]), 450 - 380*first_s])
    # Not a uniform split of the image pixels or the original sample indices.
    assert not np.isclose(result.pixels[0, 1], 450 - 380/19)


def test_source_indices_exclude_nonfinite_and_duplicate_xyz_pixel_samples():
    values = inputs()
    clean = segment_image_pixels(**values)
    pixels = values['extended_yx_coords']
    # Nonfinite and duplicate XYZ points have already been rejected by the
    # reconstruction. Their image coordinates must not enter interpolation.
    pixels = np.insert(pixels, 150, [[np.nan, np.nan], [-999, -999]], axis=0)
    values['extended_yx_coords'] = pixels
    values['source_indices'] = np.r_[np.arange(150), np.arange(152, len(pixels))]
    actual = segment_image_pixels(**values)
    np.testing.assert_array_equal(actual.pixels, clean.pixels)
    np.testing.assert_array_equal(actual.centers, clean.centers)


@pytest.mark.parametrize('error', [-8e-16, 4e-16])
def test_roundoff_at_normalized_endpoint_keeps_valid_geometry(error):
    values = inputs()
    values['raw_s_3d'][-1] += error
    result = segment_image_pixels(**values)
    np.testing.assert_allclose(result.pixels[-1], [300, 50], atol=1e-10)


@pytest.mark.parametrize('key,value', [
    ('raw_s_3d', np.zeros(400)),
    ('raw_s_3d', np.linspace(1, 0, 400)),
    ('raw_s_3d', np.full(400, np.nan)),
    ('source_indices', np.zeros(400, dtype=int)),
    ('source_indices', np.arange(400, dtype=float)),
    ('source_indices', np.arange(1, 401)),
    ('s_segment', np.linspace(0, 1, 19)),
    ('s_segment', np.zeros(20)),
    ('s_segment_center', np.linspace(0, 1, 19)),
    ('extended_yx_coords', np.zeros((3, 3))),
    ('extended_yx_coords', np.full((400, 2), np.nan)),
    ('extended_yx_coords', np.full((400, 2), 640)),
    ('image_shape', (0, 640, 3)),
    ('image_shape', (480, 20000, 3)),
    ('samples_per_segment', 1),
    ('samples_per_segment', 65),
    ('samples_per_segment', 3.5),
])
def test_invalid_geometry_is_rejected_without_mutating_inputs(key, value):
    values = inputs()
    values[key] = value
    with pytest.raises(ValueError):
        segment_image_pixels(**values)


def make_reconstructor():
    from estimation_pkg.postprocess import RBSC
    rbsc = RBSC.__new__(RBSC)
    path = Path(__file__).parents[1] / 'estimation_pkg/config.json'
    rbsc.config = json.loads(path.read_text())
    rbsc.joint_kalman_states = None
    rbsc.joint_kalman_covariances = None
    rbsc.joint_kalman_timestamp_sec = None
    return rbsc


def test_reconstruction_source_correspondence_does_not_change_angles_or_geometry():
    clean_rbsc = make_reconstructor()
    dirty_rbsc = make_reconstructor()
    t = np.linspace(0, 1, 200)
    points = np.column_stack((0.09*t, 0.01*t*t, -0.006*t*t*t))
    dirty_points = np.insert(points, 70, [points[69], [np.nan, 0, 0]], axis=0)
    dirty_original = dirty_points.copy()
    clean_rbsc.reconstruct_3d_segments(points)
    dirty_rbsc.reconstruct_3d_segments(dirty_points)
    expected_indices = np.r_[np.arange(70), np.arange(72, 202)]
    np.testing.assert_array_equal(dirty_rbsc.raw_s_3d_source_indices, expected_indices)
    np.testing.assert_array_equal(clean_rbsc.raw_s_3d_source_indices, np.arange(200))
    np.testing.assert_array_equal(dirty_points, dirty_original)
    names = ('coef_x', 'coef_y', 'coef_z', 'raw_s_3d', 's_segment', 's_segment_center',
             'segment_points_xyz', 'segment_center_points_xyz', 'segment_directions_xyz',
             'segment_directions_projected_xyz', 'segment_directions_filtered_xyz',
             'pan_relative_rad', 'tilt_relative_rad', 'dh_joint_angles_filtered_rad',
             'dh_joint_frame_rotations_filtered', 'tip_position_unscaled_xyz')
    before = {name: getattr(dirty_rbsc, name).copy() for name in names}
    for name in names:
        np.testing.assert_array_equal(getattr(clean_rbsc, name), before[name])
    pixels = np.column_stack((450-400*t, 200+100*t))
    dirty_pixels = np.insert(pixels, 70, [[-99, -99], [np.nan, np.nan]], axis=0)
    result = segment_image_pixels(
        dirty_pixels, dirty_rbsc.raw_s_3d_source_indices, dirty_rbsc.raw_s_3d,
        dirty_rbsc.s_segment, dirty_rbsc.s_segment_center, (480, 640, 3))
    assert result.pixels.shape == (162, 2)
    for name in names:
        np.testing.assert_array_equal(getattr(dirty_rbsc, name), before[name])


def test_hardware_length_scaling_never_reprojects_pixels():
    rbsc = make_reconstructor()
    t = np.linspace(0, 1, 200)
    # Deliberately differs from the hardware length; all pixel endpoints must
    # nevertheless coincide with the ORIGINAL visible skeleton.
    rbsc.reconstruct_3d_segments(np.column_stack((0.15*t, 0*t, 0*t)))
    assert rbsc.hardware_length_scale < 0.6
    pixels = np.column_stack((450-400*t, 200+100*t))
    result = segment_image_pixels(pixels, rbsc.raw_s_3d_source_indices,
                                  rbsc.raw_s_3d, rbsc.s_segment,
                                  rbsc.s_segment_center, (480, 640))
    np.testing.assert_allclose(result.pixels[-1], [300, 50])
    np.testing.assert_allclose(result.pixels[0], [200 + 100/19, 450 - 400/19])
