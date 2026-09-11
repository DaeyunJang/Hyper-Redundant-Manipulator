import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from estimation_pkg.postprocess import RBSC


def test_straight_19_segment_layout_produces_18_joint_angles():
    config_path = (
        Path(__file__).parents[1] / 'estimation_pkg' / 'config.json'
    )
    with config_path.open(encoding='utf-8') as config_file:
        config = json.load(config_file)

    rbsc = RBSC.__new__(RBSC)
    rbsc.config = config
    rbsc.joint_kalman_states = None
    rbsc.joint_kalman_covariances = None
    rbsc.joint_kalman_timestamp_sec = None

    hardware_length_m = 1e-3 * (
        config['proximal_offset_length']
        + (config['num_of_bending_joints'] - 1)
        * config['bending_joint_spacing']
        + config['distal_offset_length']
    )
    x_coordinates = np.linspace(0.0, hardware_length_m, 200)
    straight_points = np.column_stack((
        x_coordinates,
        np.zeros_like(x_coordinates),
        np.zeros_like(x_coordinates),
    ))

    rbsc.reconstruct_3d_segments(straight_points, timestamp_sec=1.0)

    assert rbsc.segment_points_xyz.shape == (20, 3)
    assert rbsc.segment_center_points_xyz.shape == (19, 3)
    assert rbsc.segment_directions_xyz.shape == (19, 3)
    assert rbsc.segment_directions_projected_xyz.shape == (19, 3)
    assert rbsc.segment_directions_filtered_xyz.shape == (19, 3)
    assert rbsc.dh_joint_angles_filtered_rad.shape == (18,)
    assert rbsc.pan_relative_rad.shape == (18,)
    assert rbsc.tilt_relative_rad.shape == (18,)
    assert rbsc.dh_joint_frame_rotations_filtered.shape == (18, 3, 3)
    assert rbsc.segment_center_frame_rotations_filtered.shape == (19, 3, 3)
    np.testing.assert_allclose(rbsc.dh_joint_angles_filtered_rad, 0.0)
    np.testing.assert_allclose(
        rbsc.segment_center_points_xyz[0],
        [0.5 * config['length_of_segment'] * 1e-3, 0.0, 0.0],
        atol=1e-10,
    )
    np.testing.assert_allclose(
        rbsc.segment_points_xyz[-1],
        [hardware_length_m, 0.0, 0.0],
        atol=1e-10,
    )


def test_fixed_proximal_direction_is_excluded_from_18_joint_angles():
    config_path = (
        Path(__file__).parents[1] / 'estimation_pkg' / 'config.json'
    )
    with config_path.open(encoding='utf-8') as config_file:
        config = json.load(config_file)

    rbsc = RBSC.__new__(RBSC)
    rbsc.config = config
    rbsc.joint_kalman_states = None
    rbsc.joint_kalman_covariances = None
    rbsc.joint_kalman_timestamp_sec = None

    expected_angles = np.deg2rad(np.linspace(-4.0, 4.0, 18))
    base_x = np.array([1.0, 0.0, 0.0])
    moving_directions = []
    rotation = Rotation.identity()
    for index, joint_angle in enumerate(expected_angles):
        twist_degree = 0.0
        if index > 0:
            twist_degree = 90.0 if index % 2 == 1 else -90.0
        rotation = (
            rotation
            * Rotation.from_euler('x', twist_degree, degrees=True)
            * Rotation.from_euler('z', joint_angle)
        )
        moving_directions.append(rotation.apply(base_x))

    # Deliberately choose a fixed-segment measurement that cannot represent
    # q1. It must be stored for diagnostics but excluded from joint inversion.
    fixed_direction = np.array([0.0, 1.0, 0.0])
    rbsc.segment_directions_xyz = np.vstack((
        fixed_direction,
        np.asarray(moving_directions),
    ))

    rbsc.project_segment_directions_to_joint_planes(timestamp_sec=None)

    np.testing.assert_allclose(
        rbsc.fixed_base_segment_direction_raw_xyz,
        fixed_direction,
    )
    np.testing.assert_allclose(
        rbsc.dh_joint_angles_filtered_rad,
        expected_angles,
        atol=1e-12,
    )


def test_measured_curve_is_scaled_to_fixed_hardware_length():
    config_path = (
        Path(__file__).parents[1] / 'estimation_pkg' / 'config.json'
    )
    with config_path.open(encoding='utf-8') as config_file:
        config = json.load(config_file)

    rbsc = RBSC.__new__(RBSC)
    rbsc.config = config
    rbsc.joint_kalman_states = None
    rbsc.joint_kalman_covariances = None
    rbsc.joint_kalman_timestamp_sec = None

    hardware_length_m = 1e-3 * (
        config['proximal_offset_length']
        + (config['num_of_bending_joints'] - 1)
        * config['bending_joint_spacing']
        + config['distal_offset_length']
    )
    measured_length_m = 1.04 * hardware_length_m
    x_coordinates = np.linspace(0.0, measured_length_m, 200)
    measured_points = np.column_stack((
        x_coordinates,
        np.zeros_like(x_coordinates),
        np.zeros_like(x_coordinates),
    ))

    rbsc.reconstruct_3d_segments(measured_points, timestamp_sec=None)

    assert np.isclose(
        rbsc.fitted_curve_length_before_hardware_scaling_3d,
        measured_length_m,
    )
    assert np.isclose(rbsc.hardware_length_scale, 1.0 / 1.04)
    assert np.isclose(rbsc.curve_length_3d, hardware_length_m)
    np.testing.assert_allclose(
        rbsc.segment_points_xyz[:, 0],
        np.linspace(0.0, hardware_length_m, 20),
        atol=1e-10,
    )
    np.testing.assert_allclose(rbsc.segment_points_xyz[:, 1:], 0.0)
