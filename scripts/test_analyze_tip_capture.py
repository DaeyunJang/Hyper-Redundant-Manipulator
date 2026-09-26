"""Pure offline tip analysis tests: no ROS initialization or hardware access."""

import csv
import json
import math

import numpy as np
import pytest

import analyze_tip_capture as analysis


def header(stamp=1_000_000_000, frame='hrm_base'):
    return dict(stamp=dict(sec=stamp // 10**9, nanosec=stamp % 10**9), frame_id=frame)


def vector(x=0., y=0., z=0.):
    return dict(x=x, y=y, z=z)


def orientation(angle=0.):
    return dict(x=0., y=0., z=math.sin(angle / 2), w=math.cos(angle / 2))


def point(stamp=1_000_000_000, frame='hrm_base', x=.08):
    return dict(header=header(stamp, frame), point=vector(x))


def transform(stamp=1_000_000_000, child='hrm_fk_tip', frame='hrm_base', angle=0., x=.07):
    return dict(header=header(stamp, frame), child_frame_id=child,
                transform=dict(translation=vector(x), rotation=orientation(angle)))


def tag(stamp=1_000_000_000, angle=0.):
    return dict(header=header(stamp, 'ID0'),
                pose=dict(position=vector(.15, -.02, .01), orientation=orientation(angle)))


def add_tf(data, **kwargs):
    data.consume('/tf', dict(transforms=[transform(**kwargs)]), 1_100_000_000)


def test_direct_fk_position_uses_exact_stamp_tf_orientation_not_tf_translation():
    data = analysis.CaptureData()
    add_tf(data, angle=math.pi / 2, x=.071)
    data.consume('/kinematics/fk_tip_position', point(x=.080), 1_200_000_000)
    row = data.finalized()['fk_tip_base'][0]
    assert row['x_m'] == .080
    assert row['x_mm'] == 80
    assert row['bag_receive_time_ns'] == 1_200_000_000
    assert row['yaw_xyz_deg'] == pytest.approx(90)
    assert row['provenance'] == '/kinematics/fk_tip_position'
    assert row['orientation_provenance'] == '/tf:hrm_fk_tip'
    assert row['child_frame_id'] == 'hrm_fk_tip'


def test_missing_fk_point_uses_tf_and_repeated_tf_does_not_increase_count():
    data = analysis.CaptureData()
    add_tf(data)
    add_tf(data)
    add_tf(data, stamp=1_010_000_000)
    result = data.finalized()['fk_tip_base']
    assert len(result) == 2
    assert result[0]['x_m'] == .07
    assert result[0]['provenance'] == '/tf:hrm_fk_tip'
    assert data.counts['fk_tip_tf:duplicate_source_stamp'] == 1


def test_no_nearby_tf_orientation_fill_and_no_tf_fill_for_direct_point_gaps():
    data = analysis.CaptureData()
    add_tf(data, stamp=1_020_000_000)
    data.consume('/kinematics/fk_tip_position', point(), 2_000_000_000)
    row = data.finalized()['fk_tip_base'][0]
    assert 'qw' not in row
    assert len(data.finalized()['fk_tip_base']) == 1


def test_estimated_center_rotation_never_exports_center_as_tip_position():
    data = analysis.CaptureData()
    add_tf(data, child='estimated_segment_center_19', angle=.2, x=.06)
    row = data.finalized()['estimated_tip_orientation_base'][0]
    assert row['child_frame_id'] == 'estimated_segment_center_19'
    assert 'qw' in row and 'x_m' not in row and 'x_mm' not in row


def test_boundary_tangent_is_arrow_endpoint_difference_not_marker_rotation():
    data = analysis.CaptureData()
    marker = dict(header=header(), ns='curve_tangents', id=119, action=0,
                  pose=dict(orientation=orientation(0)),
                  points=[vector(.08, .01, .02), vector(.08, .016, .02)])
    data.consume(analysis.MARKER_TOPIC, dict(markers=[marker]), 1_050_000_000)
    row = data.finalized()['estimated_boundary_tangent_base'][0]
    assert row['direction_y'] == pytest.approx(1)
    assert row['direction_azimuth_deg'] == pytest.approx(90)
    assert row['x_mm'] == 80
    marker['action'] = 2  # DELETE is not a measurement.
    data.consume(analysis.MARKER_TOPIC, dict(markers=[marker]), 1_060_000_000)
    assert len(data.finalized()['estimated_boundary_tangent_base']) == 1


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_invalid_xyz_and_quaternion_are_rejected_not_zero_filled(bad):
    data = analysis.CaptureData()
    data.consume('/estimated_tip_position', point(x=bad), 1)
    malformed = tag()
    malformed['pose']['orientation']['w'] = bad
    data.consume(analysis.POSE_TOPIC, malformed, 1)
    assert not data.finalized()['estimated_tip_base']
    assert not data.finalized()['tag_tip_id0']
    assert data.counts['/estimated_tip_position:invalid_sample'] == 1


def test_zero_quaternion_and_missing_source_stamp_rejected():
    data = analysis.CaptureData()
    malformed = tag()
    malformed['pose']['orientation'] = dict(x=0, y=0, z=0, w=0)
    data.consume(analysis.POSE_TOPIC, malformed, 1)
    data.consume('/estimated_tip_position', point(stamp=0), 1)
    assert len(data.finalized()['tag_tip_id0']) == 0
    assert len(data.finalized()['estimated_tip_base']) == 0


def simple_row(stamp, frame='hrm_base', x=.08):
    return dict(source_time_ns=stamp, frame_id=frame, x_mm=x * 1000, y_mm=0., z_mm=0.)


def test_matching_exact_preferred_tolerance_inclusive_and_frames_rejected():
    left = [simple_row(stamp) for stamp in [100_000_000, 200_000_000, 300_000_000]]
    left += [simple_row(100_000_000, 'ID0')]
    right = [simple_row(stamp) for stamp in [
        99_000_000, 100_000_000, 225_000_000, 325_000_001]]
    right += [simple_row(300_000_000, 'ID0')]
    pairs = list(analysis.nearest_pairs(left, right, 25_000_000))
    assert [(a['source_time_ns'], b['source_time_ns']) for a, b in pairs] == [
        (100_000_000, 100_000_000), (200_000_000, 225_000_000)]


def test_quaternion_sign_change_is_not_motion():
    q = np.array([0., 0., 0., 1.])
    assert analysis.rotation_change_deg(q, -q) == 0.
    assert analysis.rotation_change_deg(q, np.array([0., 0., 1., 0.])) == 180.


def test_rotation_change_magnitude_is_invariant_to_fixed_mount_rotations():
    from scipy.spatial.transform import Rotation
    initial = Rotation.from_euler('xyz', [10, 20, 30], degrees=True)
    current = Rotation.from_euler('xyz', [20, 40, 60], degrees=True)
    left = Rotation.from_euler('x', 70, degrees=True)
    right = Rotation.from_euler('z', -35, degrees=True)
    original = analysis.rotation_change_deg(initial.as_quat(), current.as_quat())
    mounted = analysis.rotation_change_deg((left * initial * right).as_quat(),
                                           (left * current * right).as_quat())
    assert original == pytest.approx(mounted)


def test_detections_missing_id_is_counted_without_repeating_pose():
    data = analysis.CaptureData()
    for stamp, ids in [(1_000_000_000, [0, 1]), (1_100_000_000, [0]),
                       (1_200_000_000, [])]:
        data.consume('/detections', dict(
            header=header(stamp, 'camera'), detections=[dict(id=value) for value in ids]), stamp)
    data.consume(analysis.POSE_TOPIC, tag(), 1_010_000_000)
    assert data.detections['frames'] == 3
    assert data.detections['both_0_and_1_frames'] == 1
    assert data.detections['empty_frames'] == 1
    assert data.detected_ids[0] == 2 and data.detected_ids[1] == 1
    assert len(data.finalized()['tag_tip_id0']) == 1


def test_csv_report_preserves_frames_timestamps_nofill_and_unknown_calibration(tmp_path):
    data = analysis.CaptureData()
    data.consume(analysis.POSE_TOPIC, tag(), 1_010_000_000)
    data.consume('/estimated_tip_position', point(x=.080), 1_100_000_000)
    data.consume('/estimated_tip_position', point(stamp=2_000_000_000), 2_100_000_000)
    data.consume('/estimated_tip_position_unscaled', point(x=.090), 1_100_000_000)
    add_tf(data, x=.075)
    output = tmp_path / 'analysis'
    summary = analysis.write_analysis(data, output)
    assert summary['depth_fk']['matched'] == 1
    assert summary['depth_fk']['unmatched_estimated_samples'] == 1
    assert summary['depth_fk']['difference_norm_mm']['mean'] == pytest.approx(5)
    assert summary['calibrated_tag_position_error_mm'] is None
    assert summary['calibrated_tag_orientation_error_deg'] is None
    assert summary['streams']['tag_tip_id0']['frames'] == ['ID0']
    assert summary['streams']['estimated_tip_base']['frames'] == ['hrm_base']
    with (output / 'tag_tip_id0.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]['source_time_ns'] == '1000000000'
    assert rows[0]['bag_receive_time_ns'] == '1010000000'
    with (output / 'estimated_boundary_tangent_base.csv').open() as stream:
        assert list(csv.DictReader(stream)) == []
    assert json.loads((output / 'summary.json').read_text()) == summary
    with pytest.raises(FileExistsError):
        analysis.write_analysis(data, output)


def test_report_does_not_mix_position_statistics_of_different_frames():
    data = analysis.CaptureData()
    data.consume('/estimated_tip_position', point(), 1)
    data.consume('/estimated_tip_position', point(stamp=2_000_000_000, frame='camera'), 2)
    stats = analysis.series_summary(data.finalized()['estimated_tip_base'])
    assert stats['statistics_unavailable_reason'] == 'Mixed coordinate frames'
    assert 'position_std_mm' not in stats


def test_angular_scatter_uses_quaternion_mean_not_first_or_euler_wrap():
    data = analysis.CaptureData()
    for index, degrees in enumerate((179., 181.)):
        data.consume(analysis.POSE_TOPIC,
                     tag(1_000_000_000 + index, math.radians(degrees)), index)
    stats = analysis.series_summary(data.finalized()['tag_tip_id0'])
    assert stats['orientation_geodesic_from_mean_deg']['rms'] == pytest.approx(1.)
    assert stats['orientation_geodesic_from_mean_deg']['max'] == pytest.approx(1.)


def test_cli_rejects_loose_tolerance_without_reading_bag(tmp_path, monkeypatch):
    def forbidden(*args):
        raise AssertionError('Must not read a bag')
    monkeypatch.setattr(analysis, 'read_bag', forbidden)
    for value in ['25.001', '-1', 'nan', 'inf']:
        with pytest.raises(SystemExit) as error:
            analysis.main([str(tmp_path), '--tolerance-ms', value])
        assert error.value.code == 2


def test_joint_angles_arrays_remain_radians_json_and_not_tip_euler():
    data = analysis.CaptureData()
    message = dict(header=header(), **{name: [0.1] * 18 for name in analysis.ANGLE_FIELDS})
    data.consume(analysis.ANGLE_TOPIC, message, 1_020_000_000)
    row = data.finalized()['joint_angles'][0]
    assert json.loads(row['pan_relative']) == [0.1] * 18
    assert 'roll_xyz_deg' not in row
