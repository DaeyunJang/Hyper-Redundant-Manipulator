"""Tip feature ordering, CSV pose conventions and per-child TF freshness."""

import math
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

from estimation_pkg.force_id_inputs import FeatureHistory, feature_spec, pose_fields


@pytest.mark.parametrize('group,topic,child', [
    ('estimated_tip', '/estimated_tip_position', ''),
    ('fk_tip', '/kinematics/fk_tip_position', ''),
    ('fk_tip_tf', '/tf', 'hrm_fk_tip'),
])
@pytest.mark.parametrize('index,axis', enumerate('xyz'))
def test_position_mapping(group, topic, child, index, axis):
    feature = feature_spec(f'{group}.{axis}_m')
    assert (feature.topic, feature.field, feature.index, feature.size, feature.child_frame) == (
        topic, 'position', index, 3, child)


@pytest.mark.parametrize('group,child', [
    ('fk_tip_tf', 'hrm_fk_tip'),
    ('estimated_tip_orientation', 'estimated_segment_center_19'),
])
@pytest.mark.parametrize('index,axis', enumerate('xyzw'))
def test_quaternion_mapping(group, child, index, axis):
    feature = feature_spec(f'{group}.q{axis}')
    assert (feature.topic, feature.field, feature.index, feature.size, feature.child_frame) == (
        '/tf', 'quaternion', index, 4, child)


def test_quaternion_and_euler_match_summary_csv_not_canonicalized():
    # Negative, nonunit quaternion must remain unchanged in qx/qy/qz/qw,
    # while Euler uses its normalized value, as the recorded CSV does.
    fields = pose_fields(SimpleNamespace(x=-0.012, y=0., z=0.08),
                         SimpleNamespace(x=0., y=0., z=-1., w=-math.sqrt(3)))
    np.testing.assert_array_equal(fields['position'], [-0.012, 0., 0.08])
    np.testing.assert_array_equal(fields['quaternion'], [0., 0., -1., -math.sqrt(3)])
    np.testing.assert_allclose(fields['euler_deg'], [0., 0., 60.], atol=1e-12)
    for i, name in enumerate(('roll_deg', 'pitch_deg', 'yaw_deg')):
        spec = feature_spec('fk_tip_tf.' + name)
        assert spec.field == 'euler_deg' and spec.index == i


@pytest.mark.parametrize('rotation', [
    (0., 0., 0., 0.), (0., 0., float('nan'), 1.), (0., 0., 0., float('inf')),
])
def test_invalid_rotation_is_rejected(rotation):
    with pytest.raises(ValueError):
        pose_fields(rotation=SimpleNamespace(**dict(zip('xyzw', rotation))))


@pytest.mark.parametrize('name', [
    'estimated_tip_orientation.x_m', 'fk_tip_tf.frame_id',
    'fk_tip_tf.source_time_ns', 'estimated_tip.x_mm',
])
def test_unsupported_pose_columns_are_not_silently_substituted(name):
    with pytest.raises(ValueError):
        feature_spec(name)


def test_each_tf_child_is_a_separate_required_stream_in_bundle_order():
    history = FeatureHistory(['fk_tip_tf.z_m', 'estimated_tip_orientation.qw',
                              'fk_tip_tf.qx', 'estimated_tip.y_m'], 1)
    assert history.required_topics == {'/tf', '/estimated_tip_position'}
    assert len(history.required_sources) == 3
    stamp = 1_000_000_000

    def put(topic, values, child='', time_ns=stamp):
        history.put(topic, values, source_ns=time_ns, received_ns=time_ns,
                    received_monotonic_ns=time_ns, child_frame=child)

    put('/tf', {'position': [0.01, 0.02, 0.08], 'quaternion': [-0.5, 0., 0., -0.866]},
        'hrm_fk_tip')
    put('/estimated_tip_position', {'position': [0.02, -0.03, 0.07]})
    assert history.sample(stamp, stamp)[1].endswith('/tf:estimated_segment_center_19')
    put('/tf', {'quaternion': [0., 0., 0., -1.]}, 'estimated_segment_center_19')
    row, status = history.sample(stamp, stamp)
    assert status == 'ready'
    np.testing.assert_array_equal(row, [[0.08, -1., -0.5, -0.03]])
    put('/tf', {'quaternion': [0., 0., 0., 1.]}, 'camera_link', stamp + 10_000_000)
    assert len(history.samples) == 3
    # FK updates may not make the stopped estimated-orientation stream fresh.
    put('/tf', {'position': [0., 0., 0.08], 'quaternion': [0., 0., 0., 1.]},
        'hrm_fk_tip', stamp + 300_000_000)
    put('/estimated_tip_position', {'position': [0., 0., 0.08]},
        time_ns=stamp + 300_000_000)
    assert history.sample(stamp + 300_000_000, stamp + 300_000_000)[1] == (
        'stale_input:/tf:estimated_segment_center_19')


@pytest.fixture
def receiver(monkeypatch):
    # Exercise actual ROS message decoding without sockets or running hardware.
    pytest.importorskip('rclpy')
    from estimation_pkg import force_id_estimation as node_module
    harness = SimpleNamespace(
        history=FeatureHistory(['fk_tip_tf.qw', 'estimated_tip.x_m',
                                'estimated_tip_orientation.qz', 'fk_tip_tf.y_m'], 1),
        now_ns=1_000_000_000,
    )
    harness.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=harness.now_ns))
    harness._put_input = MethodType(node_module.ForceIdEstimationNode._put_input, harness)
    harness.receive = MethodType(node_module.ForceIdEstimationNode._receive, harness)
    monkeypatch.setattr(node_module.time, 'monotonic_ns', lambda: harness.now_ns)
    return harness


def transform(child, *, parent='hrm_base', stamp=1_000_000_000):
    from geometry_msgs.msg import TransformStamped
    result = TransformStamped()
    result.header.frame_id = parent
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 10**9)
    result.child_frame_id = child
    result.transform.translation.y = -0.03
    result.transform.rotation.z = -0.6
    result.transform.rotation.w = -0.8
    return result


def tip_point(frame='hrm_base'):
    from geometry_msgs.msg import PointStamped
    result = PointStamped()
    result.header.frame_id = frame
    result.header.stamp.sec = 1
    result.point.x = 0.075
    return result


def test_ros_callbacks_extract_only_matching_frames_and_original_stamps(receiver):
    from tf2_msgs.msg import TFMessage
    receiver.receive('/estimated_tip_position', tip_point())
    receiver.receive('/tf', TFMessage(transforms=[transform('hrm_fk_joint_01'),
                     transform('hrm_fk_tip', stamp=990_000_000)]))
    assert receiver.history.sample(receiver.now_ns, receiver.now_ns)[0] is None
    receiver.receive('/tf', TFMessage(transforms=[transform('estimated_segment_center_19')]))
    row, status = receiver.history.sample(receiver.now_ns, receiver.now_ns)
    assert status == 'ready'
    np.testing.assert_array_equal(row, [[-0.8, 0.075, -0.6, -0.03]])
    assert receiver.history.samples['/tf:hrm_fk_tip']['source_ns'] == 990_000_000
    assert set(receiver.history.samples) == {
        '/estimated_tip_position', '/tf:hrm_fk_tip', '/tf:estimated_segment_center_19'}


@pytest.mark.parametrize('stream', ['point', 'tf'])
def test_wrong_frame_is_not_treated_as_hrm_base(receiver, stream):
    from tf2_msgs.msg import TFMessage
    if stream == 'point':
        receiver.receive('/estimated_tip_position', tip_point('camera_color_optical_frame'))
        assert receiver.history.samples['/estimated_tip_position'] is None
    else:
        receiver.receive('/tf', TFMessage(transforms=[transform('hrm_fk_tip', parent='camera')]))
        assert receiver.history.samples['/tf:hrm_fk_tip'] is None
