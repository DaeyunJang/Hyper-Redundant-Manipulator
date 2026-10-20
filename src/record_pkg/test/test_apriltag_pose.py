"""Geometry and freshness checks independent of ROS, camera, and motors."""

import math

import pytest

from record_pkg.apriltag_pose_geometry import FreshTagPair, TagPose, relative_pose


def pose(tag='ID0', stamp=10, parent='camera', position=(0., 0., 0.),
         orientation=(0., 0., 0., 1.)):
    return TagPose(parent, tag, stamp, position, orientation)


def test_relative_pose_is_tag1_expressed_in_tag0():
    root_half = math.sqrt(0.5)
    tag0 = pose(position=(1., 2., 3.), orientation=(0., 0., root_half, root_half))
    tag1 = pose('ID1', position=(1., 3., 3.))
    relative = relative_pose(tag0, tag1)
    assert relative.parent == 'ID0' and relative.child == 'ID1'
    assert relative.stamp_ns == 10
    assert relative.position == pytest.approx((1., 0., 0.))
    assert relative.orientation == pytest.approx((0., 0., -root_half, root_half))


def test_separate_tf_messages_pair_without_republishing_cached_data():
    pairs = FreshTagPair()
    camera, relative = pairs.ingest(pose())
    assert camera is not None and relative is None
    camera, relative = pairs.ingest(pose('ID1', position=(1., 0., 0.)))
    assert camera is not None and relative is not None
    assert pairs.ingest(pose()) == (None, None)
    assert pairs.ingest(pose('ID1')) == (None, None)
    assert pairs.ingest(pose(stamp=9)) == (None, None)


def test_no_relative_pose_from_different_images_or_parents():
    pairs = FreshTagPair()
    pairs.ingest(pose())
    assert pairs.ingest(pose('ID1', stamp=11))[1] is None
    assert pairs.ingest(pose(stamp=11, parent='other_camera'))[1] is None
    with pytest.raises(ValueError, match='same parent and source stamp'):
        relative_pose(pose(), pose('ID1', stamp=11))


def test_missing_tag_never_produces_relative_and_cache_is_bounded():
    pairs = FreshTagPair(max_pending=3)
    for stamp in range(1, 100):
        camera, relative = pairs.ingest(pose(stamp=stamp))
        assert camera is not None and relative is None
        assert len(pairs.pending) <= 3
    assert pairs.ingest(pose('unrelated_tf')) == (None, None)


@pytest.mark.parametrize('bad_pose', [
    pose(stamp=0), pose(position=(float('nan'), 0., 0.)),
    pose(orientation=(0., 0., 0., 0.)), pose(parent=''),
    pose(orientation=(0., float('inf'), 0., 1.)),
])
def test_invalid_pose_is_rejected_without_advancing_freshness(bad_pose):
    pairs = FreshTagPair()
    with pytest.raises(ValueError):
        pairs.ingest(bad_pose)
    assert not pairs.pending and not pairs.last_stamp


def test_quaternion_is_normalized_without_changing_rotation():
    camera, _ = FreshTagPair().ingest(pose(orientation=(0., 0., 0., 2.)))
    assert camera.orientation == (0., 0., 0., 1.)


def test_publisher_replacement_allows_lower_epoch_without_cross_source_pairing():
    pairs = FreshTagPair()
    pairs.ingest(pose(stamp=100), source_id=b'old-publisher')
    camera, relative = pairs.ingest(pose('ID1', stamp=10), source_id=b'new-publisher')
    assert camera is not None and relative is None
    camera, relative = pairs.ingest(pose(stamp=10), source_id=b'new-publisher')
    assert camera is not None and relative is not None
    assert pairs.source_changes == 1
    assert pairs.ingest(pose(stamp=9), source_id=b'new-publisher') == (None, None)
    assert pairs.ingest(pose(stamp=10), source_id=b'new-publisher') == (None, None)


def test_same_stamp_never_pairs_tags_from_different_publishers():
    pairs = FreshTagPair()
    pairs.ingest(pose(), source_id=b'first')
    assert pairs.ingest(pose('ID1'), source_id=b'second')[1] is None
