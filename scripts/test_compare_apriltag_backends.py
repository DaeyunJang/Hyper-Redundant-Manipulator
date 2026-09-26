"""Pure backend metric/geometry tests; no ROS initialization or GPU execution."""

import math
import signal
from types import SimpleNamespace as Struct
from unittest.mock import Mock

import numpy as np
import pytest

from compare_apriltag_backends import (
    detection_record, normalize_pose, relative_pose, report_frames, stats, stop_owned,
)


def test_summary_p95_and_missing_backend_quality_fields():
    result = stats([1.0, 2.0, 3.0, None, float('nan')])
    assert result['count'] == 3 and result['mean'] == 2.0
    assert result['p95'] == pytest.approx(2.9)
    assert stats([None])['count'] == 0 and stats([])['p95'] is None


def test_relative_pose_rotates_translation_into_tag0_and_normalizes_quaternion():
    q = np.array([0.0, 0.0, math.sqrt(2), math.sqrt(2)])
    first = normalize_pose('camera', [1, 2, 3], q)
    assert q[2] == math.sqrt(2)  # Normalization must not mutate callers.
    second = normalize_pose('camera', [1, 3, 3], [0, 0, 0, -2])
    relative = relative_pose(first, second)
    np.testing.assert_allclose(relative['xyz_m'], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(relative['xyzw'], [0, 0, -math.sqrt(0.5), math.sqrt(0.5)],
                               atol=1e-12)
    assert relative['parent'] == 'ID0'
    assert relative_pose(first, dict(second, parent='different_camera')) is None
    assert normalize_pose('camera', [1, 2, 3], [0, 0, 0, 0]) is None


def test_isaac_pose_and_corners_no_invented_hamming_or_margin():
    pose = Struct(position=Struct(x=1., y=2., z=3.), orientation=Struct(x=0., y=0., z=0., w=2.))
    detector = Struct(
        id=1, family='tag36h11',
        corners=[Struct(x=x, y=y) for x, y in ((0, 0), (20, 0), (20, 20), (0, 20))],
        pose=Struct(header=Struct(frame_id=''), pose=Struct(pose=pose)),
    )
    record = detection_record(detector, 'camera_optical', isaac=True)
    assert record['hamming'] is None and record['decision_margin'] is None
    assert record['min_edge_pixels'] == 20.0 and record['corners_valid']
    assert record['pose']['xyzw'] == [0., 0., 0., 1.]
    assert record['pose']['parent'] == 'camera_optical'
    detector.hamming, detector.decision_margin = 1, 25.5
    record = detection_record(detector, 'camera_optical', isaac=False)
    assert record['hamming'] == 1 and record['decision_margin'] == 25.5
    assert record['pose'] is None  # CPU geometry must come from the matching TF stamp.


def test_missing_response_is_not_counted_as_detector_miss_or_connected_pose_step():
    def row(index, ids, status='response'):
        poses = ({'ID1': normalize_pose('camera', [index, 0, 0], [0, 0, 0, 1])}
                 if 1 in ids else {})
        return dict(index=index, source_stamp_ns=(index + 1) * 100_000_000,
                    status=status, ids=ids, detections=[], poses=poses, round_trip_ms=10.0)
    frames = [row(0, []), row(1, []), row(2, [], 'timeout'), row(3, []),
              row(4, [0, 1]), row(5, [1]), row(6, []), row(7, [1])]
    report = report_frames(frames, elapsed=2.0)
    assert report['response_timeouts'] == 1
    assert report['empty_response_frames'] == 4
    assert report['paired_id0_id1_frames'] == 1
    assert report['tags']['1']['detected_frames'] == 3
    run = report['longest_id1_miss_run']
    assert run == dict(frames=2, first_stamp_ns=100_000_000,
                       last_stamp_ns=200_000_000, source_span_sec=0.1)
    steps = report['tags']['1']['consecutive_pose_steps']['translation_step_m']
    assert steps['count'] == 1 and steps['mean'] == 1.0


def test_cleanup_signals_owned_group_when_launch_parent_already_exited(monkeypatch):
    import compare_apriltag_backends as module
    process = Mock(pid=12345)
    process.poll.return_value = 0
    alive, signals = True, []

    def kill_owned_group(pid, sig):
        nonlocal alive
        assert pid == process.pid
        signals.append(sig)
        if not alive:
            raise ProcessLookupError()
        if sig == signal.SIGTERM:
            alive = False
    monkeypatch.setattr(module.os, 'killpg', kill_owned_group)
    stop_owned(process)
    assert signals == [0, signal.SIGTERM, 0]
    process.send_signal.assert_not_called()
