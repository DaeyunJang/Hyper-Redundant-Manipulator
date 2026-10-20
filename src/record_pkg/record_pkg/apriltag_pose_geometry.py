"""Small ROS-independent helpers for same-image AprilTag poses (metres, xyzw)."""

from collections import OrderedDict
from dataclasses import dataclass
import math


def unit_quaternion(quaternion):
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('AprilTag quaternion must contain four finite values.')
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('AprilTag quaternion has invalid length.')
    return tuple(value / norm for value in values)


def quaternion_product(left, right):
    x, y, z, w = left
    a, b, c, d = right
    return (w*a + x*d + y*c - z*b,
            w*b - x*c + y*d + z*a,
            w*c + x*b - y*a + z*d,
            w*d - x*a - y*b - z*c)


@dataclass(frozen=True)
class TagPose:
    parent: str
    child: str
    stamp_ns: int
    position: tuple
    orientation: tuple

    def validated(self):
        position = tuple(float(value) for value in self.position)
        if (not self.parent or not self.child or self.parent == self.child
                or self.stamp_ns <= 0 or len(position) != 3
                or not all(math.isfinite(value) for value in position)):
            raise ValueError('AprilTag pose requires frames, a source stamp, and finite XYZ.')
        return TagPose(self.parent, self.child, self.stamp_ns, position,
                       unit_quaternion(self.orientation))


def relative_pose(tag0, tag1):
    """Return ^tag0 T_tag1 = inverse(^camera T_tag0) * ^camera T_tag1."""
    tag0, tag1 = tag0.validated(), tag1.validated()
    if tag0.parent != tag1.parent or tag0.stamp_ns != tag1.stamp_ns:
        raise ValueError('Relative AprilTag pose requires the same parent and source stamp.')
    x, y, z, w = tag0.orientation
    inverse = (-x, -y, -z, w)
    offset = tuple(b-a for a, b in zip(tag0.position, tag1.position))
    position = quaternion_product(
        quaternion_product(inverse, (*offset, 0.0)), tag0.orientation)[:3]
    orientation = unit_quaternion(quaternion_product(inverse, tag1.orientation))
    return TagPose(tag0.child, tag1.child, tag0.stamp_ns, position, orientation)


class FreshTagPair:
    """Reject repeated/old TFs; pair tags only at an identical image stamp.

    A bounded cache permits the detector to send the two transforms separately.
    Missing detections never create a zero pose or repeat the last valid pose.
    """

    def __init__(self, tag0_frame='ID0', tag1_frame='ID1', max_pending=64):
        if not tag0_frame or not tag1_frame or tag0_frame == tag1_frame:
            raise ValueError('AprilTag frame names must be non-empty and distinct.')
        if max_pending < 1:
            raise ValueError('max_pending must be positive.')
        self.frames = (tag0_frame, tag1_frame)
        self.max_pending = max_pending
        self.last_stamp = {}
        self.pending = OrderedDict()
        self.source_id = None
        self.source_changes = 0

    def ingest(self, pose, source_id=None):
        if pose.child not in self.frames:
            return None, None
        pose = pose.validated()
        # Both tags belong to one apriltag_ros publisher. A replacement node
        # can restart its image clock: its DDS GID marks a new source epoch.
        # A lower stamp from the SAME publisher never resets the epoch.
        if source_id is not None and source_id != self.source_id:
            if self.source_id is not None:
                self.source_changes += 1
            self.source_id = source_id
            self.last_stamp.clear()
            self.pending.clear()
        if pose.stamp_ns <= self.last_stamp.get(pose.child, -1):
            return None, None
        self.last_stamp[pose.child] = pose.stamp_ns
        key = (pose.parent, pose.stamp_ns)
        pair = self.pending.setdefault(key, {})
        pair[pose.child] = pose
        relative = None
        if all(frame in pair for frame in self.frames):
            relative = relative_pose(pair[self.frames[0]], pair[self.frames[1]])
            del self.pending[key]
        while len(self.pending) > self.max_pending:
            self.pending.popitem(last=False)
        return pose, relative
