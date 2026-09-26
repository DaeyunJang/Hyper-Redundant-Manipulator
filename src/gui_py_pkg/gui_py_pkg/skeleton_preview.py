"""Bounded, same-image tip telemetry for the read-only skeleton preview."""

from collections import OrderedDict
from dataclasses import dataclass
import math
import time


TIP_TOPICS = {
    'estimated_tip': '/estimated_tip_position',
    'fk_tip': '/kinematics/fk_tip_position',
}
TIP_MAX_AGE = 0.2
TIP_CACHE_SIZE = 8


def source_stamp(message):
    """Images use camera axes, points use base axes; compare their source time."""
    stamp = message.header.stamp
    if (not message.header.frame_id or stamp.sec < 0
            or not 0 <= stamp.nanosec < 1_000_000_000
            or (stamp.sec == 0 and stamp.nanosec == 0)):
        return None
    return stamp.sec, stamp.nanosec


@dataclass(frozen=True)
class SkeletonPreviewFrame:
    image: object
    received_at: float
    estimated_tip: object = None
    fk_tip: object = None


class SkeletonFrameBuffer:
    """Keep references only; late FK may match a recent, not the newest image."""

    def __init__(self):
        self.images = OrderedDict()
        self.points = {kind: OrderedDict() for kind in TIP_TOPICS}
        self.latest_image = None

    def _trim(self, now):
        for cache in (self.images, *self.points.values()):
            for key, (_, received) in list(cache.items()):
                if now - received > TIP_MAX_AGE:
                    del cache[key]
            while len(cache) > TIP_CACHE_SIZE:
                cache.popitem(last=False)

    def receive_image(self, msg, *, now=None):
        now = time.monotonic() if now is None else now
        self.latest_image = (msg, now)
        stamp = source_stamp(msg)
        if stamp is not None:
            self.images[stamp] = (msg, now)
            self.images.move_to_end(stamp)
        self._trim(now)

    def receive_tip(self, kind, msg, *, now=None):
        if kind not in self.points:
            raise ValueError(f'Unknown tip stream: {kind}')
        now = time.monotonic() if now is None else now
        stamp = source_stamp(msg)
        if stamp is not None:
            self.points[kind][stamp] = (msg, now)
            self.points[kind].move_to_end(stamp)
        self._trim(now)

    def snapshot(self, now):
        self._trim(now)
        if self.latest_image is None:
            return None
        # A late FK result must not be paired with a different image. Prefer
        # a complete recent set, but never hold the preview indefinitely.
        for stamp, (image, received) in reversed(self.images.items()):
            if all(stamp in cache for cache in self.points.values()):
                return SkeletonPreviewFrame(
                    image, received,
                    *(self.points[kind][stamp][0] for kind in TIP_TOPICS))
        image, received = self.latest_image
        stamp = source_stamp(image)
        points = [cache[stamp][0] if stamp in cache else None for cache in self.points.values()]
        if now - received > TIP_MAX_AGE:
            points = [None, None]
        return SkeletonPreviewFrame(image, received, *points)


def format_skeleton_tips(frame, now):
    """Render signed millimetres only; missing values are neither held nor zeroed."""
    title = 'hrm_base | XYZ [mm]'
    if frame is None:
        return title, 'Est: waiting for image', 'FK: waiting for image'
    if now - frame.received_at > TIP_MAX_AGE:
        return title, 'Est: stale', 'FK: stale'
    stamp = source_stamp(frame.image)
    if stamp is None:
        return title, 'Est: invalid image stamp', 'FK: invalid image stamp'
    lines = [title]
    for label, point in (('Est', frame.estimated_tip), ('FK', frame.fk_tip)):
        if point is None:
            lines.append(f'{label}: waiting for matching tip')
        elif point.header.frame_id != 'hrm_base':
            lines.append(f'{label}: wrong frame')
        elif source_stamp(point) != stamp:
            lines.append(f'{label}: time mismatch')
        else:
            xyz = [value * 1000. for value in (point.point.x, point.point.y, point.point.z)]
            if not all(math.isfinite(value) for value in xyz):
                lines.append(f'{label}: invalid position')
            else:
                lines.append(f'{label}: ' + ' '.join(f'{value:+.1f}' for value in xyz))
    return tuple(lines)
