"""Display-only, source-aware contact probability geometry for the skeleton tile."""

from dataclasses import dataclass
import math

from gui_py_pkg.force_id_preview import heatmap_values
from gui_py_pkg.skeleton_preview import TIP_MAX_AGE, source_stamp


CONTACT_GEOMETRY_TOPIC = '/estimated_segment_image_geometry'
CONTACT_MAX_SOURCE_GAP_SEC = 0.5
CONTACT_MAX_RECEIPT_AGE_SEC = 0.5


@dataclass(frozen=True)
class ContactOverlay:
    width: int
    height: int
    # Tuples: (contact ID, raw probability, sampled (u, v) path, centre (u, v)).
    segments: tuple
    center_id: int


def contact_overlay(frame, snapshot, now, *, max_source_gap=CONTACT_MAX_SOURCE_GAP_SEC):
    """Use exact image geometry and bounded recent inference, never a control ID.

    Inference uses the newest selected input time, not necessarily a camera time.
    Its timestamp therefore need not equal the image/geometry source timestamp.
    """
    if frame is None or snapshot is None or not snapshot.valid:
        return None
    if (snapshot.load_state == 1
            or (snapshot.contact_id == 0 and snapshot.load_state != 2)):
        return None  # Unloaded operating ID18 is not detected tip contact.
    # Force-range policy can mark a class0 result LOADED while its raw argmax
    # remains zero. In that case the publisher's positive-ID candidate is shown
    # using the original probabilities, without conditioning or renormalization.
    try:
        geometry = frame.geometry
        image = frame.image
        stamp = source_stamp(image)
        if (geometry is None or stamp is None
                or source_stamp(geometry) != stamp
                or geometry.header.frame_id != image.header.frame_id
                or snapshot.source_stamp is None
                or frame.geometry_received_at is None
                or snapshot.received_at is None):
            return None
        ages = (now - frame.received_at, now - frame.geometry_received_at,
                now - snapshot.received_at)
        if (not all(math.isfinite(age) and age >= 0 for age in ages)
                or max(ages[:2]) > TIP_MAX_AGE
                or ages[2] > CONTACT_MAX_RECEIPT_AGE_SEC):
            return None
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        if (not math.isfinite(max_source_gap) or max_source_gap < 0
                or not isinstance(snapshot.source_stamp, int) or snapshot.source_stamp <= 0
                or abs(snapshot.source_stamp - stamp_ns) > max_source_gap * 1e9):
            return None
        width, height = int(geometry.width), int(geometry.height)
        count = int(geometry.samples_per_segment)
        if (width != image.width or height != image.height or min(width, height) <= 0
                or not 2 <= count <= 64 or len(geometry.pixels) != 18 * count
                or len(geometry.centers) != 18):
            return None
        points = tuple((float(p.x), float(p.y)) for p in geometry.pixels)
        centers = tuple((float(p.x), float(p.y)) for p in geometry.centers)
        # Partly off-image projections are allowed and painter-clipped; reject
        # nonfinite/unreasonable coordinates rather than drawing arbitrary lines.
        if not all(math.isfinite(u) and math.isfinite(v)
                   and -width <= u <= 2 * width and -height <= v <= 2 * height
                   for u, v in (*points, *centers)):
            return None
        if (len(snapshot.probabilities) != 19
                or not 1 <= snapshot.heatmap_center_id <= 18
                or not all(math.isfinite(p) and 0 <= p <= 1
                           for p in snapshot.probabilities)):
            return None
        values = heatmap_values(snapshot)
        segments = tuple((index + 1, probability,
                          points[index * count:(index + 1) * count], centers[index])
                         for index, probability in enumerate(values)
                         if probability is not None and probability > 0)
        return ContactOverlay(width, height, segments, snapshot.heatmap_center_id)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
