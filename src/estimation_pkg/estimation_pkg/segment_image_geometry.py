"""Visualization-only correspondence between contact IDs and crop pixels.

Use the original extended skeleton, not a projection of the length-scaled
metric curve. Its retained sample indices and 3D curve parameter provide the
correspondence even when finite/depth/duplicate samples were removed.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SegmentImagePixels:
    """Crop-local (u, v) polylines, with blocks in contact ID 1..18 order."""

    width: int
    height: int
    samples_per_segment: int
    pixels: np.ndarray
    centers: np.ndarray


def segment_image_pixels(extended_yx_coords, source_indices, raw_s_3d,
                         s_segment, s_segment_center, image_shape,
                         samples_per_segment=9):
    """Interpolate the original pixels at the existing reconstruction samples.

    The physical segment at index zero is the fixed proximal offset. Contact
    ID i is physical segment i, so ID 1 starts at boundary 1 and ID 18 ends
    at boundary 19. Coordinates are pixels, never metres or normalized XYZ.
    Invalid geometry raises ValueError; callers must treat this as an optional
    visualization failure, not as an angle/shape-estimation failure.
    """
    if len(image_shape) < 2:
        raise ValueError('The crop image must have height and width.')
    height, width = image_shape[:2]
    if any(not isinstance(value, (int, np.integer)) or isinstance(value, bool)
           or not 1 <= value <= 16384 for value in (height, width)):
        raise ValueError('Crop dimensions must be positive bounded integers.')
    if (not isinstance(samples_per_segment, (int, np.integer))
            or isinstance(samples_per_segment, bool)
            or not 2 <= samples_per_segment <= 64):
        raise ValueError('samples_per_segment must be an integer from 2 to 64.')

    original_pixels = np.asarray(extended_yx_coords, dtype=float)
    indices = np.asarray(source_indices)
    raw_s = np.asarray(raw_s_3d, dtype=float)
    boundaries = np.asarray(s_segment, dtype=float)
    centers_s = np.asarray(s_segment_center, dtype=float)
    if (original_pixels.ndim != 2 or original_pixels.shape[1] != 2
            or not 2 <= len(original_pixels) <= 1000000):
        raise ValueError('Expected a bounded N-by-2 extended skeleton.')
    if (indices.ndim != 1 or not np.issubdtype(indices.dtype, np.integer)
            or len(indices) < 2 or np.any(indices < 0)
            or np.any(indices >= len(original_pixels))
            or np.any(np.diff(indices) <= 0)):
        raise ValueError('Retained source indices must be ordered and unique.')
    if (raw_s.shape != indices.shape or not np.all(np.isfinite(raw_s))
            or np.any(np.diff(raw_s) <= 0)):
        raise ValueError('The retained curve parameter must strictly increase.')
    if (boundaries.shape != (20,) or centers_s.shape != (19,)
            or not np.all(np.isfinite(boundaries))
            or not np.all(np.isfinite(centers_s))
            or np.any(np.diff(boundaries) <= 0)
            or np.any(centers_s <= boundaries[:-1])
            or np.any(centers_s >= boundaries[1:])):
        raise ValueError('Expected 20 boundaries and 19 interior segment centers.')
    # cumsum/sum differ by a few ulps at the final normalized sample.
    epsilon = 1e-10
    if (raw_s[0] < -epsilon or raw_s[-1] > 1.0 + epsilon
            or boundaries[0] < raw_s[0] - epsilon
            or boundaries[-1] > raw_s[-1] + epsilon):
        raise ValueError('Segment parameters must lie within the raw curve range.')

    # Use exactly the samples retained by reconstruct_3d_segments. This is
    # important if an interior point is nonfinite or a duplicate in XYZ.
    pixels_xy = original_pixels[indices][:, [1, 0]]
    if (not np.all(np.isfinite(pixels_xy))
            or np.any(pixels_xy < 0.0)
            or np.any(pixels_xy[:, 0] > width - 1)
            or np.any(pixels_xy[:, 1] > height - 1)):
        raise ValueError('Retained skeleton pixels must lie inside the crop.')

    query = np.linspace(boundaries[1:-1], boundaries[2:],
                        int(samples_per_segment), axis=1)

    def interpolate(values):
        return np.column_stack((
            np.interp(values, raw_s, pixels_xy[:, 0]),
            np.interp(values, raw_s, pixels_xy[:, 1]),
        ))

    return SegmentImagePixels(
        width=int(width), height=int(height),
        samples_per_segment=int(samples_per_segment),
        pixels=interpolate(query.reshape(-1)),
        centers=interpolate(centers_s[1:]),
    )
