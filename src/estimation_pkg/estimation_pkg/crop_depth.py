"""Calibration for lossless aligned depth ROI publication (no depth filtering)."""

from copy import deepcopy
import math


def crop_depth_calibration(camera_info, header, encoding, scale, source_shape, roi):
    """Return crop intrinsics and explicit source ROI/unit metadata.

    CameraInfo describes the already-cropped pixels, so ROI is reset to zero
    to prevent consumers from subtracting its offset twice. The original ROI
    is stored separately in metadata, together with full source dimensions.
    """
    x, y, width, height = roi
    source_height, source_width = source_shape[:2]
    if encoding not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError('Only 16UC1/mono16/32FC1 depth can be archived.')
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Depth scale must be finite and positive.')
    if (camera_info is None or camera_info.header.frame_id != header.frame_id
            or camera_info.width != source_width or camera_info.height != source_height):
        raise ValueError('Aligned depth and color CameraInfo frame/dimensions differ.')
    if (min(x, y) < 0 or min(width, height) <= 0
            or x + width > source_width or y + height > source_height):
        raise ValueError('Invalid depth ROI.')
    if camera_info.k[0] <= 0 or camera_info.k[4] <= 0:
        raise ValueError('Camera intrinsics are not calibrated.')
    info = deepcopy(camera_info)
    info.header = deepcopy(header)
    info.width, info.height = width, height
    info.k[2] -= x
    info.k[5] -= y
    info.p[2] -= x
    info.p[6] -= y
    info.roi.x_offset = info.roi.y_offset = 0
    info.roi.width = info.roi.height = 0
    info.roi.do_rectify = False
    info.binning_x = info.binning_y = 0
    stamp = header.stamp
    metadata = dict(
        schema_version=1, source_time_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
        frame_id=header.frame_id, encoding=encoding, width=width, height=height,
        source_width=source_width, source_height=source_height,
        roi=dict(x=x, y=y, width=width, height=height),
        depth_scale_m_per_unit=float(scale),
        intrinsics_reference='cropped_pixels',
        k=list(info.k), p=list(info.p), r=list(info.r), d=list(info.d),
        distortion_model=info.distortion_model,
        camera_info_source_time_ns=(camera_info.header.stamp.sec * 1_000_000_000
                                    + camera_info.header.stamp.nanosec),
    )
    return info, metadata
