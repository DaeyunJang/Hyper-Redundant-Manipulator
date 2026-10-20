"""Operator labels and same-sample angle selection; no force classification."""

import math


RELATIVE_COLUMNS = tuple(f'relative_angle_{i}' for i in range(1, 19))


def validate_contact_segment_id(value):
    if type(value) is not int or not 0 <= value <= 18:
        raise ValueError('contact_segment_id must be an integer in 0..18.')
    return value


def session_contact_segment_id(session):
    snapshot = session.get('snapshot', {})
    if 'contact_segment_id' not in snapshot:
        return ''  # Legacy data is unlabeled, not automatically a free-motion trial.
    return validate_contact_segment_id(snapshot['contact_segment_id'])


def relative_angles(pan, tilt):
    """Fixed 1-based odd tilt / even pan selection, retaining sign and units."""
    result = {}
    for i, column in enumerate(RELATIVE_COLUMNS):
        source = tilt if i % 2 == 0 else pan
        value = ''
        if isinstance(source, (list, tuple)) and len(source) == 18:
            candidate = source[i]
            if (not isinstance(candidate, bool) and isinstance(candidate, (int, float))
                    and math.isfinite(candidate)):
                value = candidate
        result[column] = value
    return result
