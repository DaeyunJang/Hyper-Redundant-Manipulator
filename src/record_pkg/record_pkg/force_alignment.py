"""Session-scoped force axis mapping for derived CSV columns only.

Each axis entry specifies one OUTPUT base component in terms of an INPUT sensor
component: ['+x', '-z', '+y'] means Fb = [Fs.x, -Fs.z, Fs.y]. No raw message,
force units, action/reaction sign, torque or torque reference point is changed.
"""

from collections.abc import Mapping
import math
from numbers import Real


SOURCE_FRAME = 'fts_sensor'
TARGET_FRAME = 'hrm_base'
FORCE_TOPICS = ('/fts_data', '/fts_data_kalman_filter')
ALIGNMENT_NOTE = (
    'User-specified fixed sensor-to-base rotation for force vectors only. '
    'Raw values are preserved; no unit conversion, action/reaction sign change, '
    'torque rotation or torque-origin transformation is applied. '
    'The configured mounting must remain valid during the session.')


def _finite_real(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def create_alignment(enabled=False, axes=None):
    """Create a validated proper signed-permutation rotation (not a reflection)."""
    if type(enabled) is not bool:
        raise ValueError('force_alignment_enabled must be a boolean.')
    axes = ['+x', '+y', '+z'] if axes is None else axes
    allowed = ('+x', '-x', '+y', '-y', '+z', '-z')
    if (not isinstance(axes, (list, tuple)) or len(axes) != 3
            or any(not isinstance(axis, str) or axis not in allowed for axis in axes)):
        raise ValueError(
            'force_alignment_axes must contain three signed axes from +x/-x/+y/-y/+z/-z.')
    if len({axis[1] for axis in axes}) != 3:
        raise ValueError('Each sensor axis must be used exactly once in force_alignment_axes.')
    matrix = [[0, 0, 0] for _ in range(3)]
    for row, axis in zip(matrix, axes):
        row['xyz'.index(axis[1])] = 1 if axis[0] == '+' else -1
    a, b, c = matrix
    determinant = (a[0] * (b[1]*c[2] - b[2]*c[1])
                   - a[1] * (b[0]*c[2] - b[2]*c[0])
                   + a[2] * (b[0]*c[1] - b[1]*c[0]))
    if determinant != 1:
        raise ValueError(
            'Force axis mapping must be a proper rotation (determinant +1), not a reflection.')
    return dict(
        schema_version=1, enabled=enabled, axes=list(axes),
        matrix_base_from_sensor=matrix, source_frame=SOURCE_FRAME,
        target_frame=TARGET_FRAME, topics=list(FORCE_TOPICS), units='unchanged',
        note=ALIGNMENT_NOTE,
    )


def validate_alignment(alignment):
    """Validate frozen session metadata without guessing missing schema fields."""
    if not isinstance(alignment, Mapping):
        raise ValueError('force_alignment snapshot must be an object.')
    required = ('schema_version', 'enabled', 'axes', 'matrix_base_from_sensor',
                'source_frame', 'target_frame', 'topics', 'units')
    missing = [key for key in required if key not in alignment]
    if missing:
        raise ValueError('Incomplete force_alignment snapshot: missing ' + ', '.join(missing))
    if type(alignment['schema_version']) is not int or alignment['schema_version'] != 1:
        raise ValueError('Unsupported force_alignment schema_version.')
    if alignment['axes'] is None:
        raise ValueError('Saved force_alignment axes cannot be null.')
    canonical = create_alignment(alignment['enabled'], alignment['axes'])
    for key in ('source_frame', 'target_frame', 'topics', 'units'):
        if alignment[key] != canonical[key]:
            raise ValueError(f'Unexpected force_alignment {key}; cannot reinterpret saved data.')
    matrix = alignment['matrix_base_from_sensor']
    if (not isinstance(matrix, (list, tuple)) or len(matrix) != 3
            or any(not isinstance(row, (list, tuple)) or len(row) != 3 for row in matrix)
            or any(not _finite_real(value) for row in matrix for value in row)
            or [list(row) for row in matrix] != canonical['matrix_base_from_sensor']):
        raise ValueError('Saved force-alignment matrix must match the signed-axis mapping exactly.')
    return canonical


def aligned_force_columns(force_dict, alignment):
    """Return derived force columns; invalid force samples stay blank, not zero."""
    alignment = validate_alignment(alignment)
    if not alignment['enabled']:
        return {}
    columns = dict(aligned_fx='', aligned_fy='', aligned_fz='',
                   aligned_frame_id=TARGET_FRAME, aligned_force_valid=False)
    if not isinstance(force_dict, Mapping):
        return columns
    values = [force_dict.get(axis) for axis in 'xyz']
    if any(not _finite_real(value) for value in values):
        return columns
    # Exactly one +/-1 entry per row: direct signed selection avoids introducing
    # rounding or mixing components. Original force_dict is never modified.
    for name, axis in zip(('aligned_fx', 'aligned_fy', 'aligned_fz'), alignment['axes']):
        columns[name] = (1 if axis[0] == '+' else -1) * force_dict[axis[1]]
    columns['aligned_force_valid'] = True
    return columns
