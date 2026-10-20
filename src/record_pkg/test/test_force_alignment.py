"""Signed force-axis mappings are explicit, immutable-by-copy, and CSV-only."""

import copy
import itertools
import math

import pytest

from record_pkg.force_alignment import (
    aligned_force_columns, create_alignment, validate_alignment,
)


def test_default_is_disabled_identity_and_has_no_derived_columns():
    alignment = create_alignment()
    assert not alignment['enabled']
    assert alignment['axes'] == ['+x', '+y', '+z']
    assert alignment['matrix_base_from_sensor'] == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    assert aligned_force_columns({'x': 1., 'y': 2., 'z': 3.}, alignment) == {}
    assert validate_alignment(alignment) == alignment


def test_axis_order_describes_output_base_components_and_preserves_raw_force():
    alignment = create_alignment(True, ['+x', '-z', '+y'])
    force = {'x': 1.5, 'y': -2., 'z': 3.25}
    raw = copy.deepcopy(force)
    assert alignment['matrix_base_from_sensor'] == [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    assert aligned_force_columns(force, alignment) == dict(
        aligned_fx=1.5, aligned_fy=-3.25, aligned_fz=-2.,
        aligned_frame_id='hrm_base', aligned_force_valid=True)
    assert force == raw
    assert alignment['units'] == 'unchanged'
    assert alignment['topics'] == ['/fts_data', '/fts_data_kalman_filter']


def test_only_24_proper_signed_permutations_are_accepted():
    accepted = 0
    for order in itertools.permutations('xyz'):
        for signs in itertools.product('+-', repeat=3):
            axes = [sign + axis for sign, axis in zip(signs, order)]
            try:
                alignment = create_alignment(True, axes)
            except ValueError as error:
                assert 'determinant' in str(error)
                continue
            accepted += 1
            result = aligned_force_columns({'x': 1., 'y': 2., 'z': 3.}, alignment)
            assert math.isclose(sum(result[key]**2 for key in
                                   ('aligned_fx', 'aligned_fy', 'aligned_fz')), 14.)
    assert accepted == 24


@pytest.mark.parametrize('enabled, axes', [
    (1, ['+x', '+y', '+z']), ('false', ['+x', '+y', '+z']),
    (True, ['+x', '+x', '+z']), (True, ['+x', '+z', '+y']),
    (False, ['-x', '+y', '+z']), (True, ['+x', '+y']),
    (True, '+x,+y,+z'), (True, ['x', 'y', 'z']),
    (True, ['+X', '+y', '+z']), (True, ['+x', '+y', None]),
])
def test_invalid_settings_are_rejected_even_when_disabled(enabled, axes):
    with pytest.raises(ValueError):
        create_alignment(enabled, axes)


@pytest.mark.parametrize('force', [
    None, {}, {'x': 1., 'y': 2.}, {'x': True, 'y': 2., 'z': 3.},
    {'x': '1', 'y': 2., 'z': 3.}, {'x': float('nan'), 'y': 2., 'z': 3.},
    {'x': 1., 'y': float('inf'), 'z': 3.},
    {'x': 1., 'y': 2., 'z': -float('inf')}, {'x': 10**400, 'y': 2., 'z': 3.},
])
def test_invalid_force_is_blank_not_zero_or_guessed(force):
    result = aligned_force_columns(force, create_alignment(True))
    assert result == dict(aligned_fx='', aligned_fy='', aligned_fz='',
                          aligned_frame_id='hrm_base', aligned_force_valid=False)


def test_zero_force_is_valid():
    result = aligned_force_columns({'x': 0., 'y': 0., 'z': 0.}, create_alignment(True))
    assert result['aligned_force_valid']
    assert [result[key] for key in ('aligned_fx', 'aligned_fy', 'aligned_fz')] == [0.] * 3


@pytest.mark.parametrize('key, value', [
    ('schema_version', True), ('schema_version', 2), ('enabled', 1), ('axes', None),
    ('matrix_base_from_sensor', [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    ('matrix_base_from_sensor', [[True, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ('source_frame', 'camera'), ('target_frame', 'sensor'),
    ('topics', ['/fts_data']), ('units', 'N'),
])
def test_saved_configuration_cannot_silently_change_meaning(key, value):
    alignment = create_alignment(True)
    alignment[key] = value
    with pytest.raises(ValueError):
        validate_alignment(alignment)


def test_saved_matrix_is_required_and_caller_cannot_mutate_canonical_copy():
    alignment = create_alignment(True)
    validated = validate_alignment(alignment)
    alignment['axes'][0] = '-x'
    alignment['matrix_base_from_sensor'][0][0] = -1
    assert validated == create_alignment(True)
    del alignment['matrix_base_from_sensor']
    with pytest.raises(ValueError, match='matrix_base_from_sensor'):
        validate_alignment(alignment)


def test_input_axes_list_is_copied():
    axes = ['+x', '+y', '+z']
    alignment = create_alignment(True, axes)
    axes[0] = '-x'
    assert alignment['axes'] == ['+x', '+y', '+z']
