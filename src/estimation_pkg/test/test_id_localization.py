"""Operating force ranges must not change model predictions or training labels."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from estimation_pkg.force_id_inputs import contact_policy
from estimation_pkg.id_localization import LocalizationConfig, load_localization_config


def prediction(mode, winner, force):
    probabilities = np.zeros(19)
    probabilities[winner] = .8
    probabilities[18] += .2
    return SimpleNamespace(force_N=np.asarray(force), probabilities=probabilities,
                           predicted_id=winner,
                           location_id=np.argmax(probabilities[1:]) + 1)


@pytest.fixture
def config():
    return LocalizationConfig(True, ((-100., 100.), (-200., 200.), (-300., 300.)))


@pytest.mark.parametrize('mode,winner', [('masked', 9), ('masked', 18),
                                       ('class0', 0), ('class0', 9), ('class0', 18)])
@pytest.mark.parametrize('force', [(0., 0., 0.), (.1, -.2, .3), (-.1, .2, -.3)])
@pytest.mark.parametrize('body_enabled', [True, False])
def test_box_inclusive_operating_tip_preserves_raw_predictions(config, mode, winner, force, body_enabled):
    p = prediction(mode, winner, force)
    original_probabilities = p.probabilities.copy()
    result = contact_policy(p, mode, range(1, 19), 'tip', body_enabled, 2, config)
    assert result['load_state'] == 1
    assert result['control_contact_id'] == 18
    assert result['contact_id'] == winner
    assert result['tip_force_valid'] and not result['body_contact_detected']
    np.testing.assert_array_equal(p.force_N, force)
    np.testing.assert_array_equal(p.probabilities, original_probabilities)


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('winner', [1, 9, 18])
def test_any_axis_outside_uses_signed_original_force_and_model_id(config, axis, sign, winner):
    force = np.zeros(3)
    bound = config.ranges_mN[axis][1] * .001
    force[axis] = sign * np.nextafter(bound, np.inf)
    p = prediction('masked', winner, force)
    result = contact_policy(p, 'masked', range(1, 19), 'tip', True, 3, config)
    assert result['load_state'] == 2
    assert result['control_contact_id'] == winner
    assert result['body_contact_detected'] == (winner != 18)
    assert result['tip_force_valid'] == (winner == 18)
    np.testing.assert_array_equal(p.force_N, force)


def test_class0_outside_box_uses_positive_candidate_without_replacing_raw_zero(config):
    p = prediction('class0', 0, [0., 1., 0.])
    result = contact_policy(p, 'class0', range(1, 19), 'tip', False, 2, config)
    assert result['contact_id'] == 0 and result['control_contact_id'] == 18
    assert result['load_state'] == 2 and result['tip_force_valid']
    assert p.probabilities[0] == .8 and p.probabilities[18] == .2
    p.probabilities[18], p.probabilities[7], p.location_id = 0., .2, 7
    result = contact_policy(p, 'class0', range(1, 19), 'tip', True, 2, config)
    assert result['control_contact_id'] == 7 and result['body_contact_detected']
    assert not result['tip_force_valid']


def test_disabled_restores_original_native_rules(config):
    disabled = LocalizationConfig(False, config.ranges_mN)
    for mode, winner, load, control in [('masked', 9, 0, -1), ('class0', 0, 1, 18),
                                      ('class0', 9, 2, 9)]:
        result = contact_policy(prediction(mode, winner, [0., 0., 0.]), mode,
                                range(1, 19), 'tip', False, 2, disabled)
        assert (result['load_state'], result['control_contact_id']) == (load, control)


def test_non_tip_scope_never_becomes_tip_force(config):
    result = contact_policy(prediction('masked', 9, [0., 0., 0.]), 'masked', range(1, 19),
                            'body', False, 2, config)
    assert result['control_contact_id'] == 18 and not result['tip_force_valid']


def test_all_single_contact_scope_supports_tip_but_does_not_label_body_as_tip(config):
    result = contact_policy(prediction('masked', 9, [0., 0., 0.]), 'masked', range(1, 19),
                            'all_single_contact', False, 2, config)
    assert result['control_contact_id'] == 18 and result['tip_force_valid']
    result = contact_policy(prediction('masked', 9, [1., 0., 0.]), 'masked', range(1, 19),
                            'all_single_contact', False, 2, config)
    assert result['control_contact_id'] == 9 and not result['tip_force_valid']


@pytest.mark.parametrize('force', [[np.nan, 0, 0], [0, np.inf, 0], [0, 0], [0, 0, 0, 0]])
def test_invalid_force_never_becomes_no_load(config, force):
    with pytest.raises(ValueError):
        config.is_unloaded(force)


def test_asymmetric_ranges_are_not_recentered_or_force_zeroed():
    config = LocalizationConfig(True, ((10., 100.), (-200., 0.), (20., 300.)))
    assert config.is_unloaded([.01, -.2, .02])
    assert not config.is_unloaded([0., 0., 0.])


def test_operator_file_has_valid_explicit_mN_ranges():
    path = Path(__file__).parents[1] / 'estimation_pkg/id_localization_config.json'
    config = load_localization_config(path)
    # This is intentionally editable by the operator; do not pin their ranges
    # to the original training calibration or fail when they turn it off.
    assert type(config.enabled) is bool
    midpoint = [(.5 * lo + .5 * hi) * .001 for lo, hi in config.ranges_mN]
    assert config.is_unloaded(midpoint)
    midpoint[0] = np.nextafter(config.ranges_mN[0][1] * .001, np.inf)
    assert not config.is_unloaded(midpoint)


@pytest.mark.parametrize('change', [
    {'enabled': 'true'}, {'enabled': 1}, {'schema_version': True}, {'schema_version': 2},
    {'typo': 1}, {'unloaded_force_ranges_mN': {'fx': [-1, 1]}},
    {'unloaded_force_ranges_mN': {'fx': [1, -1], 'fy': [-1, 1], 'fz': [-1, 1]}},
    {'unloaded_force_ranges_mN': {'fx': [False, 1], 'fy': [-1, 1], 'fz': [-1, 1]}},
    {'unloaded_force_ranges_mN': {'fx': [-1, np.nan], 'fy': [-1, 1], 'fz': [-1, 1]}},
    {'unloaded_force_ranges_mN': {'fx': [-1, np.inf], 'fy': [-1, 1], 'fz': [-1, 1]}},
    {'unloaded_force_ranges_mN': {'fx': 'bad', 'fy': [-1, 1], 'fz': [-1, 1]}},
])
def test_bad_config_rejected_not_silently_disabled(tmp_path, change):
    data = {'schema_version': 1, 'enabled': True, 'unloaded_force_ranges_mN': {
        'fx': [-100., 100.], 'fy': [-200., 200.], 'fz': [-300., 300.]}}
    data.update(change)
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_localization_config(path)


def test_missing_or_malformed_file_fails(tmp_path):
    path = tmp_path / 'missing.json'
    with pytest.raises(FileNotFoundError):
        load_localization_config(path)
    path.write_text('{')
    with pytest.raises(ValueError):
        load_localization_config(path)


def test_whole_line_operator_comments_are_allowed_without_changing_values(tmp_path):
    path = tmp_path / 'commented.json'
    path.write_text('''{
      // Editable ranges are mN, not N.
      "schema_version": 1,
      "enabled": true,
      "unloaded_force_ranges_mN": {
        "fx": [-100, 100], "fy": [-200, 200], "fz": [-300, 300]
      }
      // Old ranges can be kept here without a second active configuration.
    }''')
    config = load_localization_config(path)
    assert config.ranges_mN == ((-100, 100), (-200, 200), (-300, 300))
    assert config.is_unloaded([.1, -.2, .3])
