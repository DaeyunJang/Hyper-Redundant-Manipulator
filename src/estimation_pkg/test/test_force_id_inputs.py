"""Feature provenance, history freshness and diagnostic contact policy tests."""

from types import SimpleNamespace

import numpy as np
import pytest

from estimation_pkg.force_id_inputs import FeatureHistory, contact_policy, feature_spec


COLUMNS = [f'wire.Cable #{i} length' for i in range(1, 5)] + [
    f'loadcell.Loadcell #{i} tension' for i in range(1, 5)]


def feed(history, stamp, *, wire=(0., -1., 2., 3.), stress=(200., 210., 220., 230.)):
    for topic, values in (('/wire_length', {'data': wire}),
                          ('/loadcell_state', {'stress': stress})):
        history.put(topic, values, source_ns=stamp, received_ns=stamp,
                    received_monotonic_ns=stamp)


def test_exact_bundle_column_order_units_signs_and_no_unused_angles():
    history = FeatureHistory(COLUMNS[::-1], 2)
    assert history.required_topics == {'/wire_length', '/loadcell_state'}
    feed(history, 1_000_000_000)
    value, state = history.sample(1_000_000_000, 1_000_000_000)
    assert value is None and state == 'warming_up'
    feed(history, 1_033_000_000)
    value, state = history.sample(1_033_000_000, 1_033_000_000)
    assert state == 'ready'
    np.testing.assert_array_equal(value[0], [230, 220, 210, 200, 3, 2, -1, 0])
    assert value.shape == (2, 8)


@pytest.mark.parametrize('joint', range(1, 19))
def test_active_relative_axis_is_fixed_odd_tilt_even_pan(joint):
    spec = feature_spec(f'relative_angle_{joint}')
    assert spec.field == ('tilt_relative' if joint % 2 else 'pan_relative')
    assert spec.index == joint - 1
    speed = feature_spec(f'relative_angular_velocity_{joint}_rad_s')
    assert speed.field == ('tilt' if joint % 2 else 'pan') + '_angular_velocity_relative'


@pytest.mark.parametrize('field', ['pan_relative', 'pan_absolute', 'tilt_relative', 'tilt_absolute'])
def test_absolute_and_relative_named_feature_mapping(field):
    spec = feature_spec(f'{field}_18_rad')
    assert spec.field == field and spec.index == 17


@pytest.mark.parametrize('name', ['fts.fx', 'fts_kalman.aligned_fz', 'source_time_ns',
                                 'contact_segment_id', 'relative_angle_19', 'wire.Cable #0 length'])
def test_unsupported_or_ground_truth_features_are_rejected(name):
    with pytest.raises(ValueError):
        feature_spec(name)


def test_stale_stream_clears_history_and_requires_full_fresh_warmup():
    history = FeatureHistory(COLUMNS, 2)
    feed(history, 1_000_000_000)
    history.sample(1_000_000_000, 1_000_000_000)
    feed(history, 1_033_000_000)
    assert history.sample(1_033_000_000, 1_033_000_000)[1] == 'ready'
    assert history.sample(1_300_000_000, 1_300_000_000)[1].startswith('stale_input:')
    assert len(history.window) == 0
    feed(history, 1_310_000_000)
    assert history.sample(1_310_000_000, 1_310_000_000)[1] == 'warming_up'


def test_same_measurement_new_timestamp_is_fresh_but_dead_stream_is_not():
    history = FeatureHistory(COLUMNS, 2)
    feed(history, 1_000_000_000)
    history.sample(1_000_000_000, 1_000_000_000)
    assert history.sample(1_010_000_000, 1_010_000_000)[1] == 'waiting_for_new_input'
    assert len(history.window) == 1
    feed(history, 1_033_000_000)
    assert history.sample(1_033_000_000, 1_033_000_000)[1] == 'ready'
    np.testing.assert_array_equal(history.window[0], history.window[1])


def test_retransmitted_source_stamp_does_not_refresh_age_or_add_row():
    history = FeatureHistory(COLUMNS, 2)
    feed(history, 1_000_000_000)
    history.sample(1_000_000_000, 1_000_000_000)
    history.put('/loadcell_state', {'stress': [0] * 4}, source_ns=1_000_000_000,
                received_ns=1_020_000_000, received_monotonic_ns=1_020_000_000)
    assert history.sample(1_020_000_000, 1_020_000_000)[1] == 'waiting_for_new_input'
    assert history.samples['/loadcell_state']['values'][feature_spec(COLUMNS[4])] == 200


@pytest.mark.parametrize('bad', [[1, 2], [float('nan')] * 4, [float('inf')] * 4])
def test_invalid_required_array_clears_history(bad):
    history = FeatureHistory(COLUMNS, 2)
    feed(history, 1_000_000_000)
    history.sample(1_000_000_000, 1_000_000_000)
    feed(history, 1_010_000_000, stress=bad)
    assert history.sample(1_010_000_000, 1_010_000_000)[1].startswith('missing_or_invalid_input:')
    assert len(history.window) == 0


def test_excess_skew_or_window_gap_does_not_join_discontinuous_history():
    history = FeatureHistory(COLUMNS, 2)
    feed(history, 1_000_000_000)
    history.sample(1_000_000_000, 1_000_000_000)
    history.put('/wire_length', {'data': [0] * 4}, source_ns=1_110_000_000,
                received_ns=1_110_000_000, received_monotonic_ns=1_110_000_000)
    assert history.sample(1_110_000_000, 1_110_000_000)[1] == 'input_time_skew'
    feed(history, 1_120_000_000)
    history.sample(1_120_000_000, 1_120_000_000)
    feed(history, 1_300_000_000)
    assert history.sample(1_300_000_000, 1_300_000_000)[1] == 'warming_up'


@pytest.mark.parametrize('mode,raw_id,load,control,eligible', [
    ('class0', 0, 1, 18, True), ('class0', 18, 2, 18, True),
    ('class0', 16, 2, 16, False), ('masked', 18, 0, -1, False),
    ('masked', 16, 0, -1, False),
])
@pytest.mark.parametrize('enabled', [False, True])
def test_contact_policy_preserves_raw_id_and_probabilities(mode, raw_id, load, control, eligible, enabled):
    probs = np.zeros(19)
    probs[raw_id] = 0.8
    probs[17] += 0.2
    prediction = SimpleNamespace(probabilities=probs, predicted_id=raw_id,
                                 location_id=int(np.argmax(probs[1:])) + 1,
                                 force_N=np.array([-1., 2., -3.]))
    original = probs.copy()
    policy = contact_policy(prediction, mode, list(range(1, 19)), 'tip', enabled, 2)
    assert policy['contact_id'] == raw_id
    assert policy['load_state'] == load
    assert policy['control_contact_id'] == control
    assert policy['tip_force_valid'] == eligible
    assert policy['body_contact_enabled'] == enabled
    np.testing.assert_array_equal(probs, original)
    np.testing.assert_array_equal(prediction.force_N, [-1., 2., -3.])
    if raw_id == 0:
        assert probs[18] == 0  # Default operating ID18 never fabricates tip confidence.
        assert policy['band_probability'] == pytest.approx(0.2)
    if raw_id == 16:
        assert policy['heatmap_center_id'] == 16  # Band including18 cannot authorize tip use.


def test_heatmap_clipped_to_supported_ids_and_original_probabilities():
    prediction = SimpleNamespace(probabilities=np.zeros(19), predicted_id=17, location_id=17)
    prediction.probabilities[[1, 16, 17, 18]] = [0.2, 0.1, 0.5, 0.2]
    policy = contact_policy(prediction, 'class0', [1, 16, 17, 18], 'tip', True, 2)
    assert policy['band_probability'] == pytest.approx(0.8)
    assert policy['body_contact_detected'] and not policy['tip_force_valid']
