"""Run using HRM_env Python; ordinary ROS-only Python skips torch tests.

Optional recorded-prediction parity (no training, hardware or ROS):
HRM_BUNDLE_PARITY_CSV=/path/to/summary_static_...seg-id-18.csv pytest -q ...
The selected CSV can be a tip-column-refreshed version: only unchanged raw
features/target finite-row selection are used, not the refreshed pose columns.
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

from estimation_pkg.force_id_runtime import BundleRuntime, discover_bundles

torch = pytest.importorskip('torch')
from estimation_pkg.force_id_models import IndependentHRM, MODEL_NAMES


@pytest.fixture(autouse=True)
def one_cpu_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def synthetic_bundle(mode='class0', active_ids=(2, 7, 18), architecture='mlp'):
    bundle = {
        'schema_version': 2, 'kind': 'independent_hrm', 'mode': mode,
        'input_dim': 2, 'num_classes': len(active_ids) + int(mode == 'class0'),
        'feature_columns': ['second', 'first'],
        'force_name': architecture, 'location_name': architecture,
        'force_params': {}, 'location_params': {},
        'config': {'input_columns': ['second', 'first'], 'window_samples': 30,
                   'active_ids': list(active_ids), 'force_frame': 'hrm_base'},
        'force_scope': 'tip', 'force_unit': 'N', 'future_hrm_control_multiplier': -1,
        'scalers': {
            'force': {'x_mean': [2.0, 3.0], 'x_std': [2.0, 4.0],
                      'y_mean': [1.0, 2.0, 3.0], 'y_std': [0.1, 0.2, 0.3]},
            'location': {'x_mean': [-1.0, -2.0], 'x_std': [3.0, 5.0]},
        },
    }
    model = IndependentHRM(bundle)
    with torch.no_grad():
        for parameter in model.parameters():
            parameter.zero_()
        model.force_net.head.bias.copy_(torch.tensor([1.0, -2.0, 0.0]))
        model.location_net.head.bias.copy_(torch.arange(bundle['num_classes']))
    bundle['state_dict'] = model.state_dict()
    return bundle


def runtime_from(tmp_path, bundle):
    path = tmp_path / 'hrm_bundle.pt'
    torch.save(bundle, path)
    return BundleRuntime(path)


@pytest.mark.parametrize('architecture', MODEL_NAMES)
@pytest.mark.parametrize('mode', ['class0', 'masked'])
def test_all_architectures_restore_and_predict(tmp_path, architecture, mode):
    runtime = runtime_from(tmp_path, synthetic_bundle(mode=mode, architecture=architecture))
    result = runtime.predict(np.zeros((30, 2)))
    np.testing.assert_allclose(result.force_N, [1.1, 1.6, 3.0], atol=1e-6)
    assert result.predicted_id == result.location_id == 18
    assert result.probabilities.shape == (19,)
    assert result.probabilities.sum() == pytest.approx(1.0, abs=1e-6)
    assert result.no_load_probability_available == (mode == 'class0')
    assert (result.probabilities[0] > 0) == (mode == 'class0')
    assert runtime.feature_columns == ('second', 'first')
    assert runtime.future_hrm_control_multiplier == -1
    assert all(not parameter.requires_grad or parameter.grad is None
               for parameter in runtime.model.parameters())


def test_separate_branch_scalers_and_raw_continuous_force(tmp_path):
    runtime = runtime_from(tmp_path, synthetic_bundle())
    captured = {}
    runtime.model.force_net.register_forward_pre_hook(
        lambda module, args: captured.update(force=args[0].cpu().numpy().copy()))
    runtime.model.location_net.register_forward_pre_hook(
        lambda module, args: captured.update(location=args[0].cpu().numpy().copy()))
    result = runtime.predict(np.tile([6.0, 11.0], (30, 1)))
    np.testing.assert_allclose(captured['force'][0], np.tile([2.0, 2.0], (30, 1)))
    np.testing.assert_allclose(captured['location'][0], np.tile([7/3, 13/5], (30, 1)), rtol=1e-6)
    np.testing.assert_allclose(result.force_N, [1.1, 1.6, 3.0], atol=1e-6)


def test_class0_and_conditional_heatmap_candidate_remain_distinct(tmp_path):
    bundle = synthetic_bundle(active_ids=(18, 3, 6))
    bundle['state_dict']['location_net.head.bias'] = torch.tensor([10., 1., 3., 2.])
    result = runtime_from(tmp_path, bundle).predict(np.zeros((30, 2)))
    assert result.predicted_id == 0
    assert result.location_id == 3
    assert result.force_N[0] != 0  # Never manufacture a zero force on unloaded class.
    assert result.probabilities[0] > result.probabilities[3] > result.probabilities[6]
    assert result.probabilities[6] > result.probabilities[18]


@pytest.mark.parametrize('window', [np.zeros((29, 2)), np.zeros((31, 2)),
                                    np.zeros((30, 3)), np.full((30, 2), np.nan),
                                    np.full((30, 2), np.inf)])
def test_missing_or_invalid_history_not_padded(tmp_path, window):
    with pytest.raises(ValueError, match='finite raw input window'):
        runtime_from(tmp_path, synthetic_bundle()).predict(window)


@pytest.mark.parametrize('branch,key,value', [
    ('force', 'x_std', [0., 1.]), ('location', 'x_std', [-1., 1.]),
    ('force', 'x_mean', [np.nan, 1.]), ('location', 'x_mean', [1.]),
    ('force', 'y_std', [1., 0., 1.]), ('force', 'y_mean', [np.inf, 0., 0.]),
])
def test_malformed_scaler_rejected(tmp_path, branch, key, value):
    bundle = synthetic_bundle()
    bundle['scalers'][branch][key] = value
    with pytest.raises(ValueError):
        runtime_from(tmp_path, bundle)


@pytest.mark.parametrize('mutation', [
    {'schema_version': 3}, {'kind': 'unknown'}, {'mode': 'unknown'},
    {'force_unit': 'mN'}, {'num_classes': 18}, {'feature_columns': ['first', 'second']},
])
def test_incompatible_contract_rejected(tmp_path, mutation):
    bundle = synthetic_bundle()
    bundle.update(mutation)
    with pytest.raises(ValueError):
        runtime_from(tmp_path, bundle)


def test_state_dict_is_strict(tmp_path):
    bundle = synthetic_bundle()
    bundle['state_dict'].pop('force_net.head.weight')
    with pytest.raises(RuntimeError, match='Missing key'):
        runtime_from(tmp_path, bundle)


def test_discovery_does_not_load_checkpoint(tmp_path):
    empty = tmp_path / 'empty' / 'hrm_bundle.pt'
    empty.parent.mkdir()
    empty.touch()
    present = tmp_path / 'experiment' / 'choice' / 'hrm_bundle.pt'
    present.parent.mkdir(parents=True)
    present.write_bytes(b'listing-only-not-a-checkpoint')
    assert discover_bundles(tmp_path) == [present]
    assert discover_bundles(present) == [present]
    assert discover_bundles(empty) == []
    assert discover_bundles(tmp_path / 'missing') == []


_MODEL_ROOT = Path(__file__).resolve().parents[1] / 'models' / 'cable_loadcell' / 'models'


@lru_cache(maxsize=2)
def _compacted_validation_rows(csv_path, columns):
    pandas = pytest.importorskip('pandas')
    frame = pandas.read_csv(csv_path, usecols=columns)
    raw = frame[list(columns)].apply(pandas.to_numeric, errors='coerce').to_numpy(dtype=float)
    return raw[np.isfinite(raw).all(axis=1)]


def test_installed_user_bundles_all_restore_when_present():
    from estimation_pkg.force_id_inputs import FeatureHistory
    paths = discover_bundles(_MODEL_ROOT.parents[1])
    if not paths:
        pytest.skip('User checkpoints are optional, not source-code fixtures.')
    for path in paths:
        runtime = BundleRuntime(path)
        # Every experiment's declared input columns must map to live data.
        history = FeatureHistory(runtime.feature_columns, runtime.window_samples)
        assert len(history.features) == runtime.input_dim, path
        output = runtime.predict(np.tile(runtime.scalers['force']['x_mean'],
                                         (runtime.window_samples, 1)))
        assert np.isfinite(output.force_N).all(), path
        assert output.probabilities.sum() == pytest.approx(1, abs=1e-6), path


@pytest.mark.parametrize('mode', ['class0', 'masked'])
@pytest.mark.parametrize('architecture', MODEL_NAMES)
def test_archived_validation_prediction_parity(mode, architecture):
    csv_path = os.environ.get('HRM_BUNDLE_PARITY_CSV')
    if not csv_path:
        pytest.skip('Set HRM_BUNDLE_PARITY_CSV for archived-prediction comparison.')
    path = _MODEL_ROOT / f'{mode}__force_{architecture}__location_{architecture}' / 'hrm_bundle.pt'
    if not path.is_file():
        pytest.skip('Optional saved training bundle not present.')
    bundle = torch.load(path, map_location='cpu', weights_only=True)
    audit = bundle['data_audit']['trials']
    trial_indices = [i for i, trial in enumerate(audit)
                     if Path(trial['path']).name == Path(csv_path).name]
    assert len(trial_indices) == 1, 'CSV basename must identify one archived validation trial.'
    trial_index = trial_indices[0]
    columns = bundle['feature_columns'] + bundle['config']['force_columns']
    raw = _compacted_validation_rows(csv_path, tuple(columns))
    assert len(raw) == audit[trial_index]['rows'], 'Numeric-compacted trial rows changed.'
    runtime = BundleRuntime(path)
    with np.load(path.parent / 'validation_predictions.npz', allow_pickle=False) as saved:
        for branch in ('force', 'location'):
            references = saved[f'{branch}_refs']
            indices = np.flatnonzero(references[:, 0] == trial_index)
            assert len(indices) > 0
            for index in indices[[0, len(indices)//2, -1]]:
                end = int(references[index, 1])
                result = runtime.predict(raw[end-runtime.window_samples+1:end+1, :runtime.input_dim])
                if branch == 'force':
                    actual, expected = result.force_N, saved['force_N'][index]
                else:
                    actual = result.probabilities[list(runtime.class_ids)]
                    expected = saved['location_probabilities'][index]
                np.testing.assert_allclose(actual, expected, rtol=2e-5, atol=2e-6,
                                           err_msg=f'{path.parent.name}, {branch}, row {end}')
