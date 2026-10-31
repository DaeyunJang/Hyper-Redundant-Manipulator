"""Inference monitoring is diagnostic, source-aware, and never implies actuation."""

import os
from pathlib import Path
from types import SimpleNamespace

from PyQt5.QtWidgets import QApplication
import pytest

from gui_py_pkg.force_id_preview import ForceIdMonitor, ForceIdPreview, heatmap_values
from gui_py_pkg.force_model_selector import (
    DEFAULT_BUNDLE_RELATIVE_PATH, ForceModelSelector, discover_bundles,
    preferred_bundle_path,
)


def estimate(**overrides):
    probabilities = [0.0] * 19
    probabilities[0] = 0.1
    probabilities[8:13] = [0.1, 0.15, 0.4, 0.2, 0.05]
    values = dict(
        header=SimpleNamespace(frame_id='hrm_base'), valid=True, status='ready',
        model_name='class0_example', mode='class0', force_scope='tip',
        external_force=SimpleNamespace(x=0.1, y=-0.2, z=0.0),
        contact_id=10, control_contact_id=10, load_state=2, tip_force_valid=False,
        body_contact_enabled=False, body_contact_detected=True,
        probabilities=probabilities, heatmap_center_id=10, id_tol=2,
        samples_in_window=30, window_samples=30)
    values.update(overrides)
    return SimpleNamespace(**values)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


def test_force_in_newtons_converted_once_without_axis_or_sign_change():
    preview = ForceIdPreview()
    msg = estimate()
    preview.receive(msg, now=10)
    assert preview.snapshot(now=10).force_mn == (100.0, -200.0, 0.0)
    assert msg.external_force.y == -0.2
    assert preview.snapshot(now=10).contact_id == 10
    assert preview.snapshot(now=10).tip_force_valid is False


def test_missing_and_stale_are_not_unloaded_and_never_keep_previous_force():
    preview = ForceIdPreview()
    assert not preview.seen and preview.snapshot(now=9).load_state == 0
    preview.receive(estimate(), now=10)
    assert preview.snapshot(now=10.5).valid
    stale = preview.snapshot(now=10.501)
    assert not stale.valid and stale.status == 'Stale' and stale.force_mn == ()
    preview.receive(estimate(valid=False, status='Warming up', samples_in_window=2), now=11)
    value = preview.snapshot(now=11)
    assert not value.valid and value.samples_in_window == 2
    assert value.load_state == 0 and value.control_contact_id == -1


def test_unloaded_default_control_id_does_not_fabricate_tip_heatmap_or_zero_force():
    probabilities = [0.0] * 19
    probabilities[0], probabilities[10] = 0.9, 0.1
    preview = ForceIdPreview()
    preview.receive(estimate(probabilities=probabilities, contact_id=0,
                             control_contact_id=18, load_state=1, tip_force_valid=True), now=1)
    snapshot = preview.snapshot(now=1)
    assert snapshot.force_mn == (100.0, -200.0, 0.0)
    assert snapshot.probabilities[18] == 0
    assert snapshot.heatmap_center_id == 10
    assert heatmap_values(snapshot)[9] == 0.1
    assert heatmap_values(snapshot)[17] is None


def test_masked_has_unknown_load_not_fake_p0():
    probabilities = [0.0] * 19
    probabilities[18] = 1.0
    preview = ForceIdPreview()
    preview.receive(estimate(mode='masked', probabilities=probabilities, contact_id=18,
                             control_contact_id=-1, heatmap_center_id=18, load_state=0), now=1)
    snapshot = preview.snapshot(now=1)
    assert snapshot.valid and snapshot.load_state == 0 and not snapshot.tip_force_valid


def test_masked_force_range_unloaded_status_preserves_force_id_and_numeric_heatmap(app):
    probabilities = [0.] * 19
    probabilities[9] = 1.
    preview = ForceIdPreview()
    message = estimate(mode='masked', probabilities=probabilities, contact_id=9,
                       heatmap_center_id=9, control_contact_id=18, load_state=1,
                       external_force=SimpleNamespace(x=.05, y=-.03, z=.01))
    preview.receive(message, now=10.)
    snapshot = preview.snapshot(now=10.)
    assert snapshot.valid and snapshot.load_state == 1 and snapshot.control_contact_id == 18
    assert snapshot.contact_id == 9 and snapshot.probabilities == tuple(probabilities)
    assert snapshot.force_mn == (50., -30., 10.)
    panel = ForceIdMonitor()
    try:
        panel.refresh(snapshot)
        assert 'Unloaded' in panel.status_label.text()
        assert 'raw ID 9' in panel.status_label.text()
        assert 'control ID 18' in panel.status_label.text()
        assert 'not proof of zero physical force' in panel.status_label.toolTip()
        assert 'rgb(255, 0, 0)' in panel.cells[8].styleSheet()
        assert 'p=1.0000' in panel.cells[8].toolTip()
        assert heatmap_values(snapshot)[17] is None
    finally:
        panel.close()


def test_class0_raw_unloaded_prediction_can_coexist_with_force_derived_loaded_state(app):
    probabilities = [.8] + [0.] * 17 + [.2]
    preview = ForceIdPreview()
    message = estimate(probabilities=probabilities, contact_id=0,
                       heatmap_center_id=18, control_contact_id=18, load_state=2,
                       external_force=SimpleNamespace(x=.3, y=0., z=0.))
    preview.receive(message, now=10.)
    snapshot = preview.snapshot(now=10.)
    assert snapshot.valid and snapshot.contact_id == 0 and snapshot.load_state == 2
    assert snapshot.probabilities[0] == .8 and snapshot.probabilities[18] == .2
    assert snapshot.force_mn == (300., 0., 0.)
    panel = ForceIdMonitor()
    try:
        panel.refresh(snapshot)
        assert 'Loaded' in panel.status_label.text()
        assert 'raw ID 0' in panel.status_label.text()
        assert 'control ID 18' in panel.status_label.text()
        assert 'rgb(255, 204, 204)' in panel.cells[17].styleSheet()
        assert 'p0=0.80' in panel.status_label.text()
    finally:
        panel.close()


def test_band_values_do_not_renormalize_or_reclassify_near_tip():
    probabilities = [0.0] * 19
    probabilities[0], probabilities[16], probabilities[18] = 0.2, 0.5, 0.3
    preview = ForceIdPreview()
    preview.receive(estimate(probabilities=probabilities, contact_id=16,
                             control_contact_id=16, heatmap_center_id=16), now=1)
    snapshot = preview.snapshot(now=1)
    assert snapshot.contact_id == 16
    assert heatmap_values(snapshot)[15] == 0.5
    assert heatmap_values(snapshot)[17] == 0.3
    assert sum(p for p in heatmap_values(snapshot) if p is not None) == 0.8


@pytest.mark.parametrize('overrides', [
    {'header': SimpleNamespace(frame_id='sensor_frame')},
    {'external_force': SimpleNamespace(x=float('nan'), y=0, z=0)},
    {'external_force': SimpleNamespace(x=1e308, y=0, z=0)},
    {'probabilities': [0.0] * 18},
    {'probabilities': [float('nan')] * 19},
    {'probabilities': [0.0] * 19},
    {'contact_id': 19}, {'load_state': 8}, {'mode': 'unknown'},
])
def test_invalid_new_message_clears_old_prediction(overrides):
    preview = ForceIdPreview()
    preview.receive(estimate(), now=1)
    preview.receive(estimate(**overrides), now=2)
    assert preview.seen
    assert not preview.snapshot(now=2).valid
    assert not preview.snapshot(now=2).force_mn


def test_heatmap_panel_keeps_raw_probabilities_when_body_monitor_off(app):
    panel = ForceIdMonitor()
    preview = ForceIdPreview()
    preview.receive(estimate(), now=1)
    panel.refresh(preview.snapshot(now=1))
    assert not panel.body_checkbox.isChecked()
    assert 'Body monitor OFF' in panel.status_label.text()
    assert 'p=0.4000' in panel.cells[9].toolTip()
    assert 'rgb(255, 153, 153)' in panel.cells[9].styleSheet()
    assert 'band Σp=0.90' in panel.status_label.text()
    changes = []
    panel.settings_requested.connect(lambda body, tol: changes.append((body, tol)))
    panel.body_checkbox.setChecked(True)
    panel.tolerance.setValue(3)
    assert changes == [(True, 2), (True, 3)]
    assert panel.substitutions() == {'body_contact_enabled': True, 'force_id_tol': 3}
    panel.refresh(preview.snapshot(now=2))
    assert 'Stale' in panel.status_label.text()
    assert all('#eeeeee' in cell.styleSheet() for cell in panel.cells)
    panel.close()


def test_contact_monitor_explains_default_visual_ids_and_optional_control_gate(app):
    panel = ForceIdMonitor()
    preview = ForceIdPreview()
    preview.receive(estimate(contact_id=9, control_contact_id=9,
                             heatmap_center_id=9, tip_force_valid=False), now=1.)
    try:
        panel.refresh(preview.snapshot(now=1.))
        assert 'display only (default)' in panel.settings_label.text()
        assert 'visual only' in panel.body_checkbox.toolTip()
        assert 'admittance/use_contact_id_gate=true' in panel.body_checkbox.toolTip()
        assert 'causes a hold' not in panel.body_checkbox.toolTip()
        details = panel.status_label.toolTip()
        assert 'ID-gated tip eligibility: False' in details
        assert 'policy diagnostic, not control authorization' in details
        assert 'admittance/use_contact_id_gate=false' in details
        assert 'physically tip-only loading' in details
        assert 'does not isolate a segment-18 force' in details
        assert 'raw ID 9' in panel.status_label.text()
        assert panel.last_snapshot.force_mn == (100., -200., 0.)
    finally:
        panel.close()


def test_discovery_only_finds_bundles_and_deduplicates_symlinks(tmp_path):
    root = tmp_path / 'models'
    first = root / 'cable_loadcell' / 'models' / 'class0__a'
    first.mkdir(parents=True)
    (first / 'hrm_bundle.pt').touch()
    (first / 'validation_predictions.npz').touch()
    (root / 'linked').symlink_to(first, target_is_directory=True)
    found = discover_bundles([root, root, tmp_path / 'missing'])
    assert len(found) == 1
    assert found[0][1] == str(first / 'hrm_bundle.pt')


def test_model_selector_requires_operator_choice_and_locks_running_model(app, tmp_path):
    bundle = tmp_path / 'models' / 'class0__demo' / 'hrm_bundle.pt'
    bundle.parent.mkdir(parents=True)
    bundle.touch()
    selector = ForceModelSelector(roots=[tmp_path])
    with pytest.raises(ValueError, match='Choose'):
        selector.for_start()
    selector.combo.setCurrentIndex(1)
    assert selector.for_start() == {'force_model_path': str(bundle)}
    selector.refresh()
    assert selector.combo.currentData() == str(bundle)
    for status in ('running', 'starting', 'stopping', 'external', 'graph_pending'):
        selector.set_process_status(status)
        assert not selector.combo.isEnabled() and not selector.refresh_button.isEnabled()
    selector.set_process_status('stopped')
    assert selector.combo.isEnabled()
    bundle.unlink()
    with pytest.raises(ValueError):
        selector.for_start()
    selector.close()


def test_model_architectures_are_visible_before_common_parent_prefix(tmp_path):
    folder = tmp_path / 'cable_loadcell' / 'models' / 'class0__force_mlp__location_cnn'
    folder.mkdir(parents=True)
    (folder / 'hrm_bundle.pt').touch()
    label, path = discover_bundles([tmp_path])[0]
    assert label == 'class0 | F:mlp / ID:cnn · cable_loadcell'
    assert path == str(folder / 'hrm_bundle.pt')


def test_agreed_masked_model_is_preselected_without_starting_it(app, tmp_path):
    bundle = tmp_path / DEFAULT_BUNDLE_RELATIVE_PATH
    bundle.parent.mkdir(parents=True)
    bundle.touch()  # GUI only discovers paths; it must not load Torch/checkpoints.
    selector = ForceModelSelector(roots=[tmp_path])
    assert selector.process_status == 'stopped'
    assert selector.for_start() == {'force_model_path': str(bundle)}
    assert selector.combo.currentText() == (
        'masked | F:kalmannet / ID:transformer · cable_loadcell_angle')
    selector.close()


def test_missing_default_does_not_silently_change_features(app, tmp_path):
    alternate = (tmp_path / 'cable_loadcell' / 'models' /
                 'masked__force_kalmannet__location_transformer' / 'hrm_bundle.pt')
    alternate.parent.mkdir(parents=True)
    alternate.touch()
    selector = ForceModelSelector(roots=[tmp_path])
    assert preferred_bundle_path([tmp_path]) == ''
    with pytest.raises(ValueError, match='Choose'):
        selector.for_start()
    selector.close()


def test_refresh_preserves_manual_model_or_explicit_empty_choice(app, tmp_path):
    default = tmp_path / DEFAULT_BUNDLE_RELATIVE_PATH
    alternate = tmp_path / 'cable_angle/models/class0__force_mlp__location_cnn/hrm_bundle.pt'
    for bundle in (default, alternate):
        bundle.parent.mkdir(parents=True)
        bundle.touch()
    selector = ForceModelSelector(roots=[tmp_path])
    selector.combo.setCurrentIndex(selector.combo.findData(str(alternate)))
    selector.refresh()
    assert selector.for_start() == {'force_model_path': str(alternate)}
    selector.combo.setCurrentIndex(0)
    selector.refresh()
    with pytest.raises(ValueError, match='Choose'):
        selector.for_start()
    selector.close()


def test_removed_default_is_not_replaced_with_another_model(app, tmp_path):
    default = tmp_path / DEFAULT_BUNDLE_RELATIVE_PATH
    default.parent.mkdir(parents=True)
    default.touch()
    selector = ForceModelSelector(roots=[tmp_path])
    default.unlink()
    selector.refresh()
    assert selector.combo.currentData() == ''
    selector.close()


def test_preferred_source_or_installed_root_is_resolved_once(tmp_path):
    source = tmp_path / 'source_models'
    installed = tmp_path / 'installed_models'
    bundle = source / DEFAULT_BUNDLE_RELATIVE_PATH
    bundle.parent.mkdir(parents=True)
    bundle.touch()
    installed.symlink_to(source, target_is_directory=True)
    assert preferred_bundle_path([installed, source]) == str(bundle.resolve())
    assert len(discover_bundles([source, installed])) == 1


def test_workspace_default_bundle_is_available_when_models_are_installed(app):
    models = Path(__file__).resolve().parents[2] / 'estimation_pkg' / 'models'
    bundle = models / DEFAULT_BUNDLE_RELATIVE_PATH
    if not bundle.is_file():
        pytest.skip('Trained model artifacts are optional outside the hardware workspace')
    selector = ForceModelSelector(roots=[models])
    assert selector.for_start() == {'force_model_path': str(bundle.resolve())}
    assert bundle.stat().st_size > 0
    selector.close()
