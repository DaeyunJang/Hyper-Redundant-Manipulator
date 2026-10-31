"""Offline Qt calculator checks; no ROS nodes, services, or motor commands."""

import os

from PyQt5.QtWidgets import QApplication, QDialog
import pytest

from gui_py_pkg.admittance_tuning_dialog import AdmittanceTuningDialog


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    return QApplication.instance() or QApplication([])


@pytest.fixture
def dialog(app):
    widget = AdmittanceTuningDialog((0.15, 20.0, 30.0))
    yield widget
    widget.close()


def test_initial_requirements_derive_from_confirmed_gains(dialog):
    assert dialog.current == (.15, 20., 30.)
    assert dialog.mode.currentData() == 'compliance'
    assert dialog.inputs['correction_speed_mm_s'].value() == 5.
    assert dialog.inputs['velocity_time_constant_s'].value() == .0075
    assert dialog.inputs['spring_error_budget_mn'].value() == 150.
    assert (dialog.candidate.mass, dialog.candidate.damping,
            dialog.candidate.stiffness) == (.15, 20., 30.)
    assert dialog.use_button.isEnabled()
    assert 'NOT automatic tuning' in dialog.findChildren(type(dialog.safety))[0].text()


def test_force_mode_sets_zero_spring_and_warns_about_independent_z_response(dialog):
    dialog.mode.setCurrentIndex(0)
    assert dialog.candidate.stiffness == 0.
    assert not dialog.inputs['spring_error_budget_mn'].isEnabled()
    assert 'does NOT disable Z' in dialog.safety.text()
    assert 'must not seek contact blindly' in dialog.safety.text()
    assert 'NOT a limit' in dialog.table.item(4, 0).text()
    assert dialog.table.item(7, 2).text() == '0.5'


def test_requirements_update_candidate_only(dialog):
    dialog.mode.setCurrentIndex(0)
    dialog.inputs['force_error_mn'].setValue(200.)
    dialog.inputs['correction_speed_mm_s'].setValue(10.)
    dialog.inputs['velocity_time_constant_s'].setValue(.03)
    assert (dialog.candidate.mass, dialog.candidate.damping,
            dialog.candidate.stiffness) == (.6, 20., 0.)
    assert dialog.current == (.15, 20., 30.)
    dialog._accept_candidate()
    assert dialog.result() == QDialog.Accepted


@pytest.mark.parametrize('key', [
    'force_error_mn', 'correction_speed_mm_s', 'velocity_time_constant_s',
    'spring_error_budget_mn', 'correction_distance_mm',
])
def test_invalid_requirement_disables_use(dialog, key):
    dialog.inputs[key].setValue(0.)
    assert dialog.candidate is None
    assert not dialog.use_button.isEnabled()
    dialog._accept_candidate()
    assert dialog.result() != QDialog.Accepted


def test_confirmed_first_order_model_is_shown_but_requires_explicit_positive_tau(app):
    widget = AdmittanceTuningDialog((0., 20., 5.))
    try:
        assert widget.table.item(0, 1).text() == '0'
        assert widget.candidate is None
        widget.inputs['velocity_time_constant_s'].setValue(.02)
        assert widget.candidate.mass == .4
    finally:
        widget.close()


def test_slow_spring_decay_is_visible_separately_from_fast_velocity_time_constant(app):
    widget = AdmittanceTuningDialog((.15, 20., 5.))
    try:
        assert widget.table.item(3, 1).text() == '0.0075'
        assert 'NOT contact settling' in widget.table.item(10, 0).text()
        assert float(widget.table.item(10, 1).text()) == pytest.approx(3.99249, abs=1.e-5)
        widget.mode.setCurrentIndex(0)
        assert widget.table.item(10, 2).text() == '—'
    finally:
        widget.close()


def test_rounding_is_visible_and_candidate_matches_editor_precision(dialog):
    dialog.inputs['velocity_time_constant_s'].setValue(.007333)
    assert dialog.candidate.mass == .1467
    assert 'four decimal places' in dialog.diagnostics.toPlainText()
    dialog.inputs['velocity_time_constant_s'].setValue(.000001)
    assert dialog.candidate is None
    assert 'too small' in dialog.diagnostics.toPlainText()


def test_compliance_must_not_silently_round_to_force_mode(dialog):
    dialog.inputs['spring_error_budget_mn'].setValue(.000001)
    assert dialog.candidate is None
    assert 'rounds to zero' in dialog.diagnostics.toPlainText()


def test_cancel_retains_no_accepted_result(dialog):
    dialog.reject()
    assert dialog.result() == QDialog.Rejected
