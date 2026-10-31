"""Offline M/B/K candidate calculator; deliberately has no ROS or robot access."""

import math

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QHeaderView, QLabel, QPlainTextEdit, QTableWidget, QTableWidgetItem,
    QVBoxLayout,
)

from .admittance_tuning import Candidate, analyze_model, propose_gains


class AdmittanceTuningDialog(QDialog):
    """Inspect assumed design requirements, then stage (never apply) a candidate."""

    METRICS = (
        ('velocity_time_constant_s', 'Velocity time constant M/B [s]'),
        ('nominal_velocity_mm_s', 'Zero-offset quasi-steady error/B [mm/s], NOT a limit'),
        ('spring_error_mn', 'Spring force K × assumed correction [mN]'),
        ('steady_offset_mm', 'Free-space error / K equilibrium [mm]'),
        ('bias_drift_mm_s', 'K=0 bias / B drift [mm/s]'),
        ('natural_frequency_rad_s', 'Free virtual model natural frequency [rad/s]'),
        ('damping_ratio', 'Free virtual model damping ratio'),
        ('relaxation_time_s', 'Free-model decay time [s], NOT contact settling'),
    )

    def __init__(self, current, parent=None):
        super().__init__(parent)
        self.current = tuple(float(value) for value in current)
        # Validate the confirmed snapshot before using it to derive initial inputs.
        analyze_model(*self.current)
        self.candidate = None
        self.setWindowTitle('Admittance tuning — calculate only, no robot action')
        self.resize(980, 880)
        layout = QVBoxLayout(self)
        intro = QLabel(
            'Choose the intended behaviour and assumed correction requirements. '
            'This is a candidate calculator, NOT automatic tuning or a stability guarantee. '
            'The confirmed settings below are the last controller read, not live telemetry.')
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form = QFormLayout()
        self.mode = QComboBox()
        self.mode.addItem('Force tracking (M–B, K = 0; drift possible)', 'force')
        self.mode.addItem('Spring compliance (M–B–K; force error can remain)', 'compliance')
        self.mode.setCurrentIndex(1 if self.current[2] > 0.0 else 0)
        form.addRow('Behaviour', self.mode)
        self.inputs = {}
        # Error and distance are editable illustrative requirements, NOT measurements.
        force_error_mn, correction_distance_mm = 100.0, 5.0
        mass, damping, stiffness = self.current
        values = (
            ('force_error_mn', 'Assumed force error [mN]', force_error_mn, 10.0),
            ('correction_speed_mm_s', 'Desired error/B velocity scale [mm/s]',
             force_error_mn / damping if damping > 0.0 else 0.0, 0.1),
            ('velocity_time_constant_s', 'Desired velocity time constant [s]',
             mass / damping if damping > 0.0 else 0.0, 0.01),
            ('spring_error_budget_mn', 'Allowed spring-force offset [mN]',
             stiffness * correction_distance_mm, 1.0),
            ('correction_distance_mm', 'Assumed correction distance [mm]',
             correction_distance_mm, 0.1),
        )
        for key, caption, value, step in values:
            spin = QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(0.0, 1.e9)
            spin.setSingleStep(step)
            spin.setValue(value)
            spin.setMinimumWidth(150)
            spin.valueChanged.connect(self.recalculate)
            self.inputs[key] = spin
            form.addRow(caption, spin)
        layout.addLayout(form)
        assumptions = QLabel(
            'Initial error = 100 mN and distance = 5 mm are editable examples, not '
            'recommended operating values. Speed, time constant and spring budget are '
            'derived from the confirmed M/B/K where possible. '
            'B = error/speed, M = B × time constant; compliance K = spring budget/distance.')
        assumptions.setWordWrap(True)
        layout.addWidget(assumptions)

        self.table = QTableWidget(3 + len(self.METRICS), 3)
        self.table.setHorizontalHeaderLabels(('Quantity', 'Confirmed current', 'Candidate'))
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setWordWrap(False)
        self.table.setMinimumHeight(310)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        layout.addWidget(self.table)
        self.diagnostics = QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.diagnostics.setFixedHeight(100)
        layout.addWidget(self.diagnostics)
        self.safety = QLabel(
            'Shared Y/Z gains: a zero Fz target does NOT disable Z admittance. '
            'Force mode (K=0) can drift under bias and must not seek contact blindly. '
            'These free-model calculations do not include the environment, inner position '
            'loop, delay, saturation or cross-axis coupling. They cannot guarantee convergence '
            'or physical speed. Stop motor output before controlled tuning trials. '
            'Use candidate only fills the M/B/K boxes; leave Admittance mode and explicitly '
            'Apply separately to change the controller.')
        self.safety.setWordWrap(True)
        self.safety.setStyleSheet('color: #8b3b00;')
        layout.addWidget(self.safety)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.use_button = self.buttons.addButton('Use candidate', QDialogButtonBox.AcceptRole)
        self.buttons.accepted.connect(self._accept_candidate)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)
        self.mode.currentIndexChanged.connect(self.recalculate)
        self.recalculate()

    @staticmethod
    def _format(value):
        return '—' if value is None else f'{value:.6g}'

    def _show_table(self, current_analysis, candidate_analysis):
        current_values = list(self.current)
        candidate_values = ([self.candidate.mass, self.candidate.damping,
                             self.candidate.stiffness] if self.candidate else [None] * 3)
        labels = ['M [kg]', 'B [N s/m]', 'K [N/m]']
        for key, label in self.METRICS:
            labels.append(label)
            current_values.append(current_analysis.get(key))
            candidate_values.append(candidate_analysis.get(key))
        for row, label in enumerate(labels):
            for column, text in enumerate((label, self._format(current_values[row]),
                                           self._format(candidate_values[row]))):
                item = QTableWidgetItem(text)
                if column:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table.setItem(row, column, item)
        self.table.resizeRowsToContents()

    def recalculate(self, *_unused):
        values = {key: spin.value() for key, spin in self.inputs.items()}
        mode = self.mode.currentData()
        self.inputs['spring_error_budget_mn'].setEnabled(mode == 'compliance')
        analysis_kwargs = dict(force_error_mn=values['force_error_mn'],
                               displacement_mm=values['correction_distance_mm'], bias_mn=10.0)
        current_analysis = analyze_model(*self.current, **analysis_kwargs)
        self.table.horizontalHeaderItem(1).setToolTip(
            '\n'.join(current_analysis.get('warnings', ())))
        self.candidate = None
        candidate_analysis = {}
        try:
            candidate = propose_gains(mode=mode, **values)
            # Match the existing panel's four-decimal editors BEFORE analysing/staging.
            numbers = tuple(round(value, 4) for value in
                            (candidate.mass, candidate.damping, candidate.stiffness))
            if any(not math.isfinite(value) or value > 1.e9 for value in numbers):
                raise ValueError('Candidate exceeds the existing M/B/K editor range.')
            if numbers[1] <= 0.0 or (candidate.mass > 0.0 and numbers[0] <= 0.0):
                raise ValueError('Candidate is too small for the four-decimal M/B/K editors.')
            if mode == 'compliance' and numbers[2] <= 0.0:
                raise ValueError('Compliance stiffness rounds to zero; choose a larger budget.')
            self.candidate = Candidate(*numbers)
            candidate_analysis = analyze_model(*numbers, **analysis_kwargs)
            notes = list(candidate_analysis.get('warnings', ()))
            notes.insert(0, 'Candidate rounded to the M/B/K editors’ four decimal places. '
                         'Bias diagnostics assume an illustrative 10 mN residual force.')
            self.diagnostics.setPlainText('\n'.join(notes))
            self.diagnostics.setStyleSheet('color: #666;')
        except ValueError as error:
            self.candidate = None
            self.diagnostics.setPlainText(f'No candidate: {error}')
            self.diagnostics.setStyleSheet('color: #d93025;')
        self._show_table(current_analysis, candidate_analysis)
        self.use_button.setEnabled(self.candidate is not None)

    def _accept_candidate(self):
        if self.candidate is not None:
            self.accept()
