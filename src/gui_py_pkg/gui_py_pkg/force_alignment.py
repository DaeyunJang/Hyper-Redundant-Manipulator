"""Experiment settings, CSV-only force axes and acknowledged recorder startup."""

from concurrent.futures import Future

from PyQt5.QtWidgets import QCheckBox, QComboBox, QGroupBox, QHBoxLayout
from PyQt5.QtWidgets import QLabel, QSizePolicy, QSpinBox, QVBoxLayout
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from record_pkg.force_alignment import create_alignment
from record_pkg.experiment_labels import validate_contact_segment_id


class ForceAlignmentPanel(QGroupBox):
    """Output-axis mapping; never rotates live sensor or control messages."""

    def __init__(self, parent=None):
        super().__init__('Recording label / force axes (sensor → hrm_base)', parent)
        layout = QVBoxLayout(self)
        self.enabled_checkbox = QCheckBox('Add aligned_fx / aligned_fy / aligned_fz')
        options = QHBoxLayout()
        options.addWidget(self.enabled_checkbox)
        options.addStretch()
        self.save_images_checkbox = QCheckBox('Save crop RGB/depth')
        self.save_images_checkbox.setChecked(True)
        self.save_images_checkbox.setToolTip(
            'Save HRM crop color and depth files. OFF keeps numeric bag, CSV and '
            'metadata; no crop image is required. Frozen until Record/Stop/export finishes.')
        options.addWidget(self.save_images_checkbox)
        options.addWidget(QLabel('contact_segment_id'))
        self.contact_segment_id = QSpinBox()
        self.contact_segment_id.setRange(0, 18)
        self.contact_segment_id.setToolTip(
            '0: free motion; 1–18: externally loaded segment. '
            'Operator label, independent of measured force. Fixed until Record Stop/export.')
        options.addWidget(self.contact_segment_id)
        layout.addLayout(options)
        self.axis_combos = []
        row = QHBoxLayout()
        row.setSpacing(0)
        for axis in 'xyz':
            if self.axis_combos:
                row.addSpacing(16)
            label = QLabel(f'Base F{axis} =')
            label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
            row.addWidget(label)
            row.addSpacing(4)
            combo = QComboBox()
            combo.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
            for source in 'xyz':
                for sign in ('+', '-'):
                    combo.addItem(f'{sign} sensor F{source}', f'{sign}{source}')
            combo.setCurrentIndex(combo.findData(f'+{axis}'))
            combo.currentIndexChanged.connect(self.update_summary)
            self.axis_combos.append(combo)
            row.addWidget(combo)
        row.addStretch()
        layout.addLayout(row)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.enabled_checkbox.toggled.connect(self.update_summary)
        self.update_summary()

    def settings(self):
        enabled = self.enabled_checkbox.isChecked()
        # With the option off, a half-edited mapping must not block raw capture.
        axes = [combo.currentData() for combo in self.axis_combos] if enabled else None
        return create_alignment(enabled, axes)

    def update_summary(self):
        enabled = self.enabled_checkbox.isChecked()
        for combo in self.axis_combos:
            combo.setEnabled(enabled)
        try:
            config = self.settings()
        except ValueError as error:
            self.summary.setText(f'Invalid axis mapping: {error}')
            self.summary.setStyleSheet('color: #d93025;')
            return
        if config['enabled']:
            equations = ', '.join(
                f'aligned_f{axis} = {source[0]}sensor_f{source[1]}'
                for axis, source in zip('xyz', config['axes']))
            self.summary.setText(
                equations + '\nOriginal columns/units unchanged; frozen per Record session.')
        else:
            self.summary.setText('OFF — original force columns only. No live/control conversion.')
        self.summary.setStyleSheet('color: #666;')

    def show_session(self, config):
        """Reflect an externally started session's actual frozen configuration."""
        if not isinstance(config, dict):
            return
        try:
            config = create_alignment(config['enabled'], config['axes'])
        except (KeyError, TypeError, ValueError):
            return
        self.enabled_checkbox.setChecked(config['enabled'])
        for combo, source in zip(self.axis_combos, config['axes']):
            combo.setCurrentIndex(combo.findData(source))
        self.update_summary()


def request_record_start(parameter_client, record_client, alignment, contact_segment_id=0,
                         save_images=True):
    """Apply settings atomically; request capture only after positive ACK.

    Returns a future containing the usual SetBool response, keeping the Qt loop
    nonblocking. Failed/unavailable settings never fall back to a stale mapping.
    """
    alignment = create_alignment(alignment['enabled'], alignment['axes'])
    validate_contact_segment_id(contact_segment_id)
    if type(save_images) is not bool:
        raise ValueError('save_images must be boolean.')
    if not parameter_client.service_is_ready() or not record_client.service_is_ready():
        return None
    result = Future()
    request = SetParametersAtomically.Request(parameters=[
        Parameter('force_alignment_enabled', value=alignment['enabled']).to_parameter_msg(),
        Parameter('force_alignment_axes', value=alignment['axes']).to_parameter_msg(),
        Parameter('contact_segment_id', value=contact_segment_id).to_parameter_msg(),
        Parameter('save_images', value=save_images).to_parameter_msg(),
    ])

    def fail(message):
        if not result.done():
            result.set_result(SetBool.Response(success=False, message=message))

    def record_finished(future):
        if result.done():
            return
        try:
            result.set_result(future.result())
        except Exception as error:
            fail(f'Record start failed: {error}')

    def settings_finished(future):
        if result.done():
            return
        try:
            response = future.result().result
            if not response.successful:
                fail(f'Force-axis settings rejected: {response.reason}')
                return
            if not record_client.service_is_ready():
                fail('Recorder service disappeared after applying force-axis settings.')
                return
            record_client.call_async(SetBool.Request(data=True)).add_done_callback(record_finished)
        except Exception as error:
            fail(f'Force-axis configuration failed: {error}')

    try:
        parameter_client.call_async(request).add_done_callback(settings_finished)
    except Exception as error:
        fail(f'Force-axis configuration failed: {error}')
    return result
