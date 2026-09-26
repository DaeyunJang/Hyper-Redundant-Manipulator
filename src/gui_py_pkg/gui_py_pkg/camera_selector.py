"""Compact, startup-only RealSense selection; never probes or opens hardware."""

import re

from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLineEdit, QWidget


class CameraSelector(QWidget):
    """Camera filters for the next CUDA launch, not the currently detected device."""

    SERIAL_HELP = (
        'Optional camera Serial Number from RealSense Viewer, not ASIC Serial '
        'Number or the USB sysfs serial. Both model and serial must match. '
        'Leave empty to select by model; use a serial for two identical models.')

    def __init__(self, parent=None, *, default_model='D405', substitution_prefix='camera'):
        super().__init__(parent)
        self.default_model = default_model
        self.substitution_prefix = substitution_prefix
        self.launch_selection = None
        self.process_status = 'stopped'
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self.combo = QComboBox()
        # The driver uses regex_search: the suffix keeps D435 distinct from D455.
        for model in ('D405', 'D435i', 'D435', 'D455', 'D415'):
            self.combo.addItem(model, model + '$')
        index = self.combo.findText(default_model)
        if index < 0:
            raise ValueError(f'Unsupported default camera model: {default_model}')
        self.combo.setCurrentIndex(index)
        self.combo.setMaximumWidth(90)
        self.serial_edit = QLineEdit()
        self.serial_edit.setMinimumWidth(80)
        self.serial_edit.setPlaceholderText('Serial (optional)')
        self.serial_edit.setToolTip(self.SERIAL_HELP)
        row.addWidget(self.combo)
        row.addWidget(self.serial_edit, 1)
        self.serial_edit.textChanged.connect(self._clear_error)
        self.set_process_status('stopped')

    def selected_device_type(self):
        return self.combo.currentData()

    def substitutions(self):
        # ROS launch requires a nonempty CLI argument and YAML must not turn
        # an all-digit serial (possibly with leading zeros) into an integer.
        serial = self.serial_edit.text().strip()
        if serial.startswith('_'):
            serial = serial[1:]
        return {
            f'{self.substitution_prefix}_device_type': self.selected_device_type(),
            f'{self.substitution_prefix}_serial_no': '_' + serial if serial else "''",
        }

    def for_start(self):
        serial = self.serial_edit.text().strip()
        if serial and not re.fullmatch(r'_?[0-9]+', serial):
            error = 'Camera serial must contain digits only, or be left empty.'
            self.serial_edit.setStyleSheet('border: 1px solid #d93025;')
            self.serial_edit.setToolTip(error)
            raise ValueError(error)
        return self.substitutions()

    def _clear_error(self):
        self.serial_edit.setStyleSheet('')
        self.serial_edit.setToolTip(self.SERIAL_HELP)

    def mark_started(self, substitutions):
        self.launch_selection = {
            name: substitutions[name]
            for name in (f'{self.substitution_prefix}_device_type',
                         f'{self.substitution_prefix}_serial_no')
        }
        self.set_process_status('starting')

    def set_process_status(self, status):
        self.process_status = status
        editable = status in ('stopped', 'failed')
        self.combo.setEnabled(editable)
        self.serial_edit.setEnabled(editable)
        if status == 'external':
            note = ('External camera: these fields do not describe or change its '
                    'device. Stop it in its original terminal before GUI Start.')
        elif not editable:
            note = ('Stop RealSense camera and wait for STOPPED before selecting '
                    'another device. No live camera switching is performed.')
        else:
            note = ('Next CUDA camera Start: match this model and optional serial. '
                    f'Default {self.default_model}; never fall back to another model. '
                    'Each camera has its own selection. For identical models, use '
                    'distinct serials. Recheck calibration/ROI after switching cameras.')
        self.setToolTip(note)
        self.combo.setToolTip(note)
