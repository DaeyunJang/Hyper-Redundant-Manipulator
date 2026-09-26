"""Select a serial device without opening it or changing a running connection."""

from pathlib import Path
import re

from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtWidgets import QVBoxLayout, QWidget


def discover_serial_ports(device_dir=Path('/dev')):
    """List USB/ACM device names; discovery never opens a serial connection."""
    paths = []
    for family in ('USB', 'ACM'):
        candidates = device_dir.glob(f'tty{family}[0-9]*')
        paths.extend(sorted(
            (path for path in candidates
             if re.fullmatch(r'tty(?:USB|ACM)\d+', path.name)),
            key=lambda path: int(re.search(r'\d+$', path.name).group()),
        ))
    return [str(path) for path in paths]


def validate_serial_port(value):
    """Require an existing serial device before the GUI starts its reader."""
    port = value.strip()
    if not re.fullmatch(r'/dev/(?:tty(?:USB|ACM)\d+|serial/by-id/[^/]+)', port):
        raise ValueError('Select /dev/ttyUSB*, /dev/ttyACM*, or a /dev/serial/by-id/ path.')
    if not Path(port).is_char_device():
        raise ValueError(f'{port} is not present. Connect the ESP32, Refresh, then select its port.')
    return port


class SerialPortSelector(QWidget):
    """Pending launch selection, separate from the GUI-owned launch's port."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.launch_port = None
        self.process_status = 'stopped'
        self.error = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        row.addWidget(QLabel('ESP32 port'))
        self.combo = QComboBox()
        self.combo.setEditable(True)
        self.combo.setInsertPolicy(QComboBox.NoInsert)
        self.combo.setMinimumContentsLength(18)
        # Automatically select a unique device, but never guess among several.
        self.combo.lineEdit().setPlaceholderText('Select USB / ACM port')
        self.combo.setToolTip(
            'Choose USB or ACM, or enter a stable /dev/serial/by-id/ path. '
            'Changing this selection only affects the next Serial sensors Start.'
        )
        row.addWidget(self.combo, 1)
        self.refresh_button = QPushButton('Refresh ports')
        self.refresh_button.clicked.connect(self.refresh_ports)
        row.addWidget(self.refresh_button)
        layout.addLayout(row)
        self.note = QLabel()
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.combo.currentTextChanged.connect(self._selection_changed)
        self.refresh_ports()

    def selected_port(self):
        return self.combo.currentText().strip()

    def refresh_ports(self):
        selected = self.selected_port()
        ports = discover_serial_ports()
        active = self.process_status in ('starting', 'running', 'unhealthy',
                                         'stopping', 'external')
        if not active and (not selected or not Path(selected).is_char_device()):
            selected = ports[0] if len(ports) == 1 else selected
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(ports)
        self.combo.setEditText(selected)
        self.combo.blockSignals(False)
        self._selection_changed()

    def _selection_changed(self):
        self.error = None
        self._update_note()

    def port_for_start(self):
        self.refresh_ports()
        try:
            return validate_serial_port(self.selected_port())
        except ValueError as error:
            self.error = str(error)
            self._update_note()
            raise

    def mark_started(self, port):
        self.launch_port = port
        self.error = None
        self.set_process_status('starting')

    def set_process_status(self, status):
        self.process_status = status
        self._update_note()

    def _update_note(self):
        if self.error:
            self.note.setText(self.error)
            self.note.setStyleSheet('color: #d93025;')
            return
        selected = self.selected_port()
        if self.process_status == 'external':
            text = ('External reader: its port is not controlled here. '
                    f'Next GUI Start: {selected or "(select a port)"}.')
        elif self.process_status in ('starting', 'running', 'unhealthy', 'stopping'):
            text = (f'Launch port: {self.launch_port or "unknown"}. '
                    f'Next Start: {selected or "(select a port)"}. '
                    'Stop Serial sensors before changing the connection.')
        else:
            present = bool(selected) and Path(selected).is_char_device()
            text = (f'Next Start: {selected} '
                    f'({"present" if present else "not present"}). '
                    if selected else 'Choose the ESP32 port before starting Serial sensors. ')
            text += 'A single connected port is selected automatically; choose if multiple.'
        self.note.setText(text)
        self.note.setStyleSheet('color: #666;')
