"""Discover local inference bundles without importing Torch or opening hardware."""

from pathlib import Path

from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QSizePolicy, QWidget


DEFAULT_BUNDLE_RELATIVE_PATH = Path(
    'cable_loadcell_angle/models/'
    'masked__force_kalmannet__location_transformer/hrm_bundle.pt')


def default_model_roots():
    """Prefer the workspace source models, and also support an installed package."""
    roots = [Path(__file__).resolve().parents[2] / 'estimation_pkg' / 'models']
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(Path(get_package_share_directory('estimation_pkg')) / 'models')
    except (ImportError, LookupError):
        pass
    return roots


def preferred_bundle_path(roots):
    """Exact startup default; never substitute a different input-feature group."""
    for root in roots:
        candidate = Path(root) / DEFAULT_BUNDLE_RELATIVE_PATH
        if candidate.is_file():
            return str(candidate.resolve())
    return ''


def discover_bundles(roots):
    """Return unique real paths with readable relative labels; never load .pt files."""
    found = {}
    for root in roots:
        root = Path(root)
        if not root.is_dir():
            continue
        for bundle in root.rglob('hrm_bundle.pt'):
            if bundle.is_file():
                resolved = str(bundle.resolve())
                relative = bundle.parent.relative_to(root)
                # Architecture first: a narrow combo should not show the same
                # "cable_loadcell/models/..." prefix for every experiment.
                label = (f'{relative.name} [{relative.parent}]'
                         if relative.parent != Path('.') else str(relative))
                parts = relative.name.split('__')
                if (len(parts) == 3 and parts[0] in ('class0', 'masked')
                        and parts[1].startswith('force_') and parts[2].startswith('location_')):
                    group = relative.parent
                    if group.name == 'models':
                        group = group.parent
                    suffix = '' if group == Path('.') else f' · {group}'
                    label = f'{parts[0]} | F:{parts[1][6:]} / ID:{parts[2][9:]}{suffix}'
                found.setdefault(resolved, label)
    return sorted(((label, path) for path, label in found.items()),
                  key=lambda item: (item[0].lower(), item[1]))


class ForceModelSelector(QWidget):
    """Preselect the agreed model, but require explicit start and lock running models."""

    def __init__(self, parent=None, roots=None):
        super().__init__(parent)
        self.roots = list(default_model_roots() if roots is None else roots)
        self.process_status = 'stopped'
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        self.combo = QComboBox()
        self.combo.setMinimumWidth(0)
        self.combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.combo.setMinimumContentsLength(12)
        self.refresh_button = QPushButton('↻')
        self.refresh_button.setMaximumWidth(26)
        self.refresh_button.setToolTip('Rescan estimation_pkg/models for hrm_bundle.pt')
        self.refresh_button.clicked.connect(self.refresh)
        self.combo.currentIndexChanged.connect(self._show_path)
        layout.addWidget(self.combo, 1)
        layout.addWidget(self.refresh_button)
        self.refresh()

    def refresh(self):
        selected = self.combo.currentData()
        if selected is None:
            # Only the first scan selects a default. Preserve any later manual
            # choice (including the empty placeholder) when refreshing.
            selected = preferred_bundle_path(self.roots)
        self.combo.clear()
        self.combo.addItem('Choose force / ID model…', '')
        for label, path in discover_bundles(self.roots):
            self.combo.addItem(label, path)
        index = self.combo.findData(selected)
        self.combo.setCurrentIndex(max(0, index))
        self._show_path()

    def _show_path(self):
        self.combo.setToolTip(self.combo.currentData() or
                             'Choose an explicit model; no model starts automatically.')

    def for_start(self):
        selected = self.combo.currentData()
        if not selected or not Path(selected).is_file():
            raise ValueError('Choose an existing hrm_bundle.pt in Force / contact first.')
        return {'force_model_path': selected}

    def set_process_status(self, status):
        self.process_status = status
        editable = status in ('stopped', 'failed')
        self.combo.setEnabled(editable)
        self.refresh_button.setEnabled(editable)
        self.setToolTip('Stop Force / contact before selecting another model. '
                        'This component never enables physical motor output.')
