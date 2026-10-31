"""ROS-free, portable force/contact-ID checkpoint inference.

The output is raw continuous force in the bundle's training sign (N, hrm_base).
The future controller's -1 sign conversion and tip/body policy belong outside
this loader. In particular, a body ID does not turn a tip-only force regressor
into a body-force estimator.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Prediction:
    force_N: np.ndarray
    probabilities: np.ndarray  # Fixed index = physical ID, length 19 (0..18).
    predicted_id: int          # Includes class0 when that class exists.
    location_id: int           # Conditional candidate among supported body/tip IDs.
    no_load_probability_available: bool


def discover_bundles(root):
    """List nonempty bundles without importing torch or loading every checkpoint."""
    root = Path(root).expanduser()
    if root.is_file():
        return [root] if root.name == 'hrm_bundle.pt' and root.stat().st_size else []
    return sorted(path for path in root.rglob('hrm_bundle.pt')
                  if path.is_file() and path.stat().st_size > 0)


class BundleRuntime:
    def __init__(self, path, device='cpu'):
        # Lazy import keeps GUI discovery and ordinary estimator use torch-free.
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError('PyTorch is required; launch this node with the HRM Python '
                               'environment containing torch, not system Python.') from exc
        from .force_id_models import IndependentHRM

        self.path = Path(path).expanduser().resolve(strict=True)
        self.model_id = self.path.parent.name
        self.device = torch.device(device)
        # No pickle fallback: only tensor/primitive schema-2 bundles are accepted.
        bundle = torch.load(self.path, map_location='cpu', weights_only=True)
        if not isinstance(bundle, dict) or bundle.get('kind') != 'independent_hrm':
            raise ValueError('Expected an independent_hrm checkpoint bundle.')
        if bundle.get('schema_version') != 2:
            raise ValueError(f"Unsupported bundle schema {bundle.get('schema_version')!r}.")
        config = bundle['config']
        self.feature_columns = tuple(bundle.get('feature_columns') or config['input_columns'])
        self.input_dim = int(bundle['input_dim'])
        self.window_samples = int(config['window_samples'])
        if self.input_dim < 1 or self.window_samples < 1:
            raise ValueError('input_dim and window_samples must be positive.')
        if (len(self.feature_columns) != self.input_dim or
                len(set(self.feature_columns)) != self.input_dim or
                not all(isinstance(name, str) for name in self.feature_columns)):
            raise ValueError('Feature order must contain exactly input_dim unique column names.')
        configured_columns = config.get('input_columns', list(self.feature_columns))
        if tuple(configured_columns) != self.feature_columns:
            raise ValueError('feature_columns and config.input_columns disagree.')
        self.mode = str(bundle['mode'])
        if self.mode not in ('class0', 'masked'):
            raise ValueError(f'Unsupported location mode: {self.mode}')
        self.active_ids = tuple(int(value) for value in config['active_ids'])
        if (not self.active_ids or len(set(self.active_ids)) != len(self.active_ids) or
                any(value < 1 or value > 18 for value in self.active_ids)):
            raise ValueError('active_ids must contain unique segment IDs in 1..18.')
        self.class_ids = ((0,) if self.mode == 'class0' else ()) + self.active_ids
        if len(self.class_ids) != int(bundle['num_classes']):
            raise ValueError('num_classes disagrees with mode/active_ids.')
        self.force_unit = str(bundle['force_unit'])
        self.force_frame = str(config['force_frame'])
        self.force_scope = str(bundle['force_scope'])
        if self.force_unit != 'N' or self.force_frame != 'hrm_base':
            raise ValueError('This ROS contract requires force in N and hrm_base.')
        self.future_hrm_control_multiplier = float(bundle.get('future_hrm_control_multiplier', -1))
        self.scalers = {}
        for branch in ('force', 'location'):
            saved = bundle['scalers'][branch]
            self.scalers[branch] = {
                key: self._vector(saved[key], self.input_dim, f'{branch}.{key}',
                                  positive=key == 'x_std')
                for key in ('x_mean', 'x_std')
            }
        self.force_mean = self._vector(bundle['scalers']['force']['y_mean'], 3, 'force.y_mean')
        self.force_std = self._vector(bundle['scalers']['force']['y_std'], 3,
                                      'force.y_std', positive=True)
        self.model = IndependentHRM(bundle)
        self.model.load_state_dict(bundle['state_dict'], strict=True)
        self.model.to(self.device).eval()
        # Keep only inference metadata, not large offline audit/history dictionaries.
        self.force_name = str(bundle['force_name'])
        self.location_name = str(bundle['location_name'])

    @staticmethod
    def _vector(value, length, label, positive=False):
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (length,) or not np.isfinite(array).all():
            raise ValueError(f'{label} must contain {length} finite numbers.')
        if positive and np.any(array <= 0):
            raise ValueError(f'{label} must be strictly positive.')
        return array

    def predict(self, window):
        """Infer one raw window, oldest row first; do not pad missing history."""
        import torch

        raw = np.asarray(window, dtype=np.float64)
        expected = (self.window_samples, self.input_dim)
        if raw.shape != expected or not np.isfinite(raw).all():
            raise ValueError(f'Expected a finite raw input window of shape {expected}.')
        inputs = {}
        for branch in ('force', 'location'):
            scaler = self.scalers[branch]
            normalized = ((raw - scaler['x_mean']) / scaler['x_std']).astype(np.float32)
            if not np.isfinite(normalized).all():
                raise ValueError(f'{branch} normalized inputs overflowed float32.')
            inputs[branch] = torch.as_tensor(normalized[None], device=self.device)
        with torch.inference_mode():
            standardized_force = self.model.force_net(inputs['force'])[0].cpu().numpy()
            logits = self.model.location_net(inputs['location'])[0]
            class_probabilities = torch.softmax(logits, dim=-1).cpu().numpy()
        force = standardized_force * self.force_std + self.force_mean
        if not np.isfinite(force).all() or not np.isfinite(class_probabilities).all():
            raise ValueError('Checkpoint inference returned non-finite output.')
        probabilities = np.zeros(19, dtype=np.float64)
        probabilities[list(self.class_ids)] = class_probabilities
        winner = self.class_ids[int(np.argmax(class_probabilities))]
        location_id = self.active_ids[int(np.argmax(probabilities[list(self.active_ids)]))]
        return Prediction(force, probabilities, winner, location_id, self.mode == 'class0')
