"""Configurable operating load decision from inferred hrm_base force, not FT truth.

No force overwrite, offsets, model retraining or motor action. The stateless
inclusive box rule is intentionally separate from the training calibration.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path


FORCE_RANGES_STATUS = 'ready_force_ranges'


@dataclass(frozen=True)
class LocalizationConfig:
    enabled: bool
    ranges_mN: tuple

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('enabled must be true or false')
        if len(self.ranges_mN) != 3:
            raise ValueError('Exactly three force ranges are required (fx, fy, fz)')
        for bounds in self.ranges_mN:
            if len(bounds) != 2 or any(
                    isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) for value in bounds):
                raise ValueError('Each force range must be two finite numbers in mN')
            if bounds[0] > bounds[1]:
                raise ValueError('Force range lower bound must be <= upper bound')

    def is_unloaded(self, force_N):
        """All three axes inside inclusive ranges; never confuse invalid with zero."""
        if len(force_N) != 3 or not all(math.isfinite(value) for value in force_N):
            raise ValueError('Load decision requires three finite inferred forces in N')
        # Convert config to N, not forces to rounded display mN; retain boundary
        # precision and use the same continuous value as the published force.
        return all(lower * 0.001 <= value <= upper * 0.001
                   for value, (lower, upper) in zip(force_N, self.ranges_mN))


def default_config_path():
    """Installed share resource, with a source-checkout fallback for unit tests."""
    fallback = Path(__file__).resolve().with_name('id_localization_config.json')
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        return fallback
    try:
        candidate = Path(get_package_share_directory('estimation_pkg')) / 'id_localization_config.json'
        if candidate.is_file():
            return candidate
    except PackageNotFoundError:
        pass
    return fallback


def load_localization_config(path):
    """Fail explicitly on missing/malformed settings; no silent policy fallback."""
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        # Permit whole-line // notes in this operator-edited config. Keep blank
        # lines so JSON parse errors retain their original line numbers; do not
        # strip quoted strings, inline comments, or silently repair bad syntax.
        data = json.loads(''.join('\n' if line.lstrip().startswith('//') else line
                                  for line in stream))
    keys = {'schema_version', 'enabled', 'unloaded_force_ranges_mN'}
    if not isinstance(data, dict) or set(data) != keys:
        raise ValueError('Localization config requires exactly: ' + ', '.join(sorted(keys)))
    if type(data['schema_version']) is not int or data['schema_version'] != 1:
        raise ValueError('Unsupported localization config schema_version (require 1)')
    ranges = data['unloaded_force_ranges_mN']
    if not isinstance(ranges, dict) or set(ranges) != {'fx', 'fy', 'fz'}:
        raise ValueError('unloaded_force_ranges_mN must contain fx, fy and fz')
    if any(not isinstance(ranges[axis], list) for axis in ('fx', 'fy', 'fz')):
        raise ValueError('Force ranges must be [lower, upper] arrays')
    return LocalizationConfig(data['enabled'], tuple(tuple(ranges[axis])
                              for axis in ('fx', 'fy', 'fz')))
