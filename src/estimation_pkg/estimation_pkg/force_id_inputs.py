"""Feature selection and fresh rolling histories; no ROS or actuator dependency."""

from collections import deque
from dataclasses import dataclass
import math
import re

import numpy as np


@dataclass(frozen=True)
class Feature:
    topic: str
    field: str
    index: int
    size: int
    child_frame: str = ''

    @property
    def source(self):
        # /tf carries independent streams: another child must not refresh this one.
        return source_key(self.topic, self.child_frame)


def source_key(topic, child_frame=''):
    return f'{topic}:{child_frame}' if child_frame else topic


def feature_spec(name):
    """Use the exact summary feature names, order, units and active-axis layout."""
    patterns = (
        (r'wire\.Cable #([1-4]) length', '/wire_length', 'data', 4),
        (r'wire_velocity\.Cable #([1-4]) velocity', '/wire_length_velocity', 'data', 4),
        (r'loadcell\.Loadcell #([1-4]) tension', '/loadcell_state', 'stress', 4),
        (r'loadcell\.Loadcell #([1-4]) voltage', '/loadcell_state', 'output_voltage', 4),
    )
    for pattern, topic, field, size in patterns:
        match = re.fullmatch(pattern, name)
        if match:
            return Feature(topic, field, int(match[1]) - 1, size)
    match = re.fullmatch(r'motor\.Motor #([1-4]) (position \[count\]|velocity|acceleration|torque)', name)
    if match:
        field = 'position' if match[2] == 'position [count]' else match[2]
        return Feature('/motor_state', 'actual_' + field, int(match[1]) - 1, 4)
    match = re.fullmatch(r'relative_angle_([1-9]|1[0-8])', name)
    if match:
        joint = int(match[1])
        return Feature('/estimated_segment_angle',
                       'tilt_relative' if joint % 2 else 'pan_relative', joint - 1, 18)
    match = re.fullmatch(r'relative_angular_velocity_([1-9]|1[0-8])_rad_s', name)
    if match:
        joint = int(match[1])
        axis = 'tilt' if joint % 2 else 'pan'
        return Feature('/estimated_segment_angle', axis + '_angular_velocity_relative', joint - 1, 18)
    match = re.fullmatch(r'(pan|tilt)_(relative|absolute)_(0[1-9]|1[0-8])_rad', name)
    if match:
        return Feature('/estimated_segment_angle', match[1] + '_' + match[2], int(match[3]) - 1, 18)
    match = re.fullmatch(r'(estimated_tip|fk_tip)\.([xyz])_m', name)
    if match:
        topic = ('/estimated_tip_position' if match[1] == 'estimated_tip'
                 else '/kinematics/fk_tip_position')
        return Feature(topic, 'position', 'xyz'.index(match[2]), 3)
    match = re.fullmatch(
        r'(fk_tip_tf|estimated_tip_orientation)\.([xyz]_m|q[xyzw]|roll_deg|pitch_deg|yaw_deg)', name)
    if match:
        group, column = match.groups()
        child = 'hrm_fk_tip' if group == 'fk_tip_tf' else 'estimated_segment_center_19'
        if column.endswith('_m'):
            if group != 'fk_tip_tf':
                raise ValueError('Estimated tip position uses estimated_tip, not the center TF.')
            return Feature('/tf', 'position', 'xyz'.index(column[0]), 3, child)
        if column.startswith('q'):
            return Feature('/tf', 'quaternion', 'xyzw'.index(column[1]), 4, child)
        return Feature('/tf', 'euler_deg', ('roll_deg', 'pitch_deg', 'yaw_deg').index(column), 3, child)
    raise ValueError(f'Unsupported live input feature: {name!r}. No force/ID labels or timestamps may be inputs.')


def pose_fields(position=None, rotation=None):
    """Match summary CSV units/conventions; retain original quaternion signs/values.

    XYZ is metres, quaternion order xyzw. Only Euler conversion normalizes the
    quaternion, exactly as record_pkg.summary_csv.orientation does (degrees).
    """
    fields = {}
    if position is not None:
        fields['position'] = [float(getattr(position, axis)) for axis in 'xyz']
        if not all(math.isfinite(v) for v in fields['position']):
            raise ValueError('Non-finite tip position')
    if rotation is not None:
        quaternion = [float(getattr(rotation, axis)) for axis in 'xyzw']
        norm = math.hypot(*quaternion)
        if not all(math.isfinite(v) for v in quaternion) or norm <= 1e-12 or not math.isfinite(norm):
            raise ValueError('Invalid tip quaternion')
        fields['quaternion'] = quaternion
        x, y, z, w = (value / norm for value in quaternion)
        fields['euler_deg'] = [
            math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))),
            math.degrees(math.asin(max(-1., min(1., 2 * (w*y - z*x))))),
            math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))),
        ]
    return fields


class FeatureHistory:
    """Sample required streams at a bounded rate, never extend dead-stream history.

    Cable values are mm and loadcell stresses retain training/native units.
    No zeroing, normalization, force labels or unused angle/pose prerequisites.
    """

    def __init__(self, columns, window_samples, *, timeout_sec=0.25,
                 max_skew_sec=0.1, max_gap_sec=0.15):
        self.columns = tuple(columns)
        if not self.columns or len(set(self.columns)) != len(self.columns):
            raise ValueError('Feature columns must be nonempty and unique')
        self.features = tuple(feature_spec(name) for name in self.columns)
        if isinstance(window_samples, bool) or not isinstance(window_samples, int) or window_samples < 1:
            raise ValueError('window_samples must be a positive integer')
        if any(not math.isfinite(v) or v <= 0 for v in (timeout_sec, max_skew_sec, max_gap_sec)):
            raise ValueError('Input timing limits must be positive and finite')
        self.required_topics = frozenset(f.topic for f in self.features)
        self.required_sources = frozenset(f.source for f in self.features)
        self.tf_children = frozenset(f.child_frame for f in self.features if f.topic == '/tf')
        self.window_samples = window_samples
        self.timeout_ns = round(timeout_sec * 1e9)
        self.skew_ns = round(max_skew_sec * 1e9)
        self.gap_ns = round(max_gap_sec * 1e9)
        self.samples = {}
        self.window = deque(maxlen=window_samples)
        self.last_used = {}
        self.last_sample_at = None
        self.last_source_ns = 0
        self.sequence = 0

    def put(self, topic, fields, *, source_ns, received_ns, received_monotonic_ns,
            child_frame=''):
        source = source_key(topic, child_frame)
        if source not in self.required_sources:
            return
        self.sequence += 1
        previous = self.samples.get(source)
        if source_ns <= 0:
            self.samples[source] = None
            return
        # Out-of-order or retransmitted stamped samples must not masquerade as new data.
        if previous is not None and source_ns <= previous['source_ns']:
            return
        selected = {}
        try:
            for feature in self.features:
                if feature.source != source:
                    continue
                values = fields[feature.field]
                if len(values) != feature.size:
                    raise ValueError('Wrong feature array length')
                value = float(values[feature.index])
                if not math.isfinite(value):
                    raise ValueError('Non-finite feature')
                selected[feature] = value
        except (KeyError, TypeError, ValueError, IndexError):
            self.samples[source] = None
            return
        self.samples[source] = dict(values=selected, source_ns=source_ns,
                                   received_ns=received_ns,
                                   received_monotonic_ns=received_monotonic_ns,
                                   sequence=self.sequence)

    def reset_window(self):
        self.window.clear()
        self.last_used.clear()
        self.last_sample_at = None
        self.last_source_ns = 0

    def sample(self, now_ns, monotonic_ns):
        for source in sorted(self.required_sources):
            sample = self.samples.get(source)
            if sample is None:
                self.reset_window()
                return None, 'missing_or_invalid_input:' + source
            receipt_age = monotonic_ns - sample['received_monotonic_ns']
            source_age = now_ns - sample['source_ns']
            if not 0 <= receipt_age <= self.timeout_ns or not -self.skew_ns <= source_age <= self.timeout_ns:
                self.reset_window()
                return None, 'stale_input:' + source
        stamps = [self.samples[t]['source_ns'] for t in self.required_sources]
        if max(stamps) - min(stamps) > self.skew_ns:
            self.reset_window()
            return None, 'input_time_skew'
        tokens = {t: self.samples[t]['sequence'] for t in self.required_sources}
        if any(tokens[t] == self.last_used.get(t) for t in tokens):
            return None, 'waiting_for_new_input'
        if self.last_sample_at is not None and monotonic_ns - self.last_sample_at > self.gap_ns:
            self.reset_window()
        row = [self.samples[f.source]['values'][f] for f in self.features]
        self.window.append(row)
        self.last_used = tokens
        self.last_sample_at = monotonic_ns
        self.last_source_ns = max(stamps)
        if len(self.window) < self.window_samples:
            return None, 'warming_up'
        # Match training preprocessing: branch-specific normalization happens in
        # float64 before conversion to network float32, not before scaling.
        return np.asarray(self.window, dtype=np.float64), 'ready'


def contact_policy(prediction, mode, active_ids, force_scope, body_enabled, id_tol,
                   force_ranges_config=None):
    """Diagnostic interpretation; no force overwrite, sign change or motor action."""
    probabilities = np.asarray(prediction.probabilities, dtype=float)
    if probabilities.shape != (19,) or not np.isfinite(probabilities).all():
        raise ValueError('Expected 19 finite probabilities')
    if (probabilities < 0).any() or not np.isclose(probabilities.sum(), 1.0, atol=1e-5):
        raise ValueError('Invalid probability distribution')
    raw_id = int(prediction.predicted_id)
    if mode not in ('class0', 'masked'):
        raise ValueError('Unsupported classification mode')
    center = int(prediction.location_id)
    if center not in active_ids or not 1 <= center <= 18:
        raise ValueError('Conditional location must be a supported positive ID')
    if force_ranges_config is not None and force_ranges_config.enabled:
        # Common operating policy for both architectures. The raw classifier
        # may disagree: class0 can predict0 even outside the force box, in which
        # case the positive-ID conditional argmax is the operating location.
        unloaded = force_ranges_config.is_unloaded(prediction.force_N)
        load_state = 1 if unloaded else 2
        control_id = 18 if unloaded else center
    elif mode == 'class0':
        load_state = 1 if raw_id == 0 else 2
        control_id = 18 if raw_id == 0 else raw_id
    else:
        load_state, control_id = 0, -1
    band = [i for i in active_ids if abs(i - center) <= id_tol]
    return dict(contact_id=raw_id, control_contact_id=control_id,
                load_state=load_state,
                # ID-gated eligibility diagnostic. The controller may explicitly
                # use ID only for visualization during physically tip-only tests.
                tip_force_valid=force_scope in ('tip', 'all_single_contact')
                and load_state != 0 and control_id == 18,
                body_contact_enabled=bool(body_enabled),
                body_contact_detected=load_state == 2 and 1 <= control_id < 18,
                heatmap_center_id=center, id_tol=id_tol,
                band_probability=float(probabilities[band].sum()))
