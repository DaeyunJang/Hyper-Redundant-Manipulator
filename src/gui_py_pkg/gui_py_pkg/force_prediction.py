"""Display-only force prediction cache, expressed in sensor axes and mN."""

import math
import time


class ForcePredictionPreview:
    """Cache predictions; Vector3 units/frame must be agreed with the publisher.

    sensor_axes specifies each displayed sensor component in incoming XYZ.
    Receipt age is available, but Vector3 has no acquisition stamp or frame ID.
    """

    def __init__(self, unit='mN', sensor_axes=('x', 'y', 'z'), timeout_sec=0.5):
        if unit not in ('mN', 'N'):
            raise ValueError('predicted_force_unit must be mN or N')
        axes = tuple(sensor_axes)
        if (len(axes) != 3 or any(a not in ('x', 'y', 'z', '-x', '-y', '-z')
                                  for a in axes)
                or len({a.lstrip('-') for a in axes}) != 3):
            raise ValueError('predicted_force_sensor_axes must be a signed XYZ permutation')
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('predicted_force_timeout_sec must be finite and positive')
        self.scale = 1000.0 if unit == 'N' else 1.0
        self.mapping = tuple(('xyz'.index(a[-1]), -1 if a.startswith('-') else 1)
                             for a in axes)
        self.timeout_sec = timeout_sec
        self.values = None
        self.received_at = None
        self.status = 'Waiting'

    def receive(self, message, now=None):
        """Cache finite predictions; invalid input clears the previous value."""
        values = (message.x, message.y, message.z)
        mapped = tuple(sign * values[index] * self.scale for index, sign in self.mapping)
        if not all(math.isfinite(v) for v in mapped):
            self.values = None
            self.received_at = None
            self.status = 'Invalid'
            return
        self.values = mapped
        self.received_at = time.monotonic() if now is None else now
        self.status = 'Receiving'

    def snapshot(self, now=None):
        """Return None for unavailable data so plots can leave a gap."""
        now = time.monotonic() if now is None else now
        if self.received_at is None:
            return None, self.status
        if now - self.received_at > self.timeout_sec or now < self.received_at:
            return None, 'Stale'
        return self.values, self.status
