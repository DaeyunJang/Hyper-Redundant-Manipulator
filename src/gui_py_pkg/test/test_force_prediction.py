"""Prediction display contract without ROS traffic or robot hardware."""

from types import SimpleNamespace

import pytest

from gui_py_pkg.force_prediction import ForcePredictionPreview


def message(x=0.0, y=-12.5, z=100.0):
    return SimpleNamespace(x=x, y=y, z=z)


def test_no_prediction_is_not_a_zero_prediction():
    preview = ForcePredictionPreview()
    assert preview.snapshot(now=10) == (None, 'Waiting')
    preview.receive(message(), now=10)
    assert preview.snapshot(now=10.1) == ((0.0, -12.5, 100.0), 'Receiving')


def test_stale_and_recovery():
    preview = ForcePredictionPreview(timeout_sec=0.5)
    preview.receive(message(), now=10)
    assert preview.snapshot(now=10.5)[0] is not None
    assert preview.snapshot(now=10.501) == (None, 'Stale')
    preview.receive(message(x=-3), now=11)
    assert preview.snapshot(now=11)[0] == (-3, -12.5, 100)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_invalid_does_not_reuse_old_force(bad):
    preview = ForcePredictionPreview()
    preview.receive(message(), now=10)
    preview.receive(message(y=bad), now=10.1)
    assert preview.snapshot(now=10.2) == (None, 'Invalid')


def test_explicit_newtons_base_to_sensor_and_no_message_mutation():
    # For this recorded alignment, base=[-sensor_y,-sensor_x,-sensor_z],
    # the inverse also happens to be [-y,-x,-z]. Do not assume this generally.
    preview = ForcePredictionPreview(unit='N', sensor_axes=['-y', '-x', '-z'])
    msg = message(x=0.1, y=-0.2, z=0.0)
    preview.receive(msg, now=10)
    assert preview.snapshot(now=10)[0] == (200.0, -100.0, 0.0)
    assert (msg.x, msg.y, msg.z) == (0.1, -0.2, 0.0)


def test_unit_conversion_overflow_is_invalid():
    preview = ForcePredictionPreview(unit='N')
    preview.receive(message(x=1e308), now=10)
    assert preview.snapshot(now=10) == (None, 'Invalid')


@pytest.mark.parametrize('kwargs', [
    {'unit': 'kg'}, {'sensor_axes': ['x', 'x', 'z']},
    {'sensor_axes': ['x', 'y']}, {'sensor_axes': ['x', 'y', 'q']},
    {'timeout_sec': 0}, {'timeout_sec': -1}, {'timeout_sec': float('nan')},
])
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        ForcePredictionPreview(**kwargs)
