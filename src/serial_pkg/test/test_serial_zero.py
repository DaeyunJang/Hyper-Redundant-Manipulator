"""Exercise actual serial parsing/calibration/publication with finite fake input.

No ROS node is created and no physical serial device is opened. End-of-input
always raises, so a regression that rejects every sample cannot hang pytest.
"""

import importlib
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
serial_read = importlib.import_module('serial_pkg.serial_read')
Time = pytest.importorskip('builtin_interfaces.msg').Time
DataFilterSetting = pytest.importorskip('custom_interfaces.msg').DataFilterSetting


def packet(values):
    return ('/' + ','.join(map(str, values)) + ';\n').encode()


class FakeSerial:
    def __init__(self, records, end_error=None):
        self.records = iter(records)
        self.reads = 0
        self.closed = False
        self.end_error = end_error or AssertionError('Read beyond the finite fake serial input.')

    def readline(self):
        self.reads += 1
        try:
            return next(self.records)
        except StopIteration:
            raise self.end_error

    def close(self):
        self.closed = True


def fake_node(records=(), end_error=None):
    node = SimpleNamespace(
        ser=FakeSerial(records, end_error), serial_lock=threading.Lock(),
        offset_force3d=np.zeros((1, 3)), offset_torque3d=np.zeros((1, 3)),
        offset_loadcell_weight=np.zeros((1, 4)), get_logger=lambda: Mock(),
    )
    for name in ('parse_serial_data', 'set_zero', 'publishall', 'MovingAverageFilter',
                 'create_kalman_filter'):
        setattr(node, name, MethodType(getattr(serial_read.SerialNode, name), node))
    return node


@pytest.mark.parametrize('values', [
    [0, 0, 0, 0, 0, 0, 0, 12.5, -3.25, 40],
    [0] * 10,
    [1, -2, 3, -4, 5, -6, 0, 20, 30, 40],
])
def test_zero_values_complete_initial_average_without_zeroing_loadcells(values):
    node = fake_node([packet(values)] * 130)
    assert node.set_zero(100)
    assert node.ser.reads == 130  # Existing 30-frame flush + 100 valid samples.
    np.testing.assert_array_equal(node.offset_force3d, values[:3])
    np.testing.assert_array_equal(node.offset_torque3d, values[3:6])
    np.testing.assert_array_equal(node.offset_loadcell_weight, np.zeros((1, 4)))


def test_force_offsets_still_use_the_average_of_all_valid_samples():
    low = [1, 2, 3, 4, 5, 6, 100, 200, 300, 400]
    high = [3, 4, 5, 6, 7, 8, 900, 800, 700, 600]
    node = fake_node([packet(low)] * 80 + [packet(high)] * 50)
    assert node.set_zero(100)
    np.testing.assert_array_equal(node.offset_force3d, [2, 3, 4])
    np.testing.assert_array_equal(node.offset_torque3d, [5, 6, 7])
    np.testing.assert_array_equal(node.offset_loadcell_weight, np.zeros((1, 4)))


@pytest.mark.parametrize('text', [
    '', 'Load cell output: 0.0', '/1,2,3;',
    '/' + ','.join(['1'] * 9) + ';',
    '/' + ','.join(['1'] * 11) + ';',
    '/0,0,0,0,0,0,0,0,0,nan;',
    '/0,0,0,0,0,0,0,0,0,inf;',
    '/0,0,0,0,0,0,0,0,0,-inf;',
    '/0,0,0,0,0,0,0,0,0,not_a_number;',
    '/0,0,0,0,0,0,0,0,0,;',
    '/0,0,0,0,0,0,0,0,0,0',
])
def test_parser_rejects_malformed_wrong_count_and_nonfinite_records(text):
    assert fake_node().parse_serial_data(text) is None


def test_parser_accepts_exactly_ten_finite_values_including_zero():
    values = [0., -2.5, 3., 0., 5., 0., 0., 20., -30., 40.]
    assert fake_node().parse_serial_data(packet(values).decode()) == values


@pytest.mark.parametrize('end_error', [
    KeyboardInterrupt(), serial_read.serial.SerialException('Simulated serial disconnect'),
])
def test_calibration_interruption_does_not_report_success(end_error):
    node = fake_node([packet([1] * 10)] * 30, end_error=end_error)
    try:
        result = node.set_zero(100)
    except (KeyboardInterrupt, serial_read.serial.SerialException):
        result = False  # Propagating an error is also not false success.
    assert not result
    np.testing.assert_array_equal(node.offset_force3d, np.zeros((1, 3)))
    np.testing.assert_array_equal(node.offset_torque3d, np.zeros((1, 3)))


@pytest.mark.parametrize('succeeded', [True, False])
def test_zero_service_reports_actual_calibration_result(succeeded):
    node = fake_node()
    node.set_zero = Mock(return_value=succeeded)
    response = SimpleNamespace(success=False, message='')
    result = serial_read.SerialNode.set_zero_callback(
        node, SimpleNamespace(data=True), response)
    assert result.success is succeeded


@pytest.mark.parametrize('loadcells', [[0., 12.5, -3.25, 40.], [0., 0., 0., 0.]])
def test_actual_read_and_publish_path_accepts_zero_force_and_four_raw_loadcells(loadcells):
    values = [0.] * 6 + loadcells
    node = fake_node([packet(values)] * 131, end_error=KeyboardInterrupt())
    assert node.set_zero(100)
    node.force3d = np.zeros((1, 3))
    node.torque3d = np.zeros((1, 3))
    node.force3d_kf = np.zeros((1, 3))
    node.torque3d_kf = np.zeros((1, 3))
    node.loadcell_weight = np.zeros((1, 4))
    node.size_maf = 5
    node.buffer_count = 0
    node.force3d_buffer = np.zeros((5, 3))
    node.torque3d_buffer = np.zeros((5, 3))
    node.loadcell_weight_buffer = np.zeros((5, 4))
    node.data_filter_setting = DataFilterSetting()  # LPF/MAF both disabled.
    node.kalman_filters = [node.create_kalman_filter() for _ in range(6)]
    stamp = Time(sec=123, nanosec=456)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    for name in ('fts_publisher', 'fts_kalman_publisher', 'fts_offset_publisher',
                 'loadcell_publisher', 'loadcell_offset_publisher'):
        setattr(node, name, Mock())
    serial_read.SerialNode.read_serial_data(node)
    assert node.ser.closed and node.ser.reads == 132
    for publisher in (node.fts_publisher, node.fts_kalman_publisher):
        publisher.publish.assert_called_once()
        message = publisher.publish.call_args.args[0]
        assert message.header.stamp == stamp
        assert [getattr(message.wrench.force, axis) for axis in 'xyz'] == [0.] * 3
        assert [getattr(message.wrench.torque, axis) for axis in 'xyz'] == [0.] * 3
    node.loadcell_publisher.publish.assert_called_once()
    message = node.loadcell_publisher.publish.call_args.args[0]
    assert list(message.stress) == loadcells
    assert message.header.stamp == stamp
    np.testing.assert_array_equal(node.offset_loadcell_weight, np.zeros((1, 4)))
