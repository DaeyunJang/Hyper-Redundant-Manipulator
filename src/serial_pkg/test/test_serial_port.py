"""Serial port startup parameter tests; physical devices are always mocked."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('custom_interfaces.msg')

Parameter = pytest.importorskip('rclpy.parameter').Parameter
serial_read = importlib.import_module('serial_pkg.serial_read')


@pytest.mark.parametrize('override, expected', [
    (None, '/dev/ttyUSB0'),
    ('/dev/ttyUSB1', '/dev/ttyUSB1'),
    ('/dev/ttyACM0', '/dev/ttyACM0'),
    ('/dev/serial/by-id/usb-ESP32-test', '/dev/serial/by-id/usb-ESP32-test'),
])
def test_startup_parameter_opens_exact_selected_port(monkeypatch, tmp_path, override, expected):
    monkeypatch.setenv('ROS_DOMAIN_ID', '84')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    # Patch the device-open call BEFORE creating any SerialNode. No real port
    # can be opened even if a test fails. Calibration/read loops are bypassed.
    opener = Mock(return_value=Mock(is_open=True))
    monkeypatch.setattr(serial_read.serial, 'Serial', opener)
    zero = Mock(return_value=True)
    monkeypatch.setattr(serial_read.SerialNode, 'set_zero', zero)
    monkeypatch.setattr(serial_read.SerialNode, 'read_serial_data', lambda self: None)
    args = [] if override is None else ['--ros-args', '-p', f'serial_port:={override}']
    node = None
    rclpy.init(args=args)
    try:
        node = serial_read.SerialNode()
        node.serial_thread.join(timeout=1)
        opener.assert_called_once_with(expected, 921600, timeout=1)
        zero.assert_called_once_with(100)
        assert node.serial_port == expected
        assert node.get_parameter('serial_port').value == expected
        assert node.describe_parameter('serial_port').read_only
        result = node.set_parameters([Parameter('serial_port', value='/dev/ttyUSB9')])[0]
        assert not result.successful
        assert node.get_parameter('serial_port').value == expected
        assert node.serial_port == expected
        assert opener.call_count == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_startup_with_disconnected_can_reaches_reader(monkeypatch, tmp_path):
    """Run real startup calibration, not the port-routing test's mocked zero."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '84')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    frame = b'/0,0,0,0,0,0,0,20,30,40;\n'
    connection = Mock(is_open=True)
    # A finite input prevents a regression from hanging this test indefinitely.
    connection.readline.side_effect = [frame] * 130
    monkeypatch.setattr(serial_read.serial, 'Serial', Mock(return_value=connection))
    reader = Mock()
    monkeypatch.setattr(serial_read.SerialNode, 'read_serial_data', reader)
    node = None
    rclpy.init(args=[])
    try:
        node = serial_read.SerialNode()
        node.serial_thread.join(timeout=1)
        assert connection.readline.call_count == 130
        reader.assert_called_once_with()
        assert not node.offset_force3d.any()
        assert not node.offset_torque3d.any()
        assert not node.offset_loadcell_weight.any()
    finally:
        connection.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('error', [KeyboardInterrupt, serial_read.serial.SerialException])
def test_interrupted_calibration_does_not_start_reader(monkeypatch, tmp_path, error):
    monkeypatch.setenv('ROS_DOMAIN_ID', '84')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    connection = Mock(is_open=True)
    connection.readline.side_effect = error
    monkeypatch.setattr(serial_read.serial, 'Serial', Mock(return_value=connection))
    reader = Mock()
    monkeypatch.setattr(serial_read.SerialNode, 'read_serial_data', reader)
    rclpy.init(args=[])
    try:
        with pytest.raises(error):
            serial_read.SerialNode()
        connection.close.assert_called_once_with()
        reader.assert_not_called()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('override, expected', [
    (None, '/dev/ttyUSB0'),
    ('/dev/ttyUSB1', '/dev/ttyUSB1'),
    ('/dev/ttyACM1', '/dev/ttyACM1'),
])
def test_launch_argument_forwards_a_string_without_starting_node(
        monkeypatch, tmp_path, override, expected):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch_log'))
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, LogInfo
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    path = Path(__file__).parents[1] / 'launch/_launch.py'
    spec = importlib.util.spec_from_file_location('serial_port_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = {}

    def fake_node(**kwargs):
        captured.update(kwargs)
        return LogInfo(msg='Serial execution mocked for launch parameter test.')

    monkeypatch.setattr(module, 'Node', fake_node)
    description = module.generate_launch_description()
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['serial_port'] = override
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    parameters = evaluate_parameters(context, normalize_parameters(captured['parameters']))
    assert list(parameters) == [{'serial_port': expected}]
    assert isinstance(parameters[0]['serial_port'], str)
    assert captured['package'] == 'serial_pkg'
    assert captured['executable'] == 'serial_read'
