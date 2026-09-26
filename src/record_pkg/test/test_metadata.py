"""Snapshot contracts; ROS messages with mocked service clients, no robot I/O."""

import hashlib
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.parameter import Parameter

from record_pkg.metadata import ParameterSnapshot, parameter_record, reference_file


@pytest.mark.parametrize('value', [
    True, 8, 4.33, 'CSP', [True, False], [1, 2], [1.5, 3.0], ['East', 'West'],
])
def test_parameter_types(value):
    result = parameter_record(Parameter('test', value=value).get_parameter_value())
    assert result['value'] == value
    json.dumps(result, allow_nan=False)


def test_unset_bytes_and_nonfinite_are_json_safe():
    assert parameter_record(Parameter('v').get_parameter_value())['value'] is None
    data = parameter_record(Parameter('v', value=[b'\x00', b'\xff']).get_parameter_value())
    assert data == {'type': 'byte_array', 'value': [0, 255]}
    json.dumps(data, allow_nan=False)
    for value in (float('nan'), [float('inf'), -float('inf')]):
        json.dumps(parameter_record(Parameter('v', value=value).get_parameter_value()), allow_nan=False)


def test_exact_file_snapshot_does_not_change_when_source_changes(tmp_path):
    path = tmp_path / 'hardware.hpp'
    original = b'#define OP_MODE 0x08\n'
    path.write_bytes(original)
    record = reference_file(path)
    path.write_text('#define OP_MODE 0x09\n')
    assert record['content'].encode() == original
    assert record['sha256'] == hashlib.sha256(original).hexdigest()
    assert 'not_running_state' in record['source']


def fake_node(parameters=None, ready=True):
    node = Mock()
    node.get_node_names_and_namespaces.return_value = [('control', '/')]
    parameters = parameters or {'hardware.OP_MODE': 8, 'control_mode': 3, 'motor_output_enabled': False}
    clients = {}
    def create_client(service_type, service_name):
        client = Mock()
        client.service_is_ready.return_value = ready
        future = client.call_async.return_value
        future.done.return_value = True
        if service_name.endswith('/list_parameters'):
            result = SimpleNamespace(result=SimpleNamespace(names=sorted(parameters)))
        else:
            result = SimpleNamespace(values=[
                Parameter(name, value=value).get_parameter_value()
                for name, value in sorted(parameters.items())])
        future.result.return_value = result
        clients[service_name] = client
        return client
    node.create_client.side_effect = create_client
    return node, clients


def test_async_snapshot_values_and_hardware_source():
    node, clients = fake_node()
    snapshot = ParameterSnapshot(node, ['/control'])
    assert not snapshot.done
    for _ in range(3):
        snapshot.poll()
    assert snapshot.done and snapshot.report['status'] == 'complete'
    entry = snapshot.report['nodes']['/control']
    assert entry['parameters']['control_mode']['value'] == 3
    assert entry['parameters']['motor_output_enabled']['value'] is False
    assert entry['requested_utc'] and entry['completed_utc']
    assert snapshot.hardware_constants()['sources']['/control']['OP_MODE'] == 8
    assert node.destroy_client.call_count == 2


def test_unavailable_is_not_filled_from_reference_or_notes():
    node, _ = fake_node(ready=False)
    snapshot = ParameterSnapshot(node, ['/control'])
    snapshot.deadline = 0
    snapshot.poll()
    assert snapshot.report['status'] == 'partial'
    assert snapshot.report['nodes']['/control']['status'] == 'unavailable'
    assert snapshot.hardware_constants()['status'] == 'unavailable'
    assert snapshot.hardware_constants()['sources'] == {}


def test_no_reply_times_out_and_releases_clients():
    node, clients = fake_node()
    snapshot = ParameterSnapshot(node, ['/control'])
    snapshot.poll()
    future = clients['/control/list_parameters'].call_async.return_value
    future.done.return_value = False
    snapshot.deadline = 0
    snapshot.poll()
    assert snapshot.report['nodes']['/control']['status'] == 'timeout'
    future.cancel.assert_called_once()
    assert node.destroy_client.call_count == 2


def test_cancel_partial_and_duplicate_name_not_queried():
    node, _ = fake_node()
    snapshot = ParameterSnapshot(node, ['/control'])
    snapshot.cancel()
    assert snapshot.done and snapshot.report['nodes']['/control']['status'] == 'cancelled'
    node.get_node_names_and_namespaces.return_value *= 2
    node.create_client.reset_mock()
    snapshot = ParameterSnapshot(node, ['/control'])
    snapshot.poll()
    assert snapshot.done and snapshot.report['nodes']['/control']['status'] == 'ambiguous'
    node.create_client.assert_not_called()


def test_service_exception_is_partial_not_a_recording_crash():
    node, clients = fake_node()
    snapshot = ParameterSnapshot(node, ['/control'])
    snapshot.poll()
    clients['/control/list_parameters'].call_async.return_value.result.side_effect = RuntimeError('offline')
    snapshot.poll()
    assert snapshot.report['nodes']['/control']['status'] == 'error'
    assert snapshot.done
