import json
import signal
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from gui_py_pkg.system_manager import (
    ComponentSpec, ManagedProcess, SystemProcessManager, load_component_specs,
)


def test_component_config_preserves_safe_startup_order(tmp_path):
    config = {
        'components': [
            {
                'id': 'camera',
                'label': 'Camera',
                'package': 'launcher',
                'launch_file': 'cuda_realsense.launch.py',
                'auto_start': True,
                'delay_sec': 0.0,
            },
            {
                'id': 'control',
                'label': 'Control',
                'package': 'robot_control_pkg',
                'launch_file': '_launch.py',
                'auto_start': True,
                'delay_sec': 3.0,
                'arguments': {
                    'motor_output_enabled': '{motor_output_enabled}',
                },
            },
        ]
    }
    path = tmp_path / 'components.json'
    path.write_text(json.dumps(config), encoding='utf-8')

    specs = load_component_specs(path)

    assert [spec.component_id for spec in specs] == ['camera', 'control']
    assert specs[0].delay_sec < specs[1].delay_sec
    assert specs[1].command({'motor_output_enabled': False}) == [
        'ros2',
        'launch',
        'robot_control_pkg',
        '_launch.py',
        'motor_output_enabled:=false',
    ]


def test_duplicate_component_ids_are_rejected(tmp_path):
    config = {
        'components': [
            {
                'id': 'same',
                'label': 'First',
                'package': 'one',
                'launch_file': 'one.launch.py',
            },
            {
                'id': 'same',
                'label': 'Second',
                'package': 'two',
                'launch_file': 'two.launch.py',
            },
        ]
    }
    path = tmp_path / 'components.json'
    path.write_text(json.dumps(config), encoding='utf-8')

    try:
        load_component_specs(path)
    except ValueError as error:
        assert 'Duplicate component id' in str(error)
    else:
        raise AssertionError('Duplicate component id was accepted')


@pytest.fixture
def lifecycle(monkeypatch):
    spec = ComponentSpec('control', 'Robot control', 'robot_control_pkg', '_launch.py',
                         expected_nodes=('/robot_control',))
    manager = SystemProcessManager([spec])
    managed = manager.processes['control']
    state = SimpleNamespace(now=100.0, return_code=None, group_alive=True)
    process = Mock(pid=12345)
    process.poll.side_effect = lambda: state.return_code
    popen = Mock(return_value=process)
    killpg = Mock()
    monkeypatch.setattr('gui_py_pkg.system_manager.subprocess.Popen', popen)
    monkeypatch.setattr('gui_py_pkg.system_manager.os.killpg', killpg)
    monkeypatch.setattr('gui_py_pkg.system_manager.time.monotonic', lambda: state.now)
    monkeypatch.setattr(
        managed, '_group_is_alive',
        lambda: managed.process_group_id is not None and state.group_alive)
    return SimpleNamespace(manager=manager, managed=managed, state=state,
                           process=process, popen=popen, killpg=killpg)


def test_stop_signals_only_launch_once_and_preserves_request(lifecycle):
    h = lifecycle
    assert h.manager.start('control', set())
    assert h.popen.call_args.kwargs == {
        'start_new_session': True, 'stdin': subprocess.DEVNULL}
    assert h.managed.status({'/robot_control'}) == 'running'
    assert h.manager.stop('control')
    h.process.send_signal.assert_called_once_with(signal.SIGINT)
    h.killpg.assert_not_called()
    h.state.now += 1
    assert h.manager.stop('control')
    h.process.send_signal.assert_called_once()
    assert h.managed.stop_requested_at == 100.0
    assert h.managed.status({'/robot_control'}) == 'stopping'
    assert not h.manager.start('control', set())


@pytest.mark.parametrize('return_code', [0, -signal.SIGINT, -signal.SIGTERM, 1])
def test_requested_exit_waits_for_graph_then_becomes_stopped(lifecycle, return_code):
    h = lifecycle
    h.manager.start('control', set())
    h.manager.stop('control')
    h.state.return_code = return_code
    h.state.group_alive = False
    h.manager.poll()
    assert h.managed.process is None
    assert h.managed.process_group_id is None
    assert h.managed.stop_requested_at == 100.0
    assert h.managed.status({'/robot_control'}) == 'graph_pending'
    assert not h.manager.start('control', {'/robot_control'})
    assert not h.manager.stop('control')
    h.state.now += 100
    assert h.managed.status({'/robot_control'}) == 'graph_pending'
    assert h.managed.status(set()) == 'stopped'
    assert h.manager.start('control', set())
    assert h.managed.stop_requested_at is None
    assert h.managed.last_return_code is None


def test_never_owned_external_process_is_not_started_or_signalled(lifecycle):
    h = lifecycle
    assert h.managed.status({'/robot_control'}) == 'external'
    assert not h.manager.start('control', {'/robot_control'})
    assert not h.manager.stop('control')
    h.manager.stop_all()
    h.popen.assert_not_called()
    h.killpg.assert_not_called()
    h.process.send_signal.assert_not_called()


def test_new_external_node_after_graph_clear_is_protected(lifecycle):
    h = lifecycle
    h.manager.start('control', set())
    h.manager.stop('control')
    h.state.return_code = 0
    h.state.group_alive = False
    assert h.managed.status(set()) == 'stopped'
    assert h.managed.status({'/robot_control'}) == 'external'
    assert not h.manager.start('control', {'/robot_control'})
    assert not h.manager.stop('control')
    h.killpg.assert_not_called()


def test_unexpected_failure_is_not_disguised_as_successful_stop(lifecycle):
    h = lifecycle
    h.manager.start('control', set())
    h.state.return_code = 1
    h.state.group_alive = False
    assert h.managed.status({'/robot_control'}) == 'graph_pending'
    assert h.managed.status(set()) == 'failed'


def test_orphaned_owned_children_remain_stoppable_not_external(lifecycle):
    h = lifecycle
    h.manager.start('control', set())
    h.state.return_code = 1
    assert h.managed.status({'/robot_control'}) == 'unhealthy'
    assert h.managed.is_alive
    assert h.manager.stop('control')
    h.process.send_signal.assert_not_called()
    h.killpg.assert_called_once_with(12345, signal.SIGTERM)
    assert h.managed.status({'/robot_control'}) == 'stopping'
    assert not h.manager.start('control', set())
    h.state.group_alive = False
    assert h.managed.status(set()) == 'stopped'


def test_timeout_targets_only_owned_group_without_resetting_stop_history(lifecycle):
    h = lifecycle
    h.manager.start('control', set())
    h.manager.stop('control')
    h.state.now += h.managed.STOP_TIMEOUT_SEC + 1
    h.manager.poll()
    h.killpg.assert_called_once_with(12345, signal.SIGTERM)
    assert h.managed.stop_requested_at == 100.0
    h.manager.poll()
    h.killpg.assert_called_once()


def test_partial_external_group_also_blocks_duplicate_start(lifecycle):
    h = lifecycle
    object.__setattr__(h.managed.spec, 'expected_nodes', ('/robot_control', '/helper'))
    assert h.managed.status({'/helper'}) == 'external'
    assert not h.manager.start('control', {'/helper'})


def test_group_probe_never_signals_unowned_targets(monkeypatch):
    managed = ManagedProcess(ComponentSpec('test', 'Test', 'test', 'test.launch.py'))
    killpg = Mock()
    monkeypatch.setattr('gui_py_pkg.system_manager.os.killpg', killpg)
    assert not managed._group_is_alive()
    killpg.assert_not_called()
    managed.process_group_id = 12345
    assert managed._group_is_alive()
    killpg.assert_called_once_with(12345, 0)
    killpg.side_effect = ProcessLookupError()
    assert not managed._group_is_alive()
    killpg.side_effect = PermissionError()
    assert managed._group_is_alive()
