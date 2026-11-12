# AUTHOR: Daeyun - Docker helper regression checks without Docker or hardware.
"""No daemon, ROS node, camera, serial port or subprocess is launched."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('hrm_docker_manage', ROOT / 'docker/manage.py')
manage = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(manage)


def test_environment_keeps_host_home_and_uses_absolute_data_paths(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('HRM_MODELS_DIR', str(tmp_path / 'models'))
    monkeypatch.setenv('HRM_RECORD_DIR', str(tmp_path / 'records'))
    result = manage.environment()
    assert result['HOME'] == str(tmp_path)
    assert result['HRM_MODELS_DIR'] == str(tmp_path / 'models')
    assert result['HRM_RECORD_DIR'] == str(tmp_path / 'records')
    assert result['HRM_HOME_DIR'].endswith('/.docker-local/home')


def test_device_override_contains_only_explicit_devices(tmp_path):
    device = tmp_path / 'ttyUSB0'
    device.touch()
    result = manage.runtime_override([device], [46, 20, 20])['services']['hrm']
    assert result['devices'] == [f'{device}:{device}:rw']
    assert set(result['group_add']) == {'20', '46', str(device.stat().st_gid)}
    assert 'privileged' not in result
    assert json.loads(json.dumps(result)) == result


def test_device_scan_never_includes_disk_or_other_host_devices(monkeypatch, tmp_path):
    for name in ('video0', 'media0', 'ttyUSB0', 'ttyACM0', 'sda', 'mem', 'kmsg'):
        (tmp_path / name).touch()
    monkeypatch.setattr(Path, 'is_char_device', lambda self: True)
    assert {path.name for path in manage.current_devices(tmp_path)} == {
        'video0', 'media0', 'ttyUSB0', 'ttyACM0'}


def test_missing_docker_fails_without_installing(monkeypatch):
    monkeypatch.setattr(manage.shutil, 'which', lambda name: None)
    runner = Mock(side_effect=AssertionError('Must not launch a subprocess'))
    monkeypatch.setattr(manage, 'run', runner)
    with pytest.raises(RuntimeError, match='not installed/on PATH'):
        manage.require_docker()
    runner.assert_not_called()


def test_xauth_is_private_does_not_log_cookie_and_drops_old_cookie(monkeypatch, tmp_path):
    monkeypatch.setattr(manage, 'STATE', tmp_path)
    monkeypatch.setenv('DISPLAY', ':0')
    monkeypatch.setattr(manage.shutil, 'which', lambda name: '/usr/bin/xauth')
    target = tmp_path / 'xauthority'
    target.write_text('previous cookie')
    runner = Mock(side_effect=[SimpleNamespace(stdout='0100abcdefgh\n'), None])
    monkeypatch.setattr(manage, 'run', runner)
    manage.prepare_xauthority(required=True)
    assert target.stat().st_mode & 0o777 == 0o600
    assert target.read_bytes() == b''  # The mocked xauth command does not write.
    assert runner.call_args_list[1].kwargs['input'] == 'ffffabcdefgh\n'
    assert runner.call_args_list[0].kwargs['capture_output']


def test_xauth_failure_is_clear_and_no_xhost(monkeypatch, tmp_path):
    monkeypatch.setattr(manage, 'STATE', tmp_path)
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(manage, 'run', Mock(side_effect=AssertionError('No subprocess')))
    with pytest.raises(RuntimeError, match='local DISPLAY'):
        manage.prepare_xauthority(required=True)
    manage.run.assert_not_called()


def test_compose_defaults_to_full_gui_and_preserves_host_data():
    config = yaml.safe_load((ROOT / 'compose.yaml').read_text())
    service = config['services']['hrm']
    assert service['environment']['ROS_LOCALHOST_ONLY'] == '${HRM_ROS_LOCALHOST_ONLY:-0}'
    assert service['network_mode'] == service['ipc'] == 'host'
    assert 'privileged' not in service
    assert 'auto_start_components:=false' in service['command']
    mounts = {item['target']: item for item in service['volumes']}
    assert mounts['/opt/hrm_ws/src/estimation_pkg/models']['read_only']
    assert '/opt/hrm_ws/record' in mounts
    assert '/home/hrm' in mounts
    assert '/dev' not in mounts
    assert '/var/run/docker.sock' not in mounts


def test_entrypoint_keeps_native_launch_and_persistent_config():
    text = (ROOT / 'docker/entrypoint.sh').read_text()
    assert 'if [[ ! -e "$target" ]]' in text
    assert 'exec "$@"' in text
    assert 'set -- ros2 launch launcher system.launch.py auto_start_components:=false' in text
    assert 'motor_output_enabled:=true' not in text


def test_image_build_includes_control_bridge_and_context_is_allowlisted():
    ignore = (ROOT / '.dockerignore').read_text().splitlines()
    assert '**' in ignore
    assert '!src/tcp_pkg/**' in ignore
    assert 'src/estimation_pkg/models/**' in ignore
    assert not any(line.startswith('!record') or line.startswith('!.git/') for line in ignore)
    build = (ROOT / 'docker/build_workspace.sh').read_text()
    assert 'gui_py_pkg serial_pkg record_pkg robot_control_pkg tcp_pkg launcher' in build
    assert 'HRM_CONTAINER_BUILD' in build
    recipe = (ROOT / 'docker/Dockerfile').read_text()
    assert 'COPY src/tcp_pkg/ src/tcp_pkg/' in recipe


def test_shell_config_and_doctor_help_are_not_installers():
    source = (ROOT / 'docker/manage.py').read_text()
    assert "['sudo'" not in source
    assert 'system prune' not in source
    assert 'volume prune' not in source
    assert "['stop', 'hrm']" in source


def test_runtime_keeps_native_graph_defaults(monkeypatch):
    for name in ('HRM_ROS_DOMAIN_ID', 'HRM_ROS_LOCALHOST_ONLY',
                 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'):
        monkeypatch.delenv(name, raising=False)
    result = manage.environment()
    assert result['HRM_ROS_DOMAIN_ID'] == '0'
    assert result['HRM_ROS_LOCALHOST_ONLY'] == '0'
    monkeypatch.setenv('ROS_DOMAIN_ID', '23')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    result = manage.environment()
    assert result['HRM_ROS_DOMAIN_ID'] == '23'
    assert result['HRM_ROS_LOCALHOST_ONLY'] == '1'


def test_explicit_domain_override_is_preserved(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '23')
    monkeypatch.setenv('HRM_ROS_DOMAIN_ID', '28')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('HRM_ROS_LOCALHOST_ONLY', '0')
    result = manage.environment()
    assert result['HRM_ROS_DOMAIN_ID'] == '28'
    assert result['HRM_ROS_LOCALHOST_ONLY'] == '0'


@pytest.mark.parametrize('action, expected_tail', [
    ('gui', ['up', '--no-build', 'hrm']),
    ('shell', ['run', '--rm', '--no-deps', 'hrm', 'bash']),
    ('config', ['config']),
    ('smoke', ['run', '--rm', '--no-deps', '-e', 'QT_QPA_PLATFORM=offscreen',
               'hrm', '/usr/bin/python3', '/opt/hrm_ws/docker/smoke_test.py']),
])
def test_cli_uses_single_runtime_without_starting_processes(monkeypatch, action, expected_tail):
    monkeypatch.setattr(manage.sys, 'argv', ['manage.py', action])
    monkeypatch.setattr(manage, 'require_docker', lambda: None)
    env = {'HRM_ROS_DOMAIN_ID': '23', 'HRM_ROS_LOCALHOST_ONLY': '0'}
    monkeypatch.setattr(manage, 'environment', lambda: env.copy())
    preparation = Mock()
    runner = Mock()
    monkeypatch.setattr(manage, 'prepare_runtime', preparation)
    monkeypatch.setattr(manage, 'run', runner)
    manage.main()
    assert runner.call_args.kwargs['env'] == env
    assert preparation.call_args.args[0] == env
    assert preparation.call_args.kwargs['gui'] is (action == 'gui')
    assert runner.call_args.args[0][-len(expected_tail):] == expected_tail
