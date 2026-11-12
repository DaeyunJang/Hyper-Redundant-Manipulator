#!/usr/bin/env python3
# AUTHOR: Daeyun - local device, display and data mapping for the HRM container.
"""Manage the motor-free container without installing or deleting host software."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / '.docker-local'


def run(arguments, **kwargs):
    return subprocess.run(arguments, check=True, **kwargs)


def current_devices(dev_root=Path('/dev')):
    """Only video/media/ESP32 nodes; never expose the entire host /dev tree."""
    paths = set()
    for pattern in ('video*', 'media*', 'ttyUSB*', 'ttyACM*'):
        for path in dev_root.glob(pattern):
            if path.is_char_device():
                paths.add(path.resolve())
    return sorted(paths)


def runtime_override(devices, host_groups):
    groups = {str(group) for group in host_groups}
    groups.update(str(path.stat().st_gid) for path in devices)
    return {'services': {'hrm': {
        'devices': [f'{path}:{path}:rw' for path in devices],
        'group_add': sorted(groups, key=int),
    }}}


def environment():
    env = os.environ.copy()
    env.update(HRM_UID=str(os.getuid()), HRM_GID=str(os.getgid()),
               HRM_HOME_DIR=str(STATE / 'home'),
               HRM_XAUTH_FILE=str(STATE / 'xauthority'))
    for key, fallback in (
        ('HRM_MODELS_DIR', ROOT / 'src/estimation_pkg/models'),
        ('HRM_RECORD_DIR', ROOT / 'record'),
    ):
        env[key] = str(Path(env.get(key, fallback)).expanduser().resolve())
    return env


def compose(env, override=False):
    command = ['docker', 'compose', '--project-directory', str(ROOT),
               '-f', str(ROOT / 'compose.yaml')]
    if override:
        command += ['-f', str(STATE / 'devices.json')]
    return command


def require_docker():
    if shutil.which('docker') is None:
        raise RuntimeError('Docker is not installed/on PATH. See docker/README.md; '
                           'this helper never installs host packages or asks for a password.')
    run(['docker', 'compose', 'version'], stdout=subprocess.DEVNULL)
    run(['docker', 'info'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def prepare_xauthority(required):
    target = STATE / 'xauthority'
    target.touch(mode=0o600, exist_ok=True)
    target.chmod(0o600)
    # Do not retain authorization cookies from a previous desktop session.
    target.write_bytes(b'')
    display = os.environ.get('DISPLAY', '')
    if not display or not shutil.which('xauth'):
        if required:
            raise RuntimeError('GUI requires a local DISPLAY and xauth. Use an Xorg '
                               'session (or working XWayland); do not use xhost +.')
        return
    result = run(['xauth', 'nlist', display], capture_output=True, text=True)
    cookie = result.stdout.strip()
    if not cookie:
        if required:
            raise RuntimeError('No Xauthority cookie for DISPLAY. Run from your desktop '
                               'terminal with correct DISPLAY/XAUTHORITY, without sudo.')
        return
    # FamilyWild permits the container hostname to use this display's cookie.
    cookie = '\n'.join('ffff' + line[4:] for line in cookie.splitlines()) + '\n'
    run(['xauth', '-f', str(target), 'nmerge', '-'], input=cookie, text=True,
        stdout=subprocess.DEVNULL)


def prepare_runtime(env, gui=False):
    STATE.mkdir(exist_ok=True)
    for directory in (env['HRM_HOME_DIR'], env['HRM_RECORD_DIR'], env['HRM_MODELS_DIR']):
        Path(directory).mkdir(parents=True, exist_ok=True)
    if not Path('/dev/bus/usb').is_dir():
        raise RuntimeError('/dev/bus/usb is missing; this profile targets a Linux hardware host.')
    if not Path('/tmp/.X11-unix').is_dir():
        raise RuntimeError('/tmp/.X11-unix is missing; this profile expects a Linux desktop.')
    prepare_xauthority(required=gui)
    devices = current_devices()
    override = runtime_override(devices, os.getgroups())
    (STATE / 'devices.json').write_text(json.dumps(override, indent=2) + '\n')
    print('Mapped local devices:', ', '.join(str(path) for path in devices) or '(USB bus only)', flush=True)
    print('Sensor-only mode: TCP omitted, physical motor output disabled; ROS domain '
          + env.get('HRM_ROS_DOMAIN_ID', '87'), flush=True)


def doctor():
    """Read-only preflight; does not create state, download, or probe cameras."""
    env = environment()
    print('Docker executable:', shutil.which('docker') or 'NOT FOUND')
    if shutil.which('docker'):
        for command in (['docker', 'compose', 'version'],
                        ['docker', 'info', '--format', '{{json .Runtimes}}']):
            result = subprocess.run(command, capture_output=True, text=True)
            print((result.stdout or result.stderr).strip())
    free_gib = shutil.disk_usage(ROOT).free / 1024**3
    print(f'Workspace free: {free_gib:.1f} GiB; reserve at least 25 GiB for initial build/cache '
          '(conservative estimate, not a measured image size).')
    print('Docker may store images on another filesystem; check its DockerRootDir too.')
    print('NVIDIA toolkit:', shutil.which('nvidia-ctk') or 'NOT FOUND')
    print('Display:', os.environ.get('DISPLAY') or 'NOT SET')
    print('xauth:', shutil.which('xauth') or 'NOT FOUND')
    print('Devices:', ', '.join(map(str, current_devices())) or '(no video/serial nodes)')
    models = Path(env['HRM_MODELS_DIR'])
    print('Model bundles:', sum(1 for _ in models.rglob('hrm_bundle.pt')) if models.is_dir() else 0)
    print('Recording directory:', env['HRM_RECORD_DIR'])
    print('No device was opened; cameras must not be used concurrently by native ROS/Viewer.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['doctor', 'build', 'config', 'gui', 'shell',
                                         'smoke', 'stop', 'logs'])
    parser.add_argument('--model', help='smoke only: path INSIDE container to a trusted hrm_bundle.pt')
    args = parser.parse_args()
    if args.model and args.action != 'smoke':
        parser.error('--model is only available with smoke')
    if args.action == 'doctor':
        doctor()
        return
    env = environment()
    require_docker()
    if args.action == 'build':
        jobs = env.get('HRM_BUILD_JOBS', '2')
        arch = env.get('HRM_CUDA_ARCH', '86')
        if not re.fullmatch(r'[1-9][0-9]*', jobs) or not re.fullmatch(r'[0-9]+', arch):
            raise RuntimeError('HRM_BUILD_JOBS and HRM_CUDA_ARCH must be positive integers.')
        if shutil.disk_usage(ROOT).free < 25 * 1024**3:
            print('WARNING: workspace has less than 25 GiB free. Check Docker storage space '
                  'before building; no files will be automatically deleted.', file=sys.stderr)
        run(compose(env) + ['build', 'hrm'], env=env)
    elif args.action in ('stop', 'logs'):
        # Never restart or remove unrelated containers or volumes.
        run(compose(env) + (['stop', 'hrm'] if args.action == 'stop' else ['logs', '-f', 'hrm']), env=env)
    else:
        prepare_runtime(env, gui=args.action == 'gui')
        command = compose(env, override=True)
        if args.action == 'config':
            command += ['config']
        elif args.action == 'gui':
            command += ['up', '--no-build', 'hrm']
        elif args.action == 'shell':
            command += ['run', '--rm', '--no-deps', 'hrm', 'bash']
        else:
            command += ['run', '--rm', '--no-deps', '-e', 'QT_QPA_PLATFORM=offscreen',
                        'hrm', '/usr/bin/python3', '/opt/hrm_ws/docker/smoke_test.py']
            if args.model:
                command += ['--model', args.model]
        run(command, env=env)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print(f'HRM Docker: {error}', file=sys.stderr)
        sys.exit(1)
