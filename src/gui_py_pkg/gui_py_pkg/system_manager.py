"""Lifecycle helper for ROS processes started by the operator GUI."""

from dataclasses import dataclass, field
import json
import os
import signal
import subprocess
import time


@dataclass(frozen=True)
class ComponentSpec:
    """One launchable system component loaded from the GUI configuration."""

    component_id: str
    label: str
    package: str
    launch_file: str
    expected_nodes: tuple = field(default_factory=tuple)
    auto_start: bool = False
    delay_sec: float = 0.0
    description: str = ''
    arguments: dict = field(default_factory=dict)

    def command(self, substitutions=None):
        substitutions = substitutions or {}
        command = ['ros2', 'launch', self.package, self.launch_file]
        for name, value in self.arguments.items():
            if isinstance(value, str):
                value = value.format(**substitutions)
            command.append(f'{name}:={str(value).lower()}')
        return command


def load_component_specs(config_path):
    """Read and validate the ordered component list."""

    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = json.load(config_file)

    specs = []
    seen_ids = set()
    for entry in config.get('components', []):
        component_id = entry['id']
        if component_id in seen_ids:
            raise ValueError(f'Duplicate component id: {component_id}')
        seen_ids.add(component_id)
        specs.append(ComponentSpec(
            component_id=component_id,
            label=entry['label'],
            package=entry['package'],
            launch_file=entry['launch_file'],
            expected_nodes=tuple(entry.get('expected_nodes', [])),
            auto_start=bool(entry.get('auto_start', False)),
            delay_sec=float(entry.get('delay_sec', 0.0)),
            description=entry.get('description', ''),
            arguments=dict(entry.get('arguments', {})),
        ))
    return specs


class ManagedProcess:
    """Own one ROS launch process without blocking the Qt event loop."""

    # Let ROS launch perform its own SIGINT -> TERM -> KILL cleanup first.
    STOP_TIMEOUT_SEC = 12.0
    START_TIMEOUT_SEC = 15.0

    def __init__(self, spec):
        self.spec = spec
        self.process = None
        self.started_at = None
        self.stop_requested_at = None
        self.last_return_code = None
        self.process_group_id = None
        self.exited_at = None
        self.awaiting_graph_clear = False
        self.last_stop_signal_at = None

    @property
    def is_alive(self):
        return (
            (self.process is not None and self.process.poll() is None)
            or self._group_is_alive()
        )

    def _group_is_alive(self):
        # Keep ownership if the launch parent exits before its children.
        # Never discover targets by ROS node name or signal an external group.
        if self.process_group_id is None:
            return False
        try:
            os.killpg(self.process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            # Unknown is not proof that the process has stopped.
            return True
        return True

    def start(self, substitutions=None):
        self.poll()
        if self.is_alive:
            return False

        self.process = subprocess.Popen(
            self.spec.command(substitutions),
            start_new_session=True,
            # ros2 launch detects noninteractive mode and forwards SIGINT to
            # its children; no inherited terminal or duplicate group signal.
            stdin=subprocess.DEVNULL,
        )
        self.process_group_id = self.process.pid
        self.started_at = time.monotonic()
        self.stop_requested_at = None
        self.last_return_code = None
        self.exited_at = None
        self.awaiting_graph_clear = False
        self.last_stop_signal_at = None
        return True

    def request_stop(self):
        self.poll()
        if not self.is_alive:
            return False
        if self.stop_requested_at is not None:
            return True  # Repeated Stop clicks must not resend SIGINT.
        self.stop_requested_at = time.monotonic()
        self.last_stop_signal_at = self.stop_requested_at
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            else:
                # Parent is gone, but this GUI still owns the surviving group.
                os.killpg(self.process_group_id, signal.SIGTERM)
        except ProcessLookupError:
            self.poll()
        return True

    def poll(self):
        if self.process is None:
            return

        return_code = self.process.poll()
        if return_code is not None and not self._group_is_alive():
            self.last_return_code = return_code
            self.process = None
            self.process_group_id = None
            self.exited_at = time.monotonic()
            self.awaiting_graph_clear = True
            # Preserve whether this was an operator-requested stop, even when
            # launch reports a signal/nonzero exit code.
            return

        if (
                self.stop_requested_at is not None
                and time.monotonic() - self.last_stop_signal_at
                > self.STOP_TIMEOUT_SEC):
            try:
                os.killpg(self.process_group_id, signal.SIGTERM)
            except ProcessLookupError:
                self.poll()
                return
            self.last_stop_signal_at = time.monotonic()

    def status(self, discovered_nodes):
        self.poll()
        expected = set(self.spec.expected_nodes)
        nodes_are_ready = not expected or expected.issubset(discovered_nodes)
        any_node_visible = bool(expected.intersection(discovered_nodes))

        if self.is_alive:
            if self.stop_requested_at is not None:
                return 'stopping'
            if self.process is not None and self.process.poll() is not None:
                return 'unhealthy'  # Launch exited but owned children remain.
            if nodes_are_ready:
                return 'running'
            if (
                    self.started_at is not None
                    and time.monotonic() - self.started_at
                    > self.START_TIMEOUT_SEC):
                return 'unhealthy'
            return 'starting'

        if self.awaiting_graph_clear:
            if any_node_visible:
                # A stale graph entry is neither a live process nor proof of an
                # external owner. Do not kill by name or allow duplicate starts.
                return 'graph_pending'
            self.awaiting_graph_clear = False
        if any_node_visible:
            return 'external'
        if self.stop_requested_at is not None:
            return 'stopped'
        if self.last_return_code not in (None, 0):
            return 'failed'
        return 'stopped'


class SystemProcessManager:
    """Ordered collection of processes controlled by the GUI."""

    def __init__(self, specs):
        self.specs = list(specs)
        self.processes = {
            spec.component_id: ManagedProcess(spec) for spec in self.specs
        }

    def start(self, component_id, discovered_nodes, substitutions=None):
        managed = self.processes[component_id]
        if managed.status(discovered_nodes) not in ('stopped', 'failed'):
            return False
        return managed.start(substitutions)

    def stop(self, component_id):
        return self.processes[component_id].request_stop()

    def stop_all(self):
        for spec in reversed(self.specs):
            self.stop(spec.component_id)

    def poll(self):
        for managed in self.processes.values():
            managed.poll()
