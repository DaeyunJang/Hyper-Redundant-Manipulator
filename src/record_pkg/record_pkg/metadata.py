"""Read-only, bounded metadata queries; never spin/block inside a service callback."""

import hashlib
import math
from pathlib import Path
import time

from rcl_interfaces.srv import GetParameters, ListParameters

from record_pkg.capture import utc_now


def reference_file(path):
    """Preserve exact file content; this is NOT proof of a running node's config."""
    path = Path(path)
    data = path.read_bytes()
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(data).hexdigest(),
                content=data.decode('utf-8'), captured_utc=utc_now(),
                source='file_at_recording_start_not_running_state')


def parameter_record(value):
    fields = (
        None, 'bool_value', 'integer_value', 'double_value', 'string_value',
        'byte_array_value', 'bool_array_value', 'integer_array_value',
        'double_array_value', 'string_array_value')
    field = fields[value.type]
    result = getattr(value, field) if field else None
    if field and 'array' in field:
        result = list(result)
    if field == 'byte_array_value':
        result = [item[0] if isinstance(item, bytes) else item for item in result]
    # Keep strict JSON, including unusual camera parameters with NaN/Inf values.
    def safe(item):
        return str(item) if isinstance(item, float) and not math.isfinite(item) else item
    result = [safe(item) for item in result] if isinstance(result, list) else safe(result)
    return dict(type=field.removesuffix('_value') if field else 'not_set', value=result)


class ParameterSnapshot:
    """List/get each scoped node's parameters once; poll from the ROS executor."""

    def __init__(self, node, names, timeout_sec=3.0):
        self.node = node
        self.deadline = time.monotonic() + timeout_sec
        self.report = dict(
            status='collecting', started_utc=utc_now(), timeout_sec=timeout_sec,
            note='Per-node parameter-service replies, not a globally atomic snapshot or hardware readback.',
            nodes={})
        self.jobs = {}
        discovered = [f'{ns.rstrip("/")}/{name}'
                      for name, ns in node.get_node_names_and_namespaces()]
        for name in dict.fromkeys(names):
            entry = self.report['nodes'][name] = dict(status='waiting')
            if discovered.count(name) > 1:
                entry.update(status='ambiguous', reason='Duplicate node name; cannot identify responder.')
                continue
            self.jobs[name] = dict(
                listing=node.create_client(ListParameters, name + '/list_parameters'),
                getting=node.create_client(GetParameters, name + '/get_parameters'),
                future=None, stage='waiting')

    @property
    def done(self):
        return self.report['status'] != 'collecting'

    def _finish_node(self, name, status, **details):
        entry = self.report['nodes'][name]
        entry.update(status=status, completed_utc=utc_now(), **details)
        job = self.jobs.pop(name)
        if job['future'] is not None and not job['future'].done():
            job['future'].cancel()
        self.node.destroy_client(job['listing'])
        self.node.destroy_client(job['getting'])

    def poll(self):
        if self.done:
            return
        for name, job in list(self.jobs.items()):
            entry = self.report['nodes'][name]
            try:
                if job['stage'] == 'waiting':
                    if job['listing'].service_is_ready() and job['getting'].service_is_ready():
                        job['future'] = job['listing'].call_async(ListParameters.Request(depth=0))
                        job['stage'] = 'listing'
                        entry.update(status='querying', requested_utc=utc_now())
                elif job['future'].done():
                    response = job['future'].result()
                    if job['stage'] == 'listing':
                        job['names'] = sorted(response.result.names)
                        if not job['names']:
                            self._finish_node(name, 'complete', parameters={})
                            continue
                        job['future'] = job['getting'].call_async(
                            GetParameters.Request(names=job['names']))
                        job['stage'] = 'getting'
                    else:
                        if len(response.values) != len(job['names']):
                            raise ValueError('Parameter response count does not match request.')
                        parameters = {key: parameter_record(value)
                                      for key, value in zip(job['names'], response.values)}
                        self._finish_node(name, 'complete', parameters=parameters)
            except Exception as exc:
                self._finish_node(name, 'error', reason=str(exc))
        if time.monotonic() >= self.deadline:
            for name, job in list(self.jobs.items()):
                status = 'unavailable' if job['stage'] == 'waiting' else 'timeout'
                self._finish_node(name, status, reason='Parameter service unavailable or did not reply in time.')
        if not self.jobs:
            self.report.update(
                status=('complete' if all(entry['status'] == 'complete'
                        for entry in self.report['nodes'].values()) else 'partial'),
                completed_utc=utc_now())

    def cancel(self):
        if not self.done:
            for name in list(self.jobs):
                self._finish_node(name, 'cancelled', reason='Recording stopped before metadata query completed.')
            self.report.update(status='partial', completed_utc=utc_now())

    def hardware_constants(self):
        sources = {}
        for name, entry in self.report['nodes'].items():
            parameters = entry.get('parameters', {})
            if 'hardware.OP_MODE' in parameters:
                sources[name] = {key.removeprefix('hardware.'): value['value']
                                 for key, value in parameters.items() if key.startswith('hardware.')}
        return dict(status='available' if sources else 'unavailable', sources=sources,
                    note='Compiled controller constants, NOT motor-driver readback. Reference files and recording notes are separate.')
