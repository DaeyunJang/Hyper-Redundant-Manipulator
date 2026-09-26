"""Own a rosbag recorder process; never publish or replay actuator commands."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import yaml


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def image_archive_settings(config):
    """Legacy profiles without this section retain their original bag behavior."""
    value = config.get('image_archive', {})
    if not isinstance(value, dict):
        raise ValueError('image_archive must be an object.')
    result = dict(enabled=value.get('enabled', False),
                  topic=value.get('topic', '/estimated_segment_crop_image'),
                  queue_size=value.get('queue_size', 16), required=value.get('required', True))
    if type(result['enabled']) is not bool or type(result['required']) is not bool:
        raise ValueError('image_archive enabled/required must be boolean.')
    if (not isinstance(result['topic'], str) or not result['topic'].startswith('/')
            or ' ' in result['topic']):
        raise ValueError('image_archive topic must be an absolute ROS topic.')
    if type(result['queue_size']) is not int or not 1 <= result['queue_size'] <= 128:
        raise ValueError('image_archive queue_size must be an integer in 1..128.')
    return result


def save_json(path, value):
    """Replace metadata atomically; interruption keeps the last complete copy."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


def depth_archive_settings(config):
    value = config.get('depth_archive', {})
    if not isinstance(value, dict):
        raise ValueError('depth_archive must be an object.')
    candidate = dict(value)
    candidate.setdefault('topic', '/estimated_segment_crop_depth/image_raw')
    result = image_archive_settings({'image_archive': candidate})
    result['metadata_topic'] = value.get(
        'metadata_topic', '/estimated_segment_crop_depth/metadata')
    if (not isinstance(result['metadata_topic'], str)
            or not result['metadata_topic'].startswith('/')
            or ' ' in result['metadata_topic']):
        raise ValueError('depth_archive metadata_topic must be an absolute ROS topic.')
    return result


def selected_topics(config):
    topics = []
    for group in config['enabled_groups']:
        topics.extend(config['topic_groups'][group])
    topics.extend(config.get('extra_topics', []))
    if not topics or any(not isinstance(t, str) or not t.startswith('/') for t in topics):
        raise ValueError('Select at least one absolute ROS topic name.')
    topics = list(dict.fromkeys(topics))
    if not set(config['required_topics']).issubset(topics):
        raise ValueError('Every required topic must be selected for recording.')
    for name in ('max_cache_size_mb', 'max_bag_size_mb', 'min_free_disk_mb'):
        if not isinstance(config[name], int) or config[name] <= 0:
            raise ValueError(f'{name} must be a positive integer.')
    for name in ('required_max_gap_sec', 'startup_timeout_sec'):
        value = config.get(name, 2.0 if name == 'required_max_gap_sec' else 10.0)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite.')
    if not isinstance(config.get('auto_export_csv', True), bool):
        raise ValueError('auto_export_csv must be a boolean.')
    return topics


def qos_overrides(topics):
    """Image best-effort avoids backpressure; retain latched static transforms."""
    profiles = {}
    for topic in topics:
        if ('image' in topic or 'camera_info' in topic
                or topic.endswith(('centerline_points', 'reconstruction_markers'))):
            profiles[topic] = dict(
                reliability='best_effort', durability='volatile',
                history='keep_last', depth=10)
    if '/tf_static' in topics:
        profiles['/tf_static'] = dict(
            reliability='reliable', durability='transient_local',
            history='keep_last', depth=100)
    return profiles


class CaptureSession:
    """Small, independently testable process/session lifecycle."""

    def __init__(self, config, output_root):
        self.config = config
        self.topics = selected_topics(config)
        self.output_root = Path(output_root).expanduser().resolve()
        self.process = None
        self.directory = None
        self.metadata = {}
        self.state = 'idle'
        self.detail = 'Ready; recording has not started.'
        self.log_file = None
        self.stop_started = None

    @property
    def active(self):
        return self.process is not None

    def persist(self, strict=False):
        if self.directory:
            self.metadata.update(state=self.state, detail=self.detail)
            try:
                save_json(self.directory / 'session.json', self.metadata)
            except OSError as exc:
                if strict:
                    raise
                self.metadata['metadata_write_error'] = str(exc)
                self.detail = f'{self.detail.split(" Metadata write failed:")[0]} Metadata write failed: {exc}'

    def start(self, snapshot, allow_incomplete=False):
        if self.active:
            raise RuntimeError(f'Recorder is already {self.state}.')
        available = set(snapshot['published_topics'])
        missing = sorted(set(self.config['required_topics']) - available)
        if missing and not allow_incomplete:
            raise RuntimeError('Missing required publishers: ' + ', '.join(missing))
        self.output_root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.output_root).free < self.config['min_free_disk_mb'] * 1024**2:
            raise RuntimeError('Insufficient free disk space for recording.')
        ros2 = shutil.which('ros2')
        if not ros2:
            raise RuntimeError('ros2 executable not found; source the ROS workspace.')
        local_start = datetime.now().astimezone()
        name = local_start.strftime('%Y%m%d_%H%M%S_%f')
        self.directory = self.output_root / name
        self.directory.mkdir()  # Exclusive: never overwrite an earlier dataset.
        save_json(self.directory / 'recording_config.json', self.config)
        qos_path = self.directory / 'qos_overrides.yaml'
        qos_path.write_text(yaml.safe_dump(qos_overrides(self.topics)))
        command = [
            ros2, 'bag', 'record', '--output', str(self.directory / 'bag'),
            '--storage', 'sqlite3', '--storage-preset-profile', 'resilient',
            '--max-cache-size', str(self.config['max_cache_size_mb'] * 1024**2),
            '--max-bag-size', str(self.config['max_bag_size_mb'] * 1024**2),
            '--qos-profile-overrides-path', str(qos_path), *self.topics,
        ]
        self.metadata = {
            'schema_version': 2, 'session_id': name,
            'start_requested_local': local_start.isoformat(),
            'start_requested_utc': local_start.astimezone(timezone.utc).isoformat(),
            'snapshot': snapshot, 'selected_topics': self.topics,
            'missing_required_at_start': missing,
            'missing_selected_at_start': sorted(set(self.topics) - available),
            'command': command,
            'environment': {k: os.environ.get(k) for k in (
                'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_DISTRO',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')},
        }
        self.state = 'starting'
        self.detail = 'Starting rosbag; wait for recording status before motion.'
        self.stop_started = None
        self.persist(strict=True)
        self.log_file = (self.directory / 'recorder.log').open('w')
        try:
            self.process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=self.log_file,
                stderr=subprocess.STDOUT, start_new_session=True)
        except Exception as exc:
            self.log_file.close()
            self.log_file = None
            self.state = 'failed'
            self.detail = str(exc)
            self.persist()
            raise
        return self.directory

    def request_stop(self):
        if self.active and self.stop_started is None:
            self.stop_started = time.monotonic()
            self.state = 'stopping'
            self.detail = 'Flushing bag; do not power off until stopped.'
            if self.process.poll() is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
            self.persist()

    def poll(self, ready=False):
        if not self.active:
            return
        returncode = self.process.poll()
        if returncode is not None:
            requested = self.stop_started is not None
            self.process = None
            self.log_file.close()
            self.log_file = None
            self.metadata.update(exit_code=returncode, closed_utc=utc_now())
            bag_metadata = self.directory / 'bag' / 'metadata.yaml'
            clean = (requested and returncode == 0 and bag_metadata.is_file()
                     and not self.metadata.get('forced_shutdown'))
            self.state = 'stopped' if clean else 'failed'
            self.detail = ('Bag closed.' if clean else
                           'Recorder exited or bag was not finalized; inspect recorder.log.')
            if bag_metadata.is_file():
                try:
                    info = yaml.safe_load(bag_metadata.read_text())['rosbag2_bagfile_information']
                    counts = {item['topic_metadata']['name']: item['message_count']
                              for item in info['topics_with_message_count']}
                    self.metadata['message_counts'] = counts
                    empty = [t for t in self.config['required_topics'] if not counts.get(t, 0)]
                    self.metadata['required_topics_without_messages'] = empty
                    self.metadata['selected_topics_without_messages'] = [t for t in self.topics if not counts.get(t, 0)]
                    if empty:
                        self.detail += ' INCOMPLETE required topics: ' + ', '.join(empty)
                except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
                    self.detail += f' Could not read bag summary: {exc}'
            self.persist()
        elif self.state == 'starting' and ready:
            self.state = 'recording'
            self.detail = 'Recording; source continuity must be checked in the saved bag.'
            self.metadata['subscriptions_ready_utc'] = utc_now()
            self.persist()
        elif shutil.disk_usage(self.output_root).free < self.config['min_free_disk_mb'] * 1024**2:
            self.metadata['automatic_stop_reason'] = 'low_disk_space'
            self.request_stop()

    def close(self):
        """Gracefully flush only this owned recorder, even when launch stops us."""
        self.request_stop()
        if self.active:
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.metadata['forced_shutdown'] = True
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            self.poll()
