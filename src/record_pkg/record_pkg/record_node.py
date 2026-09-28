"""GUI-compatible numeric rosbag capture with optional asynchronous crop PNGs."""

import json
import math
import os
from collections import deque
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from custom_interfaces.msg import DataFilterSetting
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image

from record_pkg.capture import CaptureSession, image_archive_settings, depth_archive_settings
from record_pkg.force_alignment import create_alignment
from record_pkg.experiment_labels import validate_contact_segment_id
from record_pkg.metadata import ParameterSnapshot, reference_file
from record_pkg.topic_health import TopicHealth


class RecordNode(Node):
    def __init__(self):
        super().__init__('record')
        share = Path(get_package_share_directory('record_pkg'))
        self.declare_parameter('config_file', str(share / 'config' / 'recording.json'))
        self.declare_parameter('output_root', str(Path.cwd() / 'record'))
        self.declare_parameter('allow_incomplete', False)
        config_path = Path(self.get_parameter('config_file').value)
        config = json.loads(config_path.read_text())
        self.session = CaptureSession(config, self.get_parameter('output_root').value)
        alignment_defaults = config.get('force_alignment', {})
        if not isinstance(alignment_defaults, dict):
            raise ValueError('force_alignment config must be an object with enabled and axes.')
        alignment_defaults = create_alignment(
            alignment_defaults.get('enabled', False), alignment_defaults.get('axes'))
        self.declare_parameter('force_alignment_enabled', alignment_defaults['enabled'])
        self.declare_parameter('force_alignment_axes', alignment_defaults['axes'])
        self.current_force_alignment()  # Also validate command-line parameter overrides.
        self.declare_parameter('contact_segment_id', 0)
        validate_contact_segment_id(self.get_parameter('contact_segment_id').value)
        self.metadata_nodes = config.get('metadata_nodes', [])
        if not isinstance(self.metadata_nodes, list) or any(
                not isinstance(name, str) or not name.startswith('/') or name.endswith('/')
                for name in self.metadata_nodes):
            raise ValueError('metadata_nodes must contain absolute node names without a trailing slash.')
        self.metadata_timeout = float(config.get('metadata_timeout_sec', 3.0))
        if not math.isfinite(self.metadata_timeout) or self.metadata_timeout <= 0.0:
            raise ValueError('metadata_timeout_sec must be positive and finite.')
        self.parameter_snapshot = None
        self.health = TopicHealth(float(config.get('required_max_gap_sec', 2.0)))
        self.health_subscriptions = {}
        self.heavy_topics = set()
        self.image_archive_settings = image_archive_settings(config)
        self.image_archive = None
        self.depth_archive_settings = depth_archive_settings(config)
        self._image_archive_defaults = self.image_archive_settings.copy()
        self._depth_archive_defaults = self.depth_archive_settings.copy()
        self.declare_parameter('save_images', True)
        self.depth_archive = None
        self.depth_metadata_cache = deque(maxlen=64)
        self.depth_subscription = None
        if self.depth_archive_settings['enabled']:
            depth_topic = self.depth_archive_settings['topic']
            if depth_topic in self.session.topics:
                raise ValueError('Depth archive image topic must not also be selected for bag.')
            self.depth_subscription = self.create_subscription(
                Image, depth_topic, self.receive_archive_depth,
                QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(
                String, self.depth_archive_settings['metadata_topic'], self.receive_depth_metadata,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._archive_finalize_pending = False
        self._bag_closed_state = None
        self.image_subscription = None
        if self.image_archive_settings['enabled']:
            image_topic = self.image_archive_settings['topic']
            if image_topic in self.session.topics:
                raise ValueError('PNG archive image topic must not also be selected for bag.')
            self.image_subscription = self.create_subscription(
                Image, image_topic, self.receive_archive_image,
                QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.last_health_warning = None
        self.record_start_monotonic = None
        self.finalizer = None
        self.finalizer_log = None
        self.apply_image_recording_settings()
        self.add_on_set_parameters_callback(self.validate_force_alignment_parameters)
        self.create_timer(0.1, self.poll_metadata)
        self.last_filter_setting = None
        self.create_subscription(DataFilterSetting, '/data_filter_setting', self.filter_callback, 10)
        self.status_pub = self.create_publisher(
            String, '/data/record_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool, '/data/record', self.record_callback)
        self.create_timer(0.5, self.poll)
        self.last_status = None
        self.publish_status()
        self.get_logger().info('Recorder ready (not recording); use GUI Record or /data/record.')

    def current_force_alignment(self):
        return create_alignment(
            self.get_parameter('force_alignment_enabled').value,
            self.get_parameter('force_alignment_axes').value)

    def apply_image_recording_settings(self):
        """Freeze effective archive config before capture; keep numeric topics intact."""
        if self.force_alignment_locked():
            raise RuntimeError('Image recording settings are frozen until export finishes.')
        enabled = self.get_parameter('save_images').value
        if type(enabled) is not bool:
            raise ValueError('save_images must be boolean.')
        self.image_archive_settings = dict(
            self._image_archive_defaults,
            enabled=enabled and self._image_archive_defaults['enabled'])
        self.depth_archive_settings = dict(
            self._depth_archive_defaults,
            enabled=enabled and self._depth_archive_defaults['enabled'])
        # CaptureSession persists this effective config; the finalizer must not
        # mistake intentionally absent images for failed/missing image files.
        self.session.config['save_images'] = enabled
        self.session.config['image_archive'] = self.image_archive_settings.copy()
        self.session.config['depth_archive'] = self.depth_archive_settings.copy()

    def force_alignment_locked(self):
        return (self.session.active or self.finalizer is not None
                or self.image_archive_busy() or self._archive_finalize_pending
                or self.session.state in ('starting', 'recording', 'stopping', 'exporting'))

    def image_archive_busy(self):
        return any(archive is not None and not archive.done
                   for archive in (self.image_archive, self.depth_archive))

    def required_live_topics(self):
        topics = set(self.session.config['required_topics'])
        settings = self.image_archive_settings
        if settings['enabled'] and settings['required']:
            topics.add(settings['topic'])
        settings = self.depth_archive_settings
        if settings['enabled'] and settings['required']:
            topics.update((settings['topic'], settings['metadata_topic']))
        return topics

    def receive_archive_image(self, message):
        topic = self.image_archive_settings['topic']
        self.health.observe(topic, message)
        if (self.image_archive is not None and self.session.active
                and self.session.stop_started is None
                and self.session.state in ('starting', 'recording')):
            self.image_archive.enqueue(message, self.get_clock().now().nanoseconds)

    def receive_archive_depth(self, message):
        self.health.observe(self.depth_archive_settings['topic'], message)
        if (self.depth_archive is not None and self.session.active
                and self.session.stop_started is None
                and self.session.state in ('starting', 'recording')):
            self.depth_archive.enqueue(message, self.get_clock().now().nanoseconds)

    def receive_depth_metadata(self, message):
        self.health.observe(self.depth_archive_settings['metadata_topic'], message)
        self.depth_metadata_cache.append(message)
        if self.depth_archive is not None and not self.depth_archive.done:
            self.depth_archive.update_metadata(message)

    def stop_image_archive(self):
        for archive in (self.image_archive, self.depth_archive):
            if archive is not None:
                archive.request_stop()

    def poll_archive_finalization(self):
        """CSV must see the final PNG manifest, never an in-flight image queue."""
        if self.image_archive is not None:
            self.session.metadata['image_archive'] = self.image_archive.report()
        if self.depth_archive is not None:
            self.session.metadata['depth_archive'] = self.depth_archive.report()
        if not self._archive_finalize_pending:
            return
        if self.image_archive_busy():
            self.session.state = 'stopping'
            self.session.detail = 'Numeric bag closed; draining accepted color/depth crop files.'
            return
        self._archive_finalize_pending = False
        self.session.state = self._bag_closed_state
        self.session.persist()
        if self.session.state == 'stopped':
            self.start_finalizer()

    def validate_force_alignment_parameters(self, parameters):
        """Validate the final atomic candidate; never mutate settings in a callback."""
        keys = ('force_alignment_enabled', 'force_alignment_axes', 'contact_segment_id',
                'save_images')
        changes = {parameter.name: parameter.value for parameter in parameters
                   if parameter.name in keys}
        if not changes:
            return SetParametersResult(successful=True)
        if self.force_alignment_locked():
            return SetParametersResult(
                successful=False,
                reason='Experiment settings are frozen until recording, bag flush and CSV export finish.')
        candidate = {key: self.get_parameter(key).value for key in keys}
        candidate.update(changes)
        try:
            create_alignment(candidate[keys[0]], candidate[keys[1]])
            validate_contact_segment_id(candidate['contact_segment_id'])
            if type(candidate['save_images']) is not bool:
                raise ValueError('save_images must be boolean.')
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def filter_callback(self, message):
        self.last_filter_setting = dict(
            received_ns=self.get_clock().now().nanoseconds,
            message=message_to_ordereddict(message))

    def snapshot(self):
        topics = dict(self.get_topic_names_and_types())
        selected = set(self.session.topics)
        if self.image_archive_settings['enabled']:
            selected.add(self.image_archive_settings['topic'])
        if self.depth_archive_settings['enabled']:
            selected.update((self.depth_archive_settings['topic'],
                             self.depth_archive_settings['metadata_topic']))
        published = {name: types for name, types in topics.items()
                     if name in selected and self.count_publishers(name)}
        configs, references = {}, {}
        for package, relative in (
            ('estimation_pkg', 'config.json'),
            ('estimation_pkg', 'config_ROI_ref.json'),
            ('gui_py_pkg', 'config/system_components.json'),
        ):
            try:
                path = Path(get_package_share_directory(package)) / relative
                reference = reference_file(path)
                references[f'{package}/{relative}'] = reference
                configs[f'{package}/{relative}'] = json.loads(reference['content'])
            except (LookupError, OSError, ValueError) as exc:
                configs[f'{package}/{relative}'] = {'unavailable': str(exc)}
        for package, relative in (
            ('robot_control_pkg', 'config/hw_definition.hpp'),
            ('robot_control_pkg', 'config/control_parameters.hpp'),
            ('launcher', 'config/apriltag.yaml'),
            ('launcher', 'config/realsense_apriltag.yaml'),
        ):
            key = f'{package}/{relative}'
            try:
                path = Path(get_package_share_directory(package)) / relative
                references[key] = reference_file(path)
            except (LookupError, OSError, ValueError) as exc:
                references[key] = {'unavailable': str(exc)}
        try:
            revision = subprocess.run(
                ['git', 'rev-parse', 'HEAD'], capture_output=True, text=True,
                timeout=2, check=True).stdout.strip()
            dirty = subprocess.run(
                ['git', 'status', '--porcelain'], capture_output=True, text=True,
                timeout=2, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            revision, dirty = None, None
        return dict(published_topics=published, package_configs=configs,
                    reference_files=references,
                    force_alignment=self.current_force_alignment(),
                    contact_segment_id=self.get_parameter('contact_segment_id').value,
                    save_images=self.get_parameter('save_images').value,
                    image_archive_settings=self.image_archive_settings.copy(),
                    depth_archive_settings=self.depth_archive_settings.copy(),
                    runtime_parameters={'status': 'pending'},
                    hardware_constants={'status': 'pending'},
                    repository_revision=revision, repository_changes=dirty,
                    last_observed_filter_setting=self.last_filter_setting,
                    ros_start_request_ns=self.get_clock().now().nanoseconds)

    def record_callback(self, request, response):
        try:
            if request.data:
                if (self.finalizer is not None or self.image_archive_busy()
                        or self._archive_finalize_pending):
                    raise RuntimeError('Wait for image flush and CSV export/integrity check to finish.')
                if not self.session.active and any(
                        name.startswith('rosbag2_recorder') for name in self.get_node_names()):
                    raise RuntimeError('Another rosbag recorder is present; leave it untouched and stop it before starting this recorder.')
                self.apply_image_recording_settings()
                self.update_health_subscriptions()
                missing_samples = self.health.stale(
                    self.required_live_topics() - self.heavy_topics)
                if missing_samples and not self.get_parameter('allow_incomplete').value:
                    raise RuntimeError('No recent progressing data on required topics: ' + ', '.join(missing_samples))
                directory = self.session.start(
                    self.snapshot(), self.get_parameter('allow_incomplete').value)
                self.image_archive = None
                self.depth_archive = None
                if self.image_archive_settings['enabled']:
                    try:
                        from record_pkg.image_archive import ImageArchive
                        self.image_archive = ImageArchive(
                            directory, topic=self.image_archive_settings['topic'],
                            queue_size=self.image_archive_settings['queue_size'],
                            contact_segment_id=self.get_parameter('contact_segment_id').value)
                        self.session.metadata['image_archive'] = self.image_archive.report()
                    except Exception as exc:
                        self.session.metadata['image_archive'] = {
                            'status': 'failed', 'error': str(exc)}
                        self.session.request_stop()
                        raise RuntimeError(f'Could not create crop PNG archive: {exc}') from exc
                if self.depth_archive_settings['enabled']:
                    try:
                        from record_pkg.depth_archive import DepthArchive
                        settings = self.depth_archive_settings
                        self.depth_archive = DepthArchive(
                            directory, topic=settings['topic'], queue_size=settings['queue_size'],
                            contact_segment_id=self.get_parameter('contact_segment_id').value)
                        # DDS can deliver calibration just before Record and its
                        # matching image just after it. Cache metadata, not images.
                        for message in self.depth_metadata_cache:
                            self.depth_archive.update_metadata(message)
                        self.session.metadata['depth_archive'] = self.depth_archive.report()
                    except Exception as exc:
                        self.session.metadata['depth_archive'] = {'status': 'failed', 'error': str(exc)}
                        self.stop_image_archive()
                        self.session.request_stop()
                        raise RuntimeError(f'Could not create crop depth archive: {exc}') from exc
                self.record_start_monotonic = time.monotonic()
                self.last_health_warning = None
                self.session.metadata['live_health_events'] = []
                try:
                    self.parameter_snapshot = ParameterSnapshot(
                        self, self.metadata_nodes, self.metadata_timeout)
                    self.save_metadata_snapshot()
                except Exception:
                    self.stop_image_archive()
                    self.session.request_stop()
                    raise
                response.message = f'Start accepted: {directory}; wait for /data/record_status recording.'
            else:
                self.finish_metadata()
                self.stop_image_archive()
                if self.session.active and self.session.stop_started is None:
                    self.session.metadata['stop_requested_ros_ns'] = self.get_clock().now().nanoseconds
                self.session.request_stop()
                response.message = ('Stop accepted; wait for PNG flush and CSV completion.'
                                    if self.session.active or self.image_archive_busy()
                                    else 'Recorder is already inactive.')
            response.success = True
        except (OSError, ValueError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        self.publish_status()
        return response

    def update_health_subscriptions(self):
        """Observe scalar progress; PNG subscription observes its own source stamps."""
        graph = dict(self.get_topic_names_and_types())
        for topic in self.required_live_topics():
            if topic not in self.heavy_topics:
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) == 1:
                    self.health.set_publisher(topic, publishers[0].endpoint_gid)
            if (topic in self.health_subscriptions or topic not in graph
                    or (self.image_subscription is not None
                        and topic == self.image_archive_settings['topic'])
                    or (self.depth_subscription is not None
                        and topic in (self.depth_archive_settings['topic'],
                                      self.depth_archive_settings['metadata_topic']))):
                continue
            types = graph[topic]
            if len(types) != 1:
                continue
            if types[0] in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2'):
                self.heavy_topics.add(topic)
                continue
            try:
                def observe(message, name=topic):
                    self.health.observe(name, message)
                self.health_subscriptions[topic] = self.create_subscription(
                    get_message(types[0]), topic, observe,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            except (ImportError, AttributeError, ValueError) as exc:
                self.get_logger().warning(f'Cannot monitor required topic {topic}: {exc}')

    def update_live_health(self):
        required = self.required_live_topics()
        stale = self.health.stale(set(required) - self.heavy_topics)
        missing = [topic for topic in required if not self.count_publishers(topic)]
        warning = sorted(set(stale + missing))
        if self.session.state == 'recording' and warning:
            self.session.metadata['live_source_gaps_detected'] = True
        self.session.metadata['live_health'] = dict(
            status='warning' if warning else 'observed', missing_or_stale=warning,
            payload_checked_after_stop=sorted(self.heavy_topics),
            observed_publisher_restarts=dict(self.health.publisher_restarts),
            note='Source progress and publisher presence, NOT proof of bag/PNG persistence.')
        if warning != self.last_health_warning:
            events = self.session.metadata.setdefault('live_health_events', [])
            if len(events) < 1000:
                events.append(dict(ros_ns=self.get_clock().now().nanoseconds,
                                   missing_or_stale=warning))
            else:
                self.session.metadata['live_health_events_truncated'] = True
            self.last_health_warning = warning
            self.session.persist()
        return warning

    def start_finalizer(self):
        self.session.state = 'exporting'
        self.session.detail = 'Bag closed; checking required data and generating CSV. Wait before shutdown.'
        self.session.metadata['postprocess'] = {'status': 'running'}
        self.session.persist()
        try:
            self.finalizer_log = (self.session.directory / 'export.log').open('w')
            self.finalizer = subprocess.Popen(
                [sys.executable, '-m', 'record_pkg.finalize_session', str(self.session.directory)],
                stdin=subprocess.DEVNULL, stdout=self.finalizer_log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except OSError as exc:
            if self.finalizer_log:
                self.finalizer_log.close()
                self.finalizer_log = None
            self.session.state = 'failed'
            self.session.detail = f'Bag preserved; could not start CSV/integrity worker: {exc}'
            self.session.metadata['postprocess'] = {'status': 'failed', 'error': str(exc)}
            self.session.persist()

    def poll_finalizer(self):
        if self.finalizer is None or self.finalizer.poll() is None:
            return
        code = self.finalizer.returncode
        self.finalizer = None
        self.finalizer_log.close()
        self.finalizer_log = None
        try:
            report = json.loads((self.session.directory / 'postprocess.json').read_text())
        except (OSError, ValueError) as exc:
            report = dict(status='failed', error=str(exc))
        integrity = report.get('integrity') or {}
        self.session.metadata['postprocess'] = report
        self.session.metadata['data_quality'] = (
            'complete' if (integrity.get('status') == 'complete'
                           and (report.get('image_integrity') or {}).get('status', 'disabled')
                           in ('complete', 'disabled')
                           and (report.get('depth_integrity') or {}).get('status', 'disabled')
                           in ('complete', 'disabled')
                           and (report.get('summary_csv') or {}).get('status') != 'no_common_start'
                           and not self.session.metadata.get('live_source_gaps_detected')
                           and not self.session.metadata.get('automatic_stop_reason')) else 'incomplete')
        self.session.state = 'stopped' if code == 0 and report.get('status') == 'complete' else 'failed'
        self.session.detail = ('Bag closed; CSV export/integrity check complete.'
                               if self.session.state == 'stopped' else
                               'Bag preserved; CSV/integrity processing failed. See export.log/postprocess.json.')
        if (report.get('summary_csv') or {}).get('status') == 'no_common_start':
            self.session.detail = ('Bag/topic CSVs preserved; summary has no common start for '
                                   'the required training signals. See summary.schema.json.')
        if self.session.metadata['data_quality'] != 'complete':
            self.session.detail += ' INCOMPLETE required data: inspect postprocess.json.'
        self.session.persist()

    def save_metadata_snapshot(self):
        snapshot = self.session.metadata['snapshot']
        snapshot['runtime_parameters'] = self.parameter_snapshot.report
        snapshot['hardware_constants'] = self.parameter_snapshot.hardware_constants()
        self.session.persist()

    def poll_metadata(self):
        if self.parameter_snapshot is not None and not self.parameter_snapshot.done:
            self.parameter_snapshot.poll()
            if self.parameter_snapshot.done:
                self.save_metadata_snapshot()
                self.get_logger().info(
                    'Initial parameter snapshot: ' + self.parameter_snapshot.report['status']
                    + ' (see session.json for unavailable nodes).')

    def finish_metadata(self):
        if self.parameter_snapshot is not None and not self.parameter_snapshot.done:
            self.parameter_snapshot.cancel()
            self.save_metadata_snapshot()

    def poll(self):
        try:
            self.update_health_subscriptions()
            self.poll_finalizer()
            # Discovery readiness is NOT a guarantee of received sensor data.
            required = self.session.config['required_topics']
            ready = bool(required) and all(
                any(info.node_name.startswith('rosbag2_recorder')
                    for info in self.get_subscriptions_info_by_topic(topic))
                for topic in required)
            if (not required or self.get_parameter('allow_incomplete').value) and self.session.directory:
                ready = (self.session.directory / 'bag').is_dir()
            metadata_ready = self.parameter_snapshot is None or self.parameter_snapshot.done
            was_active = self.session.active
            was_starting = self.session.state == 'starting'
            warning = self.update_live_health() if self.session.active else []
            samples_ready = not warning or self.get_parameter('allow_incomplete').value
            if was_starting and not (ready and metadata_ready and samples_ready):
                if time.monotonic() - self.record_start_monotonic > self.session.config.get('startup_timeout_sec', 10.0):
                    self.session.metadata['automatic_stop_reason'] = 'required_data_startup_timeout'
                    self.session.metadata['stop_requested_ros_ns'] = self.get_clock().now().nanoseconds
                    self.session.request_stop()
            self.session.poll(ready=ready and metadata_ready and samples_ready)
            if was_starting and self.session.state == 'recording':
                self.session.metadata['recording_ready_ros_ns'] = self.get_clock().now().nanoseconds
                self.session.persist()
            if self.session.stop_started is not None:
                self.stop_image_archive()
            if was_active and not self.session.active:
                self.stop_image_archive()
                self._bag_closed_state = self.session.state
                self._archive_finalize_pending = True
            self.poll_archive_finalization()
            if not self.session.active:
                self.finish_metadata()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Recording supervision error: {exc}')
            self.stop_image_archive()
            self.session.request_stop()
        self.publish_status()

    def publish_status(self):
        detail = self.session.detail
        warning = self.session.metadata.get('live_health', {}).get('missing_or_stale', [])
        if self.session.state in ('starting', 'recording') and warning:
            detail += ' DATA WARNING (recording continues): ' + ', '.join(warning)
        archive = self.session.metadata.get('image_archive', {})
        depth_archive = self.session.metadata.get('depth_archive', {})
        if (archive.get('dropped', 0) or archive.get('errors', 0)
                or archive.get('finalization_errors', 0)):
            detail += ' IMAGE WARNING: crop frames dropped/failed; inspect image manifest.'
        if (depth_archive.get('dropped', 0) or depth_archive.get('errors', 0)
                or depth_archive.get('finalization_errors', 0)
                or depth_archive.get('calibration_errors', 0)):
            detail += ' DEPTH WARNING: crop depth frames dropped/failed; inspect depth manifest.'
        if (self.depth_archive_settings['enabled']
                and self.session.state in ('starting', 'recording')
                and not depth_archive.get('saved', 0)):
            detail += ' DEPTH: no crop frames saved yet; check/restart the updated estimator.'
        elif (self.depth_archive_settings['enabled']
                and self.session.state == 'recording'
                and self.health.stale([self.depth_archive_settings['topic']])):
            detail += ' DEPTH WARNING: crop depth source is stale; other data continue recording.'
        alignment_locked = self.force_alignment_locked()
        force_alignment = (
            self.session.metadata.get('snapshot', {}).get('force_alignment')
            if alignment_locked else None)
        if force_alignment is None:
            force_alignment = self.current_force_alignment()
        payload = json.dumps(dict(
            state=self.session.state, detail=detail,
            data_quality=self.session.metadata.get('data_quality'),
            live_health=self.session.metadata.get('live_health'),
            image_archive=archive,
            depth_archive=depth_archive,
            force_alignment=force_alignment,
            save_images=(self.session.metadata.get('snapshot', {}).get('save_images')
                         if alignment_locked else self.get_parameter('save_images').value),
            contact_segment_id=(
                self.session.metadata.get('snapshot', {}).get('contact_segment_id')
                if alignment_locked else self.get_parameter('contact_segment_id').value),
            force_alignment_scope='session' if alignment_locked else 'next_session',
            directory=str(self.session.directory or '')))
        self.status_pub.publish(String(data=payload))
        if payload != self.last_status:
            self.get_logger().info(payload)
            self.last_status = payload

    def close(self):
        self.finish_metadata()
        self.stop_image_archive()
        if self.session.active and self.session.stop_started is None:
            self.session.metadata['stop_requested_ros_ns'] = self.get_clock().now().nanoseconds
        self.session.close()
        if self.image_archive is not None:
            self.image_archive.close(timeout=8.0)
            self.session.metadata['image_archive'] = self.image_archive.report()
            self.session.persist()
        if self.depth_archive is not None:
            self.depth_archive.close(timeout=8.0)
            self.session.metadata['depth_archive'] = self.depth_archive.report()
            self.session.persist()
        self.poll_finalizer()
        if self.finalizer is not None:
            if self.finalizer.poll() is None:
                try:
                    os.killpg(self.finalizer.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.finalizer.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.finalizer.pid, signal.SIGKILL)
                    self.finalizer.wait(timeout=2)
            self.finalizer = None
            self.finalizer_log.close()
            self.finalizer_log = None
            self.session.metadata['postprocess'] = {'status': 'interrupted'}
        if self.session.directory and self.session.metadata.get('postprocess', {}).get('status') not in ('complete', 'partial', 'failed'):
            self.session.metadata['postprocess'] = {'status': 'pending_or_interrupted'}
            self.session.metadata['data_quality'] = 'unverified'
            self.session.detail = 'Bag closed; CSV/integrity processing pending. Use export_csv before training.'
            if self.session.state != 'failed':
                self.session.state = 'stopped' if (self.session.directory / 'bag/metadata.yaml').is_file() else 'failed'
            self.session.persist()


def main(args=None):
    # Match the image-sized profile used by this workspace's camera/estimator.
    # Explicit operator profiles and other RMW implementations take priority.
    if (not any(os.environ.get(k) for k in (
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'))
            and os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') in (
                'rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp')):
        try:
            profile = Path(get_package_share_directory('estimation_pkg')) / 'config/fastdds_images.xml'
            if profile.is_file():
                os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        except LookupError:
            pass
    rclpy.init(args=args)
    node = None
    try:
        node = RecordNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
