"""Optional learned force/contact inference. Never publishes actuator commands."""

import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.msg import ForceIdEstimate, LoadcellState, MotorState, SegmentAngle
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from estimation_pkg.force_id_inputs import FeatureHistory, contact_policy, pose_fields
from estimation_pkg.id_localization import (
    FORCE_RANGES_STATUS, default_config_path, load_localization_config,
)


TOPIC_TYPES = {
    '/wire_length': Float64MultiArray,
    '/wire_length_velocity': Float64MultiArray,
    '/loadcell_state': LoadcellState,
    '/motor_state': MotorState,
    '/estimated_segment_angle': SegmentAngle,
    '/estimated_tip_position': PointStamped,
    '/kinematics/fk_tip_position': PointStamped,
    '/tf': TFMessage,
}


class ForceIdEstimationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('force_id_estimator', **kwargs)
        defaults = dict(model_path='', body_contact_enabled=False, id_tol=2,
                        id_localization_config=str(default_config_path()),
                        device='cpu', sample_hz=30.0, input_timeout_sec=0.25,
                        max_input_skew_sec=0.1, max_window_gap_sec=0.15)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.localization_config = load_localization_config(values['id_localization_config'])
        self.get_logger().info(
            f'ID load policy: {FORCE_RANGES_STATUS if self.localization_config.enabled else "model_native"}; '
            f'config={values["id_localization_config"]}; '
            f'predicted hrm_base XYZ bounds [mN]={self.localization_config.ranges_mN}. '
            'All axes inside inclusive bounds => operating ID18; forces/raw ID preserved. '
            'Restart inference to reload settings.')
        if not math.isfinite(values['sample_hz']) or not 1 <= values['sample_hz'] <= 200:
            raise ValueError('sample_hz must be finite and in 1..200 Hz')
        self._validate_mode(values['body_contact_enabled'], values['id_tol'])
        self.body_contact_enabled = values['body_contact_enabled']
        self.id_tol = values['id_tol']
        self.runtime = None
        self.history = None
        self._input_subscriptions = {}
        self._error = 'no_model_selected'
        self._invalid_key = None
        self._last_invalid_at = 0.0
        self._last_ros_ns = None
        self._publisher = self.create_publisher(
            ForceIdEstimate, '/force_id_estimation',
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE))
        if values['model_path']:
            try:
                self._activate(*self._prepare(values['model_path']))
            except Exception as exc:
                self._error = 'model_load_failed:' + str(exc)
                self.get_logger().error(self._error)
        self.add_on_set_parameters_callback(self._on_parameters)
        self._timer = self.create_timer(1.0 / values['sample_hz'], self._tick)
        self.get_logger().info(
            'Force/contact diagnostics only: /force_id_estimation; '
            'no legacy force publication, motor command, or admittance activation.')

    @staticmethod
    def _validate_mode(body_enabled, id_tol):
        if type(body_enabled) is not bool:
            raise ValueError('body_contact_enabled must be bool')
        if type(id_tol) is not int or not 0 <= id_tol <= 18:
            raise ValueError('id_tol must be an integer in 0..18')

    def _prepare(self, model_path):
        if not isinstance(model_path, str) or not model_path.strip():
            return None, None
        # Torch is intentionally imported only in this optional node, never GUI.
        try:
            from estimation_pkg.force_id_runtime import BundleRuntime
        except ImportError as exc:
            raise RuntimeError(
                'PyTorch runtime unavailable. Use force_id_estimation.launch.py with '
                'python_executable:=/home/daeyun/HRM_env/bin/python3, or a compatible '
                'ROS Python environment with torch installed.') from exc
        path = Path(model_path).expanduser()
        if path.is_dir():
            path = path / 'hrm_bundle.pt'
        runtime = BundleRuntime(path, device=self.get_parameter('device').value)
        if runtime.force_unit != 'N' or runtime.force_frame != 'hrm_base':
            raise ValueError('This topic requires bundle output in N / hrm_base')
        history = FeatureHistory(
            runtime.feature_columns, runtime.window_samples,
            timeout_sec=self.get_parameter('input_timeout_sec').value,
            max_skew_sec=self.get_parameter('max_input_skew_sec').value,
            max_gap_sec=self.get_parameter('max_window_gap_sec').value)
        return runtime, history

    def _activate(self, runtime, history):
        for subscription in self._input_subscriptions.values():
            self.destroy_subscription(subscription)
        self._input_subscriptions.clear()
        self.runtime, self.history = runtime, history
        self._error = 'no_model_selected' if runtime is None else 'warming_up'
        self._invalid_key = None
        if runtime is None:
            return
        for topic in sorted(history.required_topics):
            # Multiple broadcasters share /tf; depth=1 can drop the wanted child.
            qos = QoSProfile(depth=100 if topic == '/tf' else 1,
                             history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self._input_subscriptions[topic] = self.create_subscription(
                TOPIC_TYPES[topic], topic, lambda msg, topic=topic: self._receive(topic, msg), qos)
        self.get_logger().info(
            f'Loaded {runtime.model_id}: {runtime.mode}, window={runtime.window_samples}, '
            f'inputs={len(runtime.feature_columns)}, topics={sorted(history.required_topics)}. '
            'Published forces retain training sign; control-side negation is deferred.')

    def _on_parameters(self, parameters):
        updates = {p.name: p.value for p in parameters}
        allowed = {'model_path', 'body_contact_enabled', 'id_tol'}
        unknown = set(updates) - allowed
        if unknown:
            return SetParametersResult(successful=False, reason='Restart to change: ' + ', '.join(sorted(unknown)))
        try:
            body = updates.get('body_contact_enabled', self.body_contact_enabled)
            tolerance = updates.get('id_tol', self.id_tol)
            self._validate_mode(body, tolerance)
            replacement = None
            if 'model_path' in updates:
                if not isinstance(updates['model_path'], str):
                    raise ValueError('model_path must be a string')
                replacement = self._prepare(updates['model_path'])
            if replacement is not None:
                self._activate(*replacement)
            self.body_contact_enabled, self.id_tol = body, tolerance
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _receive(self, topic, message):
        if self.history is None:
            return
        now = self.get_clock().now().nanoseconds
        if topic == '/tf':
            for transform in message.transforms:
                child = transform.child_frame_id
                if child not in self.history.tf_children:
                    continue  # Never let camera/joint/tag traffic refresh tip data.
                try:
                    if transform.header.frame_id != 'hrm_base':
                        raise ValueError('Tip TF must be expressed directly in hrm_base')
                    fields = pose_fields(transform.transform.translation,
                                         transform.transform.rotation)
                except (AttributeError, TypeError, ValueError):
                    fields = {}  # Malformed matching frame invalidates its input.
                self._put_input(topic, fields, transform.header, now, child_frame=child)
            return
        if topic in ('/estimated_tip_position', '/kinematics/fk_tip_position'):
            try:
                if message.header.frame_id != 'hrm_base':
                    raise ValueError('Tip position must be expressed in hrm_base')
                fields = pose_fields(message.point)
            except (AttributeError, TypeError, ValueError):
                fields = {}
        else:
            fields = {f.field: getattr(message, f.field, ())
                      for f in self.history.features if f.topic == topic}
        self._put_input(topic, fields, getattr(message, 'header', None), now)

    def _put_input(self, topic, fields, header, now, child_frame=''):
        """Keep original source time, including each transform inside a TFMessage."""
        if header is not None:
            stamp = header.stamp
            source_ns = stamp.sec * 10**9 + stamp.nanosec
        else:
            # The original wire topics are headerless; this is reception time.
            source_ns = now
        self.history.put(topic, fields, source_ns=source_ns, received_ns=now,
                         received_monotonic_ns=time.monotonic_ns(), child_frame=child_frame)

    def _message(self):
        message = ForceIdEstimate()
        message.header.frame_id = 'hrm_base'
        message.published_stamp = self.get_clock().now().to_msg()
        if self.history and self.history.last_source_ns:
            message.header.stamp.sec = self.history.last_source_ns // 10**9
            message.header.stamp.nanosec = self.history.last_source_ns % 10**9
        message.body_contact_enabled = self.body_contact_enabled
        message.id_tol = self.id_tol
        message.contact_id = message.control_contact_id = message.heatmap_center_id = -1
        message.external_force.x = message.external_force.y = message.external_force.z = float('nan')
        if self.runtime is not None:
            message.model_name = self.runtime.model_id
            message.mode = self.runtime.mode
            message.force_scope = self.runtime.force_scope
            message.window_samples = self.runtime.window_samples
            message.samples_in_window = len(self.history.window)
        return message

    def _invalid(self, reason):
        message = self._message()
        message.status = reason
        key = (reason, message.model_name, message.samples_in_window,
               self.body_contact_enabled, self.id_tol)
        now = time.monotonic()
        if key == self._invalid_key and now - self._last_invalid_at < 0.5:
            return
        self._invalid_key, self._last_invalid_at = key, now
        self._publisher.publish(message)

    def _tick(self):
        if self.runtime is None:
            self._invalid(self._error)
            return
        now = self.get_clock().now().nanoseconds
        if self._last_ros_ns is not None and now < self._last_ros_ns:
            self.history.samples.clear()
            self.history.reset_window()
        self._last_ros_ns = now
        window, status = self.history.sample(now, time.monotonic_ns())
        if window is None:
            if status != 'waiting_for_new_input':
                self._invalid(status)
            return
        try:
            started = time.perf_counter()
            prediction = self.runtime.predict(window)
            policy = contact_policy(prediction, self.runtime.mode, self.runtime.active_ids,
                                    self.runtime.force_scope, self.body_contact_enabled, self.id_tol,
                                    force_ranges_config=self.localization_config)
            elapsed = (time.perf_counter() - started) * 1000.0
            message = self._message()
            message.valid = True
            message.status = (FORCE_RANGES_STATUS if self.localization_config.enabled else
                              'ready' if policy['load_state'] else 'ready_load_unknown')
            message.external_force.x, message.external_force.y, message.external_force.z = (
                float(v) for v in prediction.force_N)
            if not all(math.isfinite(v) for v in prediction.force_N):
                raise ValueError('Model returned non-finite force')
            message.probabilities = [float(v) for v in prediction.probabilities]
            for key, value in policy.items():
                setattr(message, key, value)
            message.inference_ms = elapsed
            self._publisher.publish(message)
            self._invalid_key = None
        except Exception as exc:
            self.history.reset_window()
            self._invalid('inference_failed:' + str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ForceIdEstimationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward SIGINT after the terminal has already delivered it.
        # Repeated shutdown requests must not turn an ordinary Stop into a crash.
        try:
            if node is not None:
                node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                rclpy.try_shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
