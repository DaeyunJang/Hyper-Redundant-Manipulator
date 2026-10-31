"""Opt-in synthetic ROS integration test; never starts hardware/control nodes.

Source the ROS and workspace setup files, then run with a PyTorch-enabled Python:
  HRM_RUN_FORCE_ID_INTEGRATION=1 python -m pytest -q \
    src/estimation_pkg/test/test_force_id_integration.py

Real-bundle contexts use DDS domain 211 (override HRM_FORCE_TEST_DOMAIN).
Controlled config-policy tests need no Torch and use domain 212 (override
HRM_FORCE_CONFIG_TEST_DOMAIN). All contexts are localhost-only and nonzero.
Only synthetic loadcell/wire/tip/TF topics are published. No camera, motor command,
legacy estimated_external_force publisher or F/T ground truth is used.
"""

import json
import math
import os
from pathlib import Path
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('HRM_RUN_FORCE_ID_INTEGRATION') != '1',
    reason='Opt-in private-domain ROS/PyTorch integration test.',
)


@pytest.mark.parametrize('group', [
    'cable_loadcell', 'cable_loadcell_fk_tip_tf', 'cable_loadcell_estimated_pose',
])
def test_real_bundles_synthetic_ros_warmup_stale_and_selection(tmp_path, monkeypatch, group):
    """Test the public ROS contract with class0 and masked model selections."""
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    from std_msgs.msg import Float64MultiArray
    from custom_interfaces.msg import ForceIdEstimate, LoadcellState
    from geometry_msgs.msg import PointStamped, TransformStamped
    from tf2_msgs.msg import TFMessage
    from estimation_pkg.force_id_estimation import ForceIdEstimationNode

    model_root = Path(os.environ.get(
        'HRM_FORCE_TEST_MODELS',
        str(Path(__file__).resolve().parents[1] / 'models' / group / 'models'),
    ))
    class0_path = model_root / 'class0__force_mlp__location_mlp/hrm_bundle.pt'
    masked_path = model_root / 'masked__force_mlp__location_mlp/hrm_bundle.pt'
    if not class0_path.is_file() or not masked_path.is_file():
        pytest.skip('Real class0/masked MLP bundles are not installed in this checkout.')

    domain = int(os.environ.get('HRM_FORCE_TEST_DOMAIN', '211'))
    if not 1 <= domain <= 232:
        pytest.fail('HRM_FORCE_TEST_DOMAIN must be an isolated nonzero DDS domain (1..232).')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    context.init(args=[], domain_id=domain)
    estimator = None
    producer = None
    executor = SingleThreadedExecutor(context=context)
    messages = []
    try:
        estimator = ForceIdEstimationNode(
            context=context,
            parameter_overrides=[
                Parameter('model_path', value=str(class0_path)),
                Parameter('device', value='cpu'),
                Parameter('sample_hz', value=30.0),
                Parameter('input_timeout_sec', value=0.25),
                Parameter('id_localization_config', value=str(
                    Path(__file__).resolve().parents[1] / 'estimation_pkg' /
                    'id_localization_config.json')),
            ],
        )
        producer = Node('force_id_synthetic_input_test', context=context)
        wire_pub = producer.create_publisher(Float64MultiArray, '/wire_length', 10)
        load_pub = producer.create_publisher(LoadcellState, '/loadcell_state', 10)
        point_pub = producer.create_publisher(PointStamped, '/estimated_tip_position', 10)
        tf_pub = producer.create_publisher(TFMessage, '/tf', 100)
        producer.create_subscription(
            ForceIdEstimate, '/force_id_estimation', messages.append, 100)
        client = producer.create_client(
            SetParametersAtomically, '/force_id_estimator/set_parameters_atomically')
        executor.add_node(estimator)
        executor.add_node(producer)

        def sample():
            # Constant real measurements are intentional: a stationary motor is
            # still a fresh sample; no input is replaced by an F/T target.
            wire = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
            load = LoadcellState()
            load.header.stamp = producer.get_clock().now().to_msg()
            load.stress = [200.0, 200.0, 200.0, 200.0]
            wire_pub.publish(wire)
            load_pub.publish(load)
            if group == 'cable_loadcell_estimated_pose':
                point = PointStamped()
                point.header.stamp = load.header.stamp
                point.header.frame_id = 'hrm_base'
                point.point.x, point.point.y, point.point.z = 0.08, -0.02, 0.003
                point_pub.publish(point)
            if group != 'cable_loadcell':
                transform = TransformStamped()
                transform.header.stamp = load.header.stamp
                transform.header.frame_id = 'hrm_base'
                transform.child_frame_id = ('hrm_fk_tip' if group.endswith('fk_tip_tf')
                                            else 'estimated_segment_center_19')
                transform.transform.translation.x = 0.075
                transform.transform.translation.y = -0.01
                transform.transform.translation.z = 0.002
                transform.transform.rotation.z = -0.6
                transform.transform.rotation.w = -0.8
                unrelated = TransformStamped()
                unrelated.header.stamp = load.header.stamp
                unrelated.child_frame_id = 'camera_link'
                tf_pub.publish(TFMessage(transforms=[unrelated, transform]))

        def pump_until(predicate, *, timeout=5.0, publish=True):
            deadline = time.monotonic() + timeout
            next_sample = time.monotonic()
            while time.monotonic() < deadline:
                if predicate():
                    return
                if publish and time.monotonic() >= next_sample:
                    sample()
                    next_sample = time.monotonic() + 0.01
                executor.spin_once(timeout_sec=0.01)
            assert predicate(), 'Timed out waiting for synthetic ROS state.'

        def set_parameters(**values):
            pump_until(client.service_is_ready)
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                  for name, value in values.items()]
            future = client.call_async(request)
            pump_until(future.done)
            assert future.exception() is None
            return future.result().result

        def next_message(predicate, *, publish=True, timeout=5.0):
            start = len(messages)
            pump_until(lambda: any(predicate(msg) for msg in messages[start:]),
                       publish=publish, timeout=timeout)
            return next(msg for msg in messages[start:] if predicate(msg))

        warmup = next_message(lambda msg: not msg.valid and 0 < msg.samples_in_window < 30)
        assert warmup.window_samples == 30
        assert warmup.contact_id == -1
        assert math.isnan(warmup.external_force.x)

        ready = next_message(lambda msg: msg.valid and msg.mode == 'class0')
        assert ready.samples_in_window == ready.window_samples == 30
        assert ready.header.frame_id == 'hrm_base'
        assert ready.force_scope == estimator.runtime.force_scope
        assert all(math.isfinite(v) for v in (
            ready.external_force.x, ready.external_force.y, ready.external_force.z))
        assert len(ready.probabilities) == 19
        assert all(0.0 <= p <= 1.0 for p in ready.probabilities)
        assert sum(ready.probabilities) == pytest.approx(1.0, abs=1e-6)
        assert ready.contact_id == int(np.argmax(ready.probabilities))
        assert 1 <= ready.heatmap_center_id <= 18

        def assert_force_ranges_policy(message):
            assert message.status == 'ready_force_ranges'
            unloaded = estimator.localization_config.is_unloaded((
                message.external_force.x, message.external_force.y, message.external_force.z))
            assert message.load_state == (ForceIdEstimate.UNLOADED if unloaded
                                          else ForceIdEstimate.LOADED)
            assert message.control_contact_id == (18 if unloaded else message.heatmap_center_id)
            assert message.tip_force_valid == (
                message.force_scope in ('tip', 'all_single_contact') and
                message.control_contact_id == 18)
            assert message.body_contact_detected == (
                not unloaded and message.control_contact_id < 18)

        assert_force_ranges_policy(ready)
        expected_row = [0.] * 4 + [200.] * 4
        if group == 'cable_loadcell_fk_tip_tf':
            expected_row += [0.075, -0.01, 0.002, 0., 0., -0.6, -0.8]
        elif group == 'cable_loadcell_estimated_pose':
            expected_row += [0.08, -0.02, 0.003, 0., 0., -0.6, -0.8]
        np.testing.assert_allclose(estimator.history.window[-1], expected_row, atol=1e-12)

        # Body mode only changes the policy/display flag, never the classifier.
        result = set_parameters(body_contact_enabled=True, id_tol=3)
        assert result.successful, result.reason
        body = next_message(lambda msg: msg.valid and msg.body_contact_enabled and msg.id_tol == 3)
        assert body.samples_in_window == 30
        assert body.contact_id == ready.contact_id
        np.testing.assert_allclose(body.probabilities, ready.probabilities, atol=1e-7)
        band_ids = range(max(1, body.heatmap_center_id - 3),
                         min(18, body.heatmap_center_id + 3) + 1)
        assert body.band_probability == pytest.approx(
            sum(body.probabilities[i] for i in band_ids), abs=1e-7)

        # Invalid selection is rejected without silently removing the good model.
        rejected = set_parameters(model_path=str(tmp_path / 'missing/hrm_bundle.pt'))
        assert not rejected.successful
        retained = next_message(lambda msg: msg.valid)
        assert retained.mode == 'class0'

        # Stop input, rather than stopping a motor. No stale force stays valid.
        stale = next_message(lambda msg: not msg.valid, publish=False, timeout=2.0)
        assert not stale.tip_force_valid
        assert stale.contact_id == -1
        assert math.isnan(stale.external_force.x)
        recovered = next_message(lambda msg: not msg.valid and 0 < msg.samples_in_window < 30)
        assert recovered.samples_in_window < recovered.window_samples
        next_message(lambda msg: msg.valid)

        result = set_parameters(model_path=str(masked_path))
        assert result.successful, result.reason
        switch_warmup = next_message(
            lambda msg: msg.mode == 'masked' and not msg.valid and 0 < msg.samples_in_window < 30)
        assert switch_warmup.samples_in_window < 30
        masked = next_message(lambda msg: msg.valid and msg.mode == 'masked')
        assert masked.probabilities[0] == 0.0
        assert sum(masked.probabilities[1:]) == pytest.approx(1.0, abs=1e-6)
        assert_force_ranges_policy(masked)
        assert 1 <= masked.contact_id <= 18
        assert masked.body_contact_enabled

        publications = producer.get_publisher_names_and_types_by_node('force_id_estimator', '/')
        topics = {name for name, _types in publications}
        assert '/force_id_estimation' in topics
        assert '/estimated_external_force' not in topics
        assert not any('motor' in topic or 'target_wire' in topic for topic in topics)
        subscriptions = producer.get_subscriber_names_and_types_by_node('force_id_estimator', '/')
        subscribed = {name for name, _types in subscriptions}
        assert '/fts_data' not in subscribed
        assert '/fts_data_kalman_filter' not in subscribed
    finally:
        executor.shutdown()
        if estimator is not None:
            estimator.destroy_node()
        if producer is not None:
            producer.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize('mode', ['masked', 'class0'])
@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('force_scope', ['tip', 'all_single_contact'])
def test_configured_force_ranges_private_ros(tmp_path, monkeypatch, mode, enabled, force_scope):
    """Exercise the real ROS node/history with controlled model predictions.

    No Torch/model checkpoint or actuator is involved. A class-0 raw winner is
    intentional: outside the force box, operational ID must be the *positive*
    location winner while the original raw ID/probabilities remain untouched.
    """
    import numpy as np
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    from std_msgs.msg import Float64MultiArray
    from custom_interfaces.msg import ForceIdEstimate, LoadcellState
    from estimation_pkg.force_id_estimation import ForceIdEstimationNode
    from estimation_pkg.force_id_inputs import FeatureHistory
    from estimation_pkg.force_id_runtime import Prediction

    config_path = tmp_path / 'id_localization_config.json'
    config_path.write_text(json.dumps({
        'schema_version': 1, 'enabled': enabled,
        'unloaded_force_ranges_mN': {
            'fx': [-100.0, 100.0], 'fy': [-200.0, 200.0], 'fz': [-300.0, 300.0],
        },
    }), encoding='utf-8')
    columns = ([f'wire.Cable #{i} length' for i in range(1, 5)] +
               [f'loadcell.Loadcell #{i} tension' for i in range(1, 5)])
    probabilities = np.zeros(19)
    probabilities[[0, 7, 18]] = [0.7, 0.2, 0.1] if mode == 'class0' else [0., 0.8, 0.2]

    class FakeRuntime:
        model_id = 'synthetic_force_range_contract'
        active_ids = tuple(range(1, 19))
        force_scope = 'tip'
        force_unit = 'N'
        force_frame = 'hrm_base'
        feature_columns = columns
        window_samples = 3
        force = np.zeros(3)

        def predict(self, window):
            assert window.shape == (3, 8)
            assert np.isfinite(window).all()
            np.testing.assert_allclose(window[-1], [0.] * 4 + [200.] * 4)
            return Prediction(self.force.copy(), probabilities.copy(),
                              int(np.argmax(probabilities)), 7, mode == 'class0')

    runtime = FakeRuntime()
    runtime.mode = mode
    runtime.force_scope = force_scope

    def prepare(node, _path):
        return runtime, FeatureHistory(
            columns, 3, timeout_sec=node.get_parameter('input_timeout_sec').value)

    monkeypatch.setattr(ForceIdEstimationNode, '_prepare', prepare)
    domain = int(os.environ.get('HRM_FORCE_CONFIG_TEST_DOMAIN', '212'))
    if not 1 <= domain <= 232:
        pytest.fail('HRM_FORCE_CONFIG_TEST_DOMAIN must be a private nonzero domain (1..232).')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    context.init(args=[], domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    estimator = producer = None
    messages = []
    invalid_input = False
    try:
        estimator = ForceIdEstimationNode(context=context, parameter_overrides=[
            Parameter('model_path', value='synthetic_no_checkpoint'),
            Parameter('id_localization_config', value=str(config_path)),
            Parameter('sample_hz', value=30.0),
            Parameter('input_timeout_sec', value=0.15),
        ])
        producer = Node('force_id_config_synthetic_input_test', context=context)
        wire_pub = producer.create_publisher(Float64MultiArray, '/wire_length', 10)
        load_pub = producer.create_publisher(LoadcellState, '/loadcell_state', 10)
        producer.create_subscription(ForceIdEstimate, '/force_id_estimation', messages.append, 100)
        client = producer.create_client(
            SetParametersAtomically, '/force_id_estimator/set_parameters_atomically')
        executor.add_node(estimator)
        executor.add_node(producer)

        def sample():
            wire_pub.publish(Float64MultiArray(data=[0.] * 4))
            load = LoadcellState()
            load.header.stamp = producer.get_clock().now().to_msg()
            load.stress = [float('nan') if invalid_input else 200.] * 4
            load_pub.publish(load)

        def pump_until(predicate, *, publish=True, timeout=3.0):
            deadline = time.monotonic() + timeout
            next_sample = time.monotonic()
            while time.monotonic() < deadline:
                if predicate():
                    return
                if publish and time.monotonic() >= next_sample:
                    sample()
                    next_sample = time.monotonic() + 0.01
                executor.spin_once(timeout_sec=0.01)
            assert predicate(), 'Timed out waiting for private-domain force/config state.'

        def next_message(predicate, *, publish=True):
            start = len(messages)
            pump_until(lambda: any(predicate(message) for message in messages[start:]),
                       publish=publish)
            return next(message for message in messages[start:] if predicate(message))

        def set_parameters(**values):
            pump_until(client.service_is_ready)
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                  for name, value in values.items()]
            future = client.call_async(request)
            pump_until(future.done)
            assert future.exception() is None
            return future.result().result

        warmup = next_message(lambda msg: not msg.valid and 0 < msg.samples_in_window < 3)
        assert warmup.window_samples == 3
        assert warmup.contact_id == warmup.control_contact_id == -1
        assert math.isnan(warmup.external_force.x)

        def assert_prediction(force):
            runtime.force = np.asarray(force, dtype=float)
            ready = next_message(lambda msg: msg.valid and np.array_equal(
                [msg.external_force.x, msg.external_force.y, msg.external_force.z], force))
            assert ready.samples_in_window == ready.window_samples == 3
            assert ready.header.frame_id == 'hrm_base'
            assert ready.force_scope == force_scope
            assert ready.header.stamp.sec > 0
            assert ready.contact_id == (0 if mode == 'class0' else 7)
            assert ready.heatmap_center_id == 7
            np.testing.assert_array_equal(ready.probabilities, probabilities)
            np.testing.assert_array_equal(
                [ready.external_force.x, ready.external_force.y, ready.external_force.z], force)
            if enabled:
                unloaded = all(-bound <= value <= bound
                               for value, bound in zip(force, (100.*.001, 200.*.001, 300.*.001)))
                assert ready.status == 'ready_force_ranges'
                assert ready.load_state == (ForceIdEstimate.UNLOADED if unloaded
                                            else ForceIdEstimate.LOADED)
                assert ready.control_contact_id == (18 if unloaded else 7)
                assert ready.tip_force_valid == unloaded
                assert ready.body_contact_detected == (not unloaded)
            elif mode == 'class0':
                assert ready.status == 'ready'
                assert ready.load_state == ForceIdEstimate.UNLOADED
                assert ready.control_contact_id == 18
                assert ready.tip_force_valid
                assert not ready.body_contact_detected
            else:
                assert ready.status == 'ready_load_unknown'
                assert ready.load_state == ForceIdEstimate.LOAD_UNKNOWN
                assert ready.control_contact_id == -1
                assert not ready.tip_force_valid
                assert not ready.body_contact_detected
            return ready

        # Both signs, exact inclusive boundaries, and *any* single outside axis.
        for force in ((0., 0., 0.), (100.*.001, 200.*.001, 300.*.001),
                      (-100.*.001, -200.*.001, -300.*.001),
                      (.100001, 0., 0.), (-.100001, 0., 0.),
                      (0., .200001, 0.), (0., 0., -.300001)):
            assert_prediction(force)

        result = set_parameters(body_contact_enabled=True, id_tol=3)
        assert result.successful, result.reason
        body = assert_prediction((.4, -.5, .6))
        assert body.body_contact_enabled and body.id_tol == 3
        assert body.band_probability == pytest.approx(probabilities[4:11].sum())

        original_config = estimator.localization_config
        rejected = set_parameters(id_localization_config=str(tmp_path / 'different.json'))
        assert not rejected.successful and 'Restart' in rejected.reason
        assert estimator.localization_config is original_config
        assert estimator.get_parameter('id_localization_config').value == str(config_path)

        # A missing source or NaN may never become a valid UNLOADED/ID18 sample.
        stale = next_message(lambda msg: not msg.valid and 'stale' in msg.status, publish=False)
        assert not stale.tip_force_valid
        assert stale.contact_id == stale.control_contact_id == -1
        assert math.isnan(stale.external_force.x)
        next_message(lambda msg: not msg.valid and 0 < msg.samples_in_window < 3)
        assert_prediction((0., 0., 0.))

        invalid_input = True
        invalid = next_message(lambda msg: not msg.valid and 'invalid_input' in msg.status)
        assert invalid.control_contact_id == -1 and not invalid.tip_force_valid
        invalid_input = False
        assert_prediction((0., 0., 0.))

        runtime.force = np.array([float('nan'), 0., 0.])
        failed = next_message(lambda msg: not msg.valid and msg.status.startswith('inference_failed:'))
        assert failed.control_contact_id == -1 and not failed.tip_force_valid
        assert math.isnan(failed.external_force.x)
        assert_prediction((0., 0., 0.))

        topics = {name for name, _types in producer.get_publisher_names_and_types_by_node(
            'force_id_estimator', '/')}
        assert '/force_id_estimation' in topics
        assert '/estimated_external_force' not in topics
        assert not any('motor' in name or 'target_wire' in name for name in topics)
        subscribed = {name for name, _types in producer.get_subscriber_names_and_types_by_node(
            'force_id_estimator', '/')}
        assert subscribed - {'/parameter_events'} == {'/wire_length', '/loadcell_state'}
    finally:
        executor.shutdown()
        if estimator is not None:
            estimator.destroy_node()
        if producer is not None:
            producer.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize('contents, error', [(None, FileNotFoundError), ('{}', ValueError)])
def test_force_config_startup_errors_are_explicit(tmp_path, monkeypatch, contents, error):
    """Bad startup settings must not silently fall back to a different ID policy."""
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from estimation_pkg.force_id_estimation import ForceIdEstimationNode

    config_path = tmp_path / 'bad_or_missing_config.json'
    if contents is not None:
        config_path.write_text(contents, encoding='utf-8')
    domain = int(os.environ.get('HRM_FORCE_CONFIG_TEST_DOMAIN', '212'))
    assert 1 <= domain <= 232, 'Use only a private nonzero DDS domain.'
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    context.init(args=[], domain_id=domain)
    # Keep a handle to the partially constructed Node so its ROS resources can
    # still be destroyed after configuration validation raises in __init__.
    estimator = ForceIdEstimationNode.__new__(ForceIdEstimationNode)
    try:
        with pytest.raises(error):
            estimator.__init__(context=context, parameter_overrides=[
                Parameter('id_localization_config', value=str(config_path)),
            ])
    finally:
        estimator.destroy_node()
        context.try_shutdown()
