"""Validate launch selection without starting ROS nodes or touching cameras."""

import importlib.util
from pathlib import Path
import re

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
from launch import LaunchContext, LaunchDescription  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
)
from launch.utilities import (  # noqa: E402
    normalize_to_list_of_substitutions, perform_substitutions,
)


ROOT = Path(__file__).resolve().parents[3]
CAMERA_CONFIG = ROOT / 'src/estimation_pkg/config/realsense_d405.yaml'
TAG_CAMERA_CONFIG = ROOT / 'src/launcher/config/realsense_apriltag.yaml'


@pytest.fixture(autouse=True)
def isolated_launch_logs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))


def load_launch(name):
    path = ROOT / 'src/launcher/launch' / (name + '.launch.py')
    spec = importlib.util.spec_from_file_location('test_' + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def camera_launch():
    return load_launch('cuda_realsense')


def resolve(value, context):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def select(module, config, model='D405$', serial="''"):
    context = LaunchContext()
    context.launch_configurations.update(device_type=model, serial_no=serial)
    actions = module._validate_camera_selection(context, config)
    for action in actions:
        action.execute(context)
    return context


@pytest.mark.parametrize('model, expected', [
    ('D405', 'D405$'), ('D405$', 'D405$'), ('D455', 'D455$'),
    ('D435i$', 'D435i$'), ('D455f', 'D455f$'), (' d405 ', 'd405$'),
])
def test_plain_and_gui_models_are_anchored(camera_launch, model, expected):
    context = select(camera_launch, CAMERA_CONFIG, model)
    assert context.launch_configurations['device_type'] == expected


def test_default_d405_cannot_match_d455(camera_launch):
    context = select(camera_launch, CAMERA_CONFIG)
    pattern = context.launch_configurations['device_type']
    assert re.search(pattern, 'Intel RealSense D405', flags=re.IGNORECASE)
    assert not re.search(pattern, 'Intel RealSense D455', flags=re.IGNORECASE)


def test_d455_does_not_match_d455f(camera_launch):
    context = select(camera_launch, CAMERA_CONFIG, 'D455')
    pattern = context.launch_configurations['device_type']
    assert re.search(pattern, 'Intel RealSense D455')
    assert not re.search(pattern, 'Intel RealSense D455f')


@pytest.mark.parametrize('model', ['', "''", '.*', 'D4..', 'D405|D455', 'D455$$', 'D４０５'])
def test_blank_or_broad_model_never_enables_fallback(camera_launch, model):
    with pytest.raises(RuntimeError, match='device_type'):
        select(camera_launch, CAMERA_CONFIG, model)


@pytest.mark.parametrize('serial', ['', "''", '""', '  '])
def test_empty_serial_keeps_model_restriction(camera_launch, serial):
    context = select(camera_launch, CAMERA_CONFIG, serial=serial)
    assert context.launch_configurations['serial_no'] == "''"
    assert context.launch_configurations['device_type'] == 'D405$'


@pytest.mark.parametrize('serial', ['043422251095', '_043422251095', ' 043422251095 '])
def test_serial_is_yaml_string_and_preserves_leading_zero(camera_launch, serial):
    context = select(camera_launch, CAMERA_CONFIG, 'D455', serial)
    emitted = context.launch_configurations['serial_no']
    assert emitted == '_043422251095'
    assert isinstance(yaml.safe_load(emitted), str)
    assert emitted[1:] == '043422251095'  # RealSense factory removes the prefix.


@pytest.mark.parametrize('serial', ['_abc', '_', '__043', '04 34', '１２３', '123.0', '-123'])
def test_invalid_serial_is_rejected(camera_launch, serial):
    with pytest.raises(RuntimeError, match='serial_no'):
        select(camera_launch, CAMERA_CONFIG, serial=serial)


@pytest.mark.parametrize('key', ['device_type', 'serial_no', 'usb_port_id'])
@pytest.mark.parametrize('value', ['', "''", 'unexpected', None])
def test_config_cannot_override_selection_even_with_empty_value(
        camera_launch, tmp_path, key, value):
    config = tmp_path / 'streams.yaml'
    config.write_text(yaml.safe_dump({key: value, 'enable_depth': True}))
    with pytest.raises(RuntimeError, match='Remove camera identity keys'):
        select(camera_launch, config)


@pytest.mark.parametrize('content', ['false', '[]', 'not-a-mapping'])
def test_invalid_config_shape_is_rejected(camera_launch, tmp_path, content):
    config = tmp_path / 'streams.yaml'
    config.write_text(content)
    with pytest.raises(RuntimeError, match='parameter mapping'):
        select(camera_launch, config)


def test_production_config_retains_profiles_and_has_no_selection_override():
    config = yaml.safe_load(CAMERA_CONFIG.read_text())
    assert not {'device_type', 'serial_no', 'usb_port_id'} & config.keys()
    assert config['depth_module.color_profile'] == '848,480,30'
    assert config['depth_module.depth_profile'] == '848,480,30'
    assert config['align_depth.enable'] is True
    assert config['enable_sync'] is True


def camera_description(module, monkeypatch, tmp_path):
    monkeypatch.setattr(module, '_find_workspace_root', lambda: tmp_path)
    estimation_share = ROOT / 'src/estimation_pkg'
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(estimation_share))
    # Isolate the launch process's intentional environment changes in this test.
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/existing/ament')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/existing/lib')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    return module.generate_launch_description()


def capture_includes(description, context=None):
    """Evaluate only launch configuration/scoping, never included ROS nodes."""
    context = context or LaunchContext()
    captured = []

    def visit(actions):
        for action in actions:
            if isinstance(action, IncludeLaunchDescription):
                snapshot = LaunchContext()
                snapshot.launch_configurations.update(context.launch_configurations)
                snapshot.captured_environment = dict(context.environment)
                captured.append((action, snapshot))
            elif not isinstance(action, LogInfo):
                visit(action.execute(context) or [])

    visit(description.entities)
    return captured


def test_launch_keeps_vendor_include_and_explicit_selection(camera_launch, monkeypatch, tmp_path):
    description = camera_description(camera_launch, monkeypatch, tmp_path)
    includes = capture_includes(description)
    assert len(includes) == 1
    include, context = includes[0]
    assert context.launch_configurations['device_type'] == 'D405$'
    assert context.launch_configurations['serial_no'] == "''"
    arguments = {key: resolve(value, context) for key, value in include.launch_arguments}
    assert arguments == {'config_file': str(CAMERA_CONFIG),
                         'device_type': 'D405$', 'serial_no': "''",
                         'camera_name': 'camera', 'camera_namespace': 'camera'}
    source = include.launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda _: LaunchDescription())
    source.get_launch_description(context)
    assert source.location == str(
        tmp_path / '.cuda_realsense/install/realsense-ros'
        / 'share/realsense2_camera/launch/rs_launch.py')


def test_cli_overrides_flow_to_vendor_without_default_replacement(
        camera_launch, monkeypatch, tmp_path):
    description = camera_description(camera_launch, monkeypatch, tmp_path)
    context = LaunchContext()
    context.launch_configurations.update(device_type='D455', serial_no='043422251095')
    include, context = capture_includes(description, context)[0]
    arguments = {key: resolve(value, context) for key, value in include.launch_arguments}
    assert arguments['device_type'] == 'D455$'
    assert arguments['serial_no'] == '_043422251095'


def test_combined_launch_forwards_selection_only_to_camera(monkeypatch):
    module = load_launch('cuda_estimation')
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(ROOT / 'src' / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(device_type='D435i$', serial_no='_123')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    includes = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 2
    camera_arguments = {k: resolve(v, context) for k, v in includes[0].launch_arguments}
    assert camera_arguments == {'device_type': 'D435i$', 'serial_no': '_123'}
    assert list(includes[1].launch_arguments) == []


def test_shell_entrypoint_forwards_cli_arguments():
    script = (ROOT / 'scripts/run_realsense_cuda.sh').read_text()
    assert 'exec ros2 launch launcher cuda_realsense.launch.py "$@"' in script


def evaluate_vendor(context, monkeypatch):
    """Capture installed vendor node arguments, without constructing a node."""
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    vendor_path = (ROOT / '.cuda_realsense/install/realsense-ros'
                   / 'share/realsense2_camera/launch/rs_launch.py')
    if not vendor_path.is_file():
        pytest.skip('Optional CUDA vendor launch is not installed in this workspace.')
    spec = importlib.util.spec_from_file_location('hrm_test_vendor_rs_launch', vendor_path)
    vendor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vendor)
    for declaration in vendor.declare_configurable_parameters(vendor.configurable_parameters):
        declaration.execute(context)
    captured = {}

    def capture_node(**kwargs):
        captured.update(kwargs)
        return object()  # No Node construction, execution, DDS, or camera access.

    monkeypatch.setattr(vendor.launch_ros.actions, 'Node', capture_node)
    vendor.launch_setup(context, vendor.set_configurable_parameters(vendor.configurable_parameters))
    evaluated = evaluate_parameters(context, normalize_parameters(captured['parameters']))
    merged = {}
    for parameters in evaluated:
        merged.update(parameters)
    return merged, captured


@pytest.mark.parametrize('model, serial, expected_model, expected_serial', [
    ('D405$', "''", 'D405$', ''),
    ('D405', '', 'D405$', ''),
    ('D455', '043422251095', 'D455$', '_043422251095'),
    ('D455$', '_043422251095', 'D455$', '_043422251095'),
])
def test_installed_vendor_parameter_evaluation_without_node_execution(
        camera_launch, monkeypatch, model, serial, expected_model, expected_serial):
    context = select(camera_launch, CAMERA_CONFIG, model, serial)
    context.launch_configurations['config_file'] = str(CAMERA_CONFIG)
    merged, captured = evaluate_vendor(context, monkeypatch)
    assert type(merged['device_type']) is str
    assert type(merged['serial_no']) is str
    assert merged['device_type'] == expected_model
    assert merged['serial_no'] == expected_serial
    assert merged['depth_module.depth_profile'] == '848,480,30'
    assert merged['depth_module.color_profile'] == '848,480,30'
    assert merged['align_depth.enable'] is True
    assert captured['package'] == 'realsense2_camera'
    assert captured['executable'] == 'realsense2_camera_node'
    assert resolve(captured['namespace'], context) == 'camera'
    assert resolve(captured['name'], context) == 'camera'


@pytest.mark.parametrize('key', [
    'camera_name', 'camera_namespace', 'base_frame_id', 'color_frame_id',
    'color_optical_frame_id', 'tf_prefix', '__node', '__ns',
])
def test_config_cannot_override_names_or_frames(camera_launch, tmp_path, key):
    config = tmp_path / 'streams.yaml'
    config.write_text(yaml.safe_dump({key: 'camera'}))
    with pytest.raises(RuntimeError, match='Remove camera name/namespace/frame keys'):
        select(camera_launch, config)


def tag_camera_description(monkeypatch):
    module = load_launch('cuda_apriltag_camera')
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(ROOT / 'src' / name))
    return module.generate_launch_description()


def test_tag_camera_launch_defaults_and_forwarding(monkeypatch):
    include, context = capture_includes(tag_camera_description(monkeypatch))[0]
    arguments = {key: resolve(value, context) for key, value in include.launch_arguments}
    assert arguments == {
        'device_type': 'D435i$', 'serial_no': "''",
        'camera_name': 'tag_camera', 'camera_namespace': 'tag_camera',
        'stream_config': str(TAG_CAMERA_CONFIG),
    }
    source = include.launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda _: LaunchDescription())
    source.get_launch_description(context)
    assert source.location == str(ROOT / 'src/launcher/launch/cuda_realsense.launch.py')


def test_tag_streams_are_rgb_only_and_separate_from_d405():
    config = yaml.safe_load(TAG_CAMERA_CONFIG.read_text())
    assert config['enable_color'] is True
    assert config['rgb_camera.color_profile'] == '1280,720,30'
    assert config['rgb_camera.color_format'] == 'RGB8'
    assert config['color_qos'] == config['color_info_qos'] == 'SENSOR_DATA'
    assert config['publish_tf'] is True
    assert 'depth_module.color_profile' not in config
    for key in ('enable_depth', 'enable_infra', 'enable_infra1', 'enable_infra2',
                'enable_gyro', 'enable_accel', 'enable_rgbd', 'pointcloud.enable',
                'align_depth.enable', 'enable_sync'):
        assert config[key] is False


@pytest.mark.parametrize('model, serial, expected_model, expected_serial', [
    ('D435i$', "''", 'D435i$', ''),
    ('D435i', '949122070024', 'D435i$', '_949122070024'),
    ('D455', '043422251095', 'D455$', '_043422251095'),
])
def test_tag_camera_reaches_vendor_with_unique_names_and_rgb_only_config(
        camera_launch, monkeypatch, tmp_path, model, serial, expected_model, expected_serial):
    tag_context = LaunchContext()
    tag_context.launch_configurations.update(device_type=model, serial_no=serial)
    include, tag_snapshot = capture_includes(tag_camera_description(monkeypatch), tag_context)[0]
    arguments = {key: resolve(value, tag_snapshot) for key, value in include.launch_arguments}
    shared_context = LaunchContext()
    shared_context.launch_configurations.update(arguments)
    vendor_include, context = capture_includes(
        camera_description(camera_launch, monkeypatch, tmp_path), shared_context)[0]
    context.launch_configurations.update({
        key: resolve(value, context) for key, value in vendor_include.launch_arguments
    })
    merged, captured = evaluate_vendor(context, monkeypatch)
    assert merged['device_type'] == expected_model
    assert merged['serial_no'] == expected_serial
    assert merged['camera_name'] == 'tag_camera'
    assert resolve(captured['name'], context) == 'tag_camera'
    assert resolve(captured['namespace'], context) == 'tag_camera'
    assert merged['rgb_camera.color_profile'] == '1280,720,30'
    assert merged['enable_color'] is True
    assert merged['enable_depth'] is False
    assert merged['align_depth.enable'] is False
    assert merged['enable_sync'] is False
    assert context.captured_environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(
        ROOT / 'src/estimation_pkg/config/fastdds_images.xml')


def test_sequential_camera_includes_do_not_leak_defaults(camera_launch, monkeypatch, tmp_path):
    description = camera_description(camera_launch, monkeypatch, tmp_path)
    context = LaunchContext()
    context.launch_configurations['unrelated'] = 'preserve'
    camera_include, camera_context = capture_includes(description, context)[0]
    assert context.launch_configurations == {'unrelated': 'preserve'}
    tag_include, tag_context = capture_includes(tag_camera_description(monkeypatch), context)[0]
    assert context.launch_configurations == {'unrelated': 'preserve'}
    camera_args = {key: resolve(value, camera_context)
                   for key, value in camera_include.launch_arguments}
    tag_args = {key: resolve(value, tag_context)
                for key, value in tag_include.launch_arguments}
    assert camera_args['device_type'] == 'D405$'
    assert camera_args['camera_name'] == 'camera'
    assert tag_args['device_type'] == 'D435i$'
    assert tag_args['camera_name'] == 'tag_camera'
