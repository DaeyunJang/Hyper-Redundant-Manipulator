"""Launch routing only: no nodes, camera hardware, or ROS context are started."""

import importlib.util
import os
from pathlib import Path
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
from ament_index_python.packages import PackageNotFoundError  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable  # noqa: E402


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def routing(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    path = ROOT / 'src/launcher/launch/apriltag.launch.py'
    spec = importlib.util.spec_from_file_location('test_apriltag_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(ROOT / 'src' / name))
    # Capture construction arguments without executing launch or loading components.
    for kind in ('Node', 'ComposableNode', 'ComposableNodeContainer'):
        monkeypatch.setattr(module, kind, lambda _kind=kind, **kwargs: dict(kind=_kind, **kwargs))
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_d435i_default_routes_through_rectifier_and_preserves_global_outputs(routing):
    context = context_for(routing)
    actions = routing._launch_pipeline(context)
    container, detector, helper = [action for action in actions if isinstance(action, dict)]
    assert isinstance(actions[0], SetEnvironmentVariable)
    actions[0].execute(context)
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(
        ROOT / 'src/estimation_pkg/config/fastdds_images.xml')
    assert (container['name'], container['namespace']) == ('container', '/apriltag_preprocess')
    component, = container['composable_node_descriptions']
    assert component['plugin'] == 'image_proc::RectifyNode'
    assert component['name'] == 'rectify_color'
    remaps = dict(component['remappings'])
    assert remaps['image'] == '/tag_camera/tag_camera/color/image_raw'
    assert remaps['image_rect'] == '/tag_camera/tag_camera/color/image_rect'
    assert remaps['camera_info'] == '/tag_camera/tag_camera/color/camera_info'
    assert dict(detector['remappings'])['image_rect'] == remaps['image_rect']
    assert detector['package'] == 'apriltag_ros' and detector['name'] == 'apriltag_node'
    assert detector['namespace'] == '/' and helper['namespace'] == '/'
    assert helper['executable'] == 'apriltag_pose'
    assert helper['parameters'][0]['tag0_frame'].perform(context) == 'ID0'
    assert helper['parameters'][0]['tag1_frame'].perform(context) == 'ID1'
    assert not any('tf' in action.get('executable', '') for action in (container, detector, helper))


def test_d405_already_rectified_input_needs_no_image_proc(routing, monkeypatch):
    context = context_for(
        routing, rectify='false', image_topic='/camera/camera/color/image_rect_raw',
        camera_info_topic='/camera/camera/color/camera_info')
    missing = Mock(side_effect=PackageNotFoundError('image_proc'))
    monkeypatch.setattr(routing, 'get_package_share_directory', missing)
    detector, helper = routing._launch_pipeline(context)
    missing.assert_not_called()
    assert detector['name'] == 'apriltag_node' and helper['name'] == 'apriltag_pose'
    assert dict(detector['remappings'])['image_rect'] == '/camera/camera/color/image_rect_raw'


def test_nonstandard_info_topic_maps_derived_image_transport_sibling(routing):
    context = context_for(routing, image_topic='/rgb/source', camera_info_topic='/calib/custom',
                          rectified_image_topic='/rectified/rgb')
    container, detector, _ = [
        action for action in routing._launch_pipeline(context) if isinstance(action, dict)]
    component, = container['composable_node_descriptions']
    assert dict(component['remappings'])['/rgb/camera_info'] == '/calib/custom'
    assert dict(detector['remappings'])['/rectified/camera_info'] == '/calib/custom'


def test_missing_rectifier_fails_clearly_before_detector_construction(routing, monkeypatch):
    context = context_for(routing)
    node = Mock()
    monkeypatch.setattr(routing, 'Node', node)
    monkeypatch.setattr(routing, 'get_package_share_directory',
                        _missing_image_proc)
    monkeypatch.setattr(routing, '_local_image_proc_prefix', lambda: None)
    with pytest.raises(RuntimeError, match='install ros-humble-image-proc'):
        routing._launch_pipeline(context)
    node.assert_not_called()


def test_installed_image_proc_is_preferred_without_environment_changes(
        routing, monkeypatch, tmp_path):
    context = context_for(routing)
    monkeypatch.setattr(routing, '_local_image_proc_prefix', lambda: tmp_path / 'local')
    monkeypatch.setattr(routing, 'get_package_share_directory',
                        lambda _: '/opt/ros/humble/share/image_proc')
    before_process, before_launch = dict(os.environ), dict(context.environment)
    assert routing._image_proc_environment(context) == []
    assert dict(os.environ) == before_process and dict(context.environment) == before_launch


def _missing_image_proc(name):
    if name == 'image_proc':
        raise PackageNotFoundError(name)
    return str(ROOT / 'src' / name)


def test_local_verified_dependency_prefix_is_forwarded_before_nodes(routing, monkeypatch, tmp_path):
    prefix = tmp_path / 'official/opt/ros/humble'
    monkeypatch.setattr(routing, 'get_package_share_directory',
                        _missing_image_proc)
    monkeypatch.setattr(routing, '_local_image_proc_prefix', lambda: prefix)
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/opt/ros/humble')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/ros/humble/lib')
    context = context_for(routing)
    actions = routing._launch_pipeline(context)
    environment_actions = [
        action for action in actions if isinstance(action, SetEnvironmentVariable)]
    assert len(environment_actions) == 3
    assert all(actions.index(action) < len(actions) - 3 for action in environment_actions)
    assert [action['kind'] for action in actions[-3:]] == [
        'ComposableNodeContainer', 'Node', 'Node']
    assert os.environ['AMENT_PREFIX_PATH'] == f'{prefix}:/opt/ros/humble'
    assert os.environ['LD_LIBRARY_PATH'] == f'{prefix}/lib:/opt/ros/humble/lib'


@pytest.mark.parametrize('library_already_present', [False, True])
def test_partially_configured_local_prefix_completes_environment_without_duplicates(
        routing, monkeypatch, tmp_path, library_already_present):
    prefix = tmp_path / 'local/opt/ros/humble'
    monkeypatch.setattr(routing, '_local_image_proc_prefix', lambda: prefix)
    monkeypatch.setattr(routing, 'get_package_share_directory',
                        lambda _: str(prefix / 'share/image_proc'))
    monkeypatch.setenv('AMENT_PREFIX_PATH', f'{prefix}:/opt/ros/humble')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/ros/humble/lib')
    context = context_for(routing)
    # A scoped parent launch may add paths absent from the Python process env.
    context.environment['AMENT_PREFIX_PATH'] = f'{prefix}:/parent/overlay:/opt/ros/humble'
    library_base = '/parent/lib:/opt/ros/humble/lib'
    context.environment['LD_LIBRARY_PATH'] = (
        f'{prefix}/lib:{library_base}' if library_already_present else library_base)
    for action in routing._image_proc_environment(context):
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    assert context.environment['AMENT_PREFIX_PATH'] == (
        f'{prefix}:/parent/overlay:/opt/ros/humble')
    assert context.environment['LD_LIBRARY_PATH'] == f'{prefix}/lib:{library_base}'
    assert os.environ['AMENT_PREFIX_PATH'] == context.environment['AMENT_PREFIX_PATH']
    assert os.environ['LD_LIBRARY_PATH'] == context.environment['LD_LIBRARY_PATH']


@pytest.mark.parametrize('name', [
    'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'])
def test_explicit_profile_in_launch_context_is_not_replaced(routing, name):
    context = context_for(routing)
    # Also cover a parent launch's SetEnvironmentVariable, not only os.environ.
    context.environment[name] = '/operator/custom.xml'
    assert routing._image_transport_environment(context) == []
    assert context.environment[name] == '/operator/custom.xml'


def test_non_fastdds_does_not_receive_fastrtps_profile(routing):
    context = context_for(routing)
    context.environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    assert routing._image_transport_environment(context) == []
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in context.environment


def test_missing_image_profile_fails_before_starting_rectifier(routing, monkeypatch, tmp_path):
    context = context_for(routing)
    monkeypatch.setattr(routing, 'get_package_share_directory', lambda _: str(tmp_path))
    with pytest.raises(RuntimeError, match='Rebuild estimation_pkg'):
        routing._launch_pipeline(context)


@pytest.mark.parametrize('overrides', [
    {'rectify': 'maybe'}, {'image_topic': 'relative/topic'}, {'image_topic': '/camera//image'},
    {'camera_info_topic': '/bad topic'}, {'rectified_image_topic': '/bad/'},
    {'rectified_image_topic': '/tag_camera/tag_camera/color/image_raw'},
    {'rectified_image_topic': '/tag_camera/tag_camera/color/camera_info'},
])
def test_invalid_routing_is_rejected(routing, overrides):
    with pytest.raises(RuntimeError):
        routing._launch_pipeline(context_for(routing, **overrides))


def test_existing_ids_sizes_and_declared_dependencies_remain():
    config = yaml.safe_load((ROOT / 'src/launcher/config/apriltag.yaml').read_text())
    assert config['/**']['ros__parameters']['size'] == 0.020
    tags = config['/**']['ros__parameters']['tag']
    assert tags == dict(ids=[0, 1], frames=['ID0', 'ID1'], sizes=[0.020, 0.020])
    dependencies = {node.text for node in ET.parse(ROOT / 'src/launcher/package.xml').getroot()
                    if node.tag == 'exec_depend'}
    assert {'image_proc', 'rclcpp_components', 'apriltag_ros', 'record_pkg'} <= dependencies
