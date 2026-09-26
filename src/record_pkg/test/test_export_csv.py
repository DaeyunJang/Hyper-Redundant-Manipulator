"""CSV contracts plus a tiny real offline bag test; no ROS graph or motor I/O."""

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from record_pkg import export_csv


def read_rows(path):
    with Path(path).open(newline='') as handle:
        return list(csv.DictReader(handle))


@pytest.mark.parametrize('topic', ['/fts_data', '/fts_data_kalman_filter'])
def test_sensor_torque_csv_one_decimal_without_changing_source(topic):
    from record_pkg.summary_csv import wrench
    record = {'wrench': {'force': {'x': 1.23456},
                        'torque': {'x': 0.30000000004, 'y': -1.27, 'z': 2.0}}}
    row = next(export_csv.message_rows(record, 'geometry_msgs/msg/WrenchStamped',
                                       123, 0, topic=topic))
    assert [row[f'wrench.torque.{axis}'] for axis in 'xyz'] == ['0.3', '-1.3', '2.0']
    assert row['wrench.force.x'] == 1.23456
    assert record['wrench']['torque']['x'] == 0.30000000004
    assert [wrench(row)[f't{axis}'] for axis in 'xyz'] == ['0.3', '-1.3', '2.0']
    assert wrench({})['tx'] == ''


def test_arrays_are_lossless_stable_columns():
    record = {'motor': {'position': [1000, 2000, 3000, 4000]},
              'pan': [0.01] * 18, 'changing': []}
    result = export_csv.flatten(record)
    assert json.loads(result['motor.position']) == [1000, 2000, 3000, 4000]
    assert len(json.loads(result['pan'])) == 18
    assert json.loads(result['changing']) == []
    record['changing'] = [1, 2, 3, 4, 5]
    assert export_csv.flatten(record).keys() == result.keys()
    assert json.loads(export_csv.compact_json([float('nan'), float('inf')])) == ['nan', 'inf']
    assert json.loads(export_csv.compact_json([b'\x00', b'\xff'])) == [0, 255]
    assert json.loads(export_csv.compact_json(b'\x00\xff')) == [0, 255]


def test_transform_rows_preserve_individual_stamps_and_frames():
    def transform(child, seconds):
        return {'header': {'stamp': {'sec': seconds, 'nanosec': 42}, 'frame_id': 'camera'},
                'child_frame_id': child, 'transform': {'rotation': {'w': 1.0}}}
    rows = list(export_csv.message_rows(
        {'transforms': [transform('ID0', 2), transform('ID1', 3)]},
        'tf2_msgs/msg/TFMessage', 5_000_000_000, 8))
    assert [row['source_time_ns'] for row in rows] == [2_000_000_042, 3_000_000_042]
    assert [row['child_frame_id'] for row in rows] == ['ID0', 'ID1']
    assert [row['item_index'] for row in rows] == [0, 1]
    assert all(row['bag_message_index'] == 8 for row in rows)


def test_headerless_is_blank_and_zero_stamp_not_fabricated():
    row = next(export_csv.message_rows({'data': 'position'}, 'std_msgs/msg/String', 123, 0))
    assert row['source_time_ns'] == ''
    row = next(export_csv.message_rows(
        {'header': {'stamp': {'sec': 0, 'nanosec': 0}}}, 'example/msg/HeaderOnly', 123, 0))
    assert row['source_time_ns'] == 0


def test_pose_quaternion_retained_with_derived_radians():
    orientation = {'x': 0.0, 'y': 0.0, 'z': math.sqrt(0.5), 'w': math.sqrt(0.5)}
    pose = {'pose': {'orientation': orientation}}
    row = next(export_csv.message_rows(pose, 'geometry_msgs/msg/PoseStamped', 1, 0))
    assert row['pose.orientation.z'] == orientation['z']
    assert row['derived_yaw_rad'] == pytest.approx(math.pi / 2)
    assert row['derived_roll_rad'] == 0.0
    pose['pose']['orientation'] = {key: 0.0 for key in orientation}
    assert not export_csv.pose_euler(pose)['derived_quaternion_valid']


def test_binary_payloads_never_become_csv_cells():
    record = {'header': {'stamp': {'sec': 3, 'nanosec': 2}, 'frame_id': 'optical'},
              'width': 2, 'height': 1, 'encoding': '16UC1', 'data': b'\x01\x00\x02\x00'}
    row = next(export_csv.message_rows(record, 'sensor_msgs/msg/Image', 123, 0))
    assert row['payload_bytes'] == 4
    assert row['encoding'] == '16UC1'
    assert 'data' not in row
    assert export_csv.topic_policy('visualization_msgs/msg/MarkerArray') == 'skipped_visualization'
    assert export_csv.topic_policy('apriltag_msgs/msg/AprilTagDetectionArray') == 'full_fields'


def test_topic_names_cannot_escape_or_collide():
    names = ['/a/b', '/a__b', '/../../escape', '/a.' + 'b' * 500]
    files = [export_csv.topic_filename(name) for name in names]
    assert len(set(files)) == len(files)
    assert all('/' not in name and '..' not in name and len(name) < 200 for name in files)


def fake_session(tmp_path, selected=None):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'metadata.yaml').write_text('test: true\n')
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': selected or ['/mode'],
        'snapshot': {'hardware_constants': {'OP_MODE': 8}},
    }))
    return tmp_path


def patch_reader(monkeypatch, records, types):
    import record_pkg.bag_integrity
    monkeypatch.setattr(record_pkg.bag_integrity, 'audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})
    reader = SimpleNamespace(
        get_all_topics_and_types=lambda: [SimpleNamespace(name=name, type=value)
                                          for name, value in types.items()],
        has_next=lambda: bool(records), read_next=lambda: records.pop(0),
    )
    monkeypatch.setattr(export_csv, 'open_reader', lambda path: reader)
    monkeypatch.setattr(export_csv, 'make_decoder', lambda type_name: lambda value: value)


def test_manifest_distinguishes_absent_empty_skipped_and_actual_zero(tmp_path, monkeypatch):
    session = fake_session(tmp_path, ['/mode', '/missing', '/empty', '/markers'])
    patch_reader(monkeypatch, [('/mode', {'data': 0}, 55), ('/markers', None, 56)], {
        '/mode': 'std_msgs/msg/Int32', '/empty': 'std_msgs/msg/Int32',
        '/markers': 'visualization_msgs/msg/MarkerArray',
    })
    manifest = export_csv.export_session(session)
    assert manifest['status'] == 'complete'
    assert manifest['topics']['/missing']['status'] == 'not_recorded'
    assert manifest['topics']['/empty']['status'] == 'no_messages'
    assert manifest['topics']['/markers']['status'] == 'skipped_visualization'
    assert manifest['topics']['/mode']['rows_written'] == 1
    row = read_rows(tmp_path / 'csv' / manifest['topics']['/mode']['file'])[0]
    assert row['data'] == '0' and row['source_time_ns'] == ''
    metadata = read_rows(tmp_path / 'csv' / 'session_metadata.csv')
    assert {'key': 'snapshot.hardware_constants.OP_MODE', 'value_json': '8'} in metadata


def test_existing_export_and_active_bag_rejected_without_modification(tmp_path, monkeypatch):
    fake_session(tmp_path)
    (tmp_path / 'csv').mkdir()
    kept = tmp_path / 'csv' / 'important.csv'
    kept.write_text('keep me')
    with pytest.raises(FileExistsError):
        export_csv.export_session(tmp_path)
    assert kept.read_text() == 'keep me'
    metadata = tmp_path / 'session.json'
    metadata.write_text(json.dumps({'state': 'recording'}))
    with pytest.raises(ValueError, match='still recording'):
        export_csv.export_session(tmp_path, tmp_path / 'other')
    assert not (tmp_path / 'other').exists()


def test_missing_type_and_decode_failure_are_partial_not_zero(tmp_path, monkeypatch):
    fake_session(tmp_path)
    patch_reader(monkeypatch, [('/mode', b'x', 55), ('/bad', b'x', 56)], {
        '/mode': 'custom_interfaces/msg/Missing', '/bad': 'std_msgs/msg/Int32',
    })

    def decoder(type_name):
        if 'Missing' in type_name:
            raise ImportError('message package unavailable')

        def decode(value):
            raise ValueError('corrupt sample')
        return decode
    monkeypatch.setattr(export_csv, 'make_decoder', decoder)
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'partial'
    assert manifest['topics']['/mode']['status'] == 'unavailable_message_type'
    assert manifest['topics']['/bad']['status'] == 'partial'
    assert not list((tmp_path / 'csv').glob('mode*.csv'))
    assert json.loads((tmp_path / 'csv' / 'manifest.json').read_text())['status'] == 'partial'


def test_real_parameter_event_octets_and_nonfinite_arrays_are_json_safe():
    pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from rcl_interfaces.msg import ParameterEvent
    event = ParameterEvent(node='/test_parameters', changed_parameters=[
        Parameter('byte_values', value=[b'\x00', b'\x7f', b'\xff']).to_parameter_msg(),
        Parameter('double_values', value=[0.125, -2.5]).to_parameter_msg(),
        Parameter('nonfinite_values', value=[float('nan'), float('inf'), -float('inf')])
        .to_parameter_msg(),
    ])
    serialized = serialize_message(event)
    record = export_csv.make_decoder('rcl_interfaces/msg/ParameterEvent')(serialized)
    row = next(export_csv.message_rows(record, 'rcl_interfaces/msg/ParameterEvent', 123, 0))
    parameters = {item['name']: item['value'] for item in json.loads(row['changed_parameters'])}
    assert parameters['byte_values']['byte_array_value'] == [0, 127, 255]
    assert parameters['double_values']['double_array_value'] == [0.125, -2.5]
    assert parameters['nonfinite_values']['double_array_value'] == ['nan', 'inf', '-inf']
    # Ensure this did not silently convert real parameter strings to octets.
    assert record['node'] == '/test_parameters'


@pytest.mark.parametrize('quality, expected_code', [('incomplete', 0), ('error', 2)])
def test_cli_reports_dataset_quality_separately(monkeypatch, capsys, quality, expected_code):
    monkeypatch.setattr(export_csv, 'export_session', lambda *args: {
        'status': 'complete', 'csv_directory': '/example/csv',
        'integrity': {'status': quality, 'required_topics_without_messages': ['/missing'],
                      'required_topics_with_receive_gaps': ['/intermittent'],
                      'errors': ['missing split file'] if quality == 'error' else []},
    })
    assert export_csv.main(['/example']) == expected_code
    output = capsys.readouterr()
    assert 'CSV export complete' in output.out
    assert f'Dataset receive-time integrity: {quality}' in output.out
    assert '/missing' in output.out and '/intermittent' in output.out
    if quality == 'error':
        assert 'missing split file' in output.err


def test_real_bag_export_has_full_channels_original_stamps_and_tag_pose(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from custom_interfaces.msg import LoadcellState, MotorState, SegmentAngle
    bag = tmp_path / 'bag'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='sqlite3', max_bagfile_duration=1),
                rosbag.ConverterOptions('', ''))
    motor = MotorState(actual_position=[1000, 2000, 3000, 4000])
    loadcell = LoadcellState(stress=[1.0, 2.0, 3.0, 4.0])
    angles = SegmentAngle()
    angles.pan_relative = [0.01] * 18
    angles.tilt_relative = [0.02] * 18
    force, filtered = WrenchStamped(), WrenchStamped()
    force.wrench.force.x, filtered.wrench.force.x = 10.0, 8.0
    tag = PoseStamped()
    tag.header.frame_id = 'ID0'
    tag.pose.position.x = 0.123
    tag.pose.orientation.w = 1.0
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'camera', 'ID0'
    transform.transform.rotation.w = 1.0
    image = Image(width=2, height=1, encoding='16UC1', step=4, data=b'\x01\x00\x02\x00')
    messages = {
        '/motor_state': motor, '/loadcell_state': loadcell, '/estimated_segment_angle': angles,
        '/fts_data': force, '/fts_data_kalman_filter': filtered,
        '/apriltag/tag1_in_tag0/pose': tag, '/tf': TFMessage(transforms=[transform]),
        '/depth': image, '/control_mode': String(data='position'),
    }
    for index, (topic, message) in enumerate(messages.items()):
        cls = type(message)
        type_name = f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'
        writer.create_topic(rosbag.TopicMetadata(
            name=topic, type=type_name, serialization_format='cdr'))
        if hasattr(message, 'header'):
            message.header.stamp.sec = 12
            message.header.stamp.nanosec = 34
        writer.write(topic, serialize_message(message),
                     13_000_000_000 + (index // 5) * 2_000_000_000 + index)
    del writer  # Finalizes metadata.yaml without creating a ROS node.
    assert len(list(bag.glob('*.db3'))) >= 2
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(messages) + ['/never_published'],
    }))
    (tmp_path / 'recording_config.json').write_text(json.dumps({
        'required_topics': ['/motor_state', '/apriltag/tag1_in_tag0/pose'],
    }))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete', manifest

    def row(topic):
        return read_rows(tmp_path / 'csv' / manifest['topics'][topic]['file'])[0]
    assert json.loads(row('/motor_state')['actual_position']) == [1000, 2000, 3000, 4000]
    assert json.loads(row('/loadcell_state')['stress']) == [1.0, 2.0, 3.0, 4.0]
    assert len(json.loads(row('/estimated_segment_angle')['pan_relative'])) == 18
    assert row('/fts_data')['wrench.force.x'] == '10.0'
    assert row('/fts_data_kalman_filter')['wrench.force.x'] == '8.0'
    assert row('/apriltag/tag1_in_tag0/pose')['header.frame_id'] == 'ID0'
    assert row('/apriltag/tag1_in_tag0/pose')['pose.position.x'] == '0.123'
    assert row('/apriltag/tag1_in_tag0/pose')['derived_yaw_rad'] == '0.0'
    assert row('/depth')['source_time_ns'] == '12000000034'
    assert row('/depth')['payload_bytes'] == '4' and 'data' not in row('/depth')
    assert row('/control_mode')['source_time_ns'] == ''
    assert manifest['topics']['/never_published']['status'] == 'not_recorded'
