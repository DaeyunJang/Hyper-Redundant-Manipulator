"""Tip CSV provenance and optional capture selection, without ROS graph/hardware."""

import csv
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from record_pkg import export_csv
from record_pkg.capture import selected_topics


ESTIMATED_TIP = '/estimated_tip_position'
UNSCALED_TIP = '/estimated_tip_position_unscaled'
FK_TIP = '/kinematics/fk_tip_position'
TIP_TOPICS = (ESTIMATED_TIP, UNSCALED_TIP, FK_TIP)
POINT_TYPE = 'geometry_msgs/msg/PointStamped'


def recording_config():
    return json.loads((Path(__file__).parents[1] / 'config/recording.json').read_text())


def read_rows(path):
    with Path(path).open(newline='') as handle:
        return list(csv.DictReader(handle))


def test_tip_topics_selected_once_but_not_new_recording_gates():
    config = recording_config()
    selected = selected_topics(config)
    for topic in TIP_TOPICS:
        assert selected.count(topic) == 1
        assert topic not in config['required_topics']
    assert ESTIMATED_TIP in config['topic_groups']['geometry']
    assert UNSCALED_TIP in config['topic_groups']['geometry']
    assert FK_TIP in config['topic_groups']['motor_and_control']
    assert '/tool_endeffector_pose' in selected
    assert '/tool_endeffector_pose' in config['required_topics']
    assert '/estimated_segment_angle' in selected


def test_tip_topics_follow_explicit_group_selection():
    config = recording_config()
    config['enabled_groups'] = ['sensors']
    config['required_topics'] = ['/fts_data', '/loadcell_state']
    assert set(TIP_TOPICS).isdisjoint(selected_topics(config))
    config['extra_topics'] = [ESTIMATED_TIP]
    selected = selected_topics(config)
    assert ESTIMATED_TIP in selected
    assert UNSCALED_TIP not in selected
    assert FK_TIP not in selected


@pytest.mark.parametrize('topic', TIP_TOPICS)
def test_point_rows_preserve_metric_xyz_frame_and_source_time(topic):
    record = {
        'header': {'stamp': {'sec': 17, 'nanosec': 123}, 'frame_id': 'hrm_base'},
        'point': {'x': 0.078, 'y': -0.006, 'z': 0.005},
    }
    rows = list(export_csv.message_rows(record, POINT_TYPE, 18_000_000_005, 7, topic=topic))
    assert export_csv.topic_policy(POINT_TYPE) == 'full_fields'
    assert len(rows) == 1
    row = rows[0]
    assert [row['point.' + axis] for axis in 'xyz'] == [0.078, -0.006, 0.005]
    assert row['header.frame_id'] == 'hrm_base'
    assert row['source_time_ns'] == 17_000_000_123
    assert row['bag_receive_time_ns'] == 18_000_000_005
    assert row['bag_message_index'] == 7
    assert not any(key.startswith('derived_') for key in row)


def test_missing_tip_topics_do_not_inherit_another_position(tmp_path, monkeypatch):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag/metadata.yaml').write_text('test: true\n')
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(TIP_TOPICS),
    }))
    records = [(ESTIMATED_TIP, {
        'header': {'stamp': {'sec': 1, 'nanosec': 2}, 'frame_id': 'hrm_base'},
        'point': {'x': 0.08, 'y': 0.0, 'z': 0.0},
    }, 2_000_000_000)]
    reader = SimpleNamespace(
        get_all_topics_and_types=lambda: [
            SimpleNamespace(name=name, type=POINT_TYPE) for name in (ESTIMATED_TIP, FK_TIP)],
        has_next=lambda: bool(records), read_next=lambda: records.pop(0),
    )
    monkeypatch.setattr(export_csv, 'open_reader', lambda _: reader)
    monkeypatch.setattr(export_csv, 'make_decoder', lambda _: lambda value: value)
    monkeypatch.setattr('record_pkg.bag_integrity.audit_bag',
                        lambda *args, **kwargs: {'status': 'complete'})
    manifest = export_csv.export_session(tmp_path)
    entries = manifest['topics']
    assert manifest['status'] == 'complete'
    assert entries[ESTIMATED_TIP]['rows_written'] == 1
    assert entries[FK_TIP]['status'] == 'no_messages'
    assert entries[UNSCALED_TIP]['status'] == 'not_recorded'
    assert 'file' not in entries[FK_TIP]
    assert 'file' not in entries[UNSCALED_TIP]


def test_real_offline_bag_preserves_three_independent_tip_series(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import PointStamped
    from rclpy.serialization import serialize_message

    bag = tmp_path / 'bag'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag.ConverterOptions('', ''))
    for topic in TIP_TOPICS:
        writer.create_topic(rosbag.TopicMetadata(
            name=topic, type=POINT_TYPE, serialization_format='cdr'))

    def write_point(bag_writer, topic, x, nanosec, receive_ns):
        point = PointStamped()
        point.header.frame_id = 'hrm_base'
        point.header.stamp.sec = 12
        point.header.stamp.nanosec = nanosec
        point.point.x, point.point.y, point.point.z = x, -0.004, 0.002
        bag_writer.write(topic, serialize_message(point), receive_ns)

    write_point(writer, ESTIMATED_TIP, 0.079, 34, 13_000_000_001)
    write_point(writer, UNSCALED_TIP, 0.089, 34, 13_000_000_002)
    write_point(writer, FK_TIP, 0.078, 34, 13_000_000_003)
    # Only the estimator reports the next sample. The exporter must not fill FK.
    write_point(writer, ESTIMATED_TIP, 0.077, 100_000_034, 13_100_000_001)
    del writer
    (tmp_path / 'session.json').write_text(json.dumps({
        'state': 'exporting', 'selected_topics': list(TIP_TOPICS),
    }))
    manifest = export_csv.export_session(tmp_path)
    assert manifest['status'] == 'complete', manifest
    entries = manifest['topics']
    assert len({entries[topic]['file'] for topic in TIP_TOPICS}) == 3
    rows = {topic: read_rows(tmp_path / 'csv' / entries[topic]['file'])
            for topic in TIP_TOPICS}
    assert [len(rows[topic]) for topic in TIP_TOPICS] == [2, 1, 1]
    assert [rows[topic][0]['point.x'] for topic in TIP_TOPICS] == ['0.079', '0.089', '0.078']
    for topic in TIP_TOPICS:
        row = rows[topic][0]
        assert row['source_time_ns'] == '12000000034'
        assert row['header.frame_id'] == 'hrm_base'
        assert row['point.y'] == '-0.004'
        assert row['point.z'] == '0.002'
    assert rows[ESTIMATED_TIP][1]['source_time_ns'] == '12100000034'
    assert rows[FK_TIP][0]['bag_receive_time_ns'] == '13000000003'
