"""Receive-time audits without ROS, payload decoding or physical hardware."""

import hashlib
import sqlite3

import pytest
import yaml

from record_pkg.bag_integrity import audit_bag


SECOND = 1_000_000_000


def make_bag(tmp_path, files, declared=('/required', '/other')):
    bag = tmp_path / 'bag'
    bag.mkdir()
    for filename, messages in files.items():
        with sqlite3.connect(str(bag / filename)) as connection:
            connection.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT)')
            connection.execute('CREATE TABLE messages '
                               '(id INTEGER PRIMARY KEY, topic_id INTEGER, '
                               'timestamp INTEGER, data BLOB)')
            connection.execute('CREATE INDEX timestamp_idx ON messages (timestamp)')
            connection.executemany('INSERT INTO topics VALUES (?, ?)', enumerate(declared, 1))
            connection.executemany(
                'INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)',
                [(topic_id, stamp, b'not a deserializable ROS message')
                 for topic_id, stamp in messages])
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3', 'relative_file_paths': list(files),
            'topics_with_message_count': [
                {'topic_metadata': {'name': name}, 'message_count': 123456}
                for name in declared],
        },
    }))
    return bag


def test_split_files_merge_timestamps_without_using_metadata_counts(tmp_path):
    bag = make_bag(tmp_path, {
        'bag_1.db3': [(1, 3 * SECOND), (2, 3 * SECOND), (1, SECOND)],
        'bag_0.db3': [(1, 0), (1, 2 * SECOND), (2, 0)],
    })
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'complete' and report['scan_complete']
    required = report['topics']['/required']
    assert required['message_count'] == 4
    assert required['first_receive_ns'] == 0
    assert required['last_receive_ns'] == 3 * SECOND
    assert required['max_internal_gap_ns'] == SECOND
    flags = report['topics']['/other']['observation_window']['coverage_flags']
    assert flags == ['internal_receive_gap']


def test_missing_required_and_absent_optional_not_fabricated(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(2, SECOND)]})
    report = audit_bag(bag, ['/required'], selected_topics=['/required', '/other', '/optional'])
    assert report['status'] == 'incomplete'
    assert report['required_topics_without_messages'] == ['/required']
    assert report['absent_selected_topics'] == ['/required', '/optional']
    assert report['topics']['/required']['first_receive_ns'] is None
    assert report['topics']['/required']['max_internal_gap_ns'] is None
    assert not report['topics']['/required']['observed']


def test_single_required_sample_then_stall_is_incomplete(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, 0), (2, SECOND), (2, 8 * SECOND)]})
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'incomplete'
    assert report['required_topics_with_receive_gaps'] == ['/required']
    window = report['topics']['/required']['observation_window']
    assert window['max_internal_gap_ns'] is None
    assert window['trailing_gap_ns'] == 8 * SECOND
    assert window['coverage_flags'] == ['trailing_receive_gap']


def test_discovery_leading_gap_only_reported_without_explicit_start(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(2, 0), (1, 5 * SECOND), (1, 6 * SECOND)]})
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'complete'
    assert report['topics']['/required']['observation_window']['leading_gap_ns'] == 5 * SECOND
    explicit = audit_bag(bag, ['/required'], observation_start_ns=0)
    assert explicit['status'] == 'incomplete'
    flags = explicit['topics']['/required']['observation_window']['coverage_flags']
    assert 'leading_receive_gap' in flags


def test_intentional_window_ignores_startup_and_finalization_gaps(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [
        (2, 0), (1, SECOND), (1, 10 * SECOND), (1, 11 * SECOND), (2, 20 * SECOND)]})
    report = audit_bag(bag, ['/required'], observation_start_ns=10 * SECOND,
                       observation_end_ns=12 * SECOND)
    assert report['status'] == 'complete'
    required = report['topics']['/required']
    assert required['message_count'] == 3 and required['max_internal_gap_ns'] == 9 * SECOND
    assert required['observation_window']['message_count'] == 2
    assert required['observation_window']['max_internal_gap_ns'] == SECOND
    assert required['observation_window']['trailing_gap_ns'] == SECOND


def test_interval_outside_bag_does_not_silently_pass(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, SECOND)]})
    report = audit_bag(bag, ['/required'], observation_start_ns=10 * SECOND,
                       observation_end_ns=12 * SECOND)
    assert report['status'] == 'incomplete'
    assert not report['required_topics_without_messages']
    assert report['required_topics_without_window_messages'] == ['/required']
    assert any('clock compatibility' in warning for warning in report['warnings'])


@pytest.mark.parametrize('damage', ['missing', 'corrupt'])
def test_missing_or_corrupt_split_file_returns_error_with_partial_counts(tmp_path, damage):
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, 0)], 'bag_1.db3': [(1, SECOND)]})
    damaged = bag / 'bag_1.db3'
    if damage == 'missing':
        damaged.unlink()
    else:
        damaged.write_bytes(b'corrupt sqlite file')
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'error' and not report['scan_complete']
    assert report['topics']['/required']['message_count'] == 1
    assert any('bag_1.db3' in error for error in report['errors'])


def test_missing_metadata_and_unsafe_split_path_report_errors(tmp_path):
    assert audit_bag(tmp_path, ['/required'])['status'] == 'error'
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, 0)]})
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
    metadata['rosbag2_bagfile_information']['relative_file_paths'] = ['../outside.db3']
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'error'
    assert 'inside the bag directory' in report['errors'][0]
    assert not (tmp_path / 'outside.db3').exists()


def test_only_metadata_listed_files_are_scanned(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, 0)]})
    (bag / 'unlisted.db3').write_bytes(b'not part of this recording')
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'complete'
    assert [item['path'] for item in report['files']] == ['bag_0.db3']


def test_unknown_topic_reference_is_an_audit_error(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': [(99, 0)]})
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'error'
    assert not report['scan_complete']
    assert 'unknown topic ID 99' in report['errors'][0]


def test_audit_does_not_modify_database_or_read_payload_columns(tmp_path, monkeypatch):
    bag = make_bag(tmp_path, {'bag_0.db3': [(1, index * SECOND) for index in range(5000)]})
    database = bag / 'bag_0.db3'
    before = hashlib.sha256(database.read_bytes()).hexdigest()
    queries = []
    original_connect = sqlite3.connect

    def traced_connect(*args, **kwargs):
        assert kwargs['uri'] is True and args[0].endswith('?mode=ro')
        connection = original_connect(*args, **kwargs)
        connection.set_trace_callback(queries.append)
        return connection

    monkeypatch.setattr('record_pkg.bag_integrity.sqlite3.connect', traced_connect)
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'complete'
    assert report['topics']['/required']['message_count'] == 5000
    assert hashlib.sha256(database.read_bytes()).hexdigest() == before
    assert not any('data' in query.lower() or '*' in query for query in queries)


@pytest.mark.parametrize('gap', [0, -1, float('nan'), float('inf'), True, '2'])
def test_invalid_gap_threshold_rejected(tmp_path, gap):
    with pytest.raises(ValueError, match='max_gap_sec'):
        audit_bag(tmp_path, ['/required'], max_gap_sec=gap)


def test_invalid_names_and_reversed_window_rejected(tmp_path):
    with pytest.raises(ValueError, match='required_topics'):
        audit_bag(tmp_path, '/required')
    with pytest.raises(ValueError, match='required_topics'):
        audit_bag(tmp_path, ['relative_topic'])
    with pytest.raises(ValueError, match='must not exceed'):
        audit_bag(tmp_path, ['/required'], observation_start_ns=2, observation_end_ns=1)


def test_empty_finalized_bag_has_no_fabricated_timestamps(tmp_path):
    bag = make_bag(tmp_path, {'bag_0.db3': []})
    report = audit_bag(bag, ['/required'])
    assert report['status'] == 'incomplete'
    assert report['bag_receive_interval'] == {'first_ns': None, 'last_ns': None}
    assert report['topics']['/required']['observation_window']['trailing_gap_ns'] is None
