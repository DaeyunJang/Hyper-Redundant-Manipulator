"""Read-only receive-time continuity audit for finalized SQLite ROS bags.

Only topic IDs and receive timestamps are read: image and other payloads are
never deserialized. A receive gap is not proof of packet loss, sensor validity,
source-clock synchronization, or detection of a particular AprilTag ID.
"""

from contextlib import ExitStack
import heapq
import math
from pathlib import Path
import sqlite3

import yaml


def _names(values, label):
    if isinstance(values, str):
        raise ValueError(f'{label} must be a sequence of absolute topic names.')
    try:
        names = list(values)
    except TypeError as exc:
        raise ValueError(f'{label} must be a sequence of absolute topic names.') from exc
    if any(not isinstance(name, str) or not name.startswith('/') for name in names):
        raise ValueError(f'{label} must contain absolute topic names.')
    return list(dict.fromkeys(names))


def _empty_stats():
    return dict(message_count=0, first_receive_ns=None, last_receive_ns=None,
                max_internal_gap_ns=None, internal_gaps_over_limit=0)


def _observe(stats, timestamp, threshold_ns):
    previous = stats['last_receive_ns']
    if previous is not None:
        gap = timestamp - previous
        stats['max_internal_gap_ns'] = max(stats['max_internal_gap_ns'] or 0, gap)
        if gap > threshold_ns:
            stats['internal_gaps_over_limit'] += 1
    else:
        stats['first_receive_ns'] = timestamp
    stats['last_receive_ns'] = timestamp
    stats['message_count'] += 1


def audit_bag(bag_path, required_topics, max_gap_sec=2.0, selected_topics=None,
              observation_start_ns=None, observation_end_ns=None):
    """Audit bag receive timestamps, with optional intentional recording bounds.

    ``observation_*`` must use the same ROS clock as bag receive timestamps,
    not monotonic or ordinary wall-clock times. Without them the observed bag
    interval is used; leading discovery gaps are reported but not failed.
    Required internal/trailing gaps exceeding the threshold are incomplete,
    including a required source that publishes once and then stops. An explicit
    start also enables leading-gap checks. Event-driven topics should not be
    listed as periodically required.

    Return ``status`` complete/incomplete/error plus errors, observed per-topic
    counts, coverage and absent topics. Complete means this receive-time check
    passed, not that the dataset is scientifically valid. Missing/corrupt split
    files return error and partial counts are explicitly marked incomplete.
    Invalid caller configuration raises ValueError.
    """
    required = _names(required_topics, 'required_topics')
    selected = None if selected_topics is None else _names(selected_topics, 'selected_topics')
    if (isinstance(max_gap_sec, bool) or not isinstance(max_gap_sec, (int, float))
            or not math.isfinite(max_gap_sec) or max_gap_sec <= 0):
        raise ValueError('max_gap_sec must be finite and positive.')
    for name, bound in (('observation_start_ns', observation_start_ns),
                        ('observation_end_ns', observation_end_ns)):
        if bound is not None and (isinstance(bound, bool) or not isinstance(bound, int)):
            raise ValueError(f'{name} must be an integer ROS receive timestamp or None.')
    if (observation_start_ns is not None and observation_end_ns is not None
            and observation_start_ns > observation_end_ns):
        raise ValueError('observation_start_ns must not exceed observation_end_ns.')
    threshold_ns = max_gap_sec * 1_000_000_000
    bag_path = Path(bag_path).expanduser().resolve()
    report = dict(
        schema_version=1, status='error', bag_path=str(bag_path), scan_complete=False,
        validity_scope='receive-time continuity only; not payload/source-time validity',
        max_gap_sec=max_gap_sec, required_topics=required, errors=[], warnings=[],
        files=[], topics={}, required_topics_without_messages=[],
        required_topics_without_window_messages=[], required_topics_with_receive_gaps=[],
        absent_selected_topics=[],
    )
    all_stats, window_stats = {}, {}
    global_first = global_last = None

    def ensure_topic(name):
        if not isinstance(name, str) or not name.startswith('/'):
            raise ValueError('Bag contains a missing or non-absolute topic name.')
        all_stats.setdefault(name, _empty_stats())
        window_stats.setdefault(name, _empty_stats())

    for name in required + (selected or []):
        ensure_topic(name)
    try:
        document = yaml.safe_load((bag_path / 'metadata.yaml').read_text())
        info = document['rosbag2_bagfile_information']
        if info.get('storage_identifier', 'sqlite3') != 'sqlite3':
            raise ValueError('Only sqlite3 bags are supported by this audit.')
        paths = info['relative_file_paths']
        if (not isinstance(paths, list) or not paths
                or any(not isinstance(path, str) or not path for path in paths)
                or len(set(paths)) != len(paths)):
            raise ValueError('relative_file_paths must list distinct SQLite split files.')
        for item in info.get('topics_with_message_count', []):
            ensure_topic(item['topic_metadata']['name'])
        if selected is None:
            selected = list(all_stats)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        report['errors'].append(f'metadata.yaml: {exc}')
        return report

    # Every cursor is timestamp-ordered. Merge split files rather than assuming
    # metadata ordering or nonoverlap; memory is O(split files + topic names).
    with ExitStack() as stack:
        heap = []
        sources = {}
        for file_index, relative in enumerate(paths):
            path = (bag_path / relative).resolve()
            file_report = dict(path=relative, status='error', message_count=0)
            report['files'].append(file_report)
            try:
                if Path(relative).is_absolute() or not path.is_relative_to(bag_path):
                    raise ValueError('Split-file path must remain inside the bag directory.')
                if not path.is_file():
                    raise OSError('Split file is missing or is not a regular file.')
                connection = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
                stack.callback(connection.close)
                connection.execute('PRAGMA query_only = ON')
                topics = dict(connection.execute('SELECT id, name FROM topics'))
                for name in topics.values():
                    ensure_topic(name)
                cursor = connection.execute(
                    'SELECT topic_id, timestamp FROM messages ORDER BY timestamp')
                sources[file_index] = (cursor, topics, file_report)
                row = cursor.fetchone()
                if row is not None:
                    if not isinstance(row[1], int):
                        raise ValueError('Non-integer receive timestamp.')
                    heapq.heappush(heap, (row[1], file_index, row[0]))
                file_report['status'] = 'scanning' if row is not None else 'complete'
            except (OSError, ValueError, sqlite3.Error) as exc:
                file_report['error'] = str(exc)
                report['errors'].append(f'{relative}: {exc}')

        while heap:
            timestamp, file_index, topic_id = heapq.heappop(heap)
            cursor, topics, file_report = sources[file_index]
            try:
                if topic_id not in topics:
                    raise ValueError(f'Message references unknown topic ID {topic_id}.')
                name = topics[topic_id]
                _observe(all_stats[name], timestamp, threshold_ns)
                if ((observation_start_ns is None or timestamp >= observation_start_ns)
                        and (observation_end_ns is None or timestamp <= observation_end_ns)):
                    _observe(window_stats[name], timestamp, threshold_ns)
                global_first = timestamp if global_first is None else global_first
                global_last = timestamp
                file_report['message_count'] += 1
                row = cursor.fetchone()
                if row is not None:
                    if not isinstance(row[1], int):
                        raise ValueError('Non-integer receive timestamp.')
                    heapq.heappush(heap, (row[1], file_index, row[0]))
                else:
                    file_report['status'] = 'complete'
            except (ValueError, sqlite3.Error) as exc:
                file_report.update(status='error', error=str(exc))
                report['errors'].append(f'{file_report["path"]}: {exc}')

    start = global_first if observation_start_ns is None else observation_start_ns
    end = global_last if observation_end_ns is None else observation_end_ns
    report['bag_receive_interval'] = dict(first_ns=global_first, last_ns=global_last)
    report['observation_interval'] = dict(
        start_ns=start, end_ns=end,
        start_source='bag' if observation_start_ns is None else 'explicit_ros_receive_time',
        end_source='bag' if observation_end_ns is None else 'explicit_ros_receive_time',
        leading_gap_check=observation_start_ns is not None,
    )
    for name, stats in sorted(all_stats.items()):
        window = window_stats[name]
        first, last = window['first_receive_ns'], window['last_receive_ns']
        leading = None if first is None or start is None else max(0, first - start)
        trailing = None if last is None or end is None else max(0, end - last)
        flags = []
        if not window['message_count']:
            flags.append('no_messages_in_observation_interval')
        if window['internal_gaps_over_limit']:
            flags.append('internal_receive_gap')
        if (observation_start_ns is not None and leading is not None
                and leading > threshold_ns):
            flags.append('leading_receive_gap')
        if trailing is not None and trailing > threshold_ns:
            flags.append('trailing_receive_gap')
        window.update(leading_gap_ns=leading, trailing_gap_ns=trailing, coverage_flags=flags)
        report['topics'][name] = dict(
            **stats, required=name in required, observation_window=window,
            observed=bool(stats['message_count']),
        )
    report['required_topics_without_messages'] = [
        name for name in required if not all_stats[name]['message_count']]
    report['required_topics_without_window_messages'] = [
        name for name in required if not window_stats[name]['message_count']]
    report['required_topics_with_receive_gaps'] = [
        name for name in required
        if any(flag.endswith('receive_gap') for flag in window_stats[name]['coverage_flags'])]
    report['absent_selected_topics'] = [
        name for name in selected if not all_stats.get(name, {}).get('message_count')]
    report['scan_complete'] = not report['errors']
    incomplete = (global_first is None or report['required_topics_without_window_messages']
                  or report['required_topics_with_receive_gaps'])
    report['status'] = ('error' if report['errors'] else
                        ('incomplete' if incomplete else 'complete'))
    report['warnings'].append(
        'Receive gaps are not proof of packet loss. Source stamps, frame identities, '
        'payload validity, sensor delay and tag visibility require separate checks.')
    if global_first is not None and (start > global_last or end < global_first):
        report['warnings'].append(
            'Requested observation interval does not overlap any bag receive timestamps; '
            'check ROS clock compatibility before interpreting absent-window messages.')
    return report
