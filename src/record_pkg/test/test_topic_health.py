from types import SimpleNamespace

from record_pkg.topic_health import TopicHealth


def message(sec):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)))


def test_receipt_freshness_and_cached_source_rejection():
    health = TopicHealth(2)
    assert health.stale(['/angle'], now=10) == ['/angle']
    health.observe('/angle', message(1), now=10)
    assert not health.stale(['/angle'], now=11)
    health.observe('/angle', message(1), now=12)
    assert health.stale(['/angle'], now=12.1) == ['/angle']
    health.observe('/angle', message(2), now=12.2)
    assert not health.stale(['/angle'], now=12.3)


def test_headerless_data_uses_receipt_and_zero_header_is_not_fresh():
    health = TopicHealth(2)
    health.observe('/wire', object(), now=2)
    health.observe('/pose', message(0), now=2)
    assert not health.stale(['/wire'], now=3)
    assert health.stale(['/pose'], now=3) == ['/pose']
    assert health.stale(['/wire'], now=5) == ['/wire']


def test_restarted_publisher_can_begin_new_stamp_epoch():
    health = TopicHealth(2)
    health.observe('/pose', message(100), now=1, publisher_gid=b'old')
    health.observe('/pose', message(2), now=2, publisher_gid=b'old')
    assert health.last_stamp['/pose'] == 100 * 10**9  # ordinary out-of-order packet
    health.observe('/pose', message(3), now=3, publisher_gid=b'new')
    assert health.last_stamp['/pose'] == 3 * 10**9
    assert not health.stale(['/pose'], now=4)
    assert health.publisher_restarts['/pose'] == 1
