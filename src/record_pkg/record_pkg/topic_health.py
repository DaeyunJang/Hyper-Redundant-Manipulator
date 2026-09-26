"""Small-message freshness observations, separate from proof of bag persistence."""

import time


class TopicHealth:
    def __init__(self, timeout_sec):
        self.timeout_sec = timeout_sec
        self.last_progress = {}
        self.last_stamp = {}
        self.publisher_gid = {}
        self.publisher_restarts = {}

    def set_publisher(self, topic, publisher_gid):
        gid = bytes(publisher_gid)
        previous = self.publisher_gid.get(topic)
        if previous is not None and previous != gid:
            self.last_stamp.pop(topic, None)
            self.publisher_restarts[topic] = self.publisher_restarts.get(topic, 0) + 1
        self.publisher_gid[topic] = gid

    def observe(self, topic, message, now=None, publisher_gid=None):
        now = time.monotonic() if now is None else now
        if publisher_gid is not None:
            self.set_publisher(topic, publisher_gid)
        header = getattr(message, 'header', None)
        if header is not None:
            stamp = header.stamp.sec * 10**9 + header.stamp.nanosec
            # Repeated cached timestamps do not count as a progressing stream.
            if stamp <= 0 or stamp <= self.last_stamp.get(topic, -1):
                return
            self.last_stamp[topic] = stamp
        self.last_progress[topic] = now

    def stale(self, topics, now=None):
        now = time.monotonic() if now is None else now
        return sorted(topic for topic in topics
                      if now - self.last_progress.get(topic, float('-inf')) > self.timeout_sec)
