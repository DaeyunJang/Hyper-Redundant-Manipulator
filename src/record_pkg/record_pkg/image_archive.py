"""Bounded, asynchronous lossless archive of the HRM crop image stream."""

import csv
import json
from pathlib import Path
import queue
import threading

import cv2
import numpy as np


INDEX_FIELDS = (
    'source_time_ns', 'received_time_ns', 'frame_id', 'filename', 'width',
    'height', 'input_encoding', 'payload_bytes', 'contact_segment_id',
)


def _positive_integer(value, name, *, allow_zero=False):
    if (isinstance(value, bool) or not isinstance(value, int)
            or value < (0 if allow_zero else 1)):
        raise ValueError(f'{name} must be a positive integer.')
    return value


def image_pixels(message):
    """Validate an 8-bit ROS Image and return PNG-ready BGR/mono pixels."""
    width = _positive_integer(message.width, 'width')
    height = _positive_integer(message.height, 'height')
    step = _positive_integer(message.step, 'step')
    encoding = message.encoding.lower()
    if encoding not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError(f'Unsupported crop image encoding: {message.encoding!r}')
    channels = 1 if encoding == 'mono8' else 3
    if step < width * channels or len(message.data) != height * step:
        raise ValueError('Image step/data size is inconsistent with its dimensions.')
    pixels = np.ndarray(
        (height, width, channels), dtype=np.uint8, buffer=message.data,
        strides=(step, channels, 1))
    if encoding == 'rgb8':
        pixels = pixels[:, :, ::-1]
    elif encoding == 'mono8':
        pixels = pixels[:, :, 0]
    return np.ascontiguousarray(pixels)


class ImageArchive:
    """Queue message references without image processing in ROS callbacks.

    ``directory`` is the new recording session root. This object exclusively
    creates ``images/hrm_crop`` and never reuses an existing archive. Each PNG
    keeps its exact pixels; the index retains original source and receipt times.
    Stop closes admission immediately and the worker drains accepted frames.
    """

    def __init__(self, directory, topic='/estimated_segment_crop_image',
                 contact_segment_id=0, queue_size=16):
        _positive_integer(queue_size, 'queue_size')
        _positive_integer(contact_segment_id, 'contact_segment_id', allow_zero=True)
        if contact_segment_id > 18:
            raise ValueError('contact_segment_id must be between 0 and 18.')
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError('The image topic must be an absolute ROS topic name.')
        self.directory = Path(directory) / 'images' / 'hrm_crop'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.topic = topic
        self.contact_segment_id = contact_segment_id
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._stopping = False
        self._stats = dict(
            received=0, accepted=0, saved=0, dropped=0, dropped_queue_full=0,
            rejected_after_stop=0, errors=0, finalization_errors=0,
            png_bytes=0, first_source_time_ns=None, last_source_time_ns=None,
            first_received_time_ns=None, last_received_time_ns=None,
            nonincreasing_source_timestamps=0, largest_source_gap_ns=0,
        )
        self._error_details = []
        self._index_file = (self.directory / 'index.csv').open('x', newline='')
        self._index = csv.DictWriter(self._index_file, fieldnames=INDEX_FIELDS)
        try:
            self._index.writeheader()
            self._index_file.flush()
        except Exception:
            self._index_file.close()
            raise
        self._worker = threading.Thread(
            target=self._run, name='hrm-crop-png-writer', daemon=True)
        self._worker.start()

    @property
    def done(self):
        return self._done.is_set()

    def enqueue(self, message, received_ns):
        """Accept a frame without blocking; expose every admission rejection."""
        with self._lock:
            self._stats['received'] += 1
            if self._stopping:
                self._stats['rejected_after_stop'] += 1
                self._stats['dropped'] += 1
                return False
            sequence = self._stats['accepted'] + 1
            try:
                self._queue.put_nowait((sequence, message, received_ns))
            except queue.Full:
                self._stats['dropped_queue_full'] += 1
                self._stats['dropped'] += 1
                return False
            self._stats['accepted'] += 1
            return True

    def request_stop(self):
        """Close admission without waiting for PNG compression or disk I/O."""
        with self._lock:
            self._stopping = True

    def close(self, timeout=5.0):
        """For process teardown/tests: request stop and wait up to timeout."""
        self.request_stop()
        self._worker.join(timeout)
        return self.done

    def report(self):
        with self._lock:
            return {
                'topic': self.topic, 'format': 'png', 'lossless': True,
                'directory': 'images/hrm_crop', 'index': 'images/hrm_crop/index.csv',
                'contact_segment_id': self.contact_segment_id,
                'queue_capacity': self._queue.maxsize,
                'pending': self._stats['accepted'] - self._stats['saved']
                - self._stats['errors'],
                'done': self.done, 'stop_requested': self._stopping,
                **self._stats, 'error_details': list(self._error_details),
            }

    def _error(self, exc, *, finalization=False):
        with self._lock:
            key = 'finalization_errors' if finalization else 'errors'
            self._stats[key] += 1
            if len(self._error_details) < 8:
                self._error_details.append(f'{type(exc).__name__}: {exc}'[:400])

    def _save(self, sequence, message, received_ns):
        stamp = message.header.stamp
        sec = _positive_integer(stamp.sec, 'source sec', allow_zero=True)
        nanosec = _positive_integer(stamp.nanosec, 'source nanosec', allow_zero=True)
        if nanosec >= 1_000_000_000 or (sec == 0 and nanosec == 0):
            raise ValueError('Image source timestamp is invalid or unset.')
        if not isinstance(message.header.frame_id, str) or not message.header.frame_id:
            raise ValueError('Image header.frame_id must not be empty.')
        received_ns = _positive_integer(received_ns, 'received_ns')
        pixels = image_pixels(message)
        valid, encoded = cv2.imencode('.png', pixels, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        if not valid:
            raise OSError('OpenCV could not encode the crop image as PNG.')
        filename = f'{sequence:08d}_{sec}_{nanosec:09d}.png'
        with (self.directory / filename).open('xb') as output:
            output.write(encoded.tobytes())
        source_ns = sec * 1_000_000_000 + nanosec
        self._index.writerow(dict(
            source_time_ns=source_ns, received_time_ns=received_ns,
            frame_id=message.header.frame_id, filename=filename,
            width=message.width, height=message.height,
            input_encoding=message.encoding, payload_bytes=len(message.data),
            contact_segment_id=self.contact_segment_id,
        ))
        self._index_file.flush()
        with self._lock:
            previous = self._stats['last_source_time_ns']
            if previous is None:
                self._stats['first_source_time_ns'] = source_ns
                self._stats['first_received_time_ns'] = received_ns
            elif source_ns <= previous:
                self._stats['nonincreasing_source_timestamps'] += 1
            else:
                self._stats['largest_source_gap_ns'] = max(
                    self._stats['largest_source_gap_ns'], source_ns - previous)
            self._stats.update(last_source_time_ns=source_ns,
                               last_received_time_ns=received_ns)
            self._stats['saved'] += 1
            self._stats['png_bytes'] += int(encoded.nbytes)

    def _run(self):
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.05)
                except queue.Empty:
                    with self._lock:
                        if self._stopping and self._queue.empty():
                            break
                    continue
                try:
                    self._save(*item)
                except Exception as exc:
                    self._error(exc)
                finally:
                    self._queue.task_done()
        finally:
            try:
                self._index_file.close()
            except Exception as exc:
                self._error(exc, finalization=True)
            try:
                report = self.report()
                report['done'] = True
                with (self.directory / 'manifest.json').open('x') as output:
                    json.dump(report, output, indent=2)
                    output.write('\n')
            except Exception as exc:
                self._error(exc, finalization=True)
            self._done.set()
