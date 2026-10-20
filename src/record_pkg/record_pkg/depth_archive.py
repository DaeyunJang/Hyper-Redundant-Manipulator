"""Lossless cropped aligned-depth archive with per-frame calibration."""

from collections import OrderedDict
import csv
import json
import math
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np

from record_pkg.image_archive import _positive_integer


DEPTH_INDEX_FIELDS = (
    'source_time_ns', 'received_time_ns', 'frame_id', 'filename', 'width', 'height',
    'input_encoding', 'payload_bytes', 'contact_segment_id',
    'depth_scale_m_per_unit', 'roi_x', 'roi_y', 'roi_width', 'roi_height',
    'source_width', 'source_height', 'k', 'p', 'r', 'd', 'distortion_model',
)


def depth_pixels(message):
    """Decode raw depth with row stride and endian; never normalize or filter."""
    width = _positive_integer(message.width, 'width')
    height = _positive_integer(message.height, 'height')
    step = _positive_integer(message.step, 'step')
    if message.encoding not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError('Unsupported depth encoding.')
    if message.is_bigendian not in (0, 1):
        raise ValueError('Depth is_bigendian must be 0 or 1.')
    kind = 'f4' if message.encoding == '32FC1' else 'u2'
    dtype = np.dtype(('>' if message.is_bigendian else '<') + kind)
    if step < width * dtype.itemsize or len(message.data) != height * step:
        raise ValueError('Depth step/data size differs from dimensions.')
    pixels = np.ndarray((height, width), dtype=dtype, buffer=message.data,
                        strides=(step, dtype.itemsize))
    # PNG encoder expects native uint16; NPY preserves floating NaN/Inf values.
    return np.ascontiguousarray(pixels, dtype=np.dtype(kind))


def validate_depth_metadata(value):
    """Validate explicit units/intrinsics; do not infer scale from filename."""
    if not isinstance(value, dict) or value.get('schema_version') != 1:
        raise ValueError('Invalid depth metadata schema.')
    for name in ('source_time_ns', 'width', 'height', 'source_width', 'source_height'):
        _positive_integer(value[name], name)
    if not isinstance(value['frame_id'], str) or not value['frame_id']:
        raise ValueError('Depth metadata needs a frame_id.')
    if value['encoding'] not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError('Invalid depth metadata encoding.')
    scale = value['depth_scale_m_per_unit']
    if isinstance(scale, bool) or not isinstance(scale, (int, float)):
        raise ValueError('Depth scale must be numeric.')
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Depth scale must be positive and finite.')
    if value.get('intrinsics_reference') != 'cropped_pixels':
        raise ValueError('Depth intrinsics must refer to cropped pixels.')
    roi = value['roi']
    for name in ('x', 'y'):
        _positive_integer(roi[name], name, allow_zero=True)
    if (roi['width'] != value['width'] or roi['height'] != value['height']
            or roi['x'] + value['width'] > value['source_width']
            or roi['y'] + value['height'] > value['source_height']):
        raise ValueError('Depth ROI/dimensions differ.')
    for name, size in (('k', 9), ('p', 12), ('r', 9), ('d', None)):
        numbers = value[name]
        if (not isinstance(numbers, list) or (size is not None and len(numbers) != size)
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in numbers)):
            raise ValueError(f'Invalid depth calibration {name}.')
    if value['k'][0] <= 0 or value['k'][4] <= 0:
        raise ValueError('Invalid depth focal length.')
    if not isinstance(value['distortion_model'], str):
        raise ValueError('Invalid distortion model.')
    return value


class DepthArchive:
    """Bounded worker archive: 16-bit PNG or float32 NPY plus exact calibration."""

    def __init__(self, directory, topic='/estimated_segment_crop_depth/image_raw',
                 contact_segment_id=0, queue_size=16):
        _positive_integer(queue_size, 'queue_size')
        _positive_integer(contact_segment_id, 'contact_segment_id', allow_zero=True)
        if contact_segment_id > 18 or not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError('Invalid depth archive label/topic.')
        self.directory = Path(directory) / 'images' / 'hrm_crop_depth'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.topic, self.contact_segment_id = topic, contact_segment_id
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._calibration_ready = threading.Condition(self._lock)
        self._calibrations = OrderedDict()
        self._done = threading.Event()
        self._stopping = False
        self._stats = dict(received=0, accepted=0, saved=0, dropped=0,
                           dropped_queue_full=0, rejected_after_stop=0, errors=0,
                           calibration_errors=0, finalization_errors=0, bytes_written=0,
                           first_source_time_ns=None, last_source_time_ns=None,
                           nonincreasing_source_timestamps=0, largest_source_gap_ns=0)
        self._error_details = []
        self._index_file = (self.directory / 'index.csv').open('x', newline='')
        self._index = csv.DictWriter(self._index_file, fieldnames=DEPTH_INDEX_FIELDS)
        try:
            self._index.writeheader()
            self._index_file.flush()
        except Exception:
            self._index_file.close()
            raise
        self._worker = threading.Thread(target=self._run, name='hrm-depth-writer', daemon=True)
        self._worker.start()

    @property
    def done(self):
        return self._done.is_set()

    def update_metadata(self, message):
        try:
            value = message if isinstance(message, dict) else json.loads(message.data)
            value = validate_depth_metadata(value)
            # Own a small immutable-by-convention snapshot, not the caller's dict.
            value = json.loads(json.dumps(value, allow_nan=False))
            key = (value['source_time_ns'], value['frame_id'])
            with self._calibration_ready:
                self._calibrations[key] = value
                self._calibrations.move_to_end(key)
                while len(self._calibrations) > 64:
                    self._calibrations.popitem(last=False)
                self._calibration_ready.notify_all()
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self._error(error, 'calibration_errors')
            return False

    def enqueue(self, message, received_ns):
        with self._lock:
            self._stats['received'] += 1
            if self._stopping:
                self._stats['rejected_after_stop'] += 1
                self._stats['dropped'] += 1
                return False
            try:
                self._queue.put_nowait((self._stats['accepted'] + 1, message, received_ns))
            except queue.Full:
                self._stats['dropped_queue_full'] += 1
                self._stats['dropped'] += 1
                return False
            self._stats['accepted'] += 1
            return True

    def request_stop(self):
        with self._lock:
            self._stopping = True

    def close(self, timeout=5.0):
        self.request_stop()
        self._worker.join(timeout)
        return self.done

    def report(self):
        with self._lock:
            return dict(
                topic=self.topic, directory='images/hrm_crop_depth',
                index='images/hrm_crop_depth/index.csv', format='uint16_png_or_float32_npy',
                lossless=True, contact_segment_id=self.contact_segment_id,
                queue_capacity=self._queue.maxsize, done=self.done,
                stop_requested=self._stopping, error_details=list(self._error_details),
                pending=self._stats['accepted'] - self._stats['saved'] - self._stats['errors'],
                **self._stats)

    def _error(self, error, key='errors'):
        with self._lock:
            self._stats[key] += 1
            if len(self._error_details) < 8:
                self._error_details.append(f'{type(error).__name__}: {error}'[:400])

    def _metadata(self, key):
        deadline = time.monotonic() + 0.25
        with self._calibration_ready:
            while key not in self._calibrations:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ValueError('No matching depth source-stamp/frame calibration.')
                self._calibration_ready.wait(remaining)
            return self._calibrations[key]

    def _save(self, sequence, message, received_ns):
        _positive_integer(received_ns, 'received_ns')
        stamp = message.header.stamp
        _positive_integer(stamp.sec, 'sec', allow_zero=True)
        _positive_integer(stamp.nanosec, 'nanosec', allow_zero=True)
        source = stamp.sec * 1_000_000_000 + stamp.nanosec
        if not source or stamp.nanosec >= 1_000_000_000 or not message.header.frame_id:
            raise ValueError('Invalid depth image header.')
        pixels = depth_pixels(message)
        calibration = self._metadata((source, message.header.frame_id))
        if (calibration['width'] != message.width or calibration['height'] != message.height
                or calibration['encoding'] != message.encoding):
            raise ValueError('Depth image and calibration dimensions/encoding differ.')
        extension = 'npy' if message.encoding == '32FC1' else 'png'
        filename = f'{sequence:08d}_{stamp.sec}_{stamp.nanosec:09d}.{extension}'
        path = self.directory / filename
        with path.open('xb') as output:
            if extension == 'npy':
                np.save(output, pixels, allow_pickle=False)
            else:
                valid, encoded = cv2.imencode('.png', pixels, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                if not valid:
                    raise OSError('Depth PNG encoding failed.')
                output.write(encoded.tobytes())
        row = dict(source_time_ns=source, received_time_ns=received_ns,
                   frame_id=message.header.frame_id, filename=filename,
                   width=message.width, height=message.height, input_encoding=message.encoding,
                   payload_bytes=len(message.data), contact_segment_id=self.contact_segment_id,
                   depth_scale_m_per_unit=calibration['depth_scale_m_per_unit'],
                   source_width=calibration['source_width'],
                   source_height=calibration['source_height'],
                   distortion_model=calibration['distortion_model'])
        row.update({f'roi_{k}': v for k, v in calibration['roi'].items()})
        row.update({k: json.dumps(calibration[k]) for k in ('k', 'p', 'r', 'd')})
        self._index.writerow(row)
        self._index_file.flush()
        written_size = path.stat().st_size
        with self._lock:
            previous = self._stats['last_source_time_ns']
            if previous is not None:
                if source <= previous:
                    self._stats['nonincreasing_source_timestamps'] += 1
                else:
                    self._stats['largest_source_gap_ns'] = max(
                        self._stats['largest_source_gap_ns'], source - previous)
            if self._stats['first_source_time_ns'] is None:
                self._stats['first_source_time_ns'] = source
            self._stats['last_source_time_ns'] = source
            self._stats['saved'] += 1
            self._stats['bytes_written'] += written_size

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
                except Exception as error:
                    self._error(error)
                finally:
                    self._queue.task_done()
        finally:
            try:
                self._index_file.close()
                report = self.report()
                report['done'] = True
                with (self.directory / 'manifest.json').open('x') as output:
                    json.dump(report, output, indent=2)
            except Exception as error:
                self._error(error, 'finalization_errors')
            self._done.set()
