#!/usr/bin/env python3
# AUTHOR: Daeyun - HRM offline segmentation export.
"""Export the current RBSC pre-skeleton masks without ROS playback or depth use.

Source ROS/workspace, then run with HRM_env Python and the dataset-parent path.
Each <session>/images/hrm_crop becomes a NEW images/hrm_crop_segmentation.
PNG filenames and source index columns are preserved; derived fields are added.
Existing output directories are never overwritten. --inspect-only writes nothing.
This reproduces current postprocess code, not necessarily the recording-date code.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import inspect
import json
from pathlib import Path
import sqlite3
import sys
import time


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def inspect_sessions(root):
    sessions = []
    for source in sorted(root.glob('*/images/hrm_crop')):
        session = source.parent.parent
        with (source / 'index.csv').open(newline='') as stream:
            reader = csv.DictReader(stream)
            fields = list(reader.fieldnames or [])
            rows = list(reader)
        names = [row['filename'] for row in rows]
        if (not names or len(set(names)) != len(names)
                or any(Path(name).name != name or Path(name).suffix.lower() != '.png'
                       for name in names)):
            raise ValueError(f'Invalid/empty/duplicate filenames in {source}/index.csv')
        if set(names) != {p.name for p in source.glob('*.png')}:
            raise ValueError(f'Index/PNG inventory mismatch: {source}')
        image_topics = []
        bags = sorted((session / 'bag').glob('*.db3'))
        for bag in bags:
            # URI read-only; no ROS replay and no write to bag metadata/data.
            with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as db:
                for topic_id, name, message_type in db.execute('SELECT id,name,type FROM topics'):
                    if ('Image' in message_type or 'binary' in name or 'skeleton' in name):
                        count = db.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',
                                           (topic_id,)).fetchone()[0]
                        image_topics.append(dict(bag=bag.name, topic=name,
                                                 type=message_type, count=count))
        sessions.append(dict(session=session, source=source, rows=rows, fields=fields,
                             output=source.with_name('hrm_crop_segmentation'),
                             bags=[str(p.relative_to(session)) for p in bags],
                             bag_image_topics=image_topics))
    if not sessions:
        raise ValueError(f'No */images/hrm_crop directories under {root}')
    return sessions


def export_session(info, rbsc, provenance):
    import cv2
    import numpy as np

    source, output = info['source'], info['output']
    output.mkdir(exist_ok=False)
    fields = info['fields'] + ['output_filename', 'segmentation_status',
                              'foreground_pixels', 'mask_encoding', 'mask_png_bytes',
                              'source_sha256', 'mask_sha256']
    saved, png_bytes = 0, 0
    failures = []
    minimum_area, maximum_area = None, 0
    started = time.perf_counter()
    with (output / 'index.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for ordinal, original_row in enumerate(info['rows'], 1):
            row = dict(original_row)
            filename = row['filename']
            data = (source / filename).read_bytes()
            row['source_sha256'] = sha256(data)
            # Archive PNG is standard color. imdecode returns BGR even when the
            # original ROS/index input_encoding was rgb8; do not swap it again.
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            error = None
            if (image is None or image.dtype != np.uint8
                    or image.shape != (int(row['height']), int(row['width']), 3)):
                error = 'Unreadable or unexpected crop shape/dtype; expected uint8 BGR.'
            else:
                mask = rbsc.postprocess(image, color_encoding='bgr8', segmentation_only=True)
                if mask is None:
                    error = rbsc.last_error or 'Segmentation failed'
            if error:
                # No silent stale-mask reuse or fabricated all-black success.
                row.update(output_filename='', segmentation_status=error)
                failures.append(dict(filename=filename, error=error))
            else:
                if (mask.shape != image.shape[:2] or mask.dtype != np.uint8
                        or not np.all((mask == 0) | (mask == 255))):
                    raise ValueError(f'Not a same-size binary mask: {filename}')
                ok, encoded = cv2.imencode('.png', mask, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                if not ok:
                    raise OSError(f'PNG encoding failed: {filename}')
                with (output / filename).open('xb') as target:
                    target.write(encoded.tobytes())
                decoded = cv2.imread(str(output / filename), cv2.IMREAD_UNCHANGED)
                if not np.array_equal(decoded, mask):
                    raise OSError(f'PNG round-trip verification failed: {filename}')
                area = int(np.count_nonzero(mask))
                minimum_area = area if minimum_area is None else min(minimum_area, area)
                maximum_area = max(maximum_area, area)
                saved += 1
                png_bytes += encoded.nbytes
                row.update(output_filename=filename, segmentation_status='ok',
                           foreground_pixels=area, mask_encoding='mono8',
                           mask_png_bytes=encoded.nbytes, mask_sha256=sha256(encoded))
            writer.writerow(row)
            if ordinal % 500 == 0:
                stream.flush()
                print(f"{info['session'].name}: {ordinal}/{len(info['rows'])}", flush=True)
    manifest = dict(
        created_utc=datetime.now(timezone.utc).isoformat(),
        source_directory='images/hrm_crop', directory='images/hrm_crop_segmentation',
        stage='RBSC.body_image after smoothing, immediately before skeletonization',
        source_index_sha256=sha256((source / 'index.csv').read_bytes()),
        input_count=len(info['rows']), saved=saved, failed=len(failures),
        png_bytes=png_bytes, foreground_pixels_min=minimum_area,
        foreground_pixels_max=maximum_area, encoding='mono8', binary_values=[0, 255],
        background=0, body=255, png_compression=3,
        same_filenames=True, same_pixel_dimensions=True,
        source_index_columns_preserved=info['fields'],
        note='input_encoding/payload_bytes describe the original RGB index; mask_* describe outputs.',
        depth_used=False, skeletonization_performed=False, crop_applied_again=False,
        historical_online_exactness_claimed=False,
        bag_files_checked=info['bags'], bag_image_topics=info['bag_image_topics'],
        algorithm=provenance, failures=failures,
        elapsed_sec=round(time.perf_counter() - started, 3))
    with (output / 'manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(dict(session=info['session'].name, saved=saved, failed=len(failures),
                          png_bytes=png_bytes, output=str(output)), ensure_ascii=False), flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset_root', type=Path)
    parser.add_argument('--inspect-only', action='store_true')
    args = parser.parse_args()
    root = args.dataset_root.resolve(strict=True)
    sessions = inspect_sessions(root)
    for info in sessions:
        print(json.dumps(dict(session=info['session'].name, crops=len(info['rows']),
                              bag_image_topics=info['bag_image_topics'])), flush=True)
    if args.inspect_only:
        return
    # Preflight every destination before creating any files; no overwrite option.
    for info in sessions:
        if info['output'].exists():
            raise FileExistsError(f"Output already exists; not overwriting: {info['output']}")
    repository = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repository / 'src/estimation_pkg'))
    import cv2
    from estimation_pkg.postprocess import RBSC
    cv2.setNumThreads(1)
    # This stage uses no config/depth/ROS state; avoid loading unrelated 3D config.
    rbsc = RBSC.__new__(RBSC)
    source_file = Path(inspect.getsourcefile(RBSC)).resolve()
    provenance = dict(source=str(source_file.relative_to(repository)),
                      source_sha256=sha256(source_file.read_bytes()),
                      implementation='RBSC.postprocess(segmentation_only=True)',
                      binary_threshold=inspect.signature(RBSC.postprocess).parameters['binary_thresh'].default,
                      smoothing_close_kernel=15, smoothing_blur_kernel=15,
                      smoothing_threshold=127, opencv_version=cv2.__version__)
    results = [export_session(info, rbsc, provenance) for info in sessions]
    print(f"TOTAL: saved={sum(r['saved'] for r in results)}, "
          f"failed={sum(r['failed'] for r in results)}, "
          f"PNG bytes={sum(r['png_bytes'] for r in results)}", flush=True)
    if any(result['failed'] for result in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
