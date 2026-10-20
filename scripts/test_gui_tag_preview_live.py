#!/usr/bin/env python3
"""Read-only Qt preview smoke test against already running camera topics.

Does not launch/stop cameras, controllers, motors or recording. Use a new output
directory. QT_QPA_PLATFORM=offscreen works without changing the operator's GUI.
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import statistics
import time


def run(args):
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    os.environ.setdefault('ROS_LOG_DIR', str(output / 'ros_logs'))
    import rclpy
    from rclpy.context import Context
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication
    from gui_py_pkg.image_preview import ImagePreviewPanel, header_key

    context = Context()
    rclpy.init(context=context)
    app = QApplication([])
    panel = None
    counts, status_counts, costs = Counter(), Counter(), []
    tag_ids, overlay_stamps, overlay_mismatches = Counter(), set(), []
    result = {'status': 'error', 'read_only': True, 'dds_environment': {
        key: os.environ.get(key) for key in (
            'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE')}}
    started = time.monotonic()
    cpu_started = time.process_time()
    try:
        panel = ImagePreviewPanel(context)
        panel.resize(args.width, args.height)
        for key, canvas in panel.canvases.items():
            original = canvas.set_rgb

            def render(pixels, name=key, setter=original):
                setter(pixels)
                counts[name] += 1
                frame = getattr(panel, 'tag_frame', None)
                if name == 'tag' and frame is not None and frame.detections is not None:
                    image_key, detection_key = header_key(frame.image), header_key(frame.detections)
                    if (image_key is None or image_key != detection_key
                            or frame.source != 'rectified'):
                        overlay_mismatches.append([image_key, detection_key, frame.source])
                    else:
                        overlay_stamps.add(image_key)
                    tag_ids.update(str(tag.id) for tag in frame.detections.detections)

            canvas.set_rgb = render
        original_refresh = panel.refresh

        def refresh():
            before = time.perf_counter()
            original_refresh()
            costs.append((time.perf_counter() - before) * 1000)
            if 'tag' in panel.status_labels:
                status_counts[panel.status_labels['tag'].text()] += 1

        panel.timer.timeout.disconnect()
        panel.timer.timeout.connect(refresh)
        panel.show()
        signal.signal(signal.SIGINT, lambda *_: app.quit())
        signal.signal(signal.SIGTERM, lambda *_: app.quit())
        QTimer.singleShot(int(args.duration * 1000), app.quit)
        app.exec_()
        elapsed = time.monotonic() - started
        saved = panel.grab().save(str(output / 'preview.png'))
        for key, canvas in panel.canvases.items():
            if not canvas.image.isNull():
                canvas.image.save(str(output / f'{key}.png'))
        result.update(
            status=('complete' if counts['tag'] > 0 and saved and not overlay_mismatches
                    and (not args.require_overlay or (tag_ids['0'] and tag_ids['1']))
                    else 'failed'),
            seconds=elapsed, process_cpu_seconds=time.process_time() - cpu_started,
            rendered_frames=dict(counts),
            overlay_tag_counts=dict(tag_ids), unique_overlay_frames=len(overlay_stamps),
            overlay_mismatches=overlay_mismatches,
            render_rate_hz={key: value / elapsed for key, value in counts.items()},
            tag_status_counts=dict(status_counts),
            final_status={key: label.text() for key, label in panel.status_labels.items()},
            refresh_ms={'mean': statistics.mean(costs) if costs else None,
                        'max': max(costs) if costs else None,
                        'p95': sorted(costs)[int(0.95 * (len(costs) - 1))] if costs else None},
            panel_size=[panel.width(), panel.height()], screenshot=str(output / 'preview.png'),
        )
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        if panel is not None:
            panel.stop()
            panel.close()
        if context.ok():
            rclpy.shutdown(context=context)
        (output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['status'] == 'complete' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--width', type=int, default=1000)
    parser.add_argument('--height', type=int, default=260)
    parser.add_argument('--require-overlay', action='store_true')
    arguments = parser.parse_args()
    if not 0 < arguments.duration <= 30 or not 500 <= arguments.width <= 2000:
        parser.error('Use duration in (0,30] seconds and width in [500,2000].')
    raise SystemExit(run(arguments))
