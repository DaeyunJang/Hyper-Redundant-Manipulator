# AprilTag ID1 detection comparison — 2026-09-18

Later CPU versus Isaac CUDA measurements on a **different 90-frame scene** are
in [APRILTAG_BACKEND_COMPARISON.md](APRILTAG_BACKEND_COMPARISON.md). The occlusion
observation below applies only to this earlier 180-frame capture.

## Setting at the time of this comparison

The runtime tag size was later corrected to **20 mm for ID0/ID1 (2026-09-26)**.
The comparison below records the original 17 mm configuration, not today's size.

`src/launcher/config/apriltag.yaml` now uses `detector.decimate: 1.0` and
`max_hamming: 0`. All other detector parameters remain unchanged: one thread,
blur 0, refine enabled, sharpening 0.25, tag36h11 IDs 0/1 and 17 mm tag size.
The installed launch config is a symlink to this file; rebuilding is not needed
for this YAML change. The already-running `/apriltag_node` also accepted the
decimate update through its parameter service, and readback verified `1.0/0`.
No node restart, camera exposure/profile change, or motor command was performed.

## Same-image comparison

Read-only capture from the running camera collected 180 RGB images over 17.905 s
(approximately 10 Hz sampled from 538 incoming color frames). Every saved image
had a CameraInfo message with exactly the same source timestamp. Images were
848×480 from `/camera/camera/color/image_rect_raw`; no crop or enhancement was
applied. The source live detector was confirmed to be `decimate=2.0/hamming=0`.

`scripts/compare_apriltag_settings.py` replayed the **same images** through the
installed `apriltag_ros` 3.2.2 C++ detector, sequentially in isolated domain 93.
All test inputs, detections and TFs were remapped to `/apriltag_compare/*`.
The real detector and camera continued running during the offline comparison.

| Quad decimation | Max accepted corrected bits | ID0 frames | ID1 frames | Response timeouts | Mean round trip |
|---|---|---|---|---|---|
| 2.0 | 0 | 180/180 | 0/180 | 0 | 5.10 ms |
| 1.0 | 0 | 180/180 | 0/180 | 0 | 14.20 ms |
| 1.0 | 1 | 180/180 | 0/180 | 0 | 14.07 ms |

Round trip includes Python publication, serialization/transport, detector work
and reception callback. It is **not** pure detector latency, a live throughput
benchmark, or an estimate of camera capture latency. ID1 had no detections, so
its pose jitter/accuracy could not be compared. ID0 had only hamming=0 results;
consecutive rotation steps averaged approximately 0.03 degrees for all three
variants (scene movement and measurement noise are not separated).

After applying `decimate=1.0/hamming=0` to the live detector, a separate six-second
observation received 180 unique detection-array timestamps: ID0 in all 180,
ID1 in none. Thus live processing continued at approximately 30 Hz, but the ID1
failure was **not resolved** by these settings in the observed scene.

## Interpretation and next check

The captured view shows a large, detectable ID0 near the bottom, and another
small angled tag partly behind the HRM body. That small tag is suspected to be
ID1; the user must confirm its identity and expose all four borders before a
reflection-only comparison. The measurements do not prove glare alone caused
the failure, and cannot isolate border detection failure from larger decoding
damage without further images/debug output.

`max_hamming` is an acceptance filter after decoding. Raising it to 1 can admit
one-bit-corrected detections, but cannot repair a missing quadrilateral or
arbitrarily damaged pattern. No benefit was observed on this capture, so the
default remains 0. First make the entire tag visible, then repeat on examples
with and without reflection, while keeping geometry/lighting comparable.

The comparison tool was independently checked using three synthetic frames:
clean tags, an ID1 with one flipped data bit, and a blank image. ID1 counts were
1/3 at 2.0/0 and 1.0/0, versus 2/3 at 1.0/1, with the extra detection reporting
hamming=1. Every variant returned all three detection arrays. This verifies the
test distinguishes missed detections from missing responses and exercises the
actual installed decoder's corrected-bit filter.

## Local artifacts / repeat

Artifacts are temporary local files, not committed datasets:

- `/tmp/hrm_apriltag_compare_11b7bizt/capture.npz`
- `/tmp/hrm_apriltag_compare_11b7bizt/capture_metadata.json`
- `/tmp/hrm_apriltag_compare_11b7bizt/first_rgb.png`
- `/tmp/hrm_apriltag_compare_11b7bizt/comparison.json`
- `/tmp/hrm-apriltag-smoke-b34hwxl5/comparison_one_bit.json`

Run after sourcing ROS and the workspace, using a free isolated domain:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 \
  scripts/compare_apriltag_settings.py \
  /tmp/hrm_apriltag_compare_11b7bizt/capture.npz --domain 93
```

The script refuses occupied domains and existing output paths. It creates and
cleans up only its own detector child processes. A saved NPZ requires RGB8
`images[N,H,W,3]`, strictly increasing `int64 stamps_ns[N]`, and scalar string
`camera_info_json` containing `message_to_ordereddict(CameraInfo)` JSON. The
camera geometry/frame must stay constant across that capture. Three consecutive
response timeouts abort the test; an empty detection array is not a timeout.
