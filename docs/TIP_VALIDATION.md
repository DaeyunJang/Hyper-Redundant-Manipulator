# Passive tip position/orientation validation

## 2026-09-21 stationary capture

No camera, estimator, controller, serial or TCP node was started/restarted.
No motor command, mode switch, enable, zeroing, or control service was issued.
The operator's already-running system was observed. D455 tag-camera runtime
parameters reported RGB 1280x720x30; no camera settings were changed.

Primary session (20 s requested, including recorder discovery):

`record/tip_validation/20260921_123441_835414/`

- Telemetry-only bag and per-topic CSV: 43 MB at initial export.
- Tag ID0->ID1: 563 samples, 29.986 Hz, maximum source interval 33.350 ms.
- Depth fitted tip and angle-derived FK tip: 565 samples each, 29.992 Hz,
  maximum source interval 33.344 ms.
- Four loadcells: 3963 samples, 199.997 Hz; both F/T streams: 3964 samples.
- Motor feedback: 4796 samples. Motor-command messages observed/recorded: zero.
- Continuity audit passed, no recorder cache-loss warning. This is a capture
  check, not a certification of sensor calibration or absolute accuracy.
- Images/PointCloud2 were intentionally excluded from this comparison session;
  camera calibration, TF, MarkerArray, angles, sensors and motor states retained.

### Full-image recording limitation found

Earlier session `record/tip_validation/20260921_123302_373699/` is retained as
an **incomplete diagnostic** (about 1.5 GB). Recording all original/debug images
with the current 64 MB rosbag cache and resilient sqlite profile lost **27,873
messages in the recorder cache**. All required topics were present, but the
continuity audit failed. A separate lightweight observer still received 600
tag poses, 600 estimated tips and 600 FK tips in 20 s; the bag retained only
178/222/204 respectively. This establishes a recording-path bottleneck, not a
camera/estimation-rate failure in that trial.

Do not use the full-image session as a complete synchronized training dataset.
The production recorder configuration was NOT silently weakened or changed.
Before full RGB/depth training collection, separately benchmark image selection,
storage throughput, cache size and sqlite transaction/profile tradeoffs; a
larger cache alone cannot cure indefinitely insufficient disk throughput.

## Files and coordinate meanings

The session's `csv/manifest.json` maps each topic to its original full-field CSV.
`analysis/` contains easier-named position/orientation CSVs and numerical results
from `scripts/analyze_tip_capture.py`.
The primary session's `RESULTS.md` is the Korean result summary with CSV links:
same-source-stamp depth/FK disagreement mean 0.2473 mm, RMS 0.2945 mm, maximum
0.9067 mm; these are NOT calibrated tag accuracy. Both tags were detected in
all 565 captured detection arrays. Pose rows are slightly fewer because topic
discovery/recording windows differ.

- Tag pose: `^ID0 T_ID1`, **not** the physical tip in hrm_base.
- Estimated tip: final fitted boundary, in hrm_base, after the existing hardware
  arc-length constraint (82.27 mm). It is not the last segment center.
- Unscaled tip: fitted endpoint before hardware-length scaling, still smoothed
  and transformed to hrm_base; not raw depth ground truth.
- FK tip: link/axis model evaluated with filtered measured angles, in hrm_base.
- `estimated_segment_center_19` TF gives the last link's accumulated DH
  orientation, but its position is the link **center**. Do not substitute that
  position for the tip. The FK tip uses the same angle-derived rotation.
- Last boundary analytic tangent comes from the `curve_tangents` ARROW endpoint
  difference, not the marker's identity `pose.orientation`. A tangent observes
  direction only: axial roll is not independently measured.

Proper marker compensation, to be provided by the user, is:

```text
^base T_tip(tag) = ^base T_ID0 * ^ID0 T_ID1 * ^ID1 T_tip
```

Both mounting **rotation and translation** can matter. Raw tag/base XYZ or Euler
subtraction is not physical tip error until these transforms are known. Euler
columns use extrinsic XYZ roll/pitch/yaw, not the controller's pan/tilt scalars.

Same-frame depth-versus-FK difference is a reconstruction/model consistency
measure, not independent ground truth. Cross-camera nearest-source-time pairing
is bounded and reports skew, but does not calibrate camera clock offset, exposure
time, filter lag or transport delay. Missing samples are never filled.

## Repeat without commanding motors

With the intended nodes already running and the GUI recorder idle:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/capture_tip_validation.py --seconds 20 --telemetry-only
python3 scripts/analyze_tip_capture.py record/tip_validation/<session>
```

The capture script only subscribes and reads parameter metadata, starts its own
rosbag recorder, and stops/exports that recorder. It does not disable an existing
active controller; the operator must leave the mechanism stationary and avoid
GUI motion commands. It refuses an already-running bag recorder. Normal recorder
requirements remain unchanged; this explicitly marked diagnostic capture permits
missing topics and reports them rather than inventing samples.
