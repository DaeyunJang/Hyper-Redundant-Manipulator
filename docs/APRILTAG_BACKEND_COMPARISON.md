# CPU AprilTag / Isaac ROS comparison — 2026-09-18

## Result so far

The isolated Isaac CUDA detector **runs on this PC**. This does not yet justify
replacing the production detector: both backends failed to detect ID1 in the
same previously captured real-camera sequence. A newly prepared scene still
needs to be captured with the operator's confirmation.

Input: 90 full RGB848x480 frames, approximately 10 Hz sampled over 9.19 seconds,
from `/camera/camera/color/image_rect_raw`, with exact-stamp color CameraInfo.
No crop, enlargement, sharpening, or frame changes between variants. The user
identified ID0 as the fixed base tag; the small upper tag is the ID1 target.
This is a **different scene** from the earlier 180-frame test described in
APRILTAG_TUNING.md. In this sequence the small tag is not hidden behind the HRM.

| Detector | Parameters | ID0 | ID1 | Response timeouts | Mean / p95 round trip |
|---|---|---|---|---|---|
| apriltag_ros 3.2.2 | decimate 1.0, max_hamming 0 | 90/90 | 0/90 | 0 | 15.59 / 17.07 ms |
| apriltag_ros 3.2.2 | decimate 1.0, max_hamming 1 | 90/90 | 0/90 | 0 | 15.56 / 16.94 ms |
| isaac_ros_apriltag 3.1.0 | size 0.017, max_tags 64, tile_size 4 | 90/90 | 0/90 | 0 | 3.60 / 5.06 ms |

All use tag36h11 and a declared 17 mm tag edge. CPU uses one thread, blur 0,
refine enabled and sharpening 0.25. Isaac 3.1's detector has no equivalent
decimate/max_hamming controls or reported hamming/decision-margin values.

Round trip measures Python publication → DDS → detector → receive callback.
It is **not** pure GPU/CPU compute time, camera latency, or live-camera FPS.
Only one image is outstanding at a time. Empty detection arrays count as valid
responses; transport timeouts are reported separately and never counted as
ordinary missing-tag images. ID1 pose stability cannot be evaluated here.

ID0's mean consecutive translation/rotation changes were 0.012 mm / 0.029°
(CPU) versus 0.789 mm / 1.169° (Isaac) on these same frames. This is an observed
sequence statistic, not a ground-truth accuracy score. Absolute tag orientation
also differs between implementations; do not replace the existing pose/TF
source or compare their axes without checking the tag-frame conventions.
Faster detection does not by itself establish better pose measurements.

An independent raw-result audit found that this Isaac sequence contained only
two distinct ID0 corner/pose sets: one corner alternated between x=334 and335
pixels, while the other corners stayed fixed; all returned corner coordinates
were integers. The two poses differed by 3.695 mm and 5.476°. CPU returned
subpixel corners. The harness does not round corners or poses. This evidence
links the large Isaac pose jumps in **this sequence** to its returned corner
sets; it does not establish that every image or newer Isaac release behaves
this way. The official 3.1 adapters copy corner/pose data without integer
rounding: [CUDA detector adapter](https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/release-3.1/gxf_isaac_fiducials/gxf/extensions/fiducials/components/cuda_april_tag_detector.cpp),
[NITROS output adapter](https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/release-3.1/isaac_ros_nitros_type/isaac_ros_nitros_april_tag_detection_array_type/src/nitros_april_tag_detection_array.cpp).

The nearly 180° difference in absolute tag orientation is consistent with
different tag-frame conventions, not proof of a 180° physical pose error.
Changing a fixed axis convention cannot remove the consecutive pose jumps.
No unverified axis correction was applied to either backend's output.

## Isolated local setup

- Ubuntu 22.04 / ROS Humble; RTX3060 12 GB; driver570.195.03; CUDA12.4.
- Official Isaac ROS3.1 packages: 23 Debian archives, 28.3 MB downloaded,
  extracted using `dpkg-deb -x` only, not installed into the system.
- Official VPI3.1.5 runtime: 70.7 MB, similarly extracted locally.
- Both archive sets checked against SHA256 values from their official HTTPS
  package indexes. Manifests are under `.isaac_ros_compare/`.
- `negotiated` and `negotiated_interfaces` built locally from
  osrf/negotiated commit `eac198b55dcd052af5988f0f174902913c5f20e7`, Release,
  tests off, `/usr/bin/python3`, two build workers.
- `.isaac_ros_compare/COLCON_IGNORE` keeps this experiment out of ordinary
  workspace builds; the whole directory is gitignored. These dependencies are
  **not included in a git clone** and must be separately prepared on another PC.
- No APT installation, CUDA/driver switch, camera restart, or production
  AprilTag/GUI/recorder/controller configuration change was made.

This unpacked-prefix setup is an **experimental local comparison environment**,
not NVIDIA's recommended Docker installation. Do not source it globally or add
it to the main launcher. The wrapper applies paths only to the requested child.

Isaac3.1 was selected for compatibility with the existing CUDA12.4 installation;
its documented minimum is CUDA12.2. Isaac3.2 documents CUDA12.6 or newer.
See [Isaac3.1 requirements](https://nvidia-isaac-ros.github.io/v/release-3.1/getting_started/hardware_setup/compute/index.html)
and [Isaac3.2 requirements](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_apriltag/index.html).
These results concern **3.1**, not the latest Isaac ROS release.

Local preparation records (not tracked): `stage_packages.py`, `manifest.json`,
`stage_vpi.py`, `vpi_runtime_manifest.json`, `elf_inspection.json` in
`.isaac_ros_compare/`. The AprilTag path has no unresolved ELF dependencies;
optional unrelated image-normalize/pad components still lack CV-CUDA and are
not part of this test. Do not launch those components with this minimal setup.

## Repeat a saved-frame comparison

Once the camera and static scene are ready, capture without changing camera
settings (use a new directory for every scene):

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/capture_apriltag_comparison.py \
  --output /tmp/hrm_tag_scene01 --frames 90 --sample-hz 10 --save-preview
```

This script subscribes only to the full RGB image and CameraInfo. It requires
exact matching source stamps, verifies constant calibration/geometry, and
returns a nonzero exit status for incomplete captures. Check
`capture_metadata.json` says `status: complete`; a timeout can leave a partial
NPZ for diagnosis and must not be mistaken for the requested full sequence.
Then use `/tmp/hrm_tag_scene01/capture.npz` in place of the older input below.

```bash
bash scripts/with_isaac_ros_compare.sh /usr/bin/python3 \
  scripts/compare_apriltag_backends.py \
  /tmp/hrm_apriltag_compare_lo3t4aw4/capture.npz \
  --domain 95 --output /tmp/apriltag_backend_new_result.json
```

Use a previously unused output filename. The harness refuses an occupied ROS
domain. Inputs, detections and TFs are private under
`/apriltag_backend_compare/*`; no motor/camera node is launched, no production
node parameters are changed, and only harness-owned processes are stopped.
`--backends cpu` runs hamming0/1, `--backends isaac` only CUDA, and `--limit 3`
allows a short smoke test. Warmup samples are excluded from the measured rows.
Three consecutive missing responses abort the test and save an error report.

The NPZ contract is RGB8 `images[N,H,W,3]`, increasing positive int64
`stamps_ns[N]`, scalar `camera_info_json`; geometry/intrinsics must stay constant.
Reports preserve per-image IDs, corners and poses, response status, timing,
missing-ID1 runs, and ID1 relative to ID0 when both are available in one frame.
Pose axes are not silently corrected to make backends agree.

## Artifacts and checks

- Real input: `/tmp/hrm_apriltag_compare_lo3t4aw4/capture.npz`
- Real comparison: `/tmp/hrm_apriltag_backends_existing90.json`
- GPU synthetic check: `/tmp/hrm_isaac_smoke_02.json` — two clean-tag frames
  returned ID0/ID1; the blank frame returned no tags; zero response timeouts.
- CPU one-bit-damage check: `/tmp/hrm-cpu-backend-fUpQz9/comparison.json` —
  ID1 1/3 at hamming0, 2/3 at hamming1; zero response timeouts.
- Pure metric/pose tests: `scripts/test_compare_apriltag_backends.py`.
- Capture and backend tests: 32 passed, including matching, calibration-change
  rejection, RGB conversion, sampling, partial-timeout status and process cleanup.
  Scoped Flake8(max100), wrapper shell syntax and git diff whitespace checks pass.
  The new capture helper has not yet been run against the newly prepared scene.

Temporary files are not permanent datasets and may disappear after cleanup.
For the next capture, hold ID0 and ID1 still with all borders visible, first
without glare and then under the troublesome lighting. Compare the same saved
frames across backends. Only a confirmed static scene supports interpreting
pose variation as stability, and neither scene supplies ground-truth accuracy.
