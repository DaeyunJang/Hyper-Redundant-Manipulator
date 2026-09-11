# Codex Handoff

Last updated: 2026-09-11 (Asia/Seoul)

## Purpose

This document carries project decisions and implementation state between Codex
sessions and computers. Read it together with the current source, `git status`,
`git diff`, and recent `git log`; the repository is the source of truth when
this document and the code differ.

## Robot and control model

- The continuum section has two controlled bending DOFs: pan and tilt.
- East/West cables form the antagonistic pair for pan.
- South/North cables form the antagonistic pair for tilt.
- The first revolute joint at the fixed base is pan. For the 18 one-axis
  joints, paper/D-H indices therefore alternate as
  `q1=pan, q2=tilt, ..., q17=pan, q18=tilt` (zero-based even indices are pan).
- The physical centerline contains one fixed proximal segment `os` followed by
  18 moving-link directions, for 19 geometric segments and 20 boundary points.
  The kinematic chain has 18 revolute joints (nine pan/tilt pairs); `q1` acts
  after the fixed `os` segment.
- X is the tool's axial direction. Its position is calculated but is not a
  position-controller input.
- End-effector Y is the pan feedback direction.
- End-effector Z is the tilt feedback direction.
- There is currently no insertion-axis control.
- `PositionController` independently uses Y and Z errors:
  - `x_desired(1) - x_actual(1)` -> pan PID -> `del_theta_pan_`
  - `x_desired(2) - x_actual(2)` -> tilt PID -> `del_theta_tilt_`

## Segment-angle interface decision

Pan/tilt angles and velocities are grouped into one synchronized custom message
instead of separate topics.

- Message: `custom_interfaces/msg/SegmentAngle.msg`
- Topic: `estimated_segment_angle`
- Every array is ordered from base to end-effector and must contain
  `NUM_OF_BENDING_JOINTS` elements.
- Units:
  - angle: rad
  - angular velocity: rad/s

Message fields:

```text
std_msgs/Header header
float64[] pan_relative
float64[] pan_absolute
float64[] tilt_relative
float64[] tilt_absolute
float64[] pan_angular_velocity_relative
float64[] pan_angular_velocity_absolute
float64[] tilt_angular_velocity_relative
float64[] tilt_angular_velocity_absolute
```

`ControlNode` initializes and validates all arrays and receives them through a
single subscriber. The RBSC estimator now publishes this message after each
successful 3D reconstruction. Every array has 18 entries. Relative arrays put
the projected DH angle in the active alternating axis and zero in the inactive
axis. Absolute pan/tilt are the Base-frame azimuth/elevation of each projected
outgoing segment. With the joint Kalman filter enabled, relative joint angular
velocities are the filter's `q_dot` states; absolute direction-angle velocities
still use wrapped temporal finite differences. With Kalman disabled, all four
velocity arrays use wrapped temporal finite differences. The first valid
message has zero velocities.

## Estimation repository layout

- The runtime ROS package was copied from
  `src/LSTM-force-estimation/src/estimation_pkg` to the first-class package
  `src/estimation_pkg` in this repository.
- Develop runtime segment-angle estimation, external-force inference, ROS
  publishers, and deployment-model loading in `src/estimation_pkg` from now on.
- Keep datasets, training scripts, multi-model experiments, comparisons, and
  research checkpoints in the `src/LSTM-force-estimation` submodule.
- `src/LSTM-force-estimation/COLCON_IGNORE` prevents colcon from discovering
  the submodule's duplicate ROS packages.
- `COLCON_IGNORE` is currently an uncommitted file inside the submodule. To
  preserve it on another computer, it must eventually be committed and pushed
  in the submodule repository, followed by committing the updated submodule
  pointer in this parent repository.

## Current forward-kinematics state

- Hardware counts are now separated into nine pan/tilt joint pairs for the
  cable IK and eighteen alternating one-axis bending joints for segment-angle
  input and D-H forward kinematics.
- The confirmed modified-DH convention matches the estimator:
  `^(i-1)T_i = Rx(alpha_(i-1)) Tx(r_(i-1)) Rz(q_i)`, with `d_i=0` and
  `alpha=[0,+90,-90,+90,...] deg`.
- `q1=pan_relative[0]`, `q2=tilt_relative[1]`, and so on through q18. Inactive
  entries in the two synchronized arrays remain zero but are selected by joint
  parity rather than added.
- Confirmed centerline distances are `os=l=le=4.33 mm`. The 18 joint boundary
  transforms start 4.33 mm from `hrm_base`; the separate tip is another
  4.33 mm after q18. The zero-angle Base-to-tip distance is therefore
  `19 * 4.33 mm = 82.27 mm`.
- `computeBaseToJointsTransformationMatrices()` now returns all 18 Base-to-
  joint transforms. `computeJointPositions()` returns their `Eigen::Vector3d`
  translations, and the separate end-effector helpers apply `le` after q18.
- `ControlNode` now passes both relative pan and tilt arrays to FK and uses the
  resulting 3D tip position for `x_actual`.
- The estimator now also broadcasts `estimated_segment_center_01` through
  `_19` directly under `hrm_base`. Their translations are the 19 measured
  per-frame 3D curve center samples. Center 1 is the fixed `os` center and uses
  the Base orientation; centers 2 through 19 use the 18 filtered sequential
  D-H frame rotations. Center translations have spatial smoothing from depth
  median and curve fitting but no separate temporal position filter.
- On every valid `estimated_segment_angle`, `ControlNode` broadcasts the 18
  model-derived joint-boundary frames `hrm_fk_joint_01` through `_18` plus
  `hrm_fk_tip`, all directly parented to the message frame (normally
  `hrm_base`) and stamped with the source image timestamp. These are model/FK
  frames, so they belong to `robot_control_pkg`; the control calculation does
  not perform TF lookups.

Current interface:

```cpp
Eigen::Matrix4d computeTransformationMatrix(
  const double& joint_angle,
  const double& twist_angle,
  const double& previous_link_length) const;

std::vector<Eigen::Matrix4d> computeBaseToJointsTransformationMatrices(
  const std::vector<double>& pan_angles,
  const std::vector<double>& tilt_angles) const;
```

## Agreed 3D curve-reconstruction stage

The implementation specification is preserved at:

```text
src/estimation_pkg/docs/3d_curve_reconstruction.md
```

The implemented scope is ordered depth-based 3D skeleton points,
cumulative-distance parameter `s`, independent quartic fits `x(s)`, `y(s)`,
`z(s)`, dense fitted-curve sampling, fitted-curve arc-length reconstruction,
19 hardware-length intervals, 20 boundary points, and 19 straight segment
directions. Direction 0 is the fixed `os` segment; a preliminary sequential DH
projection uses directions 1 through 18 to estimate and publish 18 alternating
pan/tilt joint angles. Real-camera sign, noise, and dynamics validation remain
outstanding.

Preserve the existing 2D centerline extrapolation before depth deprojection:
initial skeleton -> 2D fit/extrapolation -> conversion back to pixels -> AND
with `body_image` -> `extended_skeleton`. Apply aligned depth to the final
`extended_skeleton` pixels, not only to the initial skeleton pixels.

Current implementation details:

- `segment_angle_estimation.py` subscribes to color, aligned depth, and color
  `CameraInfo`; topic names and depth scale are ROS parameters.
- `estimation_pkg/launch/_launch.py` starts only `segment_angle_estimator`;
  `external_force_estimator` and the delayed RQT GUI are intentionally excluded.
- `postprocess.py` retains fitted-curve order through pixel conversion and the
  body-mask intersection instead of reconstructing points with scan-ordered
  `np.where`.
- Valid pixels are first pinhole-deprojected into the optical camera frame: X
  right, Y down, Z forward/depth. The first valid base-side point is then
  subtracted. The fallback orientation describes the base pose in camera axes
  as `R_camera_from_base = Ry(-90 deg) @ Rz(-45 deg)`; its inverse/transpose is
  applied to express relative points in the base frame.
- `transform_camera_points_to_base()` accepts a runtime 3x3 rotation or 4x4
  Camera-from-Base transform. Only rotation is used; base translation remains
  estimated from the first ordered 3D skeleton point. The ROS node does not yet
  look up an external live rotation for this input, so it currently uses the
  configured fallback angles `base_in_camera_rotation_y_deg` and
  `base_in_camera_rotation_z_deg`.
- The optional Base-origin filter initially averages 15 camera-frame Base
  samples, then fixes that average. Every current camera point subtracts this
  same fixed origin before rotation into `hrm_base`; it is not an offset applied
  only to the first point. During warm-up, the running mean is used. The raw
  first Base-frame point may therefore retain a small diagnostic residual after
  the origin is fixed, while the fitted curve is explicitly anchored at
  `(0, 0, 0)`.
- Base-frame coordinates are isotropically normalized by the fixed hardware
  length `os + 17*l + le` before 3D fitting. With the current equal lengths,
  this is `19 * 4.33 mm = 82.27 mm`. The configured base frame ID is
  `hrm_base`.
- After fitting, the Base-anchored curve is isotropically scaled once so its
  arc length is exactly 82.27 mm. This prevents depth/end-point length error
  from accumulating between camera reconstruction and fixed-length FK.
  `fitted_curve_length_before_hardware_scaling_3d` and
  `hardware_length_scale` retain the measured length and correction factor.
- The parametric 3D quartic is fitted without a constant term for each axis, so
  the reconstructed curve itself is constrained to `r(0) = (0, 0, 0)`.
- `points_xyz_camera` retains the deprojected camera coordinates;
  `points_xyz_base` and the compatibility alias `points_xyz` contain base-frame
  coordinates used by reconstruction.
- Runtime outputs are `points_xyz`, `segment_points_xyz` with shape `(20, 3)`,
  `segment_directions_xyz` with shape `(19, 3)`, and analytic fitted-curve
  `segment_tangents_xyz` with shape `(20, 3)`. Each hardware-length segment is
  also
  sampled at its fitted-curve midpoint, producing
  `segment_center_points_xyz` and `segment_center_tangents_xyz`, both with
  shape `(19, 3)`.
- All 19 raw `segment_directions_xyz` are preserved. Direction 0 represents
  fixed `os`; the sequential modified-DH projection uses directions 1 through
  18 with `alpha=[0,+90,-90,+90,...] deg` and produces
  `segment_directions_projected_xyz`, `dh_joint_axes_xyz`, signed
  `segment_projection_residuals`, and preliminary
  `dh_joint_angles_projected_{rad,degree}`. The orientation recursion is
  `R_B_i = R_B_(i-1) Rx(alpha_(i-1)) Rz(q_i)`, with joint 1 as pan. These
  preliminary angles remain available as raw measurements. An optional
  constant-velocity Kalman filter maintains one `[q, q_dot]` state per joint;
  its filtered angles feed `estimated_segment_angle` and regenerate
  `segment_directions_filtered_xyz`. These still require real-camera and robot
  sign/convention validation before control use.
- A configurable spatial neighborhood median is applied to depth at each
  skeleton pixel before pinhole deprojection. Zero/non-finite depth is ignored
  inside the window and still rejected if the final sample is invalid.
- All stabilization stages are independent in `config.json` under `filters`:
  `base_origin_initial_average`, `depth_neighborhood_median`, and
  `joint_kalman`. Set each stage's `enabled` value independently. Raw depth,
  raw segment directions, and projection-only joint angles remain stored for
  diagnosis.

Visualization topics are separated as follows:

- `estimated_segment_crop_image`: unmodified ROI in the camera's native RGB/BGR
  encoding
- `estimated_segment_body_binary_image`: mono8 body mask
- `estimated_segment_skeleton_image`: ROI with valid ordered skeleton overlay
- `estimated_segment_centerline_points`: ordered base-frame PointCloud2
- `estimated_segment_reconstruction_markers`: fitted curve, 19 reconstructed
  segments, 20 boundary points/tangents, and 19 segment-center
  points/tangents for RViz2. Center points use the
  `segment_center_points` namespace and center arrows use
  `segment_center_tangents`. Raw segment directions are gray vectors in
  `raw_segment_directions`; projected directions are orange vectors in
  `projected_segment_directions`; Kalman-filtered DH directions are green
  vectors in `filtered_segment_directions`, so all layers can be toggled
  separately. Each vector is now an individual `ARROW` marker because ROS has
  no `ARROW_LIST` type. Arrow shaft/head dimensions are 0.3/0.6 mm with a
  0.8 mm head length. This restores visible arrowheads but increases each
  MarkerArray by roughly 100 Marker objects, so recheck its live rate if RViz
  performance regresses. MarkerArray publication is not rate-throttled.

After each successful reconstruction, the estimator broadcasts the dynamic TF
`camera_color_optical_frame -> hrm_base` (using the actual CameraInfo frame ID).
Its translation is the running-mean Base point during the configured initial
window and then the fixed average `^C p_B`; its rotation is `^C R_B`. Internal
fitting still uses the explicitly transformed Base-frame NumPy points. This
lets RViz use the camera optical frame as its Fixed Frame while displaying the
Base-frame PointCloud2 and MarkerArray.

The crop image is published directly from the color callback, independently of
depth availability or reconstruction success. Camera inputs and visualization
outputs (images, PointCloud2, MarkerArray) use `BEST_EFFORT`, `VOLATILE`,
`KEEP_LAST(1)` sensor-style QoS. The control-facing `SegmentAngle` output stays
`RELIABLE`, `VOLATILE`, `KEEP_LAST(1)`. Set each RViz Image/Marker display's
Reliability to `Best Effort`; a stale RViz display requesting Reliable is QoS
incompatible and will show no data. Visualization messages are constructed
only while a compatible subscriber exists.
One-time reception logs identify which of the three inputs has arrived during
live-camera diagnosis. The strict RGB/depth timestamp rejection was removed;
processing uses the latest aligned-depth frame available.

## Runtime performance

- Active quartic fits use direct linear least squares. The 3D fit remains
  constrained through `(0, 0, 0)` and no longer runs iterative `curve_fit` for
  a model that is linear in its coefficients.
- D405 `rgb8` data is retained in native channel order. The callback selects
  the ROI from the passthrough NumPy view instead of converting the complete
  848x480 image to BGR first; `postprocess()` selects RGB/BGR OpenCV conversion
  codes explicitly.
- The processing worker sleeps on a new-frame event instead of polling a lock
  every 5 ms. ROS callbacks use `SingleThreadedExecutor`; reconstruction and
  visualization remain separate worker threads.
- Depth neighborhood filtering converts only selected pixel windows to float,
  and dense RViz curve output is capped independently of the 1001 internal fit
  samples.
- OpenCV is limited to one worker for these small images to avoid 24-thread
  fan-out and timing jitter. Configure this with
  `performance.opencv_num_threads` in `config.json`.
- Timing and internal input/output-rate logs are disabled by default. Enable
  `performance.debug_timing_enabled` for RBSC/visualization timing and
  `performance.debug_rate_enabled` for internal Hz logs; their intervals are
  controlled by the adjacent `performance` keys. Typical live core time after
  these changes was 11-20 ms
  (previously about 51-55 ms), with occasional 25-35 ms spikes. D405 metadata
  measured about 29.7 Hz; estimator input/output was generally 24-29 Hz on the
  currently loaded desktop. Multiple Python `ros2 topic hz` image subscribers
  materially reduce this rate and are not a passive benchmark.

The independent C++ package `src/image_rate_probe` now provides low-overhead
`image_rate_probe` and `metadata_rate_probe` executables. A clean 2026-09-11
test stopped the estimator, RViz, and existing `topic hz` subscribers while
leaving the D405 wrapper running. Depth metadata remained 29.93-30.00 Hz with
zero missing nominal periods. With alignment enabled, raw depth, raw color,
and aligned depth Image subscriptions varied roughly from 14 to 26 Hz and had
message timestamp gaps of hundreds of milliseconds; with alignment disabled,
the raw streams returned close to 30 Hz. The stock wrapper applies
`rs2::align` synchronously before publishing both aligned and raw frames, so
CPU alignment was stalling that publication path. The later CUDA librealsense
test documented below brought aligned output back to approximately 30 Hz.

Use the integrated camera/estimator launch so D405 profiles and QoS are applied
through the wrapper's config-file path:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch estimation_pkg d405_estimation_launch.py
```

It selects 848x480x30 color/depth, aligned depth, manual exposure 20000, and
`SENSOR_DATA` camera QoS. The D405 color profile parameter is
`depth_module.color_profile`; `rgb_camera.color_profile` does not select this
device's shared depth-module color stream.

`estimation_pkg/launch/_launch.py` no longer starts the full RQT perspective,
because its additional plugins and queues caused substantial live-image delay.
Run `rqt_image_view` or RViz2 separately when visualization is needed.

## Cable and motor state

- `SurgicalTool::inverse_kinematics()` already calculates East, West, South,
  and North cable lengths from pan and tilt targets.
- Motor target indices 0-3 are currently assigned those four cable results.
- Confirm real hardware index ordering and motor direction signs before physical
  operation.
- `PositionController::update()` now passes both accumulated final pan and final
  tilt inputs to `get_IK_result()`.

## Friction-model decision

- `damping_friction_model.cpp` and its legacy implementation are not used and
  are deleted in the current worktree.
- They were removed from the `robot_control` CMake target and remaining runtime
  references were removed.
- Keep the generalized dynamics equation's `tau_friction` input.
- Current behavior deliberately sets `tau_friction_ = 0.0`, so the friction
  term has no numerical effect.
- Damping remains `dynamics_params::DAMPING` and is separate from friction.

## Future external-force training dataset

- Defer runtime use of `external_force_estimation.py` until the training and
  model-comparison pipeline has been developed.
- Record the original timestamped ROS streams, preferably in rosbag: color,
  aligned depth, CameraInfo, motor/cable displacement, load-cell signals,
  external-force ground truth, TF/calibration, and the online
  `estimated_segment_angle` result.
- Build the canonical training dataset offline by replaying each complete
  sequence through the same causal `estimation_pkg` postprocess and filter
  configuration used online. Do not process isolated images because the Base
  initialization and temporal filter states depend on sequence history.
- Associate a reconstructed angle with the source image timestamp in its
  header, not with the later wall-clock time at which processing/publishing
  finishes.
- At each image timestamp, interpolate or select the temporally nearest motor,
  load-cell, and external-force samples within a defined tolerance. Reject
  samples outside that tolerance and calibrate fixed sensor latencies before
  training.
- Proposed feature/target mapping: motor or cable displacement + load-cell
  measurements + per-joint pan/tilt angles (and, for a temporal model, their
  causal history) as inputs; synchronized external-force sensor values as the
  target.
- Keep the online angle topic in the recording as a diagnostic so offline and
  live feature extraction can be compared for deployment consistency.

## Legacy backup

The single-angle, single-plane implementation is preserved under:

```text
src/robot_control_pkg/legacy/segment_angle_single_axis/
```

It contains the old topic/subscriber declarations, usage mapping, and the
single-theta planar transformation functions. The `.inc` files are reference
snippets and are intentionally excluded from the build.

## Deferred work

1. Validate projected DH pan/tilt signs, angle limits, residual thresholds, and
   temporal noise using the real camera and known robot poses before control.
2. Inspect the new FK TF frames in RViz and verify that XYZ axes match
   X=axial, positive q1 pan toward +Y, and positive q2 tilt toward +Z on the
   real mechanism.
3. Compare model-derived `hrm_fk_*` frames with camera-reconstructed geometry
   (`estimated_segment_center_*`) before enabling position feedback on physical
   hardware.
4. Update `PositionControl.msg`, `record_pkg`, and GUI only after the robot
   control interface stabilizes.
5. Validate motor ordering, direction, tension-array size, and physical safety
   before hardware tests.

## Verification

Last successful command:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces robot_control_pkg --symlink-install
```

Both packages built successfully. Compiler warnings remained, including PID
initializer ordering and several existing unused
values/parameters. They were not treated as build failures.

After relocating the runtime estimator, colcon discovered only this package:

```text
estimation_pkg  src/estimation_pkg  (ros.ament_python)
```

The following build also completed successfully:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces estimation_pkg --symlink-install
```

After implementing the RGB-D 3D reconstruction stage, Python compilation,
synthetic pinhole deprojection, synthetic 3D quartic reconstruction, and the
following package build completed successfully on 2026-09-06:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select estimation_pkg --symlink-install
```

After adding optional Base-origin averaging, spatial depth median filtering,
and per-joint `[q, q_dot]` Kalman filtering, focused synthetic tests and the
following build completed successfully on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces estimation_pkg --symlink-install
```

After the runtime performance work, `py_compile`, focused filter/message/marker
tests, and the following build succeeded on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select estimation_pkg --symlink-install
```

After adding the extra hardware segment, the synchronized definition is:

```text
19 geometric segments = os + 17*l + le
20 boundary points = P0 ... P19
18 revolute joints = q1 ... q18 = 9 pan/tilt pairs
os = l = le = 4.33 mm, total length = 82.27 mm
```

The following build completed successfully on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  custom_interfaces estimation_pkg robot_control_pkg
```

Three focused estimation tests verify the `(20,3)` boundary, `(19,3)` center
and direction, and 18-angle shapes; exclusion of the fixed `os` direction from
joint inversion; and correction of a 4% measured-length error to the fixed
82.27 mm hardware length. All passed. The two C++ FK gtests verify the 82.27 mm
straight tip and that q1 acts after `os`; both passed. Package-wide legacy lint
still reports pre-existing formatting failures in both packages and an online
XML schema lookup failure, but these do not affect the focused numerical tests
or the successful build.

The robot-control runtime loops were subsequently corrected on 2026-09-11:
the inactive dynamics thread now sleeps at 30 Hz instead of busy-spinning one
CPU core, and the position/admittance thread sleeps exactly once per iteration
instead of twice. Segment-angle sharing between ROS callbacks and control
threads is mutex-protected, `control_mode_` is atomic, the initial
`control_mode` parameter is applied, and reliable QoS now correctly uses a
validated depth of one. The package rebuilt successfully. Live execution was
not started automatically because the node contains physical motor-command
paths; validate its CPU and topic rates during the next supervised hardware
run.

The standalone C++ image/metadata rate probe also built successfully:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select image_rate_probe
```

### CUDA RealSense alignment

On 2026-09-11, librealsense `v2.55.1` was built with
`BUILD_WITH_CUDA=ON` for the RTX 3060 (`sm_86`), and realsense-ros `4.55.1`
was rebuilt against it in the isolated, git-ignored `.cuda_realsense/`
directory. The apt/system RealSense installation was not overwritten.

Runtime verification confirmed that the camera process mapped
`.cuda_realsense/install/librealsense/lib/librealsense2.so.2.55.1`, loaded
`libcuda.so`, and appeared as a GPU compute process using about 114 MiB.
With the camera at 848x480x30 and alignment enabled, the low-overhead C++
probe measured aligned depth at 29.60, 29.59, and 29.79 Hz over consecutive
five-second windows. The earlier CPU-alignment measurements were roughly
14-15 Hz.

Use the checked-in helpers to reproduce or run it:

```bash
./scripts/build_realsense_cuda.sh
./scripts/run_realsense_cuda.sh
```

The `launcher` package now contains `cuda_realsense.launch.py` and
`cuda_estimation.launch.py`. These locate the workspace-local CUDA build,
prepend its realsense-ros prefix and libraries to the child environment, and
include the D405 configuration without spawning a nested `ros2 launch` shell.
The primary `system.launch.py` also includes `cuda_realsense.launch.py` in
place of its former apt/CPU camera include. Legacy demo launch files retain
their previous camera settings.
After sourcing the normal workspace, use either:

```bash
ros2 launch launcher cuda_realsense.launch.py
ros2 launch launcher cuda_estimation.launch.py
```

The run helper is retained as a convenience wrapper around the first command.
The CUDA launch environment is required; otherwise the loader may select the
apt CPU library from `/opt/ros/humble`.

The integrated launch also verified the configured 848x480x30 profile and
BEST_EFFORT camera endpoints. Repeated live restarts later exposed an external
V4L2/USB disconnect (`VIDIOC_QBUF: No such device`); replug/recover the D405
before the next camera run rather than treating this driver state as an
estimator exception.

## Worktree safety

- The worktree contains uncommitted changes from both the user and Codex.
- Do not reset, restore, overwrite, or delete unrelated changes.
- Inspect `git status` and `git diff` before editing overlapping files.
- Do not commit or push unless the user explicitly requests it.
- Never put credentials, tokens, passwords, or private machine configuration in
  this file.

## Maintenance policy

Codex should update this file without waiting for a separate user request when
one of the following occurs:

- a control, coordinate-frame, topic, message, or hardware-mapping decision is
  made;
- a material implementation milestone is completed;
- build/test status changes;
- a blocker, safety concern, or important deferred task appears;
- work is being prepared for handoff to another computer or session.

Do not update it for trivial formatting, comments, or exploratory commands.
Keep it concise, factual, and consistent with the repository. Mention material
handoff updates briefly in the final response.
