# Codex Handoff

Last updated: 2026-09-14 (Asia/Seoul)

## Four-channel loadcell GUI (2026-09-14)

- Serial already publishes four values in `/loadcell_state.stress`; only the
  GUI still created/updated two rows. GUI now declares `num_loadcells=4`
  independently of motor count, with the same launch default, and displays
  all four channels in received order. Missing values are cleared to `—`.
- Sensor zero remains below the loadcell rows. No serial, motor command,
  force/torque plot, calibration or hardware connection behavior was changed.
- 34 focused GUI tests passed, including four-channel values and partial/empty
  messages clearing old values. GUI package rebuilt; operator GUI was not
  restarted during the edit.

## GUI Stop/External lifecycle fix (2026-09-14)

- A user Stop really terminated robot_control (PID 56281, SIGINT exit -2),
  but the old GUI forgot its stop history once the launch parent exited and
  interpreted any remaining ROS graph name as External.
- `system_manager.py` now retains the stop/exit history and owned process-group
  identity. Child processes outliving their launch parent remain owned and
  stoppable, never relabelled as External. No external targets are killed by
  node name. Partial external component presence also blocks duplicate starts.
- Normal stop is a single SIGINT to the ROS launch parent, with stdin DEVNULL
  guaranteeing noninteractive launch forwarding. This avoids sending SIGINT
  both directly to children and again via launch. Repeated Stop is idempotent.
  The GUI waits 12 s before fallback SIGTERM to its own group, allowing launch's
  native escalation to finish first. Stop initiation time is retained.
- UI: STOPPING while owned processes remain; STOPPED* / ROS cleanup… if the
  group has exited but names remain in the graph; STOPPED / Start after graph
  clearance. Restart is blocked during ambiguous graph cleanup. A new external
  node after observed clearance is still protected. Unexpected failures remain
  failures, while operator-requested signal exits count as stopped.
- Built gui_py_pkg. 31 focused tests passed, including duplicate signal
  prevention, stop-history persistence, stale graph/restart guards, surviving
  children, external protection and actual GUI status/button states.
- Three real dry-run robot_control start/stop cycles in ROS_DOMAIN_ID=74
  each finished in ~0.15 s, exit code 0, STOPPING → STOPPED, without External.
  Shutdown logs include `ROS node Shutdown` and `process has finished cleanly`.
  Test launches were cleaned up; no active hardware processes were stopped.

## Motor GUI layout and image previews (2026-09-14)

- Read-only diagnosis of Direct motion: TCP feedback had four positions and
  the motor_command subscriber was connected, but robot_control's
  `motor_output_enabled` was false. Direct `1000` means current encoder
  position +1000 counts; no test movement or gate change was performed.
- Moved the unchanged physical-output checkbox/acknowledgement to the top of
  Motor control. Startup remains OFF and operator confirmation is required.
  Motor control now takes roughly 60% of the window; System remains above
  Sensors on the left. Mode selections are horizontal to leave room for images.
- Moved **Set sensor zero** under the raw sensor values. It still calls
  `/serial_data/set_zero`, not a motor zero/homing service. The current serial
  implementation averages force/torque data; loadcell averaging remains
  commented out (no serial behavior change in this task).
- Added `gui_py_pkg/image_preview.py`: aligned depth and skeleton side by side
  under the motor commands, independently updated. Depth JET display uses
  near=red/far=blue, invalid=black, adjustable fixed 0.07–0.6 m range.
  16UC1 millimetres and 32FC1 metres are supported; raw depth is untouched.
- Image-only node `gui_image_preview` has its own executor/thread, sensor QoS
  (BEST_EFFORT, KEEP_LAST(1), VOLATILE), one latest-message slot per topic,
  max-640px display width and a 34 ms Qt refresh timer. Widget updates are
  confined to Qt; QImage owns copied pixels; missing/stale streams are labelled.
  Preview worker is joined before the GUI shuts down ROS.
- Built gui_py_pkg; 19 focused tests passed, including image formats/units,
  invalid depth, padded/big-endian depth, latest-only buffering, stale states,
  image-buffer lifetime and relocated buttons. New preview code passes targeted
  flake8/pydocstyle checks. Existing workspace-wide flake8/pep257 tests still
  fail on legacy/generated sources (not treated as a clean full-suite result).
- Isolated ROS_DOMAIN_ID=74 GUI test: 30 Hz synthetic image inputs plus 500 Hz
  synthetic motor feedback, ~27 Hz displayed, Qt heartbeat p95 ~22 ms and max
  ~28 ms. Live read-only preview check: 391 depth / 371 skeleton frames displayed
  in ~14 s. These are UI refresh observations, not end-to-end camera benchmarks.
  Test processes were closed; existing hardware/GUI sessions were not restarted.
- `launcher/system.launch.py` currently defaults auto_start_components=false
  due to an operator change; it was preserved. Restart GUI only after safely
  stopping physical motion. Detailed operator notes: `src/gui_py_pkg/README.md`.

## Camera/estimation real-time fix (2026-09-13 evening)

- Operator instructions and measured limits: `docs/REALTIME_ESTIMATION.md`.
- The earlier claim that 81% estimator CPU usage itself proves the camera
  bottleneck was too strong. Earlier measurements also included USB/device
  resets. A controlled synthetic A/B now demonstrates a transport bottleneck:
  default 512 KiB Fast DDS SHM is smaller than RGB (1.16 MiB) or depth
  (0.78 MiB). The same optimized estimator with 512 KiB SHM lost images;
  64 MiB SHM restored all streams to ~30 Hz.
- `estimation_pkg/config/fastdds_images.xml` supplies SHM 64 MiB / max message
  4 MiB plus UDPv4. `launcher/cuda_realsense.launch.py` applies it to the CUDA
  camera, so both GUI and `scripts/run_realsense_cuda.sh` share the fix.
  `estimation_pkg/runtime.py` applies it before rclpy.init for direct run and
  launch. Explicit operator DDS profiles/other RMW selections are preserved.
  Restart BOTH camera and estimator to apply it; no global OS changes needed.
- Lee skeletonization is restricted to the nonzero body bounding box, preserving
  exact original ROI pixels. The 2D extrapolation/body intersection, depth
  deprojection, base orientation, hardware scaling and DH/filter math remain.
- Reconstruction no longer holds the visualization handoff lock. One complete
  result snapshot replaces the previous pending result; crop publication is
  handled outside DDS callbacks. No old frame is republished just to raise Hz.
- RViz ARROWs retain existing types/namespaces/dimensions but update stable
  `(ns,id)` objects instead of DELETEALL/recreate on every frame. Missing IDs
  are deleted explicitly; only the initial publication uses DELETEALL.
- New config: `performance.blas_num_threads=1`, `max_depth_age_sec=0.1`.
  The latter rejects color/depth pairs more than 100 ms apart (0 disables);
  it is a stale-depth guard, not exact RGB-D synchronization. Invalid/empty body
  frames warn at most every 2 s and do not emit fresh angles. Debug rate logs
  run even during failures; timing/rate logs still default OFF.
- `image_rate_probe/pipeline_rate_probe` now measures color/depth/crop/skeleton/
  angles/markers rates, source age p50/p95 and repeated/backwards/missing stamps.
- Verified: 90 s / 2700 synthetic moving frames with estimator + dry-run FK +
  all probe image/marker subscribers, steady 30 Hz outputs, zero repeated or
  backwards timestamps and zero estimated missing periods in steady windows.
  Source-to-angle p95 by 5 s window was 20.2–36.4 ms. Real D405 + offscreen GUI
  + estimator + robot_control + existing RViz also sustained 30 Hz RGB/aligned
  depth/crop. REAL GEOMETRY LIMIT: the camera ROI was dark (maximum 19 vs
  threshold 120), so real-HRM angle success/accuracy needs illuminated testing
  tomorrow. Motor output stayed disabled; no TCP bridge was started in tests.
- Reproduce the isolated throughput test with
  `python3 scripts/benchmark_estimation_pipeline.py --duration 90` after sourcing
  the workspace. It uses test ROS domain 73 and cleans up its own children.
- Estimation/camera autostart remains enabled. The old recorder is now manual
  pending migration to final 3D dataset topics, as previously deferred. GUI
  close stops Qt/plot timers before destroying the ROS node to fix a teardown
  InvalidHandle exception discovered in the integration test.
- Built estimation_pkg, gui_py_pkg, launcher, image_rate_probe successfully;
  16 focused tests passed (including GUI process-manager tests). Geometry,
  skeleton equivalence, snapshot, marker and DDS override regression
  checks are in `test_realtime_pipeline.py` and `test_hardware_layout.py`.

## GUI-managed system startup (2026-09-13)

- `launcher/system.launch.py` now starts only `gui_py_pkg`; the GUI owns the
  child launch processes and provides per-component Start/Stop controls.
- Ordered defaults live in
  `src/gui_py_pkg/config/system_components.json`: CUDA camera (0 s), serial
  (1 s), TCP motor bridge (2 s), robot control (3 s), estimation (4 s).
  Recorder (optional 5 s) and AprilTag are selectable but not automatic.
- `daq_pkg` and `MasterMACS_pkg` are intentionally excluded.
- Status uses both the managed process and expected ROS graph node. An
  externally started node is shown as EXTERNAL and is never killed by GUI.
- Physical motor output always starts disabled. The Motor control safety switch
  updates `/robot_control`'s `motor_output_enabled` parameter; if control is
  stopped, its selection is passed as the next launch argument.
- The motor UI is parameterized for four motors in East, West, South, North
  order and displays the four-value IK preview.
- The black/frozen GUI seen without motor or serial data was caused by calling
  `rclpy.spin_once()` with its blocking default inside the Qt event loop. It is
  now called with `timeout_sec=0.0`; missing streams are shown as red
  `no recent data` labels instead of freezing repainting.
- The GUI is one maximized window split into two columns. The left column has
  System above Sensors, while Motor control occupies the full-height right
  column. The Sensors panel has raw values on its left and a wide plot on its
  right.

## Purpose

This document carries project decisions and implementation state between Codex
sessions and computers. Read it together with the current source, `git status`,
`git diff`, and recent `git log`; the repository is the source of truth when
this document and the code differ.

## Robot and control model

- The continuum section has two controlled bending DOFs: pan and tilt.
- East/West cables form the antagonistic pair for pan.
- South/North cables form the antagonistic pair for tilt.
- Hardware inspection corrected the physical order on 2026-09-13: the first
  revolute joint is driven by the South/North pair and is **tilt**, followed by
  the East/West **pan** joint. The 18 one-axis joints therefore alternate as
  `q1=tilt, q2=pan, ..., q17=tilt, q18=pan` (zero-based even indices are tilt).
  The estimator and FK were converted to this tilt-first convention on
  2026-09-13.
- The physical centerline contains one fixed proximal segment `os` followed by
  18 moving-link directions, for 19 geometric segments and 20 boundary points.
  The kinematic chain has 18 revolute joints (nine pan/tilt pairs); `q1` acts
  after the fixed `os` segment.
- `hrm_base` is the authoritative Cartesian frame. X is the tool's axial
  direction and is calculated but is not a position-controller input.
- `hrm_base` is also the D-H Base, with no preliminary rotation. q1 is named
  tilt, rotates about Base +Z, and positive q1 bends the axial +X direction
  toward Base +Y. q2 is named pan and is the next orthogonal one-axis joint.
- The position feedback dimensions are Base Y for tilt and Base Z for pan in
  the straight configuration.
- There is currently no insertion-axis control.
- `PositionController` independently uses Y and Z errors:
  - `x_desired(1) - x_actual(1)` -> tilt PID -> `del_theta_tilt_`
  - `x_desired(2) - x_actual(2)` -> pan PID -> `del_theta_pan_`

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
  `alpha=[0,+90,-90,+90,...] deg`. The recursion starts at identity in the
  fixed `hrm_base`; there is no initial `Rx(+90 deg)`.
- `q1=tilt_relative[0]`, `q2=pan_relative[1]`, and so on through q18. Inactive
  entries in the two synchronized arrays remain zero but are selected by joint
  parity rather than added.
- Confirmed centerline distances are `os=l=le=4.33 mm`. The 18 joint boundary
  transforms start 4.33 mm from `hrm_base`; the separate tip is another
  4.33 mm after q18. The zero-angle Base-to-tip distance is therefore
  `19 * 4.33 mm = 82.27 mm`.
- `computeBaseToJointsTransformationMatrices()` now returns all 18 Base-to-
  joint transforms. `computeJointPositions()` returns their `Eigen::Vector3d`
  translations, and the separate end-effector helpers apply `le` after q18.
  Consequently `hrm_fk_joint_01` is identity at `q1=0` and its first
  variable rotation is q1 tilt about Base `+Z`; `hrm_base` itself never
  rotates.
- `ControlNode` now passes both relative pan and tilt arrays to FK and uses the
  resulting 3D tip position for `x_actual`.
- The estimator now also broadcasts `estimated_segment_center_01` through
  `_19` directly under `hrm_base`. Their translations are the 19 measured
  per-frame 3D curve center samples. Center 1 is the fixed `os` center and uses
  the Base orientation; centers 2 through 19 use the 18 filtered sequential
  D-H joint rotations. Center translations have spatial smoothing from depth
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
  `R_B_i = R_B_(i-1) Rx(alpha_(i-1)) Rz(q_i)`, initialized with the identity
  rotation of fixed `hrm_base`, with joint 1 as tilt about Base +Z. These
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

The crop image is queued by the color callback and published by the visualization
worker, independently of depth availability or reconstruction success. Camera inputs and visualization
outputs (images, PointCloud2, MarkerArray) use `BEST_EFFORT`, `VOLATILE`,
`KEEP_LAST(1)` sensor-style QoS. The control-facing `SegmentAngle` output stays
`RELIABLE`, `VOLATILE`, `KEEP_LAST(1)`. Set each RViz Image/Marker display's
Reliability to `Best Effort`; a stale RViz display requesting Reliable is QoS
incompatible and will show no data. Visualization messages are constructed
only while a compatible subscriber exists.
One-time reception logs identify which of the three inputs has arrived during
live-camera diagnosis. The strict RGB/depth timestamp rejection was removed;
processing uses the latest aligned-depth frame available, with the configurable
100 ms stale-depth guard added on 2026-09-13.

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
- `robot_control` now defaults `motor_output_enabled=false`. All control paths
  pass actuator publication through this explicit gate, and a valid
  `motor_state` must also have been received before a real command is sent.
- `kinematics/move_tool_angle` remains usable in dry-run without a motor
  connection. Each request publishes `kinematics/target_wire_length` as a
  four-element `Float64MultiArray` ordered `[East, West, South, North]` in mm
  and logs the same values. The service inputs are degrees (the former `.srv`
  comments incorrectly said radians).
- The dry-run was exercised on 2026-09-13 with motor output disabled. The IK
  results in `[East, West, South, North]` mm were:
  - `tilt=+10, pan=0`: `[+0.00465, +0.00465, -0.63005, +0.63704]`
  - `tilt=-10, pan=0`: South/North signs reverse.
  - `tilt=0, pan=+10`: `[-0.63005, +0.63704, +0.00465, +0.00465]`
  - `tilt=-5, pan=+5`: `[-0.31474, +0.31881, +0.31881, -0.31474]`
  The preview topic was also received successfully, while every
  `motor_command` was blocked by the dry-run gate.
- Keep dry-run active for software checks. Real output requires an explicit
  `motor_output_enabled:=true` launch/parameter setting and supervised hardware
  readiness; this gate does not replace mechanical limits or emergency stop.

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

## Agreed next development roadmap

Resume the project in the following order. Do not start by enabling closed-loop
or admittance motion on hardware.

1. **Pre-motion safety and convention check**
   - Confirm motor indices and directions for East/West pan and South/North
     tilt, cable-length units, encoder zero/reference positions, pretension,
     joint/cable limits, command saturation, timeout behavior, and the physical
     emergency-stop procedure.
   - Validate the estimated pan/tilt signs and the FK axes against several known
     static poses. Tune the angle filter or invalid-frame hold/reset behavior
     before the estimate is used as feedback.
2. **Open-loop cable IK and motor test**
   - Exercise `SurgicalTool::inverse_kinematics()` with small, single-axis tilt
     commands first (South/North and q1), then pan (East/West and q2), then
     combined tilt/pan.
   - Initially inspect/log the four target cable lengths without motor output;
     then run low-speed, limited-amplitude hardware motion under supervision.
   - Compare commanded motion with load-cell/cable feedback, camera-estimated
     joint angles, and FK/TF. Resolve motor-order, sign, offset, slack, and
     workspace-limit errors before proceeding.
3. **Position feedback control**
   - Use the filtered `estimated_segment_angle` and 3D FK to obtain the current
     tip position. Keep X as the calculated axial coordinate only; control Y
     with q1 tilt and Z with q2 pan in the straight configuration.
   - Verify feedback signs before enabling PID, then tune one axis at a time at
     low gain. Add output/integrator limits, stale-estimate handling, and a safe
     fallback when reconstruction is invalid or delayed. Finally test coupled
     pan/tilt motion.
4. **Motor-era data collection and GUI revision**
   - Modify `record_pkg` and `gui_pkg` only once real motor motion and the final
     command/feedback interfaces can be observed. `gui_py_pkg` now reads the
     four-motor hardware count, labels the rows East/West/South/North, and shows
     the dry-run `kinematics/target_wire_length` values in mm. Its kinematics
     button can request a preview without a connected `motor_state`; manual
     motor operation still requires feedback. Revisit the real command path,
     bounds, and enable-state indication after supervised hardware motion.
   - A later GUI feature may start and stop package/launch groups. Implement it
     through supervised ROS launch-process management with visible process
     state and clean shutdown, not by embedding blocking shell commands in GUI
     callbacks.
   - Record raw timestamped color, aligned depth, CameraInfo, motor/cable
     displacement, load cells, external-force ground truth, TF/calibration,
     commands/control mode, and online segment-angle output. Construct the
     canonical synchronized training table offline as specified in the
     external-force dataset section above.
5. **External-force model training and runtime integration**
   - Train and compare models in the `LSTM-force-estimation` research
     workspace. Keep deployment inference in this repository's
     `estimation_pkg`.
   - Validate inference units, coordinate frame, signs, timestamp/latency,
     uncertainty or out-of-distribution behavior, and live rate before the
     estimate can drive a controller.
6. **Admittance control revision and validation**
   - Feed the validated estimated external force into the admittance model;
     confirm desired/environment-force convention and transform all force and
     displacement quantities into one declared frame.
   - Revisit the active pan/tilt dimensions of the M/B/K matrices, sampling
     time, force filtering, saturation, workspace limits, and reset behavior.
     Test offline/replay and zero-force behavior first, then restrained
     low-gain hardware contact tests. Friction remains intentionally zero in
     the generalized dynamics equation unless a later experiment justifies a
     model.

After the above, perform an integrated launcher test, check timing/QoS under
simultaneous camera, estimation, control, recording, and GUI load, and update
the public README with the stable operator-facing launch/calibration/safety
procedure. `CODEX_HANDOFF.md` remains the detailed development record until
those interfaces are stable.

## Deferred work

1. Validate projected DH pan/tilt signs, angle limits, residual thresholds, and
   temporal noise using the real camera and known robot poses before control.
2. Inspect the new FK TF frames in RViz and verify that XYZ axes match
   X=axial, q1 tilt rotating about Base +Z toward +Y, and q2 pan on the next
   orthogonal axis of the real mechanism.
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
