# Codex Handoff

Last updated: 2026-09-06 (Asia/Seoul)

## Purpose

This document carries project decisions and implementation state between Codex
sessions and computers. Read it together with the current source, `git status`,
`git diff`, and recent `git log`; the repository is the source of truth when
this document and the code differ.

## Robot and control model

- The continuum section has two controlled bending DOFs: pan and tilt.
- East/West cables form the antagonistic pair for pan.
- South/North cables form the antagonistic pair for tilt.
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
  `NUM_OF_JOINT` elements.
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
single subscriber. The RBSC estimator publisher has intentionally not been
updated yet.

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

- Joint positions now use `Eigen::Vector3d` and extract XYZ translation from
  each homogeneous transform.
- `computeTransformationMatrix()` still accepts only one scalar `theta` and
  still contains the original planar transform.
- `computeBaseToJointsTransformationMatrices()` still accepts one joint-angle
  vector.
- `ControlNode` stores both `theta_pan_actual_` and `theta_tilt_actual_`, but
  temporarily passes only the pan vector to forward kinematics.
- The user will derive the new pan/tilt 3D transformation equation separately.

Expected future interface shape:

```cpp
Eigen::Matrix4d computeTransformationMatrix(
  const double& pan,
  const double& tilt);

std::vector<Eigen::Matrix4d> computeBaseToJointsTransformationMatrices(
  const std::vector<double>& pan_angles,
  const std::vector<double>& tilt_angles);
```

Do not invent or replace the transformation equation without confirming it
with the user.

## Agreed 3D curve-reconstruction stage

The implementation specification is preserved at:

```text
src/estimation_pkg/docs/3d_curve_reconstruction.md
```

The implemented scope is ordered depth-based 3D skeleton points,
cumulative-distance parameter `s`, independent quartic fits `x(s)`, `y(s)`,
`z(s)`, dense fitted-curve sampling, fitted-curve arc-length reconstruction,
18 equal arc-length intervals, 19 boundary points, and 18 straight segment
directions. Pan/tilt angle estimation remains a later stage.

Preserve the existing 2D centerline extrapolation before depth deprojection:
initial skeleton -> 2D fit/extrapolation -> conversion back to pixels -> AND
with `body_image` -> `extended_skeleton`. Apply aligned depth to the final
`extended_skeleton` pixels, not only to the initial skeleton pixels.

Current implementation details:

- `segment_angle_estimation.py` subscribes to color, aligned depth, and color
  `CameraInfo`; topic names, depth scale, and RGB/depth time tolerance are ROS
  parameters.
- `estimation_pkg/launch/_launch.py` starts only `segment_angle_estimator` (and
  the existing delayed RQT GUI); `external_force_estimator` is intentionally
  excluded until the force-estimation pipeline is revised.
- `postprocess.py` retains fitted-curve order through pixel conversion and the
  body-mask intersection instead of reconstructing points with scan-ordered
  `np.where`.
- Valid pixels are pinhole-deprojected into the optical camera frame: X right,
  Y down, Z forward/depth. Camera-to-robot extrinsic transformation is not yet
  implemented.
- Coordinates are translated to the estimated base and isotropically
  normalized by `num_of_segments * length_of_segment` before 3D fitting.
- Runtime outputs are `points_xyz`, `segment_points_xyz` with shape `(19, 3)`,
  `segment_directions_xyz` with shape `(18, 3)`, and analytic fitted-curve
  `segment_tangents_xyz` with shape `(19, 3)`.
- Zero/non-finite depth is currently removed. Neighborhood recovery and temporal
  depth filtering remain future robustness improvements.

Visualization topics are separated as follows:

- `estimated_segment_crop_image`: unmodified BGR ROI
- `estimated_segment_body_binary_image`: mono8 body mask
- `estimated_segment_skeleton_image`: ROI with valid ordered skeleton overlay
- `estimated_segment_centerline_points`: raw ordered camera-frame PointCloud2
- `estimated_segment_reconstruction_markers`: fitted curve, 18 reconstructed
  segments, 19 boundary points, and 19 analytic tangent arrows for RViz2

The crop image is published directly from the color callback, independently of
depth availability or reconstruction success. Color, aligned-depth, and
CameraInfo subscriptions request BEST_EFFORT sensor QoS for compatibility with
RealSense publishers. Image and PointCloud2 outputs use BEST_EFFORT/KEEP_LAST(1)
to avoid stale visualization queues; MarkerArray uses RELIABLE/KEEP_LAST(1).
One-time reception logs identify which of the three inputs has arrived during
live-camera diagnosis. The strict RGB/depth timestamp rejection was removed;
processing uses the latest aligned-depth frame available.

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

## Legacy backup

The single-angle, single-plane implementation is preserved under:

```text
src/robot_control_pkg/legacy/segment_angle_single_axis/
```

It contains the old topic/subscriber declarations, usage mapping, and the
single-theta planar transformation functions. The `.inc` files are reference
snippets and are intentionally excluded from the build.

## Deferred work

1. Convert the 18 reconstructed camera-frame segment directions into the
   alternating pan/tilt joint angles, after defining camera-to-robot axes and
   the relative-rotation convention.
2. Publish the new `SegmentAngle` message on `estimated_segment_angle` from the
   relocated runtime package.
3. Implement the user-derived 3D `computeTransformationMatrix(pan, tilt)`.
4. Pass both relative pan and tilt arrays through cumulative FK.
5. Verify that XYZ axes match X=axial, Y=pan feedback, Z=tilt feedback.
6. Update `PositionControl.msg`, `record_pkg`, and GUI only after the robot
   control interface stabilizes.
7. Validate motor ordering, direction, tension-array size, and physical safety
   before hardware tests.

## Verification

Last successful command:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces robot_control_pkg --symlink-install
```

Both packages built successfully. Compiler warnings remained, including PID
initializer ordering, an unused planar-transform variable, and several unused
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
