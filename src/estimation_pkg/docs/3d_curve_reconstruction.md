# 3D Curve Reconstruction Specification

## Scope

This is the agreed RGB-D reconstruction and preliminary pan/tilt estimation
stage. It reconstructs 19 straight HRM segments from an ordered 3D skeleton.
The first segment is the fixed proximal offset `os`; the remaining 18 segment
directions are projected into the alternating DH joint planes to estimate one
relative angle per one-DOF joint. The result still requires real-hardware sign
and convention validation before control use.

## Input assumptions

- Input `points` has shape `(M, 3)` and uses one physical unit consistently.
- Points are ordered from base to tip along the skeleton path.
- Invalid depth samples (`NaN`, `inf`, zero depth) and consecutive duplicate
  points are removed before parameterization. A configurable spatial median
  can first replace each skeleton-pixel depth with the valid median from its
  local image neighborhood.
- The RGB skeleton pixels are deprojected using depth aligned to the RGB image
  and the corresponding camera intrinsics.
- Segment/joint counts and `os`, `l`, `le` come from `config.json`.
- The fixed hardware centerline length is:

```python
hardware_length = os + (n_bending_joints - 1) * l + le
```

For the current equal-length hardware this is `19 * 4.33 mm = 82.27 mm`.

Use this fixed value for isotropic coordinate normalization. After fitting,
apply one Base-anchored isotropic scale so the fitted centerline arc length is
exactly the fixed hardware length rather than the changing endpoint distance
or noisy raw skeleton length.

## Preserved 2D preprocessing and extrapolation

Keep the existing RGB-image pipeline through construction of
`extended_skeleton`:

```text
body mask
-> smoothing
-> initial skeleton / longest backbone
-> 2D Cartesian conversion and quartic fit
-> 2D endpoint extrapolation
-> inverse conversion to image pixels
-> AND with the original body mask
-> extended_skeleton
```

Depth deprojection starts from the pixels of this mask, after the current
`postprocess.py` statement:

```python
self.extended_skeleton = np.logical_and(
    body_image,
    self.extended_curve,
).astype(int)
```

Do not deproject only the initial `longest_backbone_image`; doing so would skip
the existing extrapolation that recovers the centerline within the visible HRM
body. Extract the final skeleton pixels from `extended_skeleton`, order them
base-to-tip, obtain robust aligned-depth values, and then deproject them to 3D.

## Reconstruction pipeline

### 1. Parameterize raw points by cumulative 3D distance

```python
raw_diff = np.diff(points, axis=0)
raw_ds = np.linalg.norm(raw_diff, axis=1)
D_raw = np.concatenate(([0.0], np.cumsum(raw_ds)))
L_raw = D_raw[-1]
s_raw = D_raw / L_raw
```

`s_raw` is a fitting parameter in `[0, 1]`; it is not guaranteed to remain an
arc-length parameter after polynomial fitting.

### 2. Fit a quartic parametric 3D curve

Fit each coordinate independently against the same parameter:

```python
coef_x = np.polyfit(s_raw, points[:, 0], 4)
coef_y = np.polyfit(s_raw, points[:, 1], 4)
coef_z = np.polyfit(s_raw, points[:, 2], 4)
```

The resulting curve is:

```text
r(s) = [x(s), y(s), z(s)]
```

Start with quartic polynomials to match the legacy approach. If later tests
show excessive residuals, endpoint oscillation, or underfitting for complex
spatial shapes, evaluate a parametric cubic/B-spline without changing the
arc-length resampling contract.

### 3. Densely sample the fitted curve

Use a vectorized sample such as `N_DENSE = 1001`:

```python
s_dense = np.linspace(0.0, 1.0, N_DENSE)
curve_dense = np.column_stack((
    np.polyval(coef_x, s_dense),
    np.polyval(coef_y, s_dense),
    np.polyval(coef_z, s_dense),
))
```

### 4. Recompute fitted-curve arc length

```python
dense_diff = np.diff(curve_dense, axis=0)
dense_ds = np.linalg.norm(dense_diff, axis=1)
D_curve = np.concatenate(([0.0], np.cumsum(dense_ds)))
L_curve = D_curve[-1]
```

`D_curve` must be strictly usable as a monotonic interpolation coordinate.
Zero-length consecutive dense intervals must be rejected or removed.

### 5. Divide by equal fitted-curve arc length

First scale the Base-anchored fitted curve from its measured length to the fixed
hardware length. Then generate 20 boundary locations using the hardware
segment-length vector `[os] + 17 * [l] + [le]`:

```python
curve_scale = hardware_length / L_curve
curve_dense *= curve_scale
D_curve *= curve_scale
L_curve = D_curve[-1]
target_lengths = np.concatenate(([0.0], np.cumsum(segment_lengths)))
s_segment = np.interp(target_lengths, D_curve, s_dense)
```

Do not use `np.linspace(0, 1, 19)` directly for boundary parameters because
the fitted curve is not generally parameterized by arc length.

### 6. Reconstruct boundary points and straight segments

```python
segment_points = np.column_stack((
    np.polyval(coef_x, s_segment),
    np.polyval(coef_y, s_segment),
    np.polyval(coef_z, s_segment),
))

segment_vectors = np.diff(segment_points, axis=0)
segment_lengths = np.linalg.norm(segment_vectors, axis=1)
segment_directions = segment_vectors / segment_lengths[:, None]
```

Expected shapes:

```text
segment_points      (20, 3)
segment_vectors     (19, 3)
segment_lengths     (19,)
segment_directions  (19, 3)
```

## Required outputs

- `coef_x`, `coef_y`, `coef_z`
- `curve_dense`
- `L_raw` and `L_curve`
- `s_segment`
- `segment_points`
- `segment_vectors`
- `segment_lengths`
- `segment_directions`
- analytic fitted-curve tangents at all 20 boundary points

## Important implementation notes

- The existing `np.where(extended_skeleton > 0)` ordering is image scan order,
  not base-to-tip path order. The current implementation therefore resamples
  the fitted 2D curve monotonically in its curve parameter, retains that order
  through the body-mask intersection, and reverses it when the legacy base
  origin is closer to the final point. Replace this provisional base rule with
  an explicit base marker or previous-frame tracking if tests show ambiguity.
- Establish which skeleton endpoint is the physical base using a known base
  pixel/ROI, previous-frame tracking, or another explicit rule. Reverse the
  ordered path when necessary.
- The fitted curve is first scaled isotropically about P0 so its centerline arc
  length equals `os + 17*l + le = 82.27 mm`. It is then divided using the
  configured hardware-length vector. Because all current values are 4.33 mm,
  the 19 arc-length intervals are equal. The pre-correction fitted length and
  applied scale remain available for diagnostics.
- Reject degenerate inputs: too few valid points for degree four, near-zero
  `L_raw`, non-finite coefficients, near-zero `L_curve`, or zero-length
  reconstructed segments.
- Keep the curve fitting and resampling vectorized for the 30 FPS target.
- Depth deprojection first produces camera optical-frame points. Before 3D
  fitting, subtract the Base origin selected by the optional initial-average
  filter. It uses a running mean during the initial window and fixes that mean
  afterward; the same origin is subtracted from every point in each frame. The
  current fallback
  describes the Base-in-Camera orientation as
  `R_camera_from_base = Ry(-90 deg) @ Rz(-45 deg)`; its inverse is applied to
  express the relative points in `hrm_base`. The conversion also accepts a
  runtime 3x3 rotation or 4x4 Camera-from-Base transform. Translation from a
  supplied 4x4 transform is intentionally ignored at this stage.
- Each coordinate of the parametric 3D quartic has no constant term. The first
  fitting input is also explicitly anchored at zero. Therefore the fitted
  curve satisfies `r(0) = (0, 0, 0)` even though the current raw first point can
  have a small residual relative to the fixed averaged Base origin.
- RViz debug visualization publishes base-frame 3D points, the dense fitted
  curve, 20 boundary points, 19 connected straight segments, and analytic
  tangent arrows at all boundary points. It also samples 19 fitted-curve points
  at the arc-length center of the segments and publishes a unit tangent at each
  center. These are separate from the 20 boundary samples.
- The runtime node also broadcasts `Camera -> hrm_base` as a dynamic TF after a
  successful reconstruction. Translation is the first ordered Camera-frame
  point and rotation is `^C R_B`; internal fitting continues to use Base-frame
  NumPy coordinates rather than performing a TF lookup.
- It additionally broadcasts `estimated_segment_center_01` through `_19`
  directly under `hrm_base`. Each translation is the corresponding measured
  fitted-curve center point. The fixed first center uses the Base orientation;
  centers 2 through 19 use the 18 filtered sequential D-H frame rotations.
  These measured center frames are geometrically distinct from the
  `hrm_fk_joint_*` boundary frames produced by `robot_control_pkg`.
- Preserve all 19 raw boundary-to-boundary segment directions. Direction 0 is
  the fixed `os` segment and is excluded from joint-angle inversion. For the
  remaining 18 directions, recursively construct each local joint axis with
  `alpha=[0,+90,-90,+90,...] deg`, remove the raw direction component parallel
  to that axis, and normalize the result. RViz displays raw directions as gray
  arrows and projected directions as orange arrows at all 19 segment centers;
  the fixed first projected/filtered arrow is the Base +X direction.
  Projection residuals remain internal diagnostic arrays. A configurable
  constant-velocity Kalman filter maintains `[q, q_dot]` for each joint and
  regenerates filtered DH direction arrows in green. Preliminary sequential
  joint angles are published on `estimated_segment_angle`: relative arrays
  contain one active alternating axis per joint, absolute arrays contain
  Base-frame segment azimuth/elevation, relative velocities use filtered
  `q_dot` when Kalman is enabled, and absolute velocities use wrapped temporal
  finite differences.
- The three filters are independently selectable in `config.json`:
  `base_origin_initial_average`, `depth_neighborhood_median`, and
  `joint_kalman`. MarkerArray output is not throttled; raw, projected, and
  filtered direction namespaces remain available simultaneously. Each vector
  is an individual `ARROW` marker so its direction is visually explicit.

## Runtime performance contract

- Quartic models are solved by linear least squares; the 3D model omits its
  constant term so `r(0)=(0,0,0)` remains exact.
- The ROS color callback keeps native `rgb8`/`bgr8`, crops a passthrough NumPy
  view first, and lets `postprocess()` select the correct conversion codes.
- New color frames wake the processing worker through an event. A
  `SingleThreadedExecutor` handles the short ROS callbacks while reconstruction
  and subscriber-gated visualization run in their own workers.
- Camera inputs and visualization outputs use BEST_EFFORT/VOLATILE/
  KEEP_LAST(1); `SegmentAngle` remains RELIABLE/VOLATILE/KEEP_LAST(1).
- Internal reconstruction uses all configured dense samples, while RViz curve
  output is independently capped by `visualization_curve_max_points`.
- `debug_timing_enabled` controls RBSC/visualization timing output and
  `debug_rate_enabled` controls internal Hz output; both default to false.
  Their intervals, `opencv_num_threads`, and the RViz point cap live under
  `performance` in `config.json`.
- With the live D405, core reconstruction normally measured 11-20 ms after the
  changes versus about 51-55 ms before them. Sensor metadata remained near
  30 Hz; large Python diagnostic subscribers such as simultaneous
  `ros2 topic hz` calls can themselves introduce frame loss.

## Explicitly deferred

- real-camera validation of projected DH angle signs, residuals, and noise
- tuning filters and defining a robust invalid-frame hold/reset policy
- forward kinematics
