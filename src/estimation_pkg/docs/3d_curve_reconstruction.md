# 3D Curve Reconstruction Specification

## Scope

This is the agreed intermediate stage between RGB-D skeleton extraction and
pan/tilt joint-angle estimation. It reconstructs 18 straight HRM segments from
an ordered 3D skeleton. It does not yet estimate joint angles or implement
kinematics.

## Input assumptions

- Input `points` has shape `(M, 3)` and uses one physical unit consistently.
- Points are ordered from base to tip along the skeleton path.
- Invalid depth samples (`NaN`, `inf`, zero depth) and consecutive duplicate
  points are removed before parameterization.
- The RGB skeleton pixels are deprojected using depth aligned to the RGB image
  and the corresponding camera intrinsics.
- `n_segments` and `length_of_segment` come from `config.json`.
- The fixed hardware centerline length is:

```python
hardware_length = n_segments * length_of_segment
```

Use this fixed value for isotropic coordinate normalization and validation, not
the changing endpoint distance or noisy raw skeleton length.

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

For 18 reconstructed segments, generate 19 boundary locations:

```python
target_lengths = np.linspace(0.0, L_curve, n_segments + 1)
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
segment_points      (19, 3)
segment_vectors     (18, 3)
segment_lengths     (18,)
segment_directions  (18, 3)
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
- analytic fitted-curve tangents at all 19 boundary points

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
- The fitted curve is divided into equal fractions of its measured fitted
  length. The fixed hardware length is a normalization and validation reference;
  it does not automatically force `L_curve / 18` to equal the physical segment
  length. If exact physical boundary distances are later required, define how
  missing endpoints and length mismatch are corrected or extrapolated.
- Reject degenerate inputs: too few valid points for degree four, near-zero
  `L_raw`, non-finite coefficients, near-zero `L_curve`, or zero-length
  reconstructed segments.
- Keep the curve fitting and resampling vectorized for the 30 FPS target.
- RViz debug visualization publishes raw 3D points, the dense fitted curve, 19
  boundary points, 18 connected straight segments, and analytic tangent arrows
  at all boundary points in the camera optical frame.

## Explicitly deferred

- pan/tilt joint-angle estimation
- alternating joint-axis inference
- DH or homogeneous transformations
- forward kinematics
