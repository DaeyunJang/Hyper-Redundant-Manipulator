# Force / contact-ID inference

`force_id_estimation.py` is an optional ROS node named `/force_id_estimator`.
It restores one selected `hrm_bundle.pt`, consumes live input features, and
publishes one `custom_interfaces/msg/ForceIdEstimate` on `/force_id_estimation`.
It does **not** publish `/estimated_external_force`, motor commands or mode
requests. As of 2026-10-15, `robot_control_pkg` can consume eligible predictions
in explicitly selected Y/Z ADMITTANCE mode; motor enable remains separate.
See [controller contract and test procedure](../../robot_control_pkg/README.md#learned-force-yz-admittance-2026-10-15).

## Build and start

`src/estimation_pkg/models/` is local-only and excluded from Git. On a new PC,
copy the required `hrm_bundle.pt` files from the training project into that
directory separately (preserving the paths below for the default selection).
Offline `validation.xlsx` and `validation_predictions.npz` are not required for
inference. Without supplied bundles the GUI has no models to select; a Git pull
alone does not install them. Existing local files are unaffected by this rule.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_interfaces estimation_pkg gui_py_pkg robot_control_pkg \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

In the GUI, **Packages → Force / contact** defaults to the Masked bundle with
force=KalmanNet, ID=Transformer and cable+loadcell+joint-angle inputs:
`cable_loadcell_angle/models/masked__force_kalmannet__location_transformer/hrm_bundle.pt`.
Review that selection (or choose another bundle), then press the component's
**Start**. It is not started automatically by default. Stop that
component before choosing another model. Refresh rescans `estimation_pkg/models`.
The body-monitor switch and `±ID` display tolerance can change while running.
Starting this component alone never enables motors; other GUI components retain
their existing independent auto-start settings.

Standalone example (only starts inference):

```bash
ros2 launch estimation_pkg force_id_estimation.launch.py \
  model_path:="$(pwd)/src/estimation_pkg/models/cable_loadcell_angle/models/masked__force_kalmannet__location_transformer/hrm_bundle.pt"
```

The launch uses `HRM_INFERENCE_PYTHON` when set, otherwise `~/HRM_env/bin/python3`
if present, otherwise its current Python. Override with `python_executable:=...`.
The current machine's system Python has ROS but no PyTorch; HRM_env has both after
sourcing the ROS/workspace setups. No new Torch installation is required here.
The GUI itself does not import Torch. Default device is CPU, with one BLAS/OMP
thread in the launch to avoid competing with image processing.

`model_path` accepts the bundle file or its immediate parent directory. It can
also be changed using ROS parameters; an invalid selection is rejected, and a
successful change clears the entire input history. A blank path unloads inference.

```bash
ros2 topic echo /force_id_estimation
ros2 param set /force_id_estimator body_contact_enabled true
ros2 param set /force_id_estimator id_tol 2
```

## Configurable operating no-load region (2026-10-26)

Edit `src/estimation_pkg/estimation_pkg/id_localization_config.json`:

```json
{
  "schema_version": 1,
  "enabled": true,
  "unloaded_force_ranges_mN": {
    "fx": [-200.0, 200.0],
    "fy": [-200.0, 200.0],
    "fz": [-200.0, 200.0]
  }
}
```

The three inclusive ranges apply to **inferred** XYZ in hrm_base, with the model's
published/training sign, before the controller's force_sign conversion. Config
uses mN; inference topic stays N. No rounded GUI values or measured FT labels are
used. All three axes inside => UNLOADED and operating `control_contact_id=18`.
Any axis outside => LOADED and the positive-ID classifier winner. This common
rule applies to Masked and Class-0. If Class-0's raw winner is0 outside the box,
the highest original probability among positive supported IDs is used for the
operating ID. Raw ID, all probabilities and continuous force remain unchanged.

The original seed used training calibration ranges (±134.326/126.814/192.670 mN);
the operator subsequently selected **±200 mN on each axis**, as shown above.
The editable file is authoritative. These are **not validated live detection
thresholds for every replacement bundle**. They define
an operating classification, not proof of zero physical force or tip contact.
There is no implicit hysteresis, debounce or force-zeroing. The current default
KalmanNet/Transformer bundle actually reports `force_scope=all_single_contact`;
old sibling experiment metadata reporting `tip` is not authoritative.

Settings are read once at startup. Restart Force / contact after editing; a
symlink build requires no rebuild for later JSON-value edits. The new file needs
one initial package build. An alternate file can be selected at launch with
`id_localization_config:=/absolute/path.json` (also a ROS node parameter).
Live parameter changes are rejected. Missing/malformed/nonfinite/reversed ranges
fail explicitly rather than silently falling back. `enabled:false` restores
native Class-0 load classification and Masked LOAD_UNKNOWN.
Whole-line `//` comments are accepted in this operator config; inline/block
comments are not. Ordinary JSON remains supported.

Valid range-policy messages have exact machine-readable
`status="ready_force_ranges"`. Existing message fields/wire layout are unchanged
to preserve bag compatibility; the original classifier is still `contact_id`,
and the operating decision is `control_contact_id`/`load_state`.

**Controller policy explicitly requested by the user:**
`admittance/use_contact_id_gate=false` is now the default. ID/load classification
is visual only and does not interrupt valid force feedback, even if the classifier
predicts a body ID during a physically tip-loaded experiment. The single predicted
Fy/Fz vector drives Y/Z admittance; this does not extract a distinct ID18 force
from a set of 18 force vectors. The experiment must physically confine contact to
the tip. Unknown/stale/invalid **force data** remain blocked (unknown localization
alone is not invalid force). Optional `use_contact_id_gate=true` restores ID-gated
operation; change only outside Admittance. See the controller README.

## Models and input provenance

Put models under `src/estimation_pkg/models/<experiment>/.../hrm_bundle.pt`.
Discovery is recursive. The package installs only `.pt` bundles, not the large
offline prediction NPZ files. In a symlink workspace the GUI also scans source
models, so newly added bundles can be selected after Refresh without copying the
whole experiment into install. A standalone deployment needs an install/rebuild
or an explicit external `model_path`.

The current six input groups contain 1,452 schema-2 `independent_hrm` bundles,
each comprising class0/masked and 11 force × 11 location architectures.
The loader supports the current saved
architectures, rejects unsupported structures, uses `weights_only=True`, strict
state loading, evaluation mode and no-gradient inference. No sibling training
repository is required at runtime.

Feature order, 30-row window, active IDs, architecture parameters and branch
scalers are read from the bundle, **not inferred from filenames**. Force and
location inputs are normalized separately. Force outputs are inverse-scaled.
`.npz` files contain saved validation predictions/row references, **not scalers**.
Several copied top-level experiment JSON/YAML files are empty; they are not used
for inference. All current `.pt` files were readable and validated.

The original cable/loadcell input group uses:

| Input | ROS source | Units / processing |
|---|---|---|
| `wire.Cable #1..4 length` | `/wire_length`, `data[0..3]` | Original mm, no offset/conversion |
| `loadcell.Loadcell #1..4 tension` | `/loadcell_state`, `stress[0..3]` | Original training/native units, no zeroing |

Supported additional bundle features include motor state, cable velocity, raw pan/tilt
relative/absolute angles, and active-axis relative angles/angular velocities.
The fixed active-axis mapping remains odd **tilt**, even **pan**. Unsupported
features, force targets, contact labels and timestamp inputs are rejected.
Only required feature topics are subscribed. Current cable/loadcell models do
not require F/T ground truth, camera images, segment angles or tip/TF poses.

### Tip pose models and automatic input ordering

The added input groups are cable_angle (22), loadcell_angle (22),
cable_loadcell_angle (26), cable_loadcell_fk_tip_tf (15), and
cable_loadcell_estimated_pose (15). Counts are features per sample, not window
length. Select the corresponding trained bundle; no manual input-array edits.

| Bundle column | Live source | Convention |
|---|---|---|
| `estimated_tip.x_m/y_m/z_m` | `/estimated_tip_position` (`PointStamped`) | Final fitted boundary, m / hrm_base |
| `fk_tip.x_m/y_m/z_m` | `/kinematics/fk_tip_position` (`PointStamped`) | FK endpoint, m / hrm_base |
| `fk_tip_tf.x_m/y_m/z_m`, `qx/qy/qz/qw` | `/tf`, child `hrm_fk_tip` | Direct parent hrm_base; XYZ + xyzw |
| `estimated_tip_orientation.qx/qy/qz/qw` | `/tf`, child `estimated_segment_center_19` | Direct parent hrm_base; xyzw |

The estimated-pose bundle intentionally combines the final boundary's position
with the last segment center's orientation, exactly as summary.csv did. Do not
replace the boundary position with the center's TF translation. Quaternion
values/signs are retained, with no canonicalization or extra smoothing. Pose
`roll_deg/pitch_deg/yaw_deg` columns are also supported with the same normalized
quaternion-to-Euler formulas as the recorder; current bundles use quaternion
columns. Zero/non-finite quaternions and wrong parent frames are invalid inputs.
The node does not silently assume a camera-frame point is in hrm_base or perform
a new TF-chain conversion.

Only requested topics are subscribed. Each required TF child has its own cache,
source timestamp and freshness check, even when both poses occur in one model.
Unrelated joint/camera/tag TF messages do not refresh missing tip transforms.
Tip models require the appropriate estimator/FK publishers running, not motor
enable or admittance. Adding pose inputs does not enable actuator output.

The automatic part is simply building the same numeric array from the selected
bundle's ordered list. A simplified illustration (not the full ROS callback):

```python
columns = bundle['feature_columns']  # e.g. ["estimated_tip.x_m", "wire.Cable #1 length"]
latest = {
    'wire.Cable #1 length': wire_message.data[0],
    'estimated_tip.x_m': tip_message.point.x,
}
row = [latest[name] for name in columns]  # [tip_x, cable_1], not dictionary order
```

Actual code maps names using `feature_spec()` and forms `row` in
`FeatureHistory.sample()`. It appends rows to the required time window, then the
runtime normalizes them independently for force/location before network calls.
Unknown columns fail explicitly; changing a bundle's names alone cannot change
the inputs the model was trained on.

## Sampling and missing data

`sample_hz=30` samples the latest required measurements. Each required stream
must have a new sample since the previous input row. Repeated stationary values
are valid; reusing an old message is not a new history row. Wire messages have no
header, so their time is local receipt time. Stamped sources retain source time.

Defaults are `input_timeout_sec=0.25`, `max_input_skew_sec=0.1`, and
`max_window_gap_sec=0.15`. Non-finite/malformed, missing, stale, clock-reset or
excessively skewed data clear the history. After recovery another full window is
required. Small delays without a new sample simply wait; they do not publish a
new copy of the previous valid force. Diagnostic invalid messages are throttled.
Change sampling/timing settings at node startup, not during an experiment.

The training summaries were approximately camera-frame cadence, whereas this
optional node's timer does not require an unused camera stream. This is not an
exact source-time join/replay of training: live rate, sensor delays and any
deployment domain shift still need experimental validation. `header.stamp` is
the newest selected input time; `published_stamp` is output time. Invalid output
has `valid=false`, IDs -1 and NaN force, never a fabricated zero-force observation.

## Output and heatmap semantics

- `external_force`: XYZ in **N**, frame **hrm_base**, retaining model/training sign.
  The control-side `-1` multiplier is **not applied by the publisher**. GUI converts N to mN
  and labels predictions as base-frame; measured F/T remains sensor-frame. Their
  plotted components are not a calibrated frame-aligned accuracy comparison.
- `contact_id`: original classifier argmax; full `probabilities[0..18]` preserved.
- With the default force-range configuration, either model uses the operating
  load rule above. With the config disabled, Class0 rawID0 means model-classified
  unloaded/controlID18; Masked remains LOAD_UNKNOWN/controlID-1. `p18` is never
  overwritten by an operating fallback and the force is never set to zero.
- `tip_force_valid` expresses model-scope/policy eligibility only. It does **not**
  certify control stability, calibration or motor authorization. Recognized
  scopes `tip` and `all_single_contact` can qualify when the operating ID is18.
  This ID-gated diagnostic is not an enable signal; visualization-only controller
  mode intentionally does not use it to gate otherwise valid force feedback.
- Body monitoring OFF/ON never disables raw ID inference or overwrites it with
  18. It changes monitoring, not the controller's separate use_contact_id_gate
  policy. Neither setting enables motors. No location-specific body-force
  response controller or per-contact force separation has been implemented.
- Heatmap center is argmax among active IDs1..18, band is ±`id_tol` (default2),
  and cell colors show the **original** probability on a fixed 0..1 scale.
  The band is clipped to valid IDs, not renormalized. Band probability mass is
  not measured ±n accuracy. A band around16 including18 does not authorize tip
  force use. Class0 probability and unknown/invalid/stale states remain distinct.

## Verification and deferred work

The GUI can also place the same raw-probability band on the HRM skeleton image
using `/estimated_segment_image_geometry`. That topic contains **crop pixels**,
not metric PointCloud/TF geometry. Exact image/geometry matching and separate
bounded inference freshness prevent old highlights from being held indefinitely.
UNLOADED in either model is not rendered as a detected tip contact. This optional
display does not modify the inference topic or any controller eligibility rule.
See [GUI overlay details](../../gui_py_pkg/README.md#contact-probabilities-on-the-skeleton-image-2026-10-26).

Verification includes all 1,452 actual bundles' input mapping and strict
restore/finite inference, all 11 architectures × both modes of the original
cable/loadcell group compared against archived validation
predictions, feature/history/policy unit tests, and isolated-domain synthetic ROS
warm-up, stale recovery, model switching, parameter changes and publisher checks.
The Transformer activation differs from the older training repository: archived
schema-2 predictions require tanh-approximate GELU, caught by numerical comparison.
These checks verify implementation parity and transport, not experimental force
or localization accuracy. No camera or physical motor was started for testing.

The 2026-10-15 controller integration uses this combined topic, applies the sign
once and drives **Y/Z admittance only**, with X observable but not controlled.
Legacy `/estimated_external_force` no longer authorizes/refreshes admittance.
The 2026-10-26 visualization-only ID policy supersedes the old class0-only gate.
Invalid/stale/out-of-range force uses the existing software hold/latch; body or
unknown localization alone does not when use_contact_id_gate=false. Threshold
detection accuracy, physical force/axis calibration, stability and powered
experiments remain unvalidated. Controller details are in the linked README.

Focused test commands (source ROS/workspace first):

```bash
PYTHONPATH="src/estimation_pkg:$PYTHONPATH" ~/HRM_env/bin/python3 -m pytest -q \
  src/estimation_pkg/test/test_force_id_inputs.py \
  src/estimation_pkg/test/test_force_id_pose_inputs.py \
  src/estimation_pkg/test/test_force_id_runtime.py \
  src/estimation_pkg/test/test_force_id_shutdown.py
HRM_RUN_FORCE_ID_INTEGRATION=1 PYTHONPATH="src/estimation_pkg:$PYTHONPATH" \
  OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 ~/HRM_env/bin/python3 -m pytest -q \
  src/estimation_pkg/test/test_force_id_integration.py
```

The integration tests use a private DDS domain (211 by default), synthetic
wire/loadcell/tip/TF inputs for cable, FK-pose and estimated-pose bundles, and no
robot controller, camera, legacy force publisher or
motor command. Archived numerical parity tests additionally require
`HRM_BUNDLE_PARITY_CSV` pointing to the matching recorded ID18 summary.
