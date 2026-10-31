# HRM operator GUI

## Position PD / admittance parameter layout (2026-10-27)

Motor control places **Position PD** on the left and **Admittance M/B/K** on the
right of a vertical separator. A horizontal separator distinguishes this settings
strip from the existing Fy/Fz target-force row below it. The motor area and three
image previews remain on screen without a motor-panel scrollbar at 1366×768.

The new panel edits the controller's existing shared Y/Z scalar parameters:

- **M**: `admittance/M_d`, virtual mass [kg].
- **B**: `admittance/B_d`, damping [N s/m].
- **K**: `admittance/K_d`, stiffness [N/m]. This is the stiffness parameter
  referred to as `D_d` in the GUI request; no separate D parameter is introduced.

Read the running controller values, edit, then press **Apply**. Application sends
only these three parameters atomically and requires an ACK and confirming
readback. Values must be finite, nonnegative and not all zero. Unedited values
retain their full server precision. Startup/reconnect and the two-second refresh
only read parameters; refresh preserves pending edits. These are runtime values,
not saved startup defaults, and the GUI never automatically changes modes.

The existing backend restriction remains: **leave Admittance mode before applying
M/B/K**. `Leave Admit` indicates why Apply is unavailable; after changing mode,
Read or wait for refresh. Fy/Fz target forces remain independently editable during
Admittance. No force-sign, filter, matrix/axis calculation, controller limits or
PID behavior changed. The two-plot, 10 Hz display and bounded ROS callback polling
are retained. Safely stop motion/recording before restarting the GUI to load this
Python-only change; no live hardware settings were changed during development.

## Admittance target contact force (2026-10-27)

**Motor control → Admittance F* [mN]**, below Position PD, sets signed Fy/Fz
targets for force **on the HRM**, expressed in `hrm_base` (X is not controlled).
Read current values, edit, then press **Apply**. Inputs alone never send targets.
The two values are converted from mN to N and written atomically to
`admittance/target_force_y_N` and `admittance/target_force_z_N` on `/robot_control`.
Application requires an ACK and confirming readback; hover the status for errors.
The controller's sign and maximum force-vector norm are read-only in this row.
Applying during Admittance can immediately affect motion, but the panel never
enables motors or changes modes. These are runtime settings, not persisted defaults.

Thin red horizontal lines on the grouped force graph show the **confirmed** Fy/Fz
targets in the existing
prediction convention: `plot_target_mN = target_N / force_sign * 1000`.
For `force_sign=-1`, a controller target **+200 mN** appears at **−200 mN**.
Measurements and predictions are not negated or rotated by this display change;
raw F/T measurements still use sensor axes. Reference lines appear only after
controller settings and a unified `hrm_base` prediction source are known. Pending
edits do not move them; an unconfirmed write/offline controller hides them.
They show configured targets even outside Admittance, not evidence of active
contact control. Stale prediction samples remain gaps while confirmed targets
can remain visible. Read-only refresh every two seconds follows external parameter
edits without overwriting pending GUI edits. The nine original data channels,
topics, and recorded numeric values are unchanged.

`Position control: ACTIVE [ID display-only; assumes physical tip contact]` is
the shared inner-controller status in Admittance: ID is visual only and forces
are assumed physically applied at the tip. It is not a fault or a localization
success claim. The user-reported log alternated this with
`[admittance offset/velocity limited]`; transitions in that limit flag re-log ACTIVE.
The combined flag does not identify which axis or which limit was active.
The inspected runtime bounds were ±5 mm compliant offset and ±2 mm/s compliant
offset velocity (not the position sine or motor speed limits).

## Lightweight force / torque graphs (2026-10-27 rollback)

The six-panel experiment was rolled back after a live responsiveness regression.
Sensor monitoring again uses two vertically stacked plots on one canvas:

- Top: fx/fy/fz and fx_pred/fy_pred/fz_pred [mN], plus confirmed Fy/Fz targets.
- Bottom: tx/ty/tz [mN m], with a separate scale.

Force limits include all measured/predicted force components and visible targets;
torque limits include only torque samples. Unchanged axis limits are not rewritten.
Light gray horizontal lines follow the major Y-axis ticks; legends stay in the
upper-right corner. Measurements use 1.5-point solid lines; predictions use
1.1-point densely dashed lines in a darker shade of the same axis color.

Only graph rendering/sampling is limited to **10 Hz** (100 ms); its 50-sample
visual history spans about five seconds when Qt keeps up. This is not the sensor,
inference, control or recording rate. ROS polling still runs from the 5 ms Qt
timer, but drains up to 16 callbacks per tick, stopping before starting another
callback once a 2 ms budget is exhausted (a running callback cannot be preempted).
The old one-callback-per-tick cap was only 200 callbacks/s, below the combined
sensor and motor stream rate. No new subscriptions or QoS changes were introduced.
Invalid/stale predictions still create NaN gaps, while zero and negative values
remain valid. Numeric readouts, topics, CSV and motor control are unchanged.
The caption identifies sensor measurement axes versus unified `hrm_base`
prediction axes; overlaying them does **not** perform an axis alignment.

Read-only live audit before rollback: F/T and loadcell ~200 Hz each, inference
and segment angles ~30 Hz each, motor state ~242 Hz independently received while
GUI CPU was ~117%. Thus the measured fault was GUI processing/display latency,
not a demonstrated publisher rate drop. Mocked same-size canvas benchmark:
six plots mean 27.58 ms / p95 28.75 ms per redraw, restored two plots mean
13.44 ms / p95 15.29 ms. These are synthetic render timings, not a measurement
of the updated GUI on hardware. No live GUI restart or motor command was made.

Plots retain a minimum height of 160 px. Only the graph area scrolls vertically
when a short window cannot accommodate both; a sufficiently tall window
shows them together without a scrollbar. The motor and image panels are unchanged.
Reopen the GUI after updating the Python source; do not restart it during motor
motion or recording, since closing it can stop GUI-managed components.

## Operating no-load range and visual-only contact ID (2026-10-26)

The inference node reads `estimation_pkg/estimation_pkg/id_localization_config.json`
once on startup. Edit its per-axis mN ranges, then restart Force / contact.
All inferred Fx/Fy/Fz inside inclusive ranges => UNLOADED/controlID18;
otherwise the positive classifier candidate supplies the operating ID. Numeric
raw ID/probabilities and predicted forces remain unchanged. The skeleton contact
overlay is hidden while UNLOADED for both Masked and Class-0; a tip fallback is
not painted as a confidently detected tip contact.

Per user request, robot_control now defaults to
`admittance/use_contact_id_gate=false`: ID/load classification is for visualization,
not for interrupting otherwise valid force feedback. This assumes the experiment
physically applies force at the tip. The network provides one XYZ vector, not
18 independently selectable force vectors. Body monitor does not select a force
source or enable motors. Optional controller ID gating remains available outside
Admittance mode; GUI `tip_force_valid` is only that gated-policy diagnostic.

## Position / Admittance Y/Z sine (2026-10-26)

The single sine row below **Move Tip** changes with the selected control mode:
Kinematics uses the existing degree-based IK sine; Position and Admittance use
**Y/Z sine** in metres. Both sets of settings are retained independently.

1. Enter Position or Admittance and wait for valid feedback/controller readiness.
2. Set the upper Move Tip Y/Z fields to the desired **absolute hrm_base centres**.
   Sine Start uses absolute centres even if manual Publish is set to Relative;
   there is no need to press Publish first. Angle and position text values are
   separately remembered so degrees cannot carry over as metres on a mode change.
3. Set each amplitude A [m], phase φ [deg], and common period T [s], then Start.
   Defaults: both A=0.010 m, Y φ=0°, Z φ=90°, T=20 s. With zero centres this is
   ±10 mm per axis after the initial approach. Set one amplitude to zero for
   single-axis motion; that axis holds its chosen centre, not its current pose.
4. Stop freezes the last nominal reference. Position PD / admittance still runs;
   this button is **not** a motor disable or emergency stop and does not return
   the HRM to zero.

Start atomically applies `motion.position_sine.{y,z,period_sec}` and then calls
`/position/sine_motion`. No automatic mode/enable change or per-frame GUI calls.
The backend ramps the nominal reference to the first phase, limits each axis to
0.005 m/s by default, and validates amplitude/period/workspace. This reference
limit is distinct from the unchanged PD angular limit of 5 deg/s; it does not
guarantee actual tracking speed or reachability. In Admittance, force-derived
offsets are added to this nominal waveform; this is not a sinusoidal force target.
Camera gaps freeze its phase and automatically resume; controller faults or
mode/output-gate changes stop it. Stop before changing waveform settings or
sending a manual position goal. Hover the status label for rejection details.

Rebuild `robot_control_pkg` and `gui_py_pkg`, then restart both when motor output
is off and no recording/test is active. The new controls do not update an already
running controller binary.

## Contact probabilities on the skeleton image (2026-10-26)

The existing image controls include **Contact overlay** (ON by default). The
HRM skeleton tile draws semitransparent red curve segments with intensity from
the original contact probabilities, within the same ±ID tolerance band as the
numeric heatmap. The selected candidate is outlined/labeled. This is a classifier
visualization, not a pixelwise contact detector or an experimental accuracy map;
probabilities are not renormalized inside the displayed band.

- `/estimated_segment_image_geometry` (`custom_interfaces/SegmentImageGeometry`)
  supplies crop-local pixel paths and centers for contact IDs 1–18. The fixed
  proximal `os` segment is excluded; ID18 covers the final moving/tip segment.
- Image geometry and skeleton image must have matching source stamps, camera
  frames and dimensions. Estimator interpolation uses retained skeleton pixels
  at the existing 3D segment parameters, so hardware-length scaling cannot shift
  the highlight away from the original image skeleton. No extra TF query or
  projection of length-scaled XYZ is performed in the GUI.
- The existing force/ID subscription is shared, with no extra inference. Its
  source stamp is the newest selected model input, not necessarily an image
  stamp: a bounded source-time gap is allowed (0.5 s), not exact equality.
  Image/geometry receipt ages must remain within 0.2 s and inference within
  0.5 s. Invalid, missing or stale data remove the overlay, including when no
  new image arrives. Prediction-only updates redraw without converting RGB.
- Class0 unloaded/contact-ID0 does not draw a false tip contact from the
  control-side fallback ID18. Masked predictions remain diagnostic localization
  with load status unknown. Body-monitor OFF does not hide diagnostic predictions.
- The overlay is a separate GUI paint layer. Published/recorded skeleton pixels,
  segmentation, fitting, angles, FK, force routing and motor control are unchanged.
  Disable the checkbox to see the unchanged skeleton display.

Build `custom_interfaces`, `estimation_pkg` and `gui_py_pkg`, then restart Segment
estimation and the GUI when safe (never restart during recording or active motor
tests). No change/restart of robot_control is required for this display feature.

## Position PID gains and default inference model (2026-10-26)

The compact **Position PD** controls are below **Move Tip / IK sine** in Motor
control. **Pan (Z)** edits Z position feedback; **Tilt (Y)** edits Y feedback.
These are the inner position-controller gains, shared by Position and Admittance,
not the Dynamics PID or the outer admittance M/B/K settings.

- **Read** gets the running `/robot_control` gains; it does not send a motion.
- Edit P/D, then **Apply** sends all six pan/tilt P/I/D parameters together via
  `/robot_control/set_parameters_atomically`. Inputs alone never apply settings.
- I is displayed but fixed at **0** because the controller currently accepts PD
  only. P/D must be finite and nonnegative. Acknowledgements, rejection reasons,
  offline and unknown/timed-out results are shown (hover status for details).
- Startup/reconnect reads values, never applies cached settings or enables motors.
  Read current values before applying. Settings are runtime-only: a controller
  restart reloads its configured startup gains. No goals/modes/speed limits change.
- The 2026-10-27 restored controller uses angular increments: P units rad/m,
  D units rad*s/m. Its output is added to the preceding command angle without
  an extra multiplication by image dt; the separate angular-speed guard remains.
  Numeric user gains are retained, not automatically retuned. Gains from the
  preceding rate-PD controller are NOT equivalent; verify new settings with motor
  output disabled first. Applying gains in an active loop affects subsequent commands.

**Packages → Force / contact** preselects the requested bundle:
`cable_loadcell_angle/models/masked__force_kalmannet__location_transformer/hrm_bundle.pt`.
Inputs are cable state + loadcell + joint angles; the bundle owns the exact
feature order/scalers. Other input/model combinations remain selectable.
Selecting a default does not start inference or change an already running model;
auto-start remains OFF. Missing default files must not silently select a different
input group or architecture. Manual selection is preserved when rescanning.
Standalone ROS launch/run still accepts an explicit `model_path`.

## Sensor numeric colours (2026-09-28)

Measured F/T (fx/fy/fz/tx/ty/tz) and all loadcell numeric fields use magnitude:
absolute value <=1000 is black, >1000 and <1500 orange, >=1500 red.
Thresholds use the existing displayed units (force mN, torque mN m, loadcell g),
not a shared physical safety limit. Signed numbers and existing formatting remain
unchanged. Missing/nonfinite values clear threshold colouring to black; this is
not a sensor-validity indication. Prediction fields, graphs, recorded values and
motor safety logic are unchanged. GUI restart loads the change.

## Optional image recording (2026-09-27)

Before Record, set **Save crop RGB/depth** in the Recording settings row.
Default is checked, preserving the existing crop color + crop depth recording.
Unchecked saves only the existing numeric bag, CSV (including summary.csv), and
metadata under the default profile: no `images/hrm_crop` or `images/hrm_crop_depth`
files are created, and missing crop streams do not block Record or fail export.
CameraInfo and depth calibration metadata may still be recorded as small numeric
messages. Camera operation, inference, GUI/RViz display and motor control are
unchanged. Without saved images, offline image/depth reprocessing is unavailable.

The GUI sends `save_images` atomically with force-axis settings and contact ID,
then starts capture only after acknowledgement. The checkbox locks through
recording, stopping, image flushing and CSV export; change it before the next
Record. An externally started active session displays its frozen choice. Idle
heartbeats do not overwrite your next-session selection. GUI restart restores
the checked default. Restart both the GUI and idle Data recorder after updating;
never restart while recording/exporting. Existing sessions are not modified.

## Live predicted-force preview (2026-09-27)

The sensor values now pair `fx | tx`, `fy | ty`, `fz | tz` in two equal-width
columns; `fx_pred`, `fy_pred`, `fz_pred` occupy the three rows below. The upper
force plots overlay measured solid lines and predicted dense dashed lines in
matching axis colours, with fixed legends. As of 2026-10-27, fx/fy/fz and
tx/ty/tz each have their own subplot (see the per-axis graph section above).

The GUI subscribes to `/estimated_external_force` (`geometry_msgs/msg/Vector3`),
the existing legacy force topic. No model/publisher is started. Prediction QoS is
BEST_EFFORT / KEEP_LAST(1) / VOLATILE so either reliable or best-effort live
publishers can feed the preview. Before receipt the fields show `—`; nonfinite
values show `invalid`, and after 0.5 s without receipt they show `stale`. Missing
prediction samples are NaN plot gaps, not fabricated zeros or indefinitely held
values. Zero and negative finite predictions are displayed normally.

Startup ROS parameters (restart GUI after changing them):

| Parameter | Default | Meaning |
|---|---|---|
| `predicted_force_topic` | `/estimated_external_force` | Vector3 input topic |
| `predicted_force_unit` | `mN` | Incoming `mN` or `N`; display always mN |
| `predicted_force_sensor_axes` | `[x, y, z]` | Each displayed sensor-axis component expressed in incoming XYZ |
| `predicted_force_timeout_sec` | `0.5` | Receipt-age display timeout, not a controller safety setting |

Existing raw readings remain in **sensor axes**, not hrm_base. The default assumes
predictions use these same axes and mN. If a future model emits hrm_base/N, set unit
`N` and configure the **inverse** of the training sensor-to-base axis mapping.
For the dataset mapping `base=[-sensor_y,-sensor_x,-sensor_z]`, the inverse happens
to be `[-y,-x,-z]` as well. Do not assume all mappings are self-inverse. Only signed
axis permutations are supported here, not arbitrary rotations. These settings are
independent of the CSV-only Recording force-axis controls, and must be matched
explicitly when the trained model is integrated. Vector3 cannot verify frame,
unit, or source timestamp; arrival freshness is not acquisition freshness.

This is a live visual comparison at GUI refresh times, not timestamp-aligned
evaluation. Nothing is republished, sent to motors, or added to recording by this
change. Tests must use an isolated ROS domain or a remapped test topic: a fake
publication on the production `/estimated_external_force` could also reach the
existing robot controller. Real-time force inference/admittance integration is a
separate task; this preview does not make those control paths ready.

Validation: 383 GUI functional tests passed (including three screen sizes,
zero/negative/stale/nonfinite values, unit/axis conversion and fixed legends),
gui_py_pkg symlink build passed. Isolated ROS domain186, remapped force/motor
topics: both RELIABLE and BEST_EFFORT synthetic publishers were received and
timed out correctly; zero motor messages. No production nodes were restarted.

## Existing controls

Recording label (2026-09-25): set `contact_segment_id` beside the force-axis
checkbox before Record (integer 0..18; 0 = free-motion experiment). The label is
sent atomically with recording settings and stays fixed through capture/export,
regardless of measured force. Stop and finish exporting before changing it.
It is saved in session metadata, per-topic time-series CSVs and summary.csv.
The control shares the existing row; no additional scrolling is introduced.

For IK-based tilt-only/pan-only/two-axis sine motion, use the compact **IK sine**
row immediately below **Move Tip**. **A** is each axis amplitude [deg], **φ** its
phase [deg], and **T** the common period [s]. Defaults: Pan A=45, φ=90; Tilt A=45,
φ=0; T=20 (±45° around zero). Set an axis A=0, φ=0 for single-axis motion; this
targets zero on that axis, so an initially bent axis can move to zero first.

Select **Kinematics**, then **Start** atomically applies settings and starts the
existing `/kinematics/sine_motion` service only after successful acknowledgement.
No mode switch, motor enabling or speed-limit increase is performed automatically.
**Stop** stops new targets without returning to zero; it is NOT an emergency stop.
Settings lock while running/pending. Status distinguishes Running, Dry run,
Stopped, rejection and unknown result; hover it for the controller's full reason.
Responses are asynchronous (3 s deadline); uncertain starts request Stop and
late acknowledgements cannot trigger a new start. Pending settings must settle
before another Start can overwrite them. No repeated/automatic Start requests.
GUI edits alone do nothing; they are local until Start and reset on GUI restart.
The waveform ignores the manual Move Tip Absolute/Relative selector: it always
uses absolute aggregate IK angles about zero, not per-joint or Cartesian targets.
See [IK sine instructions](../../docs/IK_SINE_MOTION.md) for existing safety gates
and terminal commands. Stop and confirm before closing an externally managed
controller; closing the GUI is not a guaranteed motion stop.

Recording update: System → Data recorder starts an idle recorder;
Record starts a timestamp-preserving numeric rosbag session plus **HRM crop RGB/depth**
in `images/hrm_crop/` and `images/hrm_crop_depth/`. The depth is raw uint16 PNG or
float32 NPY, not colorized. Indexes retain source stamps/frame IDs, contact label,
depth calibration/scale. Full HRM RGB/depth, tag images, binary/skeleton and
3D visualization are excluded from the default bag. Existing `csv/summary.csv`,
per-topic numeric CSV and metadata are preserved; old sessions are not changed.
The crops support ROI-aware 3D reprocessing with time matching; discarded full-image
pixels and online filter history during unrecorded intervals are not recoverable. The button follows real
`/data/record_status` states and shows startup errors, flushing and stale status.
Wait for `recording` before data-collection motion and `stopped` before shutdown.
Use existing Stop followed by a new Record when moving the F/T sensor; pause/resume
and interval IDs are not added (user's final choice). Form temporal training windows
within each saved file before pooling samples. A missing new crop depth publisher
does not prevent legacy recording startup, but yields a visible depth coverage warning.
Record Stop flushes the numeric bag and image archive, then enters `exporting` while a separate worker
checks required data and generates per-topic CSV. New Record is disabled until
completion. DATA WARNING/INCOMPLETE uses amber; it is not a healthy-data claim.
Stopping the component/GUI before this completes preserves the bag but can leave
CSV pending/interrupted; the record_pkg manual exporter can finish it later.
See [record_pkg instructions](../record_pkg/README.md) for saved topics/settings.
Position/admittance modes are **not yet cleared for physical closed-loop use**;
see [control readiness audit](../../docs/CONTROL_READINESS.md).

Start the system with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch launcher system.launch.py
```

`system.launch.py` now starts only the GUI. The GUI starts and owns the ROS 2
components listed in `config/system_components.json`. Each row shows whether
the expected ROS node is starting, running, stopped, failed, or was started by
another terminal. Externally started processes are displayed but are never
stopped by the GUI.

Stop now sends SIGINT **only to the owned ROS launch parent**; launch forwards
the request to its nodes. GUI launches use noninteractive stdin so forwarding
does not depend on the terminal used to open the GUI. Repeated Stop clicks do
not send duplicate SIGINTs. The GUI retains the process-group ownership if
children outlive their launch parent; after 12 s it may send SIGTERM only to
that owned group, allowing ROS launch's own shutdown escalation to run first.

Status after Stop is normally **STOPPING → STOPPED**. If the owned group has
exited but ROS still lists its node names, **STOPPED\*** / **ROS cleanup…** is
shown until those names disappear. This is not classified as External, and a
duplicate restart is blocked while the graph is ambiguous. Nodes that were
never GUI-owned, or appear again after the old names cleared, remain External
and cannot be stopped here. An unexpected launch failure remains FAILED after
graph cleanup; signal exit codes from a requested Stop are not treated as a
new failure.

Shutdown validation (2026-09-14): 31 focused GUI tests passed. Three real
robot_control launch/stop cycles in isolated ROS_DOMAIN_ID=74, with motor output
disabled, each completed in ~0.15 s with exit code 0 and STOPPING → STOPPED.
No production motor processes were changed during this test.

The ordered default schedule is:

1. HRM camera (CUDA, D405): 0 s
2. Tag camera (CUDA, D435i): 0.5 s
3. serial sensors: 1 s
4. motor TCP bridge: 2 s
5. robot control: 3 s
6. segment estimation: 4 s
7. AprilTag rectification + detector + stamped pose helper: 6 s

The recorder component is manual-only; its optional scheduled delay remains 5 s.
The Record/Stop button controls data capture independently. This schedule is used by **Start auto
components**, or when launched with `auto_start_components:=true`. The current
`launcher/system.launch.py` default is `false` (operator setting, preserved).

AprilTag is in the automatic component list (existing global auto-start toggle
is unchanged). It uses 20 mm ID0/ID1 tags (outer black-border side, excluding the white margin). The default recording profile
requires both tag camera poses and ID1-in-ID0 relative pose; put both tags in view.
`daq_pkg` and `MasterMACS_pkg` are intentionally not present. Edit
`auto_start` and `delay_sec` in the JSON file to change the defaults, then
restart the GUI. A rebuild is required only if the installed config is not a
symlink.

The actuator gate is OFF on startup. IK target lengths remain visible, but
`robot_control_pkg` blocks `motor_command` until **Enable physical motor
output** is explicitly confirmed at the top of the **Motor control** panel.
The four motor rows are ordered East, West, South, North. Direct/manual input
is a relative encoder-count increment, not an absolute motor position.

## CUDA camera selection (2026-09-18)

In **System → Packages**, **HRM camera (CUDA)** and **Tag camera (CUDA)** each
have their own compact model selector and optional **Serial** field inside the
component row, not a new settings panel or scroll area. Defaults are **D405**
for HRM and **D455** for tags. A missing/mismatched camera is not replaced with
a different model. Changing one row does not change the other camera.

Choose the model before **Start**. With two cameras of the same model, enter
the camera **Serial Number** displayed by RealSense Viewer (not its **ASIC
Serial Number**, firmware-update ID, or Linux USB sysfs serial). Leave the
serial empty to match only the selected model. Both filters must match when
the serial is specified. Selection is local to this GUI session; restarting
the GUI restores D405/D455 respectively with no serial filters. The controls do not enumerate or
open camera devices and cannot block GUI startup while USB is unstable.

The inputs are locked while the component is running, starting, stopping or
externally managed. Stop the camera and wait for **STOPPED** before changing
the selection. An **External** camera must be stopped in its original terminal;
the displayed selection does not identify or reconfigure that external camera.
No selection change automatically restarts a device or changes USB speed.

### Live camera exposure (2026-09-25)

The compact **Exposure** row at the bottom of Packages controls the running
**HRM** (`/camera/camera`) or **Tag** (`/tag_camera/tag_camera`) camera:

1. Select HRM or Tag. **Read** fetches current driver parameters and valid limits;
   the row also reads once when the selected camera first appears.
2. Check **Auto** for automatic exposure, or uncheck it and enter a manual time
   in **ms**. Editing the box/value does not send any setting.
3. Press **Apply**. The GUI rechecks the camera module/range, waits for each ACK,
   then reads parameters back. **Applied** means parameter readback matched.
   Hover the status/exposure value for details; errors/partial changes require Read.

The running node's parameters determine the sensor module, not the next-launch
camera model selector. D435i/D435/D455/D415 RGB uses `rgb_camera` (0.1 ms per driver
unit); D405 uses `depth_module` (0.001 ms per unit) and affects color **and depth**.
While Auto is ON, the grey exposure field is the stored manual setting, **not
the currently measured automatic exposure**. Supported limits/steps are queried.
Auto ON sends only the auto flag; manual Apply sends Auto OFF first, then exposure.
Gain, frame profile and auto-exposure-priority are not changed. Long manual
exposures may reduce FPS; at 30 FPS a frame period is about 33.3 ms.

All service discovery/requests are asynchronous with a 3 s timeout per phase;
offline cameras do not freeze the GUI. An explicit Apply can configure an
externally launched camera, but does not stop/restart it. Settings are **live
only**: camera restart uses the existing launch YAML defaults. No camera writes
occur on GUI startup, model selection, Read, or checkbox/value edits alone.

Right-side force mappings keep each label beside its combo (4 px), with 16 px
between axis groups. Operation/Actuation Mode choices use natural widths,
12 px gaps and trailing free space; control behavior is unchanged.

CUDA librealsense, Fast DDS image transport settings and existing `/camera/camera`
topic names remain unchanged. Terminal launches use the same D405 default:

```bash
./scripts/run_realsense_cuda.sh
./scripts/run_realsense_cuda.sh device_type:=D455 serial_no:=_043422251095
```

The leading `_` keeps numeric serials as strings, including leading zeros;
the GUI adds it automatically. Existing D405 stream settings are retained.
When intentionally using a different camera, verify its supported stream
profiles, intrinsic calibration and ROI, then restart estimation to reset its
base calibration and filter state. Stop motion/recording before switching.

The second camera is independently managed. Start **Tag camera (CUDA)** before
**AprilTag**, and stop RealSense Viewer streaming from that camera first to
avoid competing for the device. The current D435i SDK serial is `949122070024`.
This launches RGB8 **1280×720 at 30 fps** only; depth, alignment, infrared, IMU
and pointcloud processing are off because AprilTag pose uses RGB + intrinsics.
CUDA librealsense is used for this camera too; that does **not** make the
existing CPU `apriltag_ros` detector or `image_proc` rectifier a CUDA algorithm.

| Use | Node | RGB input / optical frame |
| --- | --- | --- |
| HRM estimation | `/camera/camera` | `/camera/camera/color/image_rect_raw` / `camera_color_optical_frame` |
| AprilTag | `/tag_camera/tag_camera` | `/tag_camera/tag_camera/color/image_raw` / `tag_camera_color_optical_frame` |

The D435i raw RGB is rectified with its matching `color/camera_info` (K/D/R/P),
then `/tag_camera/tag_camera/color/image_rect` feeds the detector. This preserves
source image timestamps, frame and resolution. The same CameraInfo's P matrix
provides rectified intrinsics for tag pose. No crop or artificial camera-to-camera
identity TF is applied. Existing `/detections`, `ID0`/`ID1`, and `/apriltag/.../pose`
outputs stay unchanged; poses now refer to the tag camera, and ID1-in-ID0 remains
camera-placement independent. Both tags must be visible in the same frame.
To compare with `hrm_base`, calibrate the ID0-to-base and ID1-to-tip mount offsets;
camera timestamps still need offline alignment, not publish-time pairing.
The rectifier launch independently enables the existing image-sized FastDDS
profile; it does not rely on inheriting settings from the separate camera process.
Operator-supplied DDS profiles or another RMW implementation are retained.

```bash
ros2 launch launcher cuda_apriltag_camera.launch.py serial_no:=_949122070024
ros2 launch launcher apriltag.launch.py
```

Live validation on 2026-09-18: D435i serial `949122070024`, USB3.2, CUDA SDK
library confirmed. In a 15 s static-scene test, RGB, rectified RGB and detections
each delivered 450 frames (~29.98 Hz); ID0 and ID1 were both detected in all
450 frames. Individual poses and ID1-in-ID0 relative pose also delivered ~30 Hz.
The reported distortion coefficients were zero, so the rectifier used its
calibrated pass-through path. This verifies the camera/tag pipeline, not 1 mm
pose accuracy, moving-tag robustness or simultaneous D405/estimator throughput.
Only the test-owned processes were stopped afterward. To repeat without motors:

```bash
# Use a NEW output directory and an otherwise empty ROS domain.
python3 scripts/test_d435i_apriltag_live.py --serial-no 949122070024 \
  --domain-id 96 --duration 15 --require-tag1 \
  --output-dir /tmp/hrm_d435i_apriltag_new_test
```

The recorder stores the second CameraInfo and tag detections/poses, camera
parameters and profile YAML, **not its raw or rectified RGB** in the lightweight
default profile. Required CameraInfo and two-tag poses are unchanged; tag
redetection from these recordings is not possible without separately saved images.
The standard rectification dependency is `ros-humble-image-proc`; the launch
must not silently pass distorted RGB to a detector expecting rectified pixels.
On this PC it is installed in the ignored workspace prefix `.ros_image_proc/`
because system package installation was unavailable. The launch automatically
uses this prefix only when the system package is absent. Official ROS repository
signatures and package SHA256 checksums were verified; no system ROS libraries,
APT sources or keyrings were replaced. This local dependency is not copied by Git.
On another Jammy/amd64 Humble PC, install `ros-humble-image-proc` normally, or use:

```bash
bash scripts/install_image_proc_local.sh
# Existing local installation: check without reinstalling
bash scripts/install_image_proc_local.sh --verify-only
```

For a deliberate single-D405 fallback with an already rectified input:

```bash
ros2 launch launcher apriltag.launch.py rectify:=false \
  image_topic:=/camera/camera/color/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

## ESP32 serial port selection (2026-09-17)

In **System → ESP32 port**, click **Refresh ports** after plugging in the ESP32,
choose its `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`, etc., then click
**Serial sensors → Start**. The editable field also accepts an existing
`/dev/serial/by-id/...` path if a stable device identifier is available.
Enumeration only lists devices; it never opens them to identify the ESP32.

At GUI startup and Serial Start, ports are refreshed automatically. A single
connected USB/ACM port is selected if the previous selection is empty or absent.
With multiple ports, choose explicitly; device enumeration cannot identify which
is the ESP32. An unselected or
missing port skips only the serial component, with an explanation next to the
selector. Other component schedules and motor output gating are unchanged.
The GUI does not save the choice across restarts or guess which device is ESP32.

While running, **Launch port** is the path passed to the current GUI-owned reader,
not a claim that sensor data is arriving. **Next Start** is the pending selection.
Changing the selection or refreshing does not reconnect/stop the active reader.
To change connections, **Stop Serial sensors → wait for STOPPED → select port →
Start**. Refresh never switches an active reader. When stopped, an absent port
can be replaced by the sole available port. External readers are not reconfigured
or stopped by this GUI.

F/T plots share their existing canvas: force (fx/fy/fz) across the top and torque
(tx/ty/tz) across the bottom, each component with independent vertical scales,
light horizontal grids and fixed upper-right legends. Torque numeric
readouts display one decimal; plotting and ROS messages retain full precision.

The selected path is passed as the serial node's read-only `serial_port` ROS
parameter. Its value is included in the recorder's existing `/serial_read`
runtime-parameter snapshot when that node is available. Terminal launch remains
available (default port is still `/dev/ttyUSB0` outside the GUI):

```bash
ros2 launch serial_pkg _launch.py serial_port:=/dev/ttyACM0
```

Port permissions and reconnect/read-loop behavior are unchanged. There is no
automatic `sudo chmod`, and baudrate remains 921600. The port selector did not
change sensor zero/filter behavior or ESP32 firmware. Tests use mocked serial devices/processes;
the hardware must still be checked by selecting the actual port and watching
the existing sensor receiving indicators.

Follow-up zero-value fix (2026-09-17): startup F/T zeroing now accepts zero-valued
channels, so an absent CAN module reporting zeros no longer blocks loadcell
publication. Frames must contain exactly ten finite numbers; invalid lengths,
NaN/Inf and malformed input are rejected. Thirty initial reads are still flushed,
then 100 accepted F/T samples averaged; loadcells are not zeroed. Stop/error no
longer falsely reports successful zero setup. Missing serial data can still wait;
this is not an automatic reconnect or timeout implementation.

After connecting CAN, let the F/T readings settle and use **Set sensor zero**.
Zero force values with CAN disconnected are NOT valid training labels. The
current serial protocol has no CAN-health bit; fresh ROS topics (and recorder
freshness checks) alone cannot prove the F/T sensor is connected.

## Estimation crop / ROI editor (2026-09-18)

Estimation uses the D405's full `/camera/camera/color/image_rect_raw` image,
then crops it internally along with aligned depth. AprilTag now uses the
**separate tag camera's full rectified RGB**. ROI changes below affect only
estimation, not either camera's original stream or AprilTag detection.

1. Stop physical motion, disable motor output and finish any recording/export.
2. Stop **Segment estimation** and wait for **STOPPED**. Keep the camera running;
   AprilTag can also remain running. Stop externally launched estimators in their
   own terminal first.
3. Open **System → Estimation ROI…**. Drag a rectangle on the full color snapshot,
   or enter `x`, `y`, `w`, `h`: **top-left** pixel position and width/height,
   not center coordinates. Include the whole HRM/base and its intended motion
   range; exclude a competing AprilTag white region where possible.
4. Click **Apply ROI**, then **Segment estimation → Start**. This is deliberately
   startup-only: a fresh estimator resets its base averaging and filter history.
   It does not reuse points or camera offsets from the previous crop.

**Save as startup ROI** is checked by default. Apply then updates only `x/y/w/h`
in the installed `config_ROI_ref.json`, following symlinks to the source file
for a symlink build. Other JSON fields are preserved. Uncheck to use the choice
only for subsequent starts in this GUI session; Cancel changes nothing.

The editor takes one full-resolution snapshot and releases its temporary color
subscription; **Refresh snapshot** takes another. A snapshot is required to
validate the rectangle. Letterboxing/display scaling does not change the saved
pixel coordinates. An out-of-bounds ROI is rejected, never silently clipped.
The estimator also checks incoming RGB/depth dimensions and warns/skips invalid
frames if the camera resolution changes. Choose a new valid ROI before use.

GUI startup passes `roi_x`, `roi_y`, `roi_width`, `roi_height` as read-only ROS
parameters. Direct `_launch.py` uses ROI JSON defaults unless these arguments
are supplied. The recorder's existing `/segment_angle_estimator` runtime
parameter snapshot captures the active ROI; the config file alone is not proof
of which override was running. Recording and estimator auto-start are blocked
while the editor is open; changing ROI during recording is not supported.

Restart the GUI after upgrading. Hardware was not started during validation;
functional tests use synthetic images, offscreen Qt and mocked processes.
Both estimation_pkg and gui_py_pkg builds passed, together with 320 focused
functional tests across GUI/estimator/recorder (legacy style tests excluded).

## Layout and live images

### Per-record force axes (2026-09-18)

Above the Record button, **Force axes for CSV (sensor → hrm_base)** optionally
adds `aligned_fx`, `aligned_fy`, `aligned_fz` alongside the original force fields.
Check **Add aligned_fx / aligned_fy / aligned_fz**, then choose each **Base output
axis = signed sensor input axis**. Example: `Fx=+sensor Fx, Fy=-sensor Fy,
Fz=-sensor Fz`. For swapped Y/Z: `Fx=+sensor Fx, Fy=-sensor Fz, Fz=+sensor Fy`.
The original columns and units are never replaced or rescaled.

The default is OFF; selections persist between sessions while this GUI stays
open, not across GUI restarts. Each sensor axis must occur once and the mapping
must be a right-handed rotation (not a reflection). Incorrect combinations are
displayed as errors and cannot start Record when alignment is enabled.

Record first waits for recorder acknowledgement of both settings, then starts
the bag. Settings are locked during capture/flush/export and saved separately
for each session. If recording is started externally, the panel shows its frozen
configuration while locked. Restart both GUI and the idle Data recorder component
after upgrading; old recorder versions reject these new parameters.

After Stop, the recorder automatically exports separate raw/Kalman force CSVs
with derived columns, target `hrm_base` frame and numeric-validity flag. Original
bag, live sensor data, control and protection are unchanged. This is fixed-mount
axis alignment, not torque-origin conversion, units/gain calibration, action/
reaction correction or CAN-health detection. Keep sensor orientation fixed
within a session. See [recorder details](../record_pkg/README.md) for the saved
matrix, CSV column names and offline interpretation.

The left column keeps System above Sensors; the right Motor control column
starts at roughly 60% of the window width. Drag the splitter to adjust it.
The right-hand scroll container has been removed: compact motor rows, filter
controls and the Record/status row leave room for all three image tiles.
The actuator-enable checkbox stays at the top; no motor safety or command
behavior was changed. Layout bounds were checked at 1366×768, 1500×900 and
1920×1080 (standard desktop scaling, force-axis settings on/off, two-line image
status). Very small screens or increased OS font scaling may need a larger
window. The existing left raw-sensor scroll area is retained.
**Set sensor zero** is below the raw sensor readings and still calls
`/serial_data/set_zero`. It does not home or zero motors. The current serial
implementation averages force/torque samples; its loadcell-offset averaging
is commented out and was not changed by this UI update.

Raw loadcell display has **four channels**, `Loadcell #1–#4 (g)`, matching
`/loadcell_state.stress[0:4]` in message order. The independent GUI parameter
`num_loadcells` defaults to 4 (also set in the launch file). Missing channels
show `—` rather than retaining old numbers. Serial publishing and the
force/torque plot are unchanged. The focused GUI suite now has 34 passing tests.

Below the motor/record controls are three aspect-preserving previews, left to right:

- `/camera/camera/aligned_depth_to_color/image_raw`: depth colorized with JET,
  near = red, far = blue, invalid/zero depth = black. Adjustable fixed range
  defaults to **0.070–0.600 m** to avoid frame-to-frame auto-scaling flicker.
  `16UC1` is interpreted in millimetres and `32FC1` in metres. This only changes
  display colors, never estimator inputs or published depth values.
- `/estimated_segment_skeleton_image`: the estimator's existing image,
  displayed as RGB (mono/BGR/RGBA encodings are converted for display).
  Its top dark margin shows **hrm_base | XYZ [mm]** and two signed one-decimal
  position rows: **Est** from `/estimated_tip_position` (the length-constrained
  fitted final boundary), and **FK** from `/kinematics/fk_tip_position`
  (robot_control's measured-angle forward kinematics). No centre point, unscaled
  endpoint, or headerless held control value is substituted for either tip.
  The GUI keeps up to eight references per image/point stream, preferring the
  newest complete exact-source-stamp set with all receipts at most 0.2 s old.
  If no complete set is available, it shows the latest image and each tip row
  independently when matched; it never attaches a different frame's position.
  Points must be in `hrm_base`, while the image keeps its camera frame. Missing,
  invalid, stale or wrong-frame values show a status instead of numbers. Image
  conversion failure clears both coordinate rows; point-only updates do not
  reconvert the same image. No FK, TF lookup, image processing or publisher is
  added. The header stays inside the existing canvas without another layout row.
- Tag camera: `/tag_camera/tag_camera/color/image_rect`, with the existing
  `/detections` corners and IDs drawn on it. ID0 is green, ID1 is yellow. No
  detector runs in the GUI and no overlay image is republished. The already
  installed `apriltag_msgs` dependency is declared; no new package installation.
  Start **Tag camera (CUDA)** then **AprilTag** for overlays. If the rectified
  stream is absent for one second, the preview subscribes to raw RGB instead and
  displays **Raw RGB — no overlay**; that fallback subscription is removed once
  rectified frames return. Raw images never receive rectified-pixel overlays.

  The tag tile's top dark margin also shows **ID0 -> ID1** position as
  `XYZ [mm]` and orientation as `RPY [deg]`, signed to one decimal. This subscribes
  to the existing `/apriltag/tag1_in_tag0/pose` helper, which starts with the
  AprilTag component; recording does not need to be running. RPY is extrinsic
  XYZ Euler, `Rz(yaw) Ry(pitch) Rx(roll)`, not HRM joint pan/tilt. Values refer
  to the tag frames, without tag-to-HRM mounting compensation. Euler angles can
  wrap at +/-180 degrees and are singular at pitch +/-90 degrees.
  Numbers are shown only for both tags in the displayed exposure and an
  exactly timestamp-matched pose with parent `ID0`. Missing/invalid/stale poses
  show a status instead of held numbers. The header stays inside the existing
  canvas; if the letterbox is too small, the whole image shrinks proportionally
  below it. No new row, scrollbar, detector, package, or published topic is added.

The image-only `gui_image_preview` node uses a separate executor thread with
BEST_EFFORT / KEEP_LAST(1) / VOLATILE subscriptions. Depth/skeleton retain one
pending frame. Tags retain at most eight image/detection/pose references each, to
match asynchronous results by **exact source timestamp AND optical frame**.
A matched pair must be at most 0.2 s old by receipt time; the display uses that
pair's source image, never old corners on newer pixels. Without a match the
latest image stays visible without an overlay. Empty detections clear the tag
outlines; missing/stale detector output is distinguished from **No tags detected**.
The pose cache also expires after 0.2 s; a pose-only arrival refreshes its text
without repeating image conversion or corner drawing.
If image conversion fails, pose numbers also clear so a new pose cannot label an
older retained image. This display feature was checked with 92 offline
image/pose/layout tests and a synthetic 1366x768 screenshot (2026-09-25);
the GUI package build passed. No live cameras or motors were run for this change.
The Qt display is capped at approximately 30 Hz. Images are downscaled to the
tile width (at most 640 pixels) before coloring and drawing. Missing/stale images
are explicitly labelled, and each preview works without the other streams. Controls and image
rendering stay on the Qt thread; [QImage](https://doc.qt.io/archives/qt-5.15/qimage.html)
owns a copy of the display bytes so it never references a released NumPy buffer.

Validation (2026-09-18 late evening): `gui_py_pkg` rebuilt; 316 focused
GUI/launch/recorder/probe tests passed. Actual running D405 depth and D435i RGB
remained ~30 Hz. A read-only offscreen Qt preview rendered depth/tag images at
~28.6/~28.5 Hz; refresh mean 3.78 ms, p95 5.52 ms (skeleton producer was off).
The current physical scene was dark and the detector correctly returned empty
arrays, so live tag detection was not claimed. Separately, an earlier illuminated
D435i image with its original calibration was replayed through the real
AprilTag detector in isolated domain96: 296 displayed frames had both ID0/ID1
outlines, zero timestamp/frame mismatches; tag-only refresh mean 1.37 ms.
This replay proves overlay routing/drawing, not moving-scene accuracy. Existing
operator GUI and both cameras were left running untouched; only test-owned
AprilTag/replay/preview processes were closed. No motor commands were sent.
For a read-only check against already running producers:

```bash
QT_QPA_PLATFORM=offscreen python3 scripts/test_gui_tag_preview_live.py \
  --duration 15 --require-overlay --output-dir /tmp/hrm_preview_new_test
```

Use a new output directory; omit `--require-overlay` to test raw-only viewing.
Restart the GUI to load the new layout/preview code when safe to stop its owned
components. The currently open operator window is not hot-reloaded.

Validation (2026-09-14): 19 focused GUI tests passed and the package built.
An isolated ROS domain test displayed both synthetic 30 Hz streams at ~27 Hz
while receiving synthetic 500 Hz motor feedback; Qt heartbeat p95 ~22 ms,
maximum ~28 ms. A separate read-only live-camera check displayed 391 depth
and 371 skeleton frames in ~14 s. This measures preview refresh, not camera
capture rate. No motor commands, gate changes, or hardware restarts were made
during these checks. Existing workspace-wide flake8/pep257 tests still fail
on legacy/generated sources; the new preview module passes targeted checks.

Stop physical motion safely before restarting the operator GUI to load changes.
Closing the GUI also stops the component processes it owns; it is not an
emergency-stop function.

The CUDA camera launch and estimator automatically load the image-sized Fast DDS
shared-memory profile. See [runtime checks](../../docs/REALTIME_ESTIMATION.md).

## Force / contact inference preview (2026-10-11)

The optional **Force / contact** row in Packages starts the standalone
`estimation_pkg/force_id_estimation.launch.py`. Auto-start is OFF. Put trusted
bundles below `src/estimation_pkg/models` (nested directories are supported),
press the refresh button, review the default or choose another model, then press Start. The
selector looks for `hrm_bundle.pt`; it does not load Torch into the GUI and does
not use validation `.npz` files. Stop the component before switching models.

The **Contact ID** strip above the sensor plots displays 18 cells using the
original `p_i` values from `/force_id_estimation`
(`custom_interfaces/msg/ForceIdEstimate`). Only the candidate positive-ID
argmax's ±ID tolerance band is colored; the tooltip of every cell retains its
original probability. Colors use a fixed 0..1 scale, without per-frame or
conditional renormalization. `band Σp` is probability mass, not accuracy.
Class0 displays p0; masked models explicitly show that p0/load state are
unavailable. An unloaded control ID18 is a default operating ID, not detected
tip contact. Invalid/warming/stale inputs clear the heatmap and displayed force;
they are never interpreted as no load.

**Body monitor** and ±ID settings can be changed live through the estimator's
atomic parameter service. Before the node is running, they configure its next
GUI Start. Raw probabilities remain visible with Body monitor OFF. Neither the
toggle nor the preview enables motors. Separately, robot_control now subscribes
to the unified force topic in explicitly selected ADMITTANCE mode; valid tip
force produces Y/Z goals, but moves motors only with the independent gate enabled. Body
force is ineligible in both monitor modes and currently causes a software hold.
See [controller setup](../robot_control_pkg/README.md#learned-force-yz-admittance-2026-10-15).

Unified predictions are displayed in **hrm_base, mN**, converting the message's
N by 1000 without changing axes or applying the controller's force-sign
inversion. Existing F/T traces remain in **sensor axes**, so the graph labels
explicitly distinguish their frames; their overlaid curves are not automatically
aligned ground truth. Tip-only models' body/unknown-load force output remains
diagnostic, not a validated body force. Once a unified message has arrived, the
GUI never substitutes a legacy `/estimated_external_force` value for a stale or
invalid unified estimate.

Package descriptions now live in name-label tooltips to keep model selection
and all camera/motor/record controls visible at 1366×768 without adding a scroll
area. Hardware-free tests cover source priority, units, unavailable data,
unloaded/masked semantics, probability bands, and compact layouts.
