# robot_control_pkg

## Current bending convention

- `hrm_base` X is the axial direction.
- The 18 one-axis bending joints alternate `q1=tilt`, `q2=pan`, through
  `q18=pan`.
- q1 tilt is driven by the South/North cable pair and rotates about
  `hrm_base +Z`. A positive q1 bends the axial +X direction toward +Y.
- q2 pan is driven by the East/West cable pair and is the next orthogonal
  one-axis joint.
- `MoveToolAngle` inputs are degrees.
- `hrm_base` is the fixed D-H Base. There is no initial rotation:
  `hrm_fk_joint_01` is aligned with `hrm_base` at zero angle and its first
  variable rotation is q1 tilt about Base `+Z`.

## Safe IK preview

Motor publication is disabled by default. Start the node in dry-run mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_control_pkg _launch.py motor_output_enabled:=false
```

Send an absolute tilt/pan request:

```bash
ros2 service call /kinematics/move_tool_angle \
  custom_interfaces/srv/MoveToolAngle \
  "{mode: 0, tiltangle: 10.0, panangle: 0.0, gripangle: 0.0}"
```

Inspect the requested cable-length changes:

```bash
ros2 topic echo /kinematics/target_wire_length
```

The four `Float64MultiArray.data` entries are always ordered
`[East, West, South, North]` and measured in millimetres. They are cable-length
changes from the IK model, not encoder counts and not motor home offsets.

Do not enable `motor_output_enabled` until motor indices, encoder/winding signs,
zero positions, pretension, travel limits, stale-state handling, and emergency
stop behavior have been verified on the hardware.

## Position PD: angular-rate output, absolute Y/Z targets (2026-10-29)

Targets and FK feedback are in **hrm_base metres**: Y error drives tilt and Z
error drives pan; X is passive. The position controller uses the scalar PID
error law, with **I=0**, optional low-pass filtering of the error derivative,
and the restored **angular-rate output**:

```text
e = desired_position - measured_FK_position
angular_rate = Kp*e + Kd*LPF((e-e_previous)/image_dt)   # rad/s
delta_q = clamp(angular_rate, -speed, speed) * step_dt
q_command = clamp(q_previous + delta_q, -limit, limit)
step_dt = image_dt   # normal frames only: 0 < image_dt <= 0.25s
# After a feedback pause or longer gap: hold command, reset derivative history,
# re-anchor source time, then resume on the next regular frame (no catch-up).
```

There is no fixed `entry_angle + PD` correction any more: persistent error
continues to change the command. Once error and its derivative vanish the
current command is held. Accumulating command increments is NOT the PID I term.
Mode-entry angle/encoder/cable offsets remain solely for bumpless initialization
and encoder conversion, not as a fixed position-control reference.

Numeric source defaults were retained (P=20.55, I=0, D=5,
derivative filter tau=0.05s, maximum angular speed=40deg/s). Runtime GUI edits
are not persisted by this change. P has units rad/(m*s) and D rad/m.
The 2026-10-27 per-frame angular-increment change is superseded: integrate the
rate once using image dt. Constant-error P action now depends on elapsed time,
not the number of frames. The separate angular-speed and absolute-bending
guards remain; these are not an assurance of closed-loop stability.
A target step contributes to D (error derivative).
Frame pacing, camera-gap handling and motor/loadcell/force safeguards remain.
This is not full Cartesian IK or cross-axis compensation.

Position and Admittance share this inner position controller. The admittance
6x6 M/B/K solver, force sign convention and force eligibility checks are unchanged.

Position/Admittance motor velocity profile retains the user's value (**35** at
restoration). Parameter
`position_control/motor_velocity_profile_percent` accepts integers1–100;
software-hold profile remains10. The40deg/s angular-command cap, motor drive
profile and Cartesian sine reference limit10mm/s are separate limits.
The former admittance offset limit5mm and offset-velocity limit2mm/s were removed.
Do not raise gains blindly to compensate actuator lag; command accumulation can
lead the real motor even with
I=0. No new tracking-error anti-windup or automatic gain changes were added.

The operator's feedback freshness timeout1.0s is preserved. It is independent
of the0.25s maximum integration interval; continuous frames slower than4Hz do
not advance this controller. Recovery preserves the goal/command, not a fresh
mode entry. The next normal frame resumes with cleared derivative history.

See `docs/CONTROL_READINESS.md` for the current law and historical diagnosis.
Offline builds/tests do not replace the running robot controller: turn motor
output off and stop that node before rebuilding/restarting it for a new test.

## Cartesian reference sine for Position / Admittance (2026-10-26)

`/position/sine_motion` (`std_srvs/SetBool`) starts/stops an absolute **hrm_base**
Y/Z reference, in metres. It is separate from `/kinematics/sine_motion` (degrees).
Start requires ready Position or Admittance mode and no other active motion.
The caller must select/prepare the mode; Start never enables motor output.

```text
y_ref = Cy + Ay * sin(2*pi*t/T + phi_y)
z_ref = Cz + Az * sin(2*pi*t/T + phi_z)
Position target   = nominal reference
Admittance target = nominal reference + force-derived offset
```

Parameters on `/robot_control`:

| Parameter | Default | Units |
|---|---|---|
| `motion.position_sine.y` | `[0.0, 0.010, 0.0]` | centre m, amplitude m, phase deg |
| `motion.position_sine.z` | `[0.0, 0.010, 90.0]` | centre m, amplitude m, phase deg |
| `motion.position_sine.period_sec` | `20.0` | seconds |
| `motion.position_sine.max_speed_m_s` | `0.010` | per-axis nominal reference m/s |

X is unchanged/passive. A=0 commands the specified centre. The reference first
approaches the initial phase from the current nominal target without a step,
then advances only on accepted source-image frames. Missing/long image intervals
freeze phase (no elapsed-time catch-up); the existing feedback policy resumes on
fresh data. No per-frame reset of PD, cable references, or admittance state occurs.

Amplitude must be nonnegative, period >=1 s, and per-axis `2*pi*A/T` must not
exceed the reference speed limit. The conservative Y/Z envelope must fit inside
the total-length radius. This does not certify coupled workspace reachability or
physical stability. Existing PD 5deg/s, bend/travel/tension/force guards remain.

Stop retains the last nominal target and leaves feedback control running. Mode
or motor-output changes and controller faults stop the waveform. Active-waveform
edits, manual position goals, and conflicting motions are rejected. Existing
Admittance force eligibility/freshness/sign checks remain authoritative.

- `/position/sine_status`: String, transient-local; RUNNING/DRY_RUN/PAUSED/STOPPED.
- `/position/sine_reference`: PointStamped, metres/hrm_base, original image stamp;
  the nominal reference before admittance offset, emitted on active valid steps.
- `/position_controller` retains its existing effective target diagnostics.

The GUI exposes amplitudes/phase/period and takes centres from its Move Tip Y/Z
fields. For a prepared controller, the service equivalents are:

```bash
ros2 service call /position/sine_motion std_srvs/srv/SetBool '{data: true}'
ros2 service call /position/sine_motion std_srvs/srv/SetBool '{data: false}'
```

Stop is not a physical emergency stop. Start can move an enabled robot; use a
clear workspace and verify small-amplitude tracking before increasing motion.

## Learned-force Y/Z admittance (2026-10-15)

Software integration is implemented and tested with synthetic feedback, **not
validated for physical closed-loop stability**. No physical motors were moved.

```text
/force_id_estimation [ForceIdEstimate, N / hrm_base]
  → eligibility + freshness checks → force_sign × XYZ
  → Y/Z admittance → Y/Z position goal → existing PD → cable IK → motor targets
                         ↑                       ↑
                  target Fy/Fz [N]       measured-angle FK tip [m]
```

X force remains observable but produces no X command. Near the straight pose,
the existing position mapping is **Y → tilt, Z → pan** (q1 rotates about base Z).
This is not full Cartesian inverse kinematics; bending-dependent coupling is
still deferred. Neither inference startup nor a target-force change enables
motors. The independent `motor_output_enabled` gate still defaults to false.

### Convention and parameters

The inference topic preserves the training/FT-sensor force sign. Per the training
handoff, `admittance/force_sign=-1.0` negates XYZ **once** inside robot_control;
there is no axis swap and no additional mN-to-N conversion. The controller's
`F_env` and target force mean **external force ON the HRM in hrm_base**, not force
exerted BY the HRM on its environment. Target zero + positive corrected Fy
produces a **positive** Y goal offset from a zero state. The earlier 2026-10-31
direct legacy-sign rollback was incompatible with this physical convention and
has been superseded. GUI prediction labels retain the original training sign,
so compare against `/admittance_controller.env_force`, not the unnegated label.

For each axis a ∈ {Y,Z}:

```text
M δa_ddot + B δa_dot + K δa = F_env,a − F_target,a
position_goal,a = reference_goal,a + δa
```

The reference is captured once at mode/output-gate entry from fresh FK feedback;
PD and admittance states are reset then. `/position/set_goal_position` can still
set the reference Y/Z goal after entry. The controller steps only on new source
image frames, using source-stamp dt; short image gaps do not duplicate steps.
The filter now restores the legacy6x6 matrix equations and M/B/K branch logic.
The node configures Y/Z entries and consumes Y/Z position offsets; the generic
filter itself also accepts other axes and coupled symmetric PSD matrices.
Inactive zero rows are excluded safely from singular solves. The legacy update
computes acceleration, then velocity, then position. For stiff/damped settings,
internal substeps refine numerical integration without clipping force, velocity
or displacement. **There is no longer an admittance offset/velocity clamp.**

The generic filter retains its legacy `desired - environment` algebra.
`AdmittanceController` passes `(-f_desired, -f_env)` at that internal boundary
to obtain the physical `F_env - F_target` RHS without changing its 6x6 dynamics.
This adapter is not a second sensor/frame conversion: node `force_sign=-1`
and public target/environment diagnostics stay unchanged; published `delta_force`
is the actual RHS `F_env-F_target`. For example, raw predicted Fy=+100 mN,
force_sign=-1 and target=0 gives corrected F_env=-100 mN and error=-100 mN.
A positive target/environment pair cancels, as does a negative matching pair.
No force deadband, offset clamp or velocity clamp is added by this correction.
Exact zero forces with zero state preserve the nominal goal, but a previous
offset/velocity evolves rather than being instantly cleared. A constant 50 mN
bias gives 2.5 mm equilibrium offset at K=20, or 1 mm at K=50; UNLOADED does not
zero this bias. These are model balances, not hardware accuracy guarantees.

Settings below are ROS parameters on `/robot_control` (double except the boolean ID gate):

| Parameter | Default | Meaning |
|---|---:|---|
| `admittance/target_force_y_N` | 0.0 | Desired external Fy on HRM, N |
| `admittance/target_force_z_N` | 0.0 | Desired external Fz on HRM, N |
| `admittance/use_contact_id_gate` | `false` | ID display-only by default; true enables tip-ID gating |
| `admittance/M_d` | 0.05 | Virtual mass, kg, both axes |
| `admittance/B_d` | 1.0 | Damping, N·s/m, both axes |
| `admittance/K_d` | 20.0 | Stiffness, N/m, both axes |
| `admittance/force_sign` | -1.0 | Training-force → force-on-HRM sign |
| `admittance/max_force_N` | 2.0 | Full XYZ estimated-force norm limit; also target Y/Z norm limit |
| `admittance/force_timeout_sec` | 0.25 | Receipt and source/publication freshness, s |

The ID gate defaults to **false** by the user's explicit 2026-10-26 choice:
contact localization is visualization only. During physically tip-only force
experiments, even a predicted body ID does not interrupt valid Fy/Fz feedback.
The model supplies one XYZ vector, not 18 separately selectable force vectors;
this does not isolate a tip component from simultaneous body/tip forces.
`true` restores the optional ID-gated policy below. Leave Admittance before
changing this boolean; changing it in an active Admittance loop is rejected.

The removed `admittance/max_offset_m` and `admittance/max_velocity_m_s` are no
longer supported. The remaining bounds are **not measured hardware safety
ratings**. Existing motor range, four-channel loadcell tension and angular
speed/angle limits remain. Target forces can change live; leave ADMITTANCE mode
before changing gains, sign or limits. Parameters are validated as a whole batch.
M/B/K must be finite, nonnegative and not all zero; timeout is in (0, 0.25] s.

### M/B/K selection assistant (2026-10-29)

The table lists source defaults, not live GUI edits or validated hardware tuning.
Read the actual parameters before a trial. GUI **Admittance → Read → Tune** opens
an offline calculator; it adds no streaming subscribers or additional plots to
the main GUI. The existing compact controller row keeps its height.

The calculator separates two objectives:

- **Force tracking (M–B):** K=0 removes spring-induced static force error, but
  persistent estimator bias or absent contact can continuously move the target.
  It is not an automatic remedy for unstable contact or poor position tracking.
- **Spring compliance (M–B–K):** K restores the nominal position; static force
  error can remain because `F_env-F_target=K*offset`.

Editable assumed requirements give the candidate:

```text
B [N s/m] = assumed force error [mN] / zero-offset velocity scale [mm/s]
M [kg]    = B * velocity time constant [s]
K [N/m]  = allowed spring-force contribution [mN] / correction distance [mm]
           (or zero in Force tracking mode)
```

The velocity relation neglects inertia and spring force; it is NOT a speed limit.
The error allowance is NOT a force deadband or a guarantee of actual error.
Initial error100mN/distance5mm are labelled illustrative assumptions; other inputs
are derived from the confirmed gains, so the dialog does not silently retune them.
It compares current and candidate values, `M/B`, `K*offset`, formal `error/K`,
K=0 bias drift and free-model decay. Example: M=.15/B30/K5 has M/B=.005s but a
slow free-model decay time near6s. Neither value is a measured contact-settling
time: environment stiffness, estimator error, inner-loop lag and coupling matter.

**Use candidate only stages the three input fields.** It never changes ROS
parameters. Leave Admittance and explicitly Apply separately after reviewing the
candidate with motor output disabled. No mode, Motor Enable, force signs, PID,
force targets, matrix formulation or protections are changed by this assistant.
Both Y/Z remain active with shared gains: Fz*=0 does NOT disable Z compliance.
Independent compliance-axis selection has not been added.

For a fixed-setting interval, use the read-only diagnostic (no rebuild):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/inspect_admittance_tuning.py --duration 15
# Optional new CSV; existing files are never overwritten:
python3 scripts/inspect_admittance_tuning.py --duration 15 --csv /tmp/hrm_tuning_trial_01.csv
```

It pairs `/admittance_controller`, `/position_controller`, `/control/reference`
by exact source-image stamps, reports missing pairs, and splits intervals on
force-target/M/B/K/nominal-reference/output-gate/reported pan-PD changes or gaps.
The legacy PositionControl message lacks separate tilt-PD gains, so keep BOTH
PD axes unchanged during a measurement. Tail statistics use at most the last2s:

- Force error RMSE/mean, spring/damper/inertia contributions, offset/velocity,
  inner-position error and command-angle peak are reported per axis.
- `SPRING_BALANCE` means observations are consistent with spring equilibrium;
  `POSITION_LAG` means effective target tracking is poor; neither proves a sole cause.
- `IN_BAND` requires every observed tail sample within the analysis tolerance,
  at least1s/10samples and output enabled; it is NOT an extrapolated stability or
  ground-truth accuracy claim. Short windows and disabled output are explicit.
- Default20mN/0.5mm tolerances affect the report only, never the controller.
  The tool reads controller-sign force (after `force_sign`), not raw model sign.
  It rejects unsupported coupled matrix rows rather than misreporting diagonal
  contributions. The controller itself retains its general6x6 solver.

Keep gains, nominal position and contact geometry fixed within each comparison.
Three-second steps are not automatically long enough to establish convergence.
First verify stable inner position tracking and the force direction/bias. Compare
one parameter change at a time; do not increase gains blindly to overcome a
static spring error. The diagnostic never starts the robot, changes any settings,
or substitutes for a hardware stop or an independent force reference.

### Commands and monitoring

For the fixed force-target staircase, run the standalone script from the workspace
root (no rebuild required):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/admittance_force_steps.py --axis fy --dry-run
python3 scripts/admittance_force_steps.py --axis fy --interval 3
# Or, in a separate trial, apply the sequence to Fz:
python3 scripts/admittance_force_steps.py --axis fz --interval 3
# Opposite signed force target, using the SAME convention as the GUI:
python3 scripts/admittance_force_steps.py --axis fz --sign -1 --interval 3
```

Each trial sends `0,200,300,400,500,400,300,200,0` mN, holding each acknowledged
target for3s, including both zeros (~27s plus service latency). Only the selected
axis is written; the other force target remains unchanged, so check/zero it in
the GUI if the experiment requires single-axis loading. The script converts mN
to N once; it does not apply the estimator's force-sign conversion to the target.
It prechecks the target-vector limit, awaits each service acknowledgement, and
stops the sequence on rejection/timeout. It never changes mode, Motor Enable,
M/B/K, position targets or PD gains. The operator must select Admittance and
verify the setup separately. Do not edit the same target concurrently in the GUI.

Completion leaves the selected target at0. Ctrl+C/SIGTERM/error after a write
attempt requests selected-axis0, with a prominent warning if that request fails.
Zero force is **not a motor stop**: the running controller can still move. Network
loss/SIGKILL/power loss can leave the last target active. Use Motor Enable or the
hardware stop for unexpected motion. `--dry-run` does not connect to ROS at all.
This is target generation only, not a guarantee of500mN safety or force tracking.

After restarting the rebuilt controller with motor output disabled and starting
the estimator, motor/loadcell feedback and a suitable force model:

```bash
# Begin with zero desired force (N), then select admittance in the GUI or here.
ros2 param set /robot_control admittance/target_force_y_N 0.0
ros2 param set /robot_control admittance/target_force_z_N 0.0
ros2 param set /robot_control control_mode 4

# Check READY (dry-run), force sign, goal offset and actuator preview first.
ros2 topic echo /position/control_status
ros2 topic echo /admittance_controller
ros2 topic echo /position/target_motor_command
```

For example, `ros2 param set /robot_control admittance/target_force_y_N 0.02`
sets +20 mN external force ON the HRM; this is a syntax example, not a validated
experimental target. Targets survive mode exit/re-entry within a running node,
but are zero again on restart unless explicitly supplied. Zero them before the
next trial. In free space a nonzero force target can move the goal to its limit
without ever establishing contact. With K>0, steady state satisfies
`F_env − F_target = K δa`, so **exact force tracking is not guaranteed**. K=0 removes
this spring bias but requires separate contact/stability validation; do not
blindly remove stiffness or increase bounds to overcome tracking error.

### Force eligibility and fault behavior

- Always require valid, finite `hrm_base` force, recognized masked/class0 mode,
  `tip` or `all_single_contact` model scope, ready status, a full input window,
  fresh source/publication timestamps and force magnitude within the limit.
  These conditions remain even when localization is visual only.
- Default `use_contact_id_gate=false`: do not gate on contact ID, load state,
  body-monitor flag or the ID-gated `tip_force_valid` diagnostic. A body-ID
  misclassification or unknown localization alone must not interrupt tip-loaded
  experiments. This assumes physical tip contact; it does not detect or safely
  resolve real body contact. Missing/invalid force is never treated as no-load.
- Optional `use_contact_id_gate=true`: accept only tip/operating-ID18. Native
  class0 uses raw0/18; new `status=ready_force_ranges` explicitly identifies the
  configured load rule for either model. All inferred axes inside inclusive
  bounds => UNLOADED/control18; outside => positive-ID winner. Raw IDs, original
  probabilities, candidate and flags are consistency checked. Loaded body IDs
  remain blocked regardless of Body monitor; a heatmap band touching18 is not
  sufficient. Unloaded force is not replaced by zero. Masked without an enabled
  load detector remains ineligible in this optional gated mode.
- Inference thresholds are in `estimation_pkg/estimation_pkg/id_localization_config.json`
  (mN, interpreted before controller sign conversion); restart inference to load.
  Thresholds are an operating convention, not validated force/contact truth.
- Before first readiness, unavailable feedback means WAITING and no command.
  Force receipt/source timeout and producer warm-up, missing/stale input or
  input-time-skew statuses pause updates and auto-resume on valid feedback.
  They do not consume the invalid payload, fabricate zero force, reset the
  goal, or advance the sine/admittance state over the missing interval.
  Other invalid force, model-name change, overforce, wrong frame/scope
  or existing motor/loadcell faults trigger the existing software hold/latch:
  hold **current motor encoder positions** if feedback is fresh, then require a
  mode exit/re-entry or output-gate transition to reset. No stale-force reuse or
  fabricated zero force; no automatic body-force evasive motion. When ID gating
  is explicitly enabled, ineligible body/unknown localization also triggers this
  hold. The default display-only policy intentionally does not.
- Camera and transient force gaps retain the prior goal and command and
  auto-resume; hardware guards still run. The motor can finish its last target
  while waiting: pausing feedback updates is not a physical stop. Integration
  velocity is cleared and no long-dt catch-up step is performed.
- Software hold is not driver shutdown or an emergency stop. If motor feedback
  itself is stale, current-position hold cannot be safely computed. Validate the
  drive-side watchdog and physical stop before powered testing.

Before hardware operation, verify position feedback directions/accuracy,
FT-to-HRM force sign, model validity near the tested poses, latency/filter delay,
contact stiffness, pretension, saturation and stop behavior. A 30-sample model
window does not itself mean 1 s of output latency, but model/filter/transport
lag must be measured. Start with zero-force compliance and one axis at a time;
target-force tuning depends on the actual contact environment.

### Verification

31 C++ tests (FK/IK, PD, sine and new Y/Z/force eligibility), plus isolated-domain
position/admittance smoke tests passed. GUI force-preview tests also passed.
Smoke tests publish only synthetic feedback, set motor output false, remap the
actuator topic, assert zero actuator commands and check domain isolation.

```bash
ctest --test-dir build/robot_control_pkg --output-on-failure \
  -R 'test_(admittance_yz|position_controller|surgical_tool_fk|ik_sine_motion)$'
# Source ROS and workspace, then use an otherwise empty private domain:
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/hrm_admittance_logs \
  /usr/bin/python3 -B src/robot_control_pkg/test/admittance_yz_smoke.py
```
