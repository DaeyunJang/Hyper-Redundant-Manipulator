# HRM operator GUI

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

1. CUDA RealSense camera: 0 s
2. serial sensors: 1 s
3. motor TCP bridge: 2 s
4. robot control: 3 s
5. segment estimation: 4 s

The legacy recorder is manual-only until its dataset interfaces are migrated;
its optional scheduled delay remains 5 s. This schedule is used by **Start auto
components**, or when launched with `auto_start_components:=true`. The current
`launcher/system.launch.py` default is `false` (operator setting, preserved).

AprilTag is available as an optional row and is not started by default.
`daq_pkg` and `MasterMACS_pkg` are intentionally not present. Edit
`auto_start` and `delay_sec` in the JSON file to change the defaults, then
restart the GUI. A rebuild is required only if the installed config is not a
symlink.

The actuator gate is OFF on startup. IK target lengths remain visible, but
`robot_control_pkg` blocks `motor_command` until **Enable physical motor
output** is explicitly confirmed at the top of the **Motor control** panel.
The four motor rows are ordered East, West, South, North. Direct/manual input
is a relative encoder-count increment, not an absolute motor position.

## Layout and live images

The left column keeps System above Sensors; the right Motor control column
starts at roughly 60% of the window width. Drag the splitter to adjust it.
**Set sensor zero** is below the raw sensor readings and still calls
`/serial_data/set_zero`. It does not home or zero motors. The current serial
implementation averages force/torque samples; its loadcell-offset averaging
is commented out and was not changed by this UI update.

Raw loadcell display has **four channels**, `Loadcell #1–#4 (g)`, matching
`/loadcell_state.stress[0:4]` in message order. The independent GUI parameter
`num_loadcells` defaults to 4 (also set in the launch file). Missing channels
show `—` rather than retaining old numbers. Serial publishing and the
force/torque plot are unchanged. The focused GUI suite now has 34 passing tests.

Below the motor commands are two independent, aspect-preserving previews:

- `/camera/camera/aligned_depth_to_color/image_raw`: depth colorized with JET,
  near = red, far = blue, invalid/zero depth = black. Adjustable fixed range
  defaults to **0.070–0.600 m** to avoid frame-to-frame auto-scaling flicker.
  `16UC1` is interpreted in millimetres and `32FC1` in metres. This only changes
  display colors, never estimator inputs or published depth values.
- `/estimated_segment_skeleton_image`: the estimator's existing image,
  displayed as RGB (mono/BGR/RGBA encodings are converted for display).

The image-only `gui_image_preview` node uses a separate executor thread with
BEST_EFFORT / KEEP_LAST(1) / VOLATILE subscriptions. Each topic has one pending
frame, and the Qt display takes only the latest at approximately 30 Hz maximum.
Images are downscaled to at most 640 pixels wide before coloring/display;
old pending images are not replayed. Missing/stale images are explicitly
labelled, and each preview works without the other stream. Controls and image
rendering stay on the Qt thread; [QImage](https://doc.qt.io/archives/qt-5.15/qimage.html)
owns a copy of the display bytes so it never references a released NumPy buffer.

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
