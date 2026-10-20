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
