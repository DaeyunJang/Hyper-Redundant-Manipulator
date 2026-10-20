# Single-axis segment-angle legacy backup

This directory preserves the segment-angle input used before the pan/tilt
message was introduced. The legacy input represents one bending plane (pan)
with four independent `std_msgs/msg/Float64MultiArray` topics:

- `estimated_segment_angle/relative`
- `estimated_segment_angle/absolute`
- `estimated_segment_angular_velocity/relative`
- `estimated_segment_angular_velocity/absolute`

The `.inc` files are reference snippets and are intentionally excluded from
the `robot_control_pkg` build. To restore this interface, copy the declarations
into `control_node.hpp` and the initialization/subscriptions into the
`ControlNode` constructor in `control_node.cpp`.

`surgical_tool_transform_legacy.cpp.inc` also preserves the original planar
forward kinematics that accepts one scalar angle per segment.

Legacy computation mapping:

```cpp
std::vector<double> theta_actual(
  segment_angle_relative_.data.begin(),
  segment_angle_relative_.data.end());

std::vector<double> omega_actual(
  segment_angular_velocity_relative_.data.begin(),
  segment_angular_velocity_relative_.data.end());

double end_effector_theta_actual = segment_angle_absolute_.data.back();
double end_effector_omega_actual =
  segment_angular_velocity_absolute_.data.back();
```
