# Requested control reference for recording; not confirmation of motor execution.
# Generation/publication time, with frame_id = hrm_base.
std_msgs/Header header
# Source-image time for Position/Admittance; zero for Kinematics.
builtin_interfaces/Time feedback_stamp
# kinematics, position, or admittance.
string control_mode
# Aggregate cable-IK command angles in radians, not tip Euler orientation.
float64 pan_rad
float64 tilt_rad
# False for Kinematics: both Cartesian points are unused zeros.
bool cartesian_reference_valid
# Nominal Cartesian reference in hrm_base, metres, before admittance offset.
geometry_msgs/Point nominal_position
# Effective Cartesian reference consumed by the current Position controller step.
# In Admittance this includes the applied admittance offset, in metres.
geometry_msgs/Point target_position
# Configured output gate only; other guards may still prevent motor publication.
bool motor_output_enabled
