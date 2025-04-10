# Position control values of multi input and multi output
std_msgs/Header header
float64 sampling_time

# input & output data
float64 p_gain
float64 i_gain
float64 d_gain

# float64[] x_desired
# float64[] x_actual
# float64[] x_error
geometry_msgs/Pose x_desired
geometry_msgs/Pose x_actual
geometry_msgs/Pose x_error
float64 dt
float64 del_theta_pan
float64 del_theta_tilt

# reference data
float64[] theta_actual_relative
float64[] theta_actual_absolute