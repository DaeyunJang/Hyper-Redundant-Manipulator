std_msgs/Header header

# Joint angles [rad], ordered from base to end-effector.
float64[] pan_relative
float64[] pan_absolute
float64[] tilt_relative
float64[] tilt_absolute

# Joint angular velocities [rad/s], ordered from base to end-effector.
float64[] pan_angular_velocity_relative
float64[] pan_angular_velocity_absolute
float64[] tilt_angular_velocity_relative
float64[] tilt_angular_velocity_absolute
