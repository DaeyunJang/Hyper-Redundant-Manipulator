# Admittance control values of multi input and multi output
std_msgs/Header header
float64 sampling_time

float64[] m_matrix
float64[] b_matrix
float64[] k_matrix
geometry_msgs/Wrench desired_force
geometry_msgs/Wrench env_force
geometry_msgs/Wrench delta_force
geometry_msgs/Pose x_ddot
geometry_msgs/Pose x_dot
geometry_msgs/Pose x
float64 dt