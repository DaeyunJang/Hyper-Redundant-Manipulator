# Admittance control values of multi input and multi output
std_msgs/Header header

geometry_msgs/Wrench desired_force
geometry_msgs/Wrench external_force
geometry_msgs/Wrench delta_force
float64[] m_matrix
float64[] b_matrix
float64[] k_matrix
geometry_msgs/Pose x_f