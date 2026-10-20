// Single bending-plane (pan) segment-angle input.
bool segment_angle_op_flag_ = false;
std_msgs::msg::Float64MultiArray segment_angle_relative_;
std_msgs::msg::Float64MultiArray segment_angle_relative_prev_;
std_msgs::msg::Float64MultiArray segment_angle_absolute_;
std_msgs::msg::Float64MultiArray segment_angle_absolute_prev_;

rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  segment_angle_relative_subscriber_;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  segment_angle_absolute_subscriber_;

bool segment_angular_velocity_op_flag_ = false;
std_msgs::msg::Float64MultiArray segment_angular_velocity_relative_;
std_msgs::msg::Float64MultiArray segment_angular_velocity_absolute_;

rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  segment_angular_velocity_relative_subscriber_;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  segment_angular_velocity_absolute_subscriber_;

