// Place in the ControlNode constructor after QoS_RKL10V is created.
segment_angle_relative_.data.resize(NUM_OF_JOINT, 0.0);

segment_angle_relative_subscriber_ =
  create_subscription<std_msgs::msg::Float64MultiArray>(
    "estimated_segment_angle/relative",
    QoS_RKL10V,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      segment_angle_relative_.data = msg->data;
      segment_angle_op_flag_ = true;
      RCLCPP_INFO_ONCE(
        get_logger(), "Subscribing the /estimated_segment_angle/relative.");
    });

segment_angle_absolute_subscriber_ =
  create_subscription<std_msgs::msg::Float64MultiArray>(
    "estimated_segment_angle/absolute",
    QoS_RKL10V,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      segment_angle_absolute_.data = msg->data;
      segment_angle_op_flag_ = true;
      RCLCPP_INFO_ONCE(
        get_logger(), "Subscribing the /estimated_segment_angle/absolute.");
    });

segment_angular_velocity_relative_subscriber_ =
  create_subscription<std_msgs::msg::Float64MultiArray>(
    "estimated_segment_angular_velocity/relative",
    QoS_RKL10V,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      segment_angular_velocity_relative_.data = msg->data;
      segment_angular_velocity_op_flag_ = true;
      RCLCPP_INFO_ONCE(
        get_logger(),
        "Subscribing the /estimated_segment_angular_velocity/relative.");
    });

segment_angular_velocity_absolute_subscriber_ =
  create_subscription<std_msgs::msg::Float64MultiArray>(
    "estimated_segment_angular_velocity/absolute",
    QoS_RKL10V,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      segment_angular_velocity_absolute_.data = msg->data;
      segment_angular_velocity_op_flag_ = true;
      RCLCPP_INFO_ONCE(
        get_logger(),
        "Subscribing the /estimated_segment_angular_velocity/absolute.");
    });

