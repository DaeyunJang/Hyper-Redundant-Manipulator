
/**
 * @file control_node.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2024-10-23
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <future>
#include <string>
#include <thread>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <memory>
#include <vector>
#include <sys/signal.h>
#include <chrono>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>
#include <signal.h>
#include <algorithm>
#include <tuple>
#include <cstdint>
#include <stdexcept>

// Surgical Tool Class
#include "surgical_tool.hpp"
#include "hw_definition.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "custom_interfaces/msg/loadcell_state.hpp"
#include "custom_interfaces/srv/move_motor_direct.hpp"
#include "custom_interfaces/srv/move_tool_angle.hpp"
// #include "tcp_node.hpp"   // using #define NUM_OF_MOTRS

typedef enum  {
  kStop = 0,
  kEnable = 1,
  kReady = 2,
  kHoming = 5,
} OpMode;

class KinematicsControlNode : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommand = custom_interfaces::msg::MotorCommand;
  using MoveMotorDirect = custom_interfaces::srv::MoveMotorDirect;
  using MoveToolAngle = custom_interfaces::srv::MoveToolAngle;

  /**
   * @brief Construct a new Kinematics Control Node object
   * @param node_options ROS
   */
  explicit KinematicsControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Kinematics Control Node object
   */
  virtual ~KinematicsControlNode();

  /**
   * @brief sugical_tool.hpp
   */
  SurgicalTool ST_;

  /**
   * @author DY, JKim
   * @def    cal_inverse_kinematics
   * @ref    A Stiffness-Adjustable Hyperredundant Manipulator Using a Variable Neutral-Line Mechanism for Minimally Invasive Surgery
   * @brief  calculate target values(length of wire) from the surgical tool kinematics,
   * @param  actual position, actual_velocity and Controller(e.g. Xbox) input
   * @return target values
  */
  void cal_inverse_kinematics(double pAngle, double tAngle, double gAngle);

  /**
   * @author DY
   * @brief multiply the ratio of all components
   * @param gear_ratio 
   * @param e_channel 
   * @param e_resolution 
   * @return float 
   */
  double gear_encoder_ratio_conversion(double gear_ratio, int e_channel, int e_resolution);

  void set_position_zero();

private:
  OpMode op_mode_ = kStop;

  /**
   * @author DY
   * @brief ROS2 objects and arguments
   */
  void publishall();

  /**
   * @author DY
   * @brief Sine wave publish function
   */
  void publish_sine_wave();
  void publish_circle_motion();
  void publish_moebius_motion();

  /**
   * @brief virtual_position
   */
  int virtual_home_pos_[NUM_OF_MOTORS] = {0,};

  /**
   * @author DY
   * @brief target values publisher for operating motors
   */
  MotorCommand motor_control_target_val_;
  rclcpp::Publisher<MotorCommand>::SharedPtr motor_control_publisher_;

  /**
   * @author DY
   * @brief actual motor status subscriber
   */
  bool motorstate_op_flag_ = false;
  MotorState motor_state_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscriber_;

  /**
   * @author DY
   * @brief kinematic info publisher
   */
  float current_tilt_angle_ = 0; 
  float current_pan_angle_ = 0; 
  float current_grip_angle_ = 0; 


  geometry_msgs::msg::Twist surgical_tool_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_publisher_;
  std_msgs::msg::Float32MultiArray wire_length_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wire_length_publisher_;

  /**
   * @author DY
   * @brief Loadcell data subscriber
   */
  bool loadcell_op_flag_ = false;
  custom_interfaces::msg::LoadcellState loadcell_data_;
  rclcpp::Subscription<custom_interfaces::msg::LoadcellState>::SharedPtr loadcell_data_subscriber_;

  /**
   * @author DY
   * @brief Service server for motion
   */
  rclcpp::Service<MoveMotorDirect>::SharedPtr move_motor_direct_service_server_;
  rclcpp::Service<MoveToolAngle>::SharedPtr move_tool_angle_service_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_sine_wave_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_circle_motion_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_moebius_motion_server_;
  rclcpp::TimerBase::SharedPtr timer_;  // 타이머는 시작과 중지를 위해 nullptr로 관리
  int timer_period_ms_ = 10;
  float amp_ = 60;
  float period_ = 20; // secs
  float count_ = 0;
  float count_add_ = timer_period_ms_ / 1000.0;
  float angle_ = 0;
};

#endif