#pragma once

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
#include <thread>
#include <numeric>
#include <iterator>

// Surgical Tool Class
#include "hw_definition.hpp"
#include "dynamics_parameters.hpp"
#include "controller.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "custom_interfaces/msg/loadcell_state.hpp"
#include "custom_interfaces/msg/dynamic_mimo_values.hpp"
#include "custom_interfaces/srv/move_motor_direct.hpp"
#include "custom_interfaces/srv/move_tool_angle.hpp"
// #include "tcp_node.hpp"   // using #define NUM_OF_MOTRS

typedef enum  {
  kStop = 0,
  kEnable = 1,
  kReady = 2,
  kHoming = 5,
} OpMode;

typedef enum  {
  kKinematics,
  kDynamics,
} ControlMode;

/**
 * @brief 
 * @author DY
 * @date 2024.10.31. 
 * @todo make choosing one which Kinematics or Dynamics version
 */
class ControlNode : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommand = custom_interfaces::msg::MotorCommand;
  using DynamicMIMOValues = custom_interfaces::msg::DynamicMIMOValues;
  using MoveMotorDirect = custom_interfaces::srv::MoveMotorDirect;
  using MoveToolAngle = custom_interfaces::srv::MoveToolAngle;

  Controller HRM_controller_;
  double theta_desired_;
  std::vector<double> theta_actual_;
  std::vector<double> dtheta_dt_actual_;
  double dt_;
  std::vector<double> force_external_;

  /**
   * @brief Construct a new Kinematics Control Node object
   * @param node_options ROS
   */
  explicit ControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Kinematics Control Node object
   */
  virtual ~ControlNode();

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
   * @brief ROS2 parameters 
   */
  ControlMode control_mode_;
  bool hrm_controller_enable_;
  rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

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
  void publish_sine_wave_1time();
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
   * @brief  Dynamics parameters of input and output
   * 
   */
  DynamicMIMOValues dynamic_MIMO_values_;
  rclcpp::Publisher<DynamicMIMOValues>::SharedPtr dynamic_MIMO_values_publisher_;

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
  std_msgs::msg::Float32MultiArray wire_length_velocity_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wire_length_velocity_publisher_;

  /**
   * @author DY
   * @brief subscriber
   */
  bool loadcell_op_flag_ = false;
  custom_interfaces::msg::LoadcellState loadcell_data_;
  rclcpp::Subscription<custom_interfaces::msg::LoadcellState>::SharedPtr loadcell_data_subscriber_;

  bool external_force_op_flag_ = false;
  geometry_msgs::msg::Vector3 external_force_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr external_force_subscriber_;

  bool segment_angle_op_flag_ = false;
  std_msgs::msg::Float32MultiArray segment_angle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr segment_angle_subscriber_;
  std_msgs::msg::Float32MultiArray segment_angle_absolute_;
  std_msgs::msg::Float32MultiArray segment_angle_absolute_prev_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr segment_angle_absolute_subscriber_;

  bool segment_angular_velocity_op_flag_ = false;
  std_msgs::msg::Float32MultiArray segment_angular_velocity_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr segment_angular_velocity_subscriber_;
  std_msgs::msg::Float32MultiArray segment_angular_velocity_absolute_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr segment_angular_velocity_absolute_subscriber_;


  /**
   * @author DY
   * @brief Service server for motion
   */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_mode_service_server_;
  rclcpp::Service<MoveMotorDirect>::SharedPtr move_motor_direct_service_server_;
  rclcpp::Service<MoveToolAngle>::SharedPtr kinematics_move_tool_angle_service_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_sine_wave_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_sine_wave_1time_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr kinematics_move_circle_motion_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr kinematics_move_moebius_motion_server_;

  rclcpp::Service<MoveToolAngle>::SharedPtr dynamics_move_tool_angle_service_server_;

  rclcpp::TimerBase::SharedPtr timer_;  // 타이머는 시작과 중지를 위해 nullptr로 관리
  int timer_period_ms_ = 10;
  float amp_ = 60;
  float period_ = 30; // secs
  float count_ = 0;
  float count_add_ = timer_period_ms_ / 1000.0;
  float angle_ = 0;

  std::thread dynamic_control_thread_;
  rclcpp::Rate loop_rate_;  // DY == initialize in the constructor of .cpp file (unit. Hz)
  void run_dynamic_control_thread();
};

#endif