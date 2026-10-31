#pragma once
#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include "PID_controller.hpp"
#include "surgical_tool.hpp"
#include "control_parameters.hpp"
#include <Eigen/Dense>

/**
 * @brief Image-paced PD: hrm_base Y -> tilt, Z -> pan; X is passive.
 * PID output is an angular velocity [rad/s], with I=0. update() limits this
 * velocity, then integrates it once using the source-frame dt into the current
 * IK command. Constant-error P action depends on elapsed time, not frame count.
 * reset() seeds that command once, NOT a fixed reference for later targets.
 * Cross-axis compensation and physical tuning remain to be validated.
 */
class PositionController {
public:
  // Motion integration has its own limit; a relaxed sensor-freshness timeout
  // must not permit accumulated motion across missing source-image frames.
  static constexpr double MAX_CONTROL_FRAME_DT_SEC = 0.25;

  PositionController();
  ~PositionController();

  void initialize();
  // Initialize the command and clear PID history once on mode entry.
  void reset(const Eigen::VectorXd& x_actual, double pan_rad, double tilt_rad);
  // Resume feedback after a pause without changing the current IK command.
  void reset_feedback_history(
    const Eigen::VectorXd& x_desired, const Eigen::VectorXd& x_actual);
  void set_limits(double max_speed_deg_s, double derivative_filter_sec);

  PIDController pid_controller_pan_;
  PIDController pid_controller_tilt_;
  SurgicalTool surgical_tool_;

  std::vector<double> theta_actual_;
  Eigen::VectorXd x_desired_;
  Eigen::VectorXd x_actual_;
  Eigen::VectorXd x_err_;
  double dt_ = 0.0;
  double del_theta_pan_;
  double del_theta_tilt_;

  /**
   * @brief compute wire length to move for desired x
   *
   * @param x_desired
   * @return std::vector<double>
   * @details Input for IK and its type is described in its .cpp file
   */
  std::vector<double> update(
    const Eigen::VectorXd& x_desired,
    const Eigen::VectorXd& x_actual,
    const double& dt);

private:
  bool initialized_ = false;
  double max_speed_rad_s_ = position_control_params::MAX_ANGULAR_SPEED_DEG_S * std::acos(-1.0) / 180.0;
  double derivative_filter_sec_ = position_control_params::DERIVATIVE_FILTER_SEC;
};
#endif  // POSITION_CONTROLLER_HPP
