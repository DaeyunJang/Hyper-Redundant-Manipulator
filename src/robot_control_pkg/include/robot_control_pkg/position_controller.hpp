#pragma once
#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include "PID_controller.hpp"
#include "surgical_tool.hpp"
#include "control_parameters.hpp"
#include <Eigen/Dense>

/**
 * @brief Image-paced PD: hrm_base Y -> tilt, Z -> pan; X is passive.
 * reset() fixes the operating-point angles on mode entry. update() applies an
 * absolute PD correction about that point, not a repeated full-output sum.
 * I=0 can leave steady-state error; cross-axis compensation is deferred.
 */
class PositionController {
public:
  PositionController();
  ~PositionController();

  void initialize();
  // Establish an operating point once on mode entry, not on every frame.
  void reset(const Eigen::VectorXd& x_actual, double pan_rad, double tilt_rad);
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
  double reference_pan_ = 0.0;
  double reference_tilt_ = 0.0;
  double max_speed_rad_s_ = position_control_params::MAX_ANGULAR_SPEED_DEG_S * std::acos(-1.0) / 180.0;
  double derivative_filter_sec_ = position_control_params::DERIVATIVE_FILTER_SEC;
};
#endif  // POSITION_CONTROLLER_HPP
