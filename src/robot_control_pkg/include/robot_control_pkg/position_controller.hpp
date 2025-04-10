#pragma once
#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include "PID_controller.hpp"
#include "surgical_tool.hpp"
#include "control_parameters.hpp"
#include <Eigen/Dense>

/**
 * @brief Position controller consist of 6 DOF x-vector.
 *  But, surgical tool is moved by pan and tilt.
 *  So when PID controller compute, desired_val & actual_val must be 1 dof variable(not vector).
 *  
 * @note
 *  This position controller for surgical tool,
 *  update() function is independent-motion control
 *  i.e. PID control is applied each elements of vector of x_desired and x_actual.
 */
class PositionController {
public:
  PositionController();
  ~PositionController();

  void initialize();

  PIDController pid_controller_pan_;
  PIDController pid_controller_tilt_;
  SurgicalTool surgical_tool_;

  std::vector<double> theta_actual_;
  Eigen::VectorXd x_desired_;
  Eigen::VectorXd x_actual_;
  Eigen::VectorXd x_err_;
  double dt_;
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
  double previous_error_ = 0;
};
#endif  // POSITION_CONTROLLER_HPP
