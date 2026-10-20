#pragma once
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Eigen/Dense>

class PIDController {
public:
  PIDController();
  PIDController(double kp, double ki, double kd);
  ~PIDController();

  void set_PID_gains(double kp, double ki, double kd);
  void reset(double desired_value = 0.0, double measured_value = 0.0);
  double compute_pd_output(
    double desired_value, double measured_value, double dt,
    double derivative_filter_sec);
  double compute_output(const double& desired_value, const double& measured_value, const double& dt);
  Eigen::VectorXd compute_output(const Eigen::VectorXd& desired_values, const Eigen::VectorXd& measured_values, const double& dt);
  // Eigen::MatrixXd compute_output(const Eigen::MatrixXd& desired_values, const Eigen::MatrixXd& measured_values, double dt);
  double get_integral();
  double get_previous_error();

  double kp_ = 0;
  double ki_ = 0;
  double kd_ = 0;

private:
  double dt_ = 0;
  double integral_ = 0;
  double previous_error_ = 0;
  double previous_measurement_ = 0;
  double filtered_measurement_velocity_ = 0;
  Eigen::VectorXd integral_vector_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd previous_error_vector_ = Eigen::VectorXd::Zero(6);
};
#endif  // PID_CONTROLLER_HPP
