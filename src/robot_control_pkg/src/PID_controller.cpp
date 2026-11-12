// AUTHOR: Daeyun
// PID implementation used by the HRM feedback controllers.
#include "PID_controller.hpp"
#include <cmath>
#include <stdexcept>

PIDController::PIDController() = default;
PIDController::~PIDController() {}


void PIDController::set_PID_gains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::reset(double desired_value, double measured_value) {
  integral_ = 0.0;
  previous_error_ = desired_value - measured_value;
  filtered_error_derivative_ = 0.0;
  dt_ = 0.0;
  integral_vector_.setZero();
  previous_error_vector_.setZero();
}

double PIDController::compute_pd_output(
  double desired_value, double measured_value, double dt,
  double derivative_filter_sec)
{
  if (!std::isfinite(desired_value) || !std::isfinite(measured_value) ||
      !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(derivative_filter_sec) || derivative_filter_sec < 0.0 ||
      !std::isfinite(kp_) || kp_ < 0.0 || !std::isfinite(kd_) || kd_ < 0.0 ||
      !std::isfinite(ki_) || ki_ != 0.0)
  {
    throw std::invalid_argument("Position PD requires finite inputs, dt>0 and I=0.");
  }
  // Compatibility helper; position and scalar PID share the same error law.
  return compute_output(desired_value, measured_value, dt, derivative_filter_sec);
}

double PIDController::get_integral() {
  return integral_;
}
double PIDController::get_previous_error() {
  return previous_error_;
}


double PIDController::compute_output(
  const double& desired_value, const double& measured_value, const double& dt,
  double derivative_filter_sec)
{
  if (!std::isfinite(desired_value) || !std::isfinite(measured_value) ||
      !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(derivative_filter_sec) || derivative_filter_sec < 0.0 ||
      !std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_))
  {
    throw std::invalid_argument("PID requires finite inputs/gains, dt>0 and filter tau>=0.");
  }
  const double error = desired_value - measured_value;
  // I=0 disables the error integral as well as its output contribution.
  const double integral = ki_ == 0.0 ? 0.0 : integral_ + error * dt;
  const double derivative = (error - previous_error_) / dt;
  const double alpha = dt / (derivative_filter_sec + dt);
  const double filtered_derivative = derivative_filter_sec == 0.0 ? derivative :
    filtered_error_derivative_ + alpha * (derivative - filtered_error_derivative_);
  const double control_signal = kp_ * error + ki_ * integral + kd_ * filtered_derivative;
  if (!std::isfinite(error) || !std::isfinite(integral) ||
      !std::isfinite(filtered_derivative) || !std::isfinite(control_signal))
  {
    throw std::invalid_argument("Non-finite PID state/output.");
  }
  integral_ = integral;
  previous_error_ = error;
  filtered_error_derivative_ = filtered_derivative;
  dt_ = dt;
  return control_signal;
}

// PID control for an Eigen vector.
Eigen::VectorXd PIDController::compute_output(const Eigen::VectorXd& desired_values, const Eigen::VectorXd& measured_values, const double& dt) {
  // Apply the same PID law to each vector element.
  Eigen::VectorXd errors = desired_values - measured_values;
  dt_ = dt;

  // Calculate the integral (I term).
  integral_vector_ += errors * dt;  // Accumulate the error integral per element.

  // Calculate the derivative (D term).
  Eigen::VectorXd derivative = (errors - previous_error_vector_) / dt;

  // PID output: P * error + I * integral + D * derivative.
  Eigen::VectorXd control_signals = kp_ * errors + ki_ * integral_vector_ + kd_ * derivative;

  // Update the previous error.
  previous_error_vector_ = errors;

  return control_signals;
}

// // PID control for an Eigen matrix.
// Eigen::MatrixXd PIDController::compute_output(const Eigen::MatrixXd& desired_values, const Eigen::MatrixXd& measured_values, double dt) {
//   // Apply the same PID law to each matrix element.
//   Eigen::MatrixXd errors = desired_values - measured_values;
//   dt_ = dt;

//   // Calculate the integral (I term).
//   integral_ += errors.sum() * dt;  // Integrate the sum of all errors.

//   // Calculate the derivative (D term).
//   Eigen::MatrixXd derivative = (errors - previous_error_) / dt;

//   // PID output: P * error + I * integral + D * derivative.
//   Eigen::MatrixXd control_signals = kp_ * errors + ki_ * integral_ + kd_ * derivative;

//   // Update the previous error.
//   previous_error_ = errors;

//   return control_signals;
// }
