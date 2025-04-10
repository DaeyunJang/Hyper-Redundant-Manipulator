#include "position_controller.hpp"

PIDController::PIDController()
  : kp_(0), ki_(0), kd_(0), integral_(0), previous_error_(0), dt_(0) {}
PIDController::~PIDController() {}


void PIDController::set_PID_gains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

double PIDController::get_integral() {
  return integral_;
}
double PIDController::get_previous_error() {
  return previous_error_;
}


double PIDController::compute_output(const double& desired_value, const double& measured_value, const double& dt) {
  // 오차 계산 (목표값 - 측정값)
  double error = desired_value - measured_value;
  dt_ = dt;
  
  // 적분 계산 (I 항)
  integral_ += error * dt;

  // 미분 계산 (D 항)
  double derivative = (error - previous_error_) / dt;

  // PID 제어 계산: P * error + I * integral + D * derivative
  double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // 이전 오차 업데이트
  previous_error_ = error;

  return control_signal;
}

// Eigen 벡터에 대한 PID 제어 함수 (벡터 타입)
Eigen::VectorXd PIDController::compute_output(const Eigen::VectorXd& desired_values, const Eigen::VectorXd& measured_values, const double& dt) {
  // 동일한 방식으로 벡터의 각 요소에 대해 PID 제어를 적용
  Eigen::VectorXd errors = desired_values - measured_values;
  dt_ = dt;

  // 적분 계산 (I 항)
  integral_vector_ += errors * dt;  // 전체 오차의 합을 적분으로 처리

  // 미분 계산 (D 항)
  Eigen::VectorXd derivative = (errors - previous_error_vector_) / dt;

  // PID 제어 계산: P * error + I * integral + D * derivative
  Eigen::VectorXd control_signals = kp_ * errors + ki_ * integral_vector_ + kd_ * derivative;

  // 이전 오차 업데이트
  previous_error_vector_ = errors;

  return control_signals;
}

// // Eigen 행렬에 대한 PID 제어 함수 (행렬 타입)
// Eigen::MatrixXd PIDController::compute_output(const Eigen::MatrixXd& desired_values, const Eigen::MatrixXd& measured_values, double dt) {
//   // 동일한 방식으로 행렬의 각 요소에 대해 PID 제어를 적용
//   Eigen::MatrixXd errors = desired_values - measured_values;
//   dt_ = dt;

//   // 적분 계산 (I 항)
//   integral_ += errors.sum() * dt;  // 전체 오차의 합을 적분으로 처리

//   // 미분 계산 (D 항)
//   Eigen::MatrixXd derivative = (errors - previous_error_) / dt;

//   // PID 제어 계산: P * error + I * integral + D * derivative
//   Eigen::MatrixXd control_signals = kp_ * errors + ki_ * integral_ + kd_ * derivative;

//   // 이전 오차 업데이트
//   previous_error_ = errors;

//   return control_signals;
// }