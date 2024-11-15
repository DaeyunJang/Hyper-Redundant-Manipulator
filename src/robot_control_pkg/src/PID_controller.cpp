#include "PID_controller.hpp"

PIDController::PIDController() {}
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


double PIDController::compute_output(double desired_value, double measured_value, double dt) {
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
