#include "controller.hpp"

// 생성자: PID 게인 초기화
Controller::Controller(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0) {}

// PID 게인을 설정하는 함수
void DynamicsController::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// PID 제어 계산 함수
double DynamicsController::compute_control(double desired_value, double measured_value, double dt) {
    // 오차 계산 (목표값 - 측정값)
    double error = desired_value - measured_value;

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
