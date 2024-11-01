#pragma once
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "PID_controller.hpp"

class PIDController {
public:
  PIDController();
  PIDController(double kp, double ki, double kd);
  ~PIDController();

  void set_PID_gains(double kp, double ki, double kd);
  double compute_output(double desired_value, double measured_value, double dt);

private:
  double kp_ = 0;
  double ki_ = 0;
  double kd_ = 0;

  double dt_ = 0;
  double integral_ = 0;
  double previous_error_ = 0;
};
#endif  // PID_CONTROLLER_HPP
