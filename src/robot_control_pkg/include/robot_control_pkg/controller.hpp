#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "dynamics_model.hpp"
#include "damping_friction_model.hpp"

/**
 * @file  controller.hpp
 * @brief This file contains the control logic for the robot.
 *
 * @image robot_control/pkg/images/controller.png
 *
 * This image shows the robot control system diagram.
 */

class Controller {
public:
    Controller(double kp, double ki, double kd);
    ~Controller();

    // PID 제어기의 게인 설정
    void set_gains(double kp, double ki, double kd);

    // 제어 신호 계산 (setpoint와 측정값을 입력으로 받아 PID 제어 계산)
    double compute_control(double setpoint, double measured_value, double dt);

private:
    double kp_ = 0;   // P 게인
    double ki_ = 0;   // I 게인
    double kd_ = 0;   // D 게인

    double integral_ = 0;        // 적분 항
    double previous_error_ = 0;  // 이전 오차
};

#endif // CONTROLLER_HPP
