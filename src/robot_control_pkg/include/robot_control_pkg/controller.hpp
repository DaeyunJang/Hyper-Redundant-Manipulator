#pragma once
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "HRM_dynamics_model.hpp"
#include "damping_friction_model.hpp"
#include "PID_controller.hpp"
#include "surgical_tool.hpp"

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
    Controller();
    ~Controller();

    // PID 제어기의 게인 설정
    PIDController pid_controller_;
    HRMDynamicsModel hrm_dynamics_model_;
    DampingFrictionModel damping_friction_model_;
    SurgicalTool surgical_tool_;

    void initialize();

    /**
     * @author DY
     * @date 2024.11.1
     * @brief
     * Compute the controller
     * 
     * @note
     * 1. compute PID
     * 2. HRM model (with F_ext, F_fric, B)
     * 3. iHRM_controller_ire length
     * 
     * the information are maybe obtained from other processes (ROS2)
     * 
     * @param theta_desired target angle
     * @param theta_actual actual angle(from image process) -> angle err = (target_angle) - (actual angle)
     * @param dtheta_dt_actual actual angular velocity (from image process)
     * @param dt sampling time (it is usually affected by curve fit algorithm)
     * @param torque_external estimated external force (LSTM)
     * @return std::v
ector<double> wire length [right(East), left(West)]
     */
    std::vector<double> compute(
        const double& theta_desired,
        const double& end_effector_theta_actual,
        const double& end_effector_omega_actual,
        const std::vector<double>& theta_actual,
        const std::vector<double>& dtheta_dt_actual,
        const std::vector<double>& cable_velocity,
        const double& dt,
        const std::vector<double>& tension,
        const std::vector<double>& force_external,
        const bool& hrm_controller_enable
        );

    double end_effector_theta_actual_;
    double end_effector_dtheta_dt_actual_;
    double torque_input_;
    std::vector<double> dandf_;

    double tau_ext_;
    double tau_friction_;
    double theta_ddot_input_;
    double theta_dot_input_;
    double theta_input_;
    double theta_input_prev_=0;
    bool hrm_controller_enable_=true;

    std::vector<double> theta_actual_;
    std::vector<double> theta_actual_prev_;
    std::vector<double> dtheta_dt_actual_;
    std::vector<double> dtheta_dt_actual_prev_;
    std::vector<double> cable_velocity_actual_;
    std::vector<double> cable_velocity_actual_prev_;
    std::vector<double> force_external_;

// private:
   
};

#endif // CONTROLLER_HPP
