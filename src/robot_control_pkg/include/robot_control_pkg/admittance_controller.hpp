#pragma once
#ifndef ADMITTANCE_CONTROLLER_HPP
#define ADMITTANCE_CONTROLLER_HPP

#include "admittance_filter.hpp"
// #include "position_controller.hpp"

/**
 * @file  controller.hpp
 * @brief This file contains the control logic for the robot.
 *
 * @image robot_control/pkg/images/controller.png
 *
 * This image shows the robot control system diagram.
 */

class AdmittanceController {
public:
  AdmittanceController();
  ~AdmittanceController();

  // Configuration class of the control system
  AdmittanceFilter admittance_filter_;


  /** @brief this f and x values are must be 6 DOF vector (e.g. VectorXd(6)) */
  Eigen::VectorXd del_f_;
  Eigen::VectorXd f_desired_;
  Eigen::VectorXd f_external_;
  Eigen::VectorXd del_xf_;

  /**
   * @brief 
   * @note
   * 1. input F_desired, F_external, x_desired and x_actual
   * 2. compute Admittance Y(s) using values set from 1.
   * 3. x_t = x_f + x_d
   * the information are maybe obtained from other processes (ROS2)
   * @param f_desired 6DOF Desired force for manipulation
   * @param f_external 6DOF External(or Environment) force
   * @return Eigen::VectorXd target pose trajectory
   */
  Eigen::VectorXd compute(
    const Eigen::VectorXd& f_desired,
    const Eigen::VectorXd& f_external
  );

private:
   
};

#endif // ADMITTANCE_CONTROLLER_HPP
