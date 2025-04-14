#pragma once
#ifndef ADMITTANCE_FILTER_HPP
#define ADMITTANCE_FILTER_HPP

#include <Eigen/Dense>
#include <iostream>
// #include "control_parameters.hpp"

class AdmittanceFilter {
public:
  /**
   * @brief Construct a new Admittance Filter object
   * @note Must be defined as a Cartesian-coordinate(6 DOF) matrix
   */
  AdmittanceFilter(); // Default(Identity) Matrix
  AdmittanceFilter(const Eigen::MatrixXd& mass_matrix,const Eigen::MatrixXd& damping_matrix, const Eigen::MatrixXd& spring_matrix);
  ~AdmittanceFilter();

  // Set functions to update M, B, K matrices
  void setMassMatrix(const Eigen::MatrixXd& mass_matrix);
  void setDampingMatrix(const Eigen::MatrixXd& damping_matrix);
  void setSpringMatrix(const Eigen::MatrixXd& spring_matrix);
  // void setDesiredForceVector(const Eigen::VectorXd& desired_force);
  // void setExternalForceVector(const Eigen::VectorXd& external_force);
  
  // Update function to compute x_t based on the given force F_t
  /**
   * @brief Solve differential equation of x_t from x_d
   * 
   * @param force 
   * @example
   * setDesiredForceMatrix(desired_force);
   * setExternalForceMatrix(external_force);
   * computeAdmittance();
   */
  Eigen::VectorXd computeAdmittance(
    const Eigen::VectorXd& desired_force,
    const Eigen::VectorXd& external_force,
    const double& dt);
  
  // Get current position x_t
  Eigen::VectorXd getXt() const;
  Eigen::VectorXd getXtDot() const;
  Eigen::VectorXd getXtDDot() const;

  Eigen::MatrixXd M_;  // Mass matrix (6x6)
  Eigen::MatrixXd B_;  // Damping matrix (6x6)
  Eigen::MatrixXd K_;  // Spring constant matrix (6x6)

  Eigen::VectorXd desired_force_;  // Desired force input
  Eigen::VectorXd external_force_;  // External force input
  Eigen::VectorXd force_error_;  // External force input
  
  Eigen::VectorXd xt_;  // Position vector (6x1)
  Eigen::VectorXd xtdot_;  // Velocity vector (6x1)
  Eigen::VectorXd xtddot_;  // Acceleration vector (6x1)
  
  double dt_;


private:
};
#endif  // ADMITTANCE_FILTER_HPP
