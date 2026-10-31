#pragma once
#ifndef ADMITTANCE_FILTER_HPP
#define ADMITTANCE_FILTER_HPP

#include <Eigen/Dense>

class AdmittanceFilter {
public:
  /**
   * @brief Construct a new Admittance Filter object
   * @note Must be defined as a Cartesian-coordinate(6 DOF) matrix
   */
  AdmittanceFilter(); // Default: passive zero matrices.
  AdmittanceFilter(const Eigen::MatrixXd& mass_matrix,const Eigen::MatrixXd& damping_matrix, const Eigen::MatrixXd& spring_matrix);
  ~AdmittanceFilter();

  // Set functions to update M, B, K matrices
  void setMassMatrix(const Eigen::MatrixXd& mass_matrix);
  void setDampingMatrix(const Eigen::MatrixXd& damping_matrix);
  void setSpringMatrix(const Eigen::MatrixXd& spring_matrix);
  void reset();
  void stopVelocity();
  // void setDesiredForceVector(const Eigen::VectorXd& desired_force);
  // void setExternalForceVector(const Eigen::VectorXd& external_force);
  
  /**
   * @brief Legacy 6D model: M*xddot + B*xdot + K*x = desired - external.
   * @note No displacement/velocity saturation is applied by this filter.
   * Matrices may be symmetric positive semidefinite and coupled. Axes with
   * zero rows in all three matrices are inactive; their force is ignored.
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
  Eigen::VectorXd env_force_;  // External force input
  Eigen::VectorXd force_error_;  // External force input
  
  Eigen::VectorXd xt_;  // Position vector (6x1)
  Eigen::VectorXd xtdot_;  // Velocity vector (6x1)
  Eigen::VectorXd xtddot_;  // Acceleration vector (6x1)
  
  double dt_ = 0.0;
};
#endif  // ADMITTANCE_FILTER_HPP
