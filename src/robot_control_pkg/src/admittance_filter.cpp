#include "admittance_filter.hpp"

// Default constructor with identity matrices
AdmittanceFilter::AdmittanceFilter()
: M_(Eigen::MatrixXd::Zero(6, 6)),
  B_(Eigen::MatrixXd::Zero(6, 6)),
  K_(Eigen::MatrixXd::Zero(6, 6)),
  desired_force_(Eigen::VectorXd::Zero(6)),
  external_force_(Eigen::VectorXd::Zero(6)),
  force_error_(Eigen::VectorXd::Zero(6)),
  xt_(Eigen::VectorXd::Zero(6)),
  xtdot_(Eigen::VectorXd::Zero(6)),
  xtddot_(Eigen::VectorXd::Zero(6)) {}

// Constructor to initialize M, B, K and initialize position, velocity, acceleration
AdmittanceFilter::AdmittanceFilter(const Eigen::MatrixXd& mass_matrix, const Eigen::MatrixXd& damping_matrix, const Eigen::MatrixXd& spring_matrix)
: M_(mass_matrix),
  B_(damping_matrix),
  K_(spring_matrix),
  desired_force_(Eigen::VectorXd::Zero(6)),
  external_force_(Eigen::VectorXd::Zero(6)),
  force_error_(Eigen::VectorXd::Zero(6)),
  xt_(Eigen::VectorXd::Zero(6)),
  xtdot_(Eigen::VectorXd::Zero(6)),
  xtddot_(Eigen::VectorXd::Zero(6)) {}

AdmittanceFilter::~AdmittanceFilter() {

}

// Set Mass matrix M
void AdmittanceFilter::setMassMatrix(const Eigen::MatrixXd& mass_matrix) {
  M_ = mass_matrix;
}

// Set Damping matrix B
void AdmittanceFilter::setDampingMatrix(const Eigen::MatrixXd& damping_matrix) {
  B_ = damping_matrix;
}

// Set Spring constant matrix K
void AdmittanceFilter::setSpringMatrix(const Eigen::MatrixXd& spring_matrix) {
  K_ = spring_matrix;
}

// void AdmittanceFilter::setDesiredForceVector(const Eigen::VectorXd& desired_force) {
//   desired_force_ = desired_force;
// }

// void AdmittanceFilter::setExternalForceVector(const Eigen::VectorXd& external_force) {
//   external_force_ = external_force;
// }


// Update function to calculate position using the admittance model
Eigen::VectorXd AdmittanceFilter::computeAdmittance(
  const Eigen::VectorXd& desired_force,
  const Eigen::VectorXd& external_force,
  const double& dt
) {
  // setDesiredForceVector(desired_force);
  // setExternalForceVector(external_force);
  this->desired_force_ = desired_force;
  this->external_force_ = external_force;
  this->force_error_ = external_force - desired_force;
  this->dt_ = dt;

  // Ensure size consistency
  if (xt_.size() == 0) {
    xt_ = Eigen::VectorXd::Zero(6);
    xtdot_ = Eigen::VectorXd::Zero(6);
    xtddot_ = Eigen::VectorXd::Zero(6);
  }
  
  const double eps = 1e-6;
  // Full admittance (MBK)
  if (M_.norm() > eps) {
    xtddot_ = M_.ldlt().solve(force_error - B_ * xtdot_ - K_ * xt_);  // M^-1(F - Bẋ - Kx)
    xtdot_ += xtddot_ * dt;
    xt_ += xtdot_ * dt;
  }
  // Damper + Spring (M = 0, 1st ODE system)
  else if (B_.norm() > eps && K_.norm() > eps) {
    xtdot_ = B_.ldlt().solve(force_error - K_ * xt_);
    xt_ += xtdot_ * dt;
    xtddot_.setZero();
  }
  // Damping control (1st ODE system)
  else if (B_.norm() > eps) {
    xtdot_ = B_.ldlt().solve(force_error);
    xt_ += xtdot_ * dt;
    xtddot_.setZero();
  }
  // Stiffness control (0 order system))
  else if (K_.norm() > eps) {
    xt_ = K_.ldlt().solve(force_error);
    xtdot_.setZero();
    xtddot_.setZero();
  }
  else {
    // Zero (M=B=K=0)
    xt_.setZero();
    xtdot_.setZero();
    xtddot_.setZero();
  }
  
  return xt_;
}

// Get the current position
Eigen::VectorXd AdmittanceFilter::getXt() const {
    return xt_;
}

// Get the current velocity
Eigen::VectorXd AdmittanceFilter::getXtDot() const {
    return xtdot_;
}

// Get the current acceleration
Eigen::VectorXd AdmittanceFilter::getXtDDot() const {
    return xtddot_;
}

