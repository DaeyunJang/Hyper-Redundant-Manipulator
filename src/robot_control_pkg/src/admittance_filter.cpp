#include "admittance_filter.hpp"

// Default constructor with identity matrices
AdmittanceFilter::AdmittanceFilter()
: M_(Eigen::MatrixXd::Zero(6, 6)),
  B_(Eigen::MatrixXd::Zero(6, 6)),
  K_(Eigen::MatrixXd::Zero(6, 6)),
  desired_force_(Eigen::VectorXd::Zero(6)),
  external_force_(Eigen::VectorXd::Zero(6)),
  prev_force_(Eigen::VectorXd::Zero(6)),
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
  prev_force_(Eigen::VectorXd::Zero(6)),
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

void AdmittanceFilter::setDesiredForceVector(const Eigen::VectorXd& desired_force) {
  desired_force_ = desired_force;
}

void AdmittanceFilter::setExternalForceVector(const Eigen::VectorXd& external_force) {
  external_force_ = external_force;
}


// Update function to calculate position using the admittance model
Eigen::VectorXd AdmittanceFilter::computeAdmittance(
  const Eigen::VectorXd& desired_force,
  const Eigen::VectorXd& external_force
) {
  setDesiredForceVector(desired_force);
  setExternalForceVector(external_force);
  
  Eigen::VectorXd del_xf = Eigen::VectorXd::Zero(6);
  /**
   * @todo update for 2nd order system equation
   *
   */
    // // Calculate the acceleration (2nd order differential equation)
    // // M * a = F(t) - B * v(t) - K * x(t)
    // // a(t) = (F(t) - B * v(t) - K * x(t)) / M
    // a_ = M_.ldlt().solve(force - B_ * v_ - K_ * x_);
    
    // // Integrate acceleration to get velocity (Euler method)
    // v_ = v_ + a_ * dt;
    
    // // Integrate velocity to get position (Euler method)
    // x_ = x_ + v_ * dt;
    
    // // Store the current force for future calculations
    // prev_force_ = force;

  return del_xf;
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

