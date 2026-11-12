// AUTHOR: Daeyun
// Six-dimensional admittance implementation for the HRM interaction controller.
#include "admittance_filter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
void validate_matrix(const Eigen::MatrixXd& matrix) {
  if (matrix.rows() != 6 || matrix.cols() != 6 || !matrix.allFinite()) {
    throw std::invalid_argument("Admittance requires finite 6x6 matrices.");
  }
  const double tolerance = 1e-12 * std::max(1.0, matrix.norm());
  if ((matrix - matrix.transpose()).norm() > tolerance) {
    throw std::invalid_argument("Admittance matrices must be symmetric.");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenvalues(matrix, Eigen::EigenvaluesOnly);
  if (eigenvalues.info() != Eigen::Success || eigenvalues.eigenvalues().minCoeff() < -tolerance) {
    throw std::invalid_argument("Admittance matrices must be positive semidefinite.");
  }
}

Eigen::VectorXd solve_supported(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& rhs) {
  // Preserve a genuine matrix solve (including cross-axis terms). The HRM's
  // inactive rows make the 6x6 matrices singular, so use a minimum-norm solve
  // there instead of dividing by a zero LDLT pivot.
  Eigen::LDLT<Eigen::MatrixXd> ldlt(matrix);
  const double pivot_tolerance = 1e-12 * matrix.norm();
  Eigen::VectorXd result;
  if (ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > pivot_tolerance) {
    result = ldlt.solve(rhs);
  } else {
    result = matrix.completeOrthogonalDecomposition().solve(rhs);
  }
  if (!result.allFinite() ||
      (matrix * result - rhs).norm() > 1e-9 * std::max(1.0, rhs.norm())) {
    throw std::runtime_error("Admittance matrix cannot represent the requested force/state.");
  }
  return result;
}

int integration_steps(
  const Eigen::MatrixXd& mass, const Eigen::MatrixXd& damping,
  const Eigen::MatrixXd& spring, double dt)
{
  constexpr double eps = 1e-3;
  double inverse_safe_step = 0.0;
  if (mass.norm() > eps) {
    const Eigen::MatrixXd inverse_mass = mass.completeOrthogonalDecomposition().pseudoInverse();
    inverse_safe_step = (inverse_mass * damping).norm() +
      std::sqrt((inverse_mass * spring).norm());
  } else if (damping.norm() > eps && spring.norm() > eps) {
    const Eigen::MatrixXd inverse_damping =
      damping.completeOrthogonalDecomposition().pseudoInverse();
    inverse_safe_step = (inverse_damping * spring).norm();
  }
  const double steps = std::max(1.0, std::ceil(dt * inverse_safe_step));
  if (!std::isfinite(inverse_safe_step) || !std::isfinite(steps) || steps > 10000.0) {
    throw std::runtime_error("Admittance parameters require an impractically small integration step.");
  }
  return static_cast<int>(steps);
}
}

// Default constructor with passive zero matrices
AdmittanceFilter::AdmittanceFilter()
: M_(Eigen::MatrixXd::Zero(6, 6)),
  B_(Eigen::MatrixXd::Zero(6, 6)),
  K_(Eigen::MatrixXd::Zero(6, 6)),
  desired_force_(Eigen::VectorXd::Zero(6)),
  env_force_(Eigen::VectorXd::Zero(6)),
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
  env_force_(Eigen::VectorXd::Zero(6)),
  force_error_(Eigen::VectorXd::Zero(6)),
  xt_(Eigen::VectorXd::Zero(6)),
  xtdot_(Eigen::VectorXd::Zero(6)),
  xtddot_(Eigen::VectorXd::Zero(6)) {
  validate_matrix(M_);
  validate_matrix(B_);
  validate_matrix(K_);
}

AdmittanceFilter::~AdmittanceFilter() {

}

// Set Mass matrix M
void AdmittanceFilter::setMassMatrix(const Eigen::MatrixXd& mass_matrix) {
  validate_matrix(mass_matrix);
  M_ = mass_matrix;
}

// Set Damping matrix B
void AdmittanceFilter::setDampingMatrix(const Eigen::MatrixXd& damping_matrix) {
  validate_matrix(damping_matrix);
  B_ = damping_matrix;
}

// Set Spring constant matrix K
void AdmittanceFilter::setSpringMatrix(const Eigen::MatrixXd& spring_matrix) {
  validate_matrix(spring_matrix);
  K_ = spring_matrix;
}

void AdmittanceFilter::reset() {
  xt_.setZero();
  stopVelocity();
  desired_force_.setZero();
  env_force_.setZero();
  force_error_.setZero();
  dt_ = 0.0;
}

void AdmittanceFilter::stopVelocity() {
  xtdot_.setZero();
  xtddot_.setZero();
}


// Update function to calculate position using the admittance model
Eigen::VectorXd AdmittanceFilter::computeAdmittance(
  const Eigen::VectorXd& desired_force,
  const Eigen::VectorXd& env_force,
  const double& dt
) {
  if (desired_force.size() != 6 || env_force.size() != 6 ||
      !desired_force.allFinite() || !env_force.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
    throw std::invalid_argument("Admittance requires finite forces and dt in (0,0.25].");
  }
  validate_matrix(M_);
  validate_matrix(B_);
  validate_matrix(K_);
  if (xt_.size() != 6 || xtdot_.size() != 6 || xtddot_.size() != 6 ||
      !xt_.allFinite() || !xtdot_.allFinite() || !xtddot_.allFinite()) {
    throw std::runtime_error("Admittance requires a finite 6D state.");
  }
  const Eigen::VectorXd force_error = desired_force - env_force;
  Eigen::VectorXd supported_error = force_error;
  Eigen::VectorXd next_x = xt_;
  Eigen::VectorXd next_v = xtdot_;
  Eigen::VectorXd next_a = Eigen::VectorXd::Zero(6);
  for (int axis = 0; axis < 6; ++axis) {
    if (M_.row(axis).isZero(0.0) && B_.row(axis).isZero(0.0) && K_.row(axis).isZero(0.0)) {
      supported_error(axis) = next_x(axis) = next_v(axis) = 0.0;
    }
  }

  // Preserve the legacy norm threshold, ODE and integration order. At 30 Hz,
  // explicit damping updates can be unstable (e.g. M=.05, B=5). Subdivide dt
  // conservatively when needed; this changes no gain and clips no motion.
  // If dt is already small enough the original single-step update is retained.
  constexpr double eps = 1e-3;
  const int steps = integration_steps(M_, B_, K_, dt);
  const double h = dt / steps;
  if (M_.norm() > eps) {
    for (int step = 0; step < steps; ++step) {
      next_a = solve_supported(M_, supported_error - B_ * next_v - K_ * next_x);
      next_v += next_a * h;
      next_x += next_v * h;
    }
  } else if (B_.norm() > eps && K_.norm() > eps) {
    for (int step = 0; step < steps; ++step) {
      next_v = solve_supported(B_, supported_error - K_ * next_x);
      next_x += next_v * h;
    }
  } else if (B_.norm() > eps) {
    next_v = solve_supported(B_, supported_error);
    next_x += next_v * dt;
  } else if (K_.norm() > eps) {
    next_x = solve_supported(K_, supported_error);
    next_v.setZero();
  } else {
    next_x.setZero();
    next_v.setZero();
  }
  if (!next_x.allFinite() || !next_v.allFinite() || !next_a.allFinite()) {
    throw std::runtime_error("Non-finite admittance state.");
  }
  desired_force_ = desired_force;
  env_force_ = env_force;
  force_error_ = force_error;
  dt_ = dt;
  xt_ = next_x;
  xtdot_ = next_v;
  xtddot_ = next_a;

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

