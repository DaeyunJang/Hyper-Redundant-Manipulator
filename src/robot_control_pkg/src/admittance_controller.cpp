// AUTHOR: Daeyun
// HRM force-feedback convention and compliant target-position generation.
#include "admittance_controller.hpp"


// Initialize admittance Y as zero matrix - zero 6x6
// AdmittanceController::AdmittanceController()
// : admittance_filter_(Eigen::MatrixXd::Zero(6, 6), Eigen::MatrixXd::Zero(6, 6), Eigen::MatrixXd::Zero(6, 6)),
//   position_controller_() {}

/**
 * @brief Construct a new Admittance Controller:: Admittance Controller object
 * @param admittance_filter_ Initialize admittance Y as pre-defined matrix
 * @ref control_parameters.hpp
 */
AdmittanceController::AdmittanceController()
: admittance_filter_(admittance_params::MASS_MATRIX,
                     admittance_params::DAMPER_MATRIX,
                     admittance_params::SPRING_MATRIX),
  del_f_(Eigen::VectorXd::Zero(6)),
  f_desired_(Eigen::VectorXd::Zero(6)),
  f_env_(Eigen::VectorXd::Zero(6)),
  del_xf_(Eigen::VectorXd::Zero(6)) {}

AdmittanceController::~AdmittanceController() {}


Eigen::VectorXd AdmittanceController::compute(
  const Eigen::VectorXd& f_desired,
  const Eigen::VectorXd& f_env,
  const double& dt
) {
  // Public forces are forces ON the HRM in hrm_base; offset is ADDED to the
  // nominal position. Compliance therefore needs RHS = environment - target:
  // with target=0 a positive external force must produce a positive offset.
  // Adapt to the legacy filter's desired-minus-environment algebra without
  // changing its 6x6 dynamics. This is NOT another sensor/frame conversion;
  // the node's force_sign conversion and the public diagnostics stay intact.
  auto del_xf = this->admittance_filter_.computeAdmittance(-f_desired, -f_env, dt);
  this->f_desired_ = f_desired;
  this->f_env_ = f_env;
  this->del_f_ = this->admittance_filter_.force_error_;
  this->dt_ = dt;
  this->del_xf_ = del_xf;
  return del_xf;
}
