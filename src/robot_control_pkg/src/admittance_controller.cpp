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
                     admittance_params::SPRING_MATRIX) {}

AdmittanceController::~AdmittanceController() {}


Eigen::VectorXd AdmittanceController::compute(
  const Eigen::VectorXd& f_desired,
  const Eigen::VectorXd& f_external
) {
  this-> f_desired_ = f_desired;
  this-> f_external_ = f_external;
  this-> del_f_ = f_desired - f_external;

  auto del_xf = this->admittance_filter_.computeAdmittance(f_desired, f_external);
  del_xf_ = del_xf;
  return del_xf_;
}