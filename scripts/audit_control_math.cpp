// Read-only characterization, no ROS node or hardware access.
// See docs/CONTROL_READINESS.md for the build command and interpretation.
#include <iomanip>
#include <iostream>

#include "admittance_filter.hpp"
#include "control_parameters.hpp"
#include "position_controller.hpp"

int main()
{
  std::cout << std::setprecision(10);
  for (int axis : {1, 2}) {
    PositionController controller;
    Eigen::VectorXd actual = Eigen::VectorXd::Zero(6);
    controller.reset(actual, 0.0, 0.0);
    Eigen::VectorXd goal = actual;
    goal(axis) = 0.001;  // 1 mm, no plant motion.
    controller.update(goal, actual, position_control_params::DT);
    std::cout << "1mm_step axis=" << axis
              << " delta_tilt_rad=" << controller.del_theta_tilt_
              << " delta_pan_rad=" << controller.del_theta_pan_
              << " command_tilt_deg=" << controller.surgical_tool_.tAngle_ * 180 / M_PI
              << " command_pan_deg=" << controller.surgical_tool_.pAngle_ * 180 / M_PI
              << std::endl;
  }
  PositionController accumulated;
  Eigen::VectorXd actual = Eigen::VectorXd::Zero(6);
  accumulated.reset(actual, 0.0, 0.0);
  Eigen::VectorXd goal = actual;
  goal(1) = 0.0001;  // Same 0.1 mm error; runtime rejects stale/duplicate frames.
  for (int i = 0; i < 10; ++i) {
    accumulated.update(goal, actual, position_control_params::DT);
  }
  std::cout << "constant_0.1mm_10_updates_tilt_deg="
            << accumulated.surgical_tool_.tAngle_ * 180 / M_PI << std::endl;
  for (int axis : {1, 2}) {
    AdmittanceFilter filter(admittance_params::MASS_MATRIX,
      admittance_params::DAMPER_MATRIX, admittance_params::SPRING_MATRIX);
    Eigen::VectorXd desired = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd environment = desired;
    environment(axis) = 0.1;
    auto displacement = filter.computeAdmittance(desired, environment, admittance_params::DT);
    std::cout << "0.1N_axis=" << axis << " displacement=" << displacement.transpose()
              << " finite=" << displacement.allFinite() << std::endl;
  }
  AdmittanceFilter biased(admittance_params::MASS_MATRIX,
    admittance_params::DAMPER_MATRIX, admittance_params::SPRING_MATRIX);
  Eigen::VectorXd desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd environment = desired;
  desired(0) = desired(1) = 0.02;  // Same hardcoded setpoint as control_node.
  Eigen::VectorXd displacement;
  for (int i = 0; i < 300; ++i) {
    displacement = biased.computeAdmittance(desired, environment, admittance_params::DT);
  }
  std::cout << "zero_external_force_10s_displacement=" << displacement.transpose() << std::endl;
  PIDController invalid_dt;
  invalid_dt.set_PID_gains(250, 5, 100);
  std::cout << "legacy_dynamics_PID_dt_zero_output=" << invalid_dt.compute_output(0.001, 0.0, 0.0) << std::endl;
}
