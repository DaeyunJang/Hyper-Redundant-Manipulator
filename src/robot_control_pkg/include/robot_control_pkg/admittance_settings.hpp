#pragma once

#include <cmath>
#include <stdexcept>
#include "control_parameters.hpp"

namespace hrm {
struct AdmittanceSettings {
  double mass = admittance_params::M_d;
  double damping = admittance_params::B_d;
  double stiffness = admittance_params::K_d;
  double target_y = 0.0, target_z = 0.0;
  double force_sign = admittance_params::FORCE_SIGN;
  double max_force = admittance_params::MAX_FORCE_N;
  double timeout = admittance_params::FORCE_TIMEOUT_SEC;
  // The current experiment assumes physical tip contact; localization is
  // display-only unless the operator explicitly enables this gate.
  bool use_contact_id_gate = false;

  void validate() const {
    for (double value : {mass, damping, stiffness, target_y, target_z, force_sign,
                         max_force, timeout}) {
      if (!std::isfinite(value)) {throw std::invalid_argument("Non-finite admittance parameter.");}
    }
    if (mass < 0.0 || damping < 0.0 || stiffness < 0.0 ||
        (mass == 0.0 && damping == 0.0 && stiffness == 0.0) ||
        (force_sign != -1.0 && force_sign != 1.0) ||
        max_force <= 0.0 || timeout <= 0.0 || timeout > 0.25 ||
        std::hypot(target_y, target_z) > max_force) {
      throw std::invalid_argument(
        "Admittance: M/B/K >=0 (not all zero), force_sign +/-1, positive limits, "
        "timeout <=0.25s, target force norm <=max_force_N.");
    }
  }
};
}  // namespace hrm
