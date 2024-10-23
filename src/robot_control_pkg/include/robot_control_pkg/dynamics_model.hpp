#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP

#include <iostream>
#include <vector>

#include "hw_definition.hpp"

/**
 * @brief Equation of Motion (dynamci equation)
 * @authors DY, YG
 * @version 1.0
 * @note    I⋅θ¨ + B⋅θ˙ + K⋅θ + τ_ext + τ_fric = τ_input
 */
class DynamcisModel {
public:
  DynamcisModel(double inertia, double damping_coef, double stiffness);
  ~DynamcisModel();
  void set_parameters(double inertia, double damping_coef, double stiffness);

  void update_inertia(double inertia);
  void update_damping(double damping_coef_);
  void update_stiffness(double stiffness);

  void compute_torque(double position, double velocity);

  /**
   * @brief // angular acceleration: θ¨ = {τ_input - (B⋅θ˙ + K⋅θ + τ_ext + τ_fric)} / I
   *
   * @param tau_input
   * @param tau_external
   * @param tau_friction
   * @return double - angular acceleration
   */
  double compute_angular_acceleration(double tau_input,
                                      double tau_external,
                                      double tau_friction,
                                      double theta,
                                      double theta_dot
  );

private:
  double intertia_;
  double damping_coef_;
  double stiffness_;

  std::vector<double> theta_;
};

#endif // DYNAMICS_MODEL_HPP