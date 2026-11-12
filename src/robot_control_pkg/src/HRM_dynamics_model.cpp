#include "HRM_dynamics_model.hpp"

// Constructor: initialize the default system parameters.
HRMDynamicsModel::HRMDynamicsModel() {}

HRMDynamicsModel::HRMDynamicsModel(double inertia, double damping_coef, double stiffness)
    : inertia_(inertia), damping_coef_(damping_coef), stiffness_(stiffness) {}
HRMDynamicsModel::~HRMDynamicsModel() {}

// Set the system parameters.
void HRMDynamicsModel::set_parameters(double inertia, double damping_coef, double stiffness) {
    inertia_ = inertia;
    damping_coef_ = damping_coef;
    stiffness_ = stiffness;
}

// Update inertia independently.
void HRMDynamicsModel::update_inertia(double inertia) {
    inertia_ = inertia;
}

// Update the damping coefficient independently.
void HRMDynamicsModel::update_damping_coefficient(double damping_coef) {
    damping_coef_ = damping_coef;
}

// Update the spring constant independently.
void HRMDynamicsModel::update_stiffness(double stiffness) {
    stiffness_ = stiffness;
}


// Dynamics: calculate torque (I * acceleration + B * velocity + K * position).
double HRMDynamicsModel::compute_torque(double position, double velocity) {
    // I * acceleration (velocity-based approximation of acceleration).
    double acceleration = velocity / inertia_;  // Simplified model assumption.

    // Torque calculation: I⋅θ¨ + B⋅θ˙ + K⋅θ + τ_ext + τ_fric = τ_input.
    return inertia_ * acceleration + damping_coef_ * velocity + stiffness_ * position;
}

double HRMDynamicsModel::compute_angular_acceleration( double tau_input,
                                                    double tau_external,
                                                    double tau_friction,
                                                    double theta,
                                                    double theta_dot )
{
    double theta_ddot = 0;
    theta_ddot = (tau_input - (damping_coef_*theta_dot + stiffness_*theta + tau_external + tau_friction) ) / inertia_;
    return theta_ddot;
}

