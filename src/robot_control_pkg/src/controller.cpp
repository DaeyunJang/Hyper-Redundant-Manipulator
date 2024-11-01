#include "controller.hpp"
#include "hw_definition.hpp"
#include "dynamics_parameters.hpp"
#include <cmath>
#include <vector>
#include <numeric>

Controller::Controller() {}
Controller::~Controller() {}

void Controller::initialize() {
    pid_controller_.set_PID_gains(KP, KI, KD);
    hrm_dynamics_model_.set_parameters(INERTIA, DAMPING_COEFFICIENT, STIFFNESS);
    surgical_tool_.init_surgical_tool(NUM_OF_JOINT, SEGMENT_ARC, SEGMENT_DIAMETER, WIRE_DISTANCE, SHIFT);
}

std::vector<double> Controller::compute(
    const double& theta_desired,
    const std::vector<double>& theta_actual,
    const std::vector<double>& dtheta_dt_actual,
    const double& dt,
    const std::vector<double>& force_external
) {
    /**
     * @todo length of manipulator should be calculated from the segment_estimation_package
     */
    const double kLength = 9;    // temp
    const double kCenterToHole = 3;    // mm
    // data initializing
    if (theta_actual_prev_.empty()) {
        theta_actual_prev_ = theta_actual;
    }
    if (dtheta_dt_actual_prev_.empty()) {
        dtheta_dt_actual_prev_ = dtheta_dt_actual;
    }

    // theta_actual and angular velocity of tip
    double end_effector_theta_actual = std::accumulate(theta_actual.begin(), theta_actual.end(), 0.0);
    double end_effector_dtheta_dt_actual = std::accumulate(dtheta_dt_actual.begin(), dtheta_dt_actual.end(), 0.0);

    // get torques_input by PID controller
    double torque_input = pid_controller_.compute_output(theta_desired, end_effector_theta_actual, dt);

    // Find B and F_friction
    std::pair<double, double> dandf = damping_friction_model_.compute_dampingCoeff_and_friction(
        theta_actual,
        dtheta_dt_actual,
        theta_actual_prev_,
        dtheta_dt_actual_prev_,
        1);
    
    hrm_dynamics_model_.update_damping_coefficient(dandf.first);

    /**
     * @brief
     * conversion forces to torques (external, frction)
     * @note
     * (-1) is multiplied, because the fx and fy calculated from "LSTM force estimation" has the F/T sensor coordinate
     */
    double tau_ext = (-1) * kLength * (force_external[0]*cos(end_effector_theta_actual) - force_external[1]*sin(end_effector_theta_actual));
    double tau_friction = kCenterToHole * dandf.second;

    // calculate angular acceleration
    double theta_ddot_input = hrm_dynamics_model_.compute_angular_acceleration(
        torque_input,
        tau_ext,
        tau_friction,
        end_effector_theta_actual,
        end_effector_dtheta_dt_actual);

    // calculate angular velocity
    double theta_dot_input = theta_ddot_input * dt;
    // calculate angle
    double theta_input = theta_dot_input * dt;
    // innverse kinematics for moving the wire
    auto wire_length_to_move = surgical_tool_.get_IK_result(theta_input, 0, 0);

    theta_actual_prev_ = theta_actual;
    dtheta_dt_actual_prev_ = dtheta_dt_actual;

    return wire_length_to_move;
}