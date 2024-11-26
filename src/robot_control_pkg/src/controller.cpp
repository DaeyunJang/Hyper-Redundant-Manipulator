#include "controller.hpp"
#include "hw_definition.hpp"
#include "dynamics_parameters.hpp"
#include <cmath>
#include <vector>
#include <numeric>
#include <iterator>

Controller::Controller() {initialize();}
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
    const std::vector<double>& tension,
    const std::vector<double>& force_external
) {
    /**
     * @todo length of manipulator should be calculated from the segment_estimation_package
     */
    double kLength = TOTAL_LENGTH * 0.001;    // temp
    double kCenterToHole = WIRE_DISTANCE * 0.001;    // mm
    // data initializing
    if (theta_actual_prev_.empty()) {
        theta_actual_prev_ = theta_actual;
    }
    if (dtheta_dt_actual_prev_.empty()) {
        dtheta_dt_actual_prev_ = dtheta_dt_actual;
    }

    // theta_actual and angular velocity of tip
    end_effector_theta_actual_ = std::accumulate(theta_actual.begin(), theta_actual.end(), 0.0);
    end_effector_dtheta_dt_actual_ = std::accumulate(dtheta_dt_actual.begin(), dtheta_dt_actual.end(), 0.0);

    // get torques_input by PID controller
    torque_input_ = pid_controller_.compute_output(theta_desired, end_effector_theta_actual_, dt);

    // Find B and F_friction
    dandf_ = damping_friction_model_.compute_dampingCoeff_and_friction(
        theta_actual,
        dtheta_dt_actual,
        theta_actual_prev_,
        dtheta_dt_actual_prev_,
        tension,
        2);
    
    // hrm_dynamics_model_.update_inertia(0.02);
    hrm_dynamics_model_.update_damping_coefficient(dandf_[0]);
    // std::cout << "<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // std::copy(dandf.begin(), dandf.end(), 
    //         std::ostream_iterator<double>(std::cout, " "));
    // std::cout << std::endl;
    // std::cout << hrm_dynamics_model_.inertia_ << std::endl;
    // std::cout << hrm_dynamics_model_.damping_coef_ << std::endl;
    // std::cout << hrm_dynamics_model_.stiffness_ << std::endl;
    // std::cout << "<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    /**
     * @brief
     * conversion forces to torques (external, frction)
     * @note
     * (-1) is multiplied, because the fx and fy calculated from "LSTM force estimation" has the F/T sensor coordinate
     * @param tau_ext : N-m
     * @param force_external : N-m
     * @param tau_friction : N-m
     */
    tau_ext_ = (-1) * kLength * (force_external[0]*cos(end_effector_theta_actual_) - force_external[1]*sin(end_effector_theta_actual_));
    tau_friction_ = kCenterToHole * dandf_[1];

    // calculate angular acceleration
    theta_ddot_input_ = hrm_dynamics_model_.compute_angular_acceleration(
        torque_input_,
        tau_ext_,
        tau_friction_,
        end_effector_theta_actual_,
        end_effector_dtheta_dt_actual_);

    // calculate angular velocity
    theta_dot_input_ = theta_ddot_input_ * dt;
    // calculate angle
    theta_input_ = theta_dot_input_ * dt;
    // calibration DY definition to Y.J. Kim kinematics definition
    theta_input_ *= -1 * surgical_tool_.todeg();
    // innverse kinematics for moving the wire
    auto wire_length_to_move = surgical_tool_.get_IK_result(theta_input_, 0, 0);

    theta_actual_prev_ = theta_actual;
    dtheta_dt_actual_prev_ = dtheta_dt_actual;

    // std::cout <<  "===========================================================" << std::endl;
    // std::cout << "KP: " << pid_controller_.kp_ << std::endl;
    // std::cout << "KI: " << pid_controller_.ki_ << std::endl;
    // std::cout << "KD: " << pid_controller_.kd_ << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "integral: " << pid_controller_.get_integral() << std::endl;
    // std::cout << "prev_error: " << pid_controller_.get_previous_error() << std::endl;
    // std::cout << "theta_desired: "    << theta_desired    << std::endl;
    // std::cout << "theta_actual: ";
    // std::copy(theta_actual.begin(), theta_actual.end(), 
    //         std::ostream_iterator<double>(std::cout, " "));
    // std::cout << std::endl;
    // std::cout <<  "dt: "               << dt               << std::endl;
    // std::cout <<  "tension: "   << tension[0] << " " << tension[1] << std::endl;
    // std::cout <<  "force_external: "   << force_external[0] << force_external[1] << std::endl;
    // std::cout <<  "torque_input: "                    << torque_input                     << std::endl;
    // std::cout <<  "tau_ext: "                         << tau_ext                          << std::endl;
    // std::cout <<  "tau_friction: "                    << tau_friction                     << std::endl;
    // std::cout <<  "end_effector_theta_actual: "       << end_effector_theta_actual        << std::endl;
    // std::cout <<  "end_effector_dtheta_dt_actual: "   << end_effector_dtheta_dt_actual    << std::endl;
    // std::cout <<  "theta_ddot_input: "                << theta_ddot_input                 << std::endl;
    // std::cout <<  "theta_dot_input: "                 << theta_dot_input                  << std::endl;
    // std::cout <<  "theta_input: "                     << theta_input                      << std::endl;

    // std::cout <<  "wire_length_to_move: "             << wire_length_to_move              << std::endl;
    // std::cout << "wire_length_to_move: ";
    // for (const auto& value : wire_length_to_move) {
    //     std::cout << value << " ";
    // }
    // std::cout << std::endl;

    return wire_length_to_move;
}