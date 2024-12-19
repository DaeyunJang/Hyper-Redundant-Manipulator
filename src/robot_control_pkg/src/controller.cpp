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
    const double& end_effector_theta_actual,
    const double& end_effector_omega_actual,
    const std::vector<double>& theta_actual,
    const std::vector<double>& dtheta_dt_actual,
    const std::vector<double>& cable_velocity_actual,
    const double& dt,
    const std::vector<double>& tension,
    const std::vector<double>& force_external,
    const bool& hrm_controller_enable
) {
    /**
     * @todo length of manipulator should be calculated from the segment_estimation_package
     */
    double kLength = TOTAL_LENGTH * 0.001;    // temp
    double kCenterToHole = WIRE_DISTANCE * 0.001;    // mm
    // data initializing
    theta_actual_ = theta_actual;
    dtheta_dt_actual_ = dtheta_dt_actual;
    cable_velocity_actual_ = cable_velocity_actual;

    if (theta_actual_prev_.empty()) {
        theta_actual_prev_ = theta_actual_;
    }
    if (dtheta_dt_actual_prev_.empty()) {
        dtheta_dt_actual_prev_ = dtheta_dt_actual_;
    }
    if (cable_velocity_actual_prev_.empty()) {
        cable_velocity_actual_prev_ = cable_velocity_actual_;
    }

    // theta_actual and angular velocity of tip
    end_effector_theta_actual_ = end_effector_theta_actual;
    end_effector_dtheta_dt_actual_ = end_effector_omega_actual;

    // get torques_input by PID controller
    torque_input_ = pid_controller_.compute_output(theta_desired, end_effector_theta_actual_, dt);

    if (hrm_controller_enable == true) {
        // Find B and F_friction
        int mode = damping_friction_model_.mode_;

        // 각 요소의 값
        double divided_value = cable_velocity_actual_[0] / NUM_OF_JOINT;
        double divided_prev_value = cable_velocity_actual_prev_[0] /NUM_OF_JOINT;
        // n개 크기의 배열 생성 및 값 할당
        std::vector<double> cable_velocity(NUM_OF_JOINT, divided_value);
        std::vector<double> cable_prev_velocity(NUM_OF_JOINT, divided_prev_value);

        auto [B, res_friction, cmode] = damping_friction_model_.compute_dampingCoeff_and_friction(
            theta_actual_,
            dtheta_dt_actual_,
            cable_velocity,
            theta_actual_prev_,
            dtheta_dt_actual_prev_,
            cable_prev_velocity,
            tension,
            mode);
        
        // hrm_dynamics_model_.update_inertia(0.02);
        hrm_dynamics_model_.update_damping_coefficient(B);

        /**
         * @brief
         * conversion forces to torques (external, frction)
         * @note
         * (-1) is multiplied, because the fx and fy calculated from "LSTM force estimation" has the F/T sensor coordinate
         * @param tau_ext : N-m
         * @param force_external : N-m
         * @param tau_friction : N-m
         */
        // std::cout << "dandf0: " << dandf_[0] << " dandf1: " << dandf_[1] << std::endl;
        tau_ext_ = (-1) * kLength * (force_external[0]*cos(end_effector_theta_actual_) - force_external[1]*sin(end_effector_theta_actual_));
        tau_friction_ = kCenterToHole * res_friction;

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
        // double final_theta_input = theta_input_ * (-1) * surgical_tool_.todeg();
        double final_theta_input = (end_effector_theta_actual_ + theta_input_) * (-1) * surgical_tool_.todeg();
        // std::cout << "final_theta_input: " << final_theta_input << std::endl;
        auto wire_length_to_move = surgical_tool_.get_IK_result(final_theta_input, 0, 0);

        theta_actual_prev_ = theta_actual_;
        dtheta_dt_actual_prev_ = dtheta_dt_actual_;
        cable_velocity_actual_prev_ = cable_velocity_actual_;

        return wire_length_to_move;
    }
    else {  // hrm_controller_enable == false
    
        double final_theta_input = (torque_input_ + end_effector_theta_actual_) * (-1) * surgical_tool_.todeg();
        auto wire_length_to_move = surgical_tool_.get_IK_result(final_theta_input, 0, 0);
        
        std::cout << "final_theta_input: " << final_theta_input << std::endl;

        return wire_length_to_move;
    }
}