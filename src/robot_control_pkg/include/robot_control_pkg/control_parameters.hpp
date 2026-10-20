#pragma once
/**
 * @file control_parameters.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2025-04-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef CONTROL_PARAMETERS_HPP_
#define CONTROL_PARAMETERS_HPP_
#include <Eigen/Dense>

/**********************************************************************
 * @brief Dynamics controller
 **********************************************************************/
namespace dynamics_params {

// PID controller ================
// slow version MasterMACS 1000-100 - double of upper values
// #define KP  150.0
// #define KI  3.0
// #define KD  5.0

// fast version MasterMACS 2000-100 - double of upper values
inline constexpr double KP  = 250.0;
inline constexpr double KI  = 5.0;
inline constexpr double KD  = 50.0;

inline constexpr double SAMPLING_HZ = 30.0;
inline constexpr double DT = 1.0 / SAMPLING_HZ;

// HRM equation of motion parameters (I B K)
// I: mass
// B: damper
// K: spring
inline constexpr double INERTIA = 0.01;
inline constexpr double DAMPING = 0.0;
inline constexpr double STIFFNESS = 2.0;

} // namespace dynamics


/**********************************************************************
 * @brief Position controller
 **********************************************************************/
namespace position_control_params {

// PID controller ================
// Initial software-test gains, NOT validated hardware tuning.
// Output is an angular correction [rad], input is tip position [m].
inline constexpr double KP  = 5.0;    // rad/m
inline constexpr double KI  = 0.0;    // PD only for the current bring-up
inline constexpr double KD  = 0.05;   // rad*s/m, derivative on measurement
inline constexpr double DERIVATIVE_FILTER_SEC = 0.05;
inline constexpr double MAX_ANGULAR_SPEED_DEG_S = 5.0;
inline constexpr double FEEDBACK_TIMEOUT_SEC = 0.25;
// Nominal reference for standalone tests; runtime dt comes from image stamps.
inline constexpr double SAMPLING_HZ = 30.0;
inline constexpr double DT = 1.0 / SAMPLING_HZ;
} // namespace position_control_params


/**********************************************************************
 * @brief Admittance controller
 **********************************************************************/
namespace admittance_params {

// Admittance equation of motion parameters (M B K)
// M_d: desired mass
// B_d: desired damper
// K_d: desired spring
// inline constexpr double M_d = 1.0; // kg
// inline constexpr double B_d = 10.0;  // N-s/m
// inline constexpr double K_d = 100.0; // N/m

inline constexpr double M_d = 0.05; // kg
inline constexpr double B_d = 1.0;  // N-s/m
inline constexpr double K_d = 20.0; // N/m
inline constexpr double SAMPLING_HZ = 30.0;
inline constexpr double DT = 1.0 / SAMPLING_HZ;
// M: Mass matrix
inline const Eigen::MatrixXd MASS_MATRIX = [] {
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6, 6);
    M(1, 1) = M_d;  // y축만 반응
    return M;
}();

// C: Damper matrix
inline const Eigen::MatrixXd DAMPER_MATRIX = [] {
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 6);
    B(1, 1) = B_d;
    return B;
}();

// K: Spring matrix
inline const Eigen::MatrixXd SPRING_MATRIX = [] {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(6, 6);
    K(1, 1) = K_d;
    return K;
}();

} // namespace admittance_params

#endif // CONTROL_PARAMETERS_HPP_
