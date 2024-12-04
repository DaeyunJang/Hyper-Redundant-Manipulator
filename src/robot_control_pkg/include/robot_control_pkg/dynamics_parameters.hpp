#pragma once
/**
 * @file dynamics_parameters.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2024-10-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef DYNAMICS_PARAMETERS_HPP_
#define DYNAMICS_PARAMETERS_HPP_

// PID controller
#define KP  150.0
#define KI  3.0
#define KD  5.0

// fast version MasterMACS 2000-100 - double of upper values
// #define KP  150.0
// #define KI  10.0
// #define KD  20.0
#define SAMPLING_HZ  30.0    // Hz
#define DT  1/SAMPLING_HZ  // sec

// HRM equation of motion parameters (I B K)
// I: mass
// B: damper
// K: spring
#define INERTIA 0.01
#define DAMPING_COEFFICIENT 0
#define STIFFNESS 1

#define FRICTION_MODE 2

#endif // DYNAMICS_PARAMETERS_HPP_