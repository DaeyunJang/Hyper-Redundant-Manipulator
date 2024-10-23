#ifndef DAMPING_FRICTION_MODEL_HPP
#define DAMPING_FRICTION_MODEL_HPP

#include <vector>

/**
 * @brief Calculate the B value and friction based on joint angles and velocities.
 *
 * This function calculates the B parameter and friction for a segment of a cable-driven continuum robot.
 *
 * @param q Vector of joint angles (size: n, 1)
 * @param dq Vector of joint velocities (size: n, 1)
 * @param p_q Vector of previous joint angles (size: n, 1)
 * @param p_dq Vector of previous joint velocities (size: n, 1)
 * @param mode Determines the friction model to use (0 = constant friction, 1 = variable friction)
 * @return std::pair<double, double> Pair where first value is B, and second is the resulting friction.
 *
 * @example
 * #include "B_and_Fric.hpp"
 * #include <iostream>
 *
 * int main() {
 *     std::vector<double> q = {0.1, 0.2, 0.3};
 *     std::vector<double> dq = {0.01, 0.02, 0.03};
 *     std::vector<double> p_q = {0.05, 0.1, 0.15};
 *     std::vector<double> p_dq = {0.005, 0.01, 0.015};
 *     int mode = 1;
 *
 *     auto result = B_and_Fric(q, dq, p_q, p_dq, mode);
 *     std::cout << "B: " << result.first << ", res_friction: " << result.second << std::endl;
 *     return 0;
 * }
 */
std::pair<double, double> DampingFrictionModel(const std::vector<double>& q,
                                  const std::vector<double>& dq,
                                  const std::vector<double>& p_q,
                                  const std::vector<double>& p_dq,
                                  int mode);

#endif  // DAMPING_FRICTION_MODEL_HPP