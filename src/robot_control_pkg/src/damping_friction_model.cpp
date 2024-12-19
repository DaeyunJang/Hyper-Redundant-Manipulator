#include "damping_friction_model.hpp"

#include <iostream>
#include <ostream>
#include <cmath>
#include <algorithm> // for std::abs
#include <utility>   // for std::pair
#include <numeric>

DampingFrictionModel::DampingFrictionModel() {}
DampingFrictionModel::~DampingFrictionModel() {}

/**
 * @author DY
 * @brief calculate B(damping coeff) and f_fric(friction force)
 * 
 * @param q
 * @param dq
 * @param q_prev
 * @param dq_prev
 * @param mode
 * @return std::pair<double, double> [Damping coefficient, friction_force]
 * @example
 * value of std::pair can be accessed by call 'first' and 'second'
 * std::pair<double, double> results = compute_dampingCoeff_and_friction(...);
 * std::cout << results.first << | << results.second << std::endl;
 */
// std::pair<double, double> DampingFrictionModel::compute_dampingCoeff_and_friction(
std::tuple<double, double, int> DampingFrictionModel::compute_dampingCoeff_and_friction(
    const std::vector<double>& q,
    const std::vector<double>& dq,
    const std::vector<double>& cable_vel,
    const std::vector<double>& q_prev,
    const std::vector<double>& dq_prev,
    const std::vector<double>& cable_vel_prev,
    const std::vector<double>& tension,
    const int& mode)
{   
    // std::cout << "Fric MODE: " << mode << std::endl;
    
    /**
     * @brief Check the vetors size are equal.
     * @author DY
     */
    if (q.size() != dq.size() || q.size() != q_prev.size() || q.size() != dq_prev.size()) {
        std::cerr << "Error: q, dq q_prev dq_prev vectors have different size!" << std::endl;
        return {0, 0, 0};
    }

    int n = q.size();

    double th_b = 0.451;
    double R = 0.05;
    double mu_s = 0.6, mu_v = 0.3, vs = 0.1, vd = 1.1;
    double del_v_hat = vd - 3 * vs;
    double del_v_u = vd - vs;
    std::vector<double> Tl(n, 0.0), Tr(n, 0.0), tau_j(n, 0.0), th_seg_dot(n, 0.0), B_seg(n, 0.0);
    std::vector<double> del_v(n, 0.0);
    std::vector<double> friction_l(n, 0.0), friction_r(n, 0.0);
    double cmode = 0;
    double stop_threshold = 0.01;
    stop_state_ = (std::abs(cable_vel[0]) < stop_threshold) ? true : false;

    // Tl[0] = 2.0;
    // Tr[0] = 1.5;
    Tl[0] = tension[0];
    Tr[0] = tension[1];

    // Compute del_v
    for (size_t i = 0; i < n; ++i) {
        del_v[i] = std::abs(cable_vel[i] - cable_vel_prev[i]);
    }

    // Segment-wise B and Friction calculation
    for (size_t j = 0; j < n; ++j) {

        // Friction calculation
        double c1 = 0.0;
        if (mode == 1) {
            if (del_v[j] < vs) {
                cmode = 1;
                c1 = 0.5 * mu_s * (2 * vs - del_v[j]) * std::pow((del_v[j] + vs), 2) / std::pow(vs, 3) + mu_s;
            } else if (del_v[j] < vd) {
                cmode = 2;
                c1 = mu_v + del_v_u * (del_v_hat + 2 * del_v[j]) * std::pow((vd - del_v[j]), 2) / std::pow(del_v_u, 3);
            } else {
                cmode = 3;
                c1 = mu_v;
            }
        } else if (mode == 2) {
            cmode = 3;
            c1 = mu_v + (mu_s - mu_v) * std::exp(-std::pow((del_v[j] / vs), 2));
        } else if (mode == 3) {
            cmode = 3;
            c1 = stop_state_ ? mu_s : mu_v;    // true -> stop -> mu_s, false -> moving : mu_v;
        }

        c1 = std::max(std::min(std::abs(c1), 1.0), 0.0);

        // B and Friction adjustment
        if (j == 0) {
            friction_l[j] = 0.0;
            friction_r[j] = 0.0;
        } else {
            Tl[j] = Tl[j - 1] * std::exp(-c1 * std::abs(q[j - 1]) / 2.0);
            Tr[j] = Tr[j - 1] * std::exp(-c1 * std::abs(q[j - 1]) / 2.0);

            friction_l[j] = Tl[j - 1] - Tl[j];
            friction_r[j] = Tr[j - 1] - Tr[j];
        }

        double del_th = q[j];
        tau_j[j] = R * (std::cos(del_th / 2.0) * (Tl[j] * std::sin(th_b) - Tr[j] * std::sin(th_b)) -
                        std::sin(del_th / 2.0) * (Tl[j] * std::cos(th_b) + Tr[j] * std::cos(th_b)));

        th_seg_dot[j] = std::abs((dq[j] - dq_prev[j]) / 2.0);
        B_seg[j] = std::abs(tau_j[j] / th_seg_dot[j]);

        if (B_seg[j] > 1.0) {
            B_seg[j] = 0.1;  // Add noise or cap the value appropriately
        }
    }

    // Adjust B_seg for stability
    for (size_t m = 0; m < B_seg.size(); ++m) {
        for (size_t k = 0; k < B_seg.size(); ++k) {
            double sumOthers = std::accumulate(B_seg.begin(), B_seg.end(), 0.0) - B_seg[k];
            if (B_seg[k] > sumOthers) {
                B_seg[k] = sumOthers / (B_seg.size() - 1);
            }
        }
    }

    // Compute final B and friction results
    double B = std::accumulate(B_seg.begin(), B_seg.end(), 0.0);
    double res_friction = std::abs(std::accumulate(friction_l.begin(), friction_l.end(), 0.0) +
                                   std::accumulate(friction_r.begin(), friction_r.end(), 0.0));

    if (std::isnan(res_friction)) res_friction = 0.0;
    if (std::isnan(B)) B = 0.0;
    
    if (mode == 0) {
        res_friction = 0;
    }

    q_prev_ = q;
    dq_prev_ = dq;
    
    B_ = B;
    res_friction_ = res_friction;
    cmode_ = cmode;
    
    return {B, res_friction, cmode};
    // return std::make_pair(B, res_friction);
}
