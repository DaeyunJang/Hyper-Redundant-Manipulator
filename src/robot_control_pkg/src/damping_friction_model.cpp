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
std::vector<double> DampingFrictionModel::compute_dampingCoeff_and_friction(
    const std::vector<double>& q,
    const std::vector<double>& dq,
    const std::vector<double>& q_prev,
    const std::vector<double>& dq_prev,
    int mode=1)
{   
    /**
     * @brief Check the vetors size are equal.
     * @author DY
     */
    if (q.size() != dq.size() || q.size() != q_prev.size() || q.size() != dq_prev.size()) {
        std::cerr << "Error: q, dq q_prev dq_prev vectors have different size!" << std::endl;
        return {0, 0};
    }

    int n = q.size();
    std::vector<double> Tl(n, 0.0), Tr(n, 0.0), tau_j(n, 0.0), th_seg_dot(n, 0.0), B_seg(n, 0.0);
    Tl[0] = 2.0;
    Tr[0] = 1.5;
    
    double th_b = 0.451;
    double R = 0.05;
    double mu_s = 0.6, mu_v = 0.3, vs = 0.1, vd = 1.1;
    double del_v_hat = vd - 3 * vs;
    double del_v_u = vd - vs;
    std::vector<double> del_v(n, 0.0);
    std::vector<double> friction_l(n, 0.0), friction_r(n, 0.0);
    double c2 = 0.02;

    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            del_v[i] = dq[i];
        } else {
            del_v[i] = dq[i] - dq[i - 1];
        }
    }

    for (int j = 0; j < n; ++j) {
        double c1 = mu_v;
        if (mode == 1) {
            if (del_v[j] < vs) {
                c1 = 0.5 * mu_s * (2 * vs - del_v[j]) * std::pow(del_v[j] + vs, 2) * std::pow(vs, -3) + mu_s;
            } else if (del_v[j] < vd) {
                c1 = mu_v + del_v[j] * (del_v_hat + 2 * del_v[j]) * std::pow(vd - del_v[j], 2) / std::pow(del_v_u, 3);
            } 
        } else if (mode == 2) {
            c1 = mu_v + (mu_s - mu_v) * std::exp(-std::pow(del_v[j] / vs, 2));
        }
        c1 = std::abs(c1);

        if (j == 0) {
            friction_l[j] = 0.0;
            friction_r[j] = 0.0;
        } else {
            Tl[j] = Tl[j - 1] * std::exp(c1 * q[j - 1]);
            Tr[j] = Tr[j - 1] * std::exp(c1 * q[j - 1]);
            friction_l[j] = Tl[j] - Tl[j - 1];
            friction_r[j] = Tr[j] - Tr[j - 1];
        }

        double del_th = (j == 0) ? q[j] : q[j] - q[j - 1];
        tau_j[j] = R * (std::cos(del_th / 2) * (Tl[j] * std::sin(th_b) - Tr[j] * std::sin(th_b)) - 
                        std::sin(del_th / 2) * (Tl[j] * std::cos(th_b) + Tr[j] * std::cos(th_b)));
        th_seg_dot[j] = dq[j] - dq_prev[j];

        if (th_seg_dot[j] != 0) {
            B_seg[j] = std::abs(tau_j[j] / th_seg_dot[j]);
        } else {
            B_seg[j] = 0.0;
        }
    }

    double B = 0.0;
    for (const auto& b : B_seg) {
        B += b;
    }

    double res_friction = std::abs(std::accumulate(friction_l.begin(), friction_l.end(), 0.0) +
                                   std::accumulate(friction_r.begin(), friction_r.end(), 0.0));

    if (std::isnan(res_friction)) res_friction = 0.0;
    if (std::isnan(B)) B = 0.0;

    q_prev_ = q;
    dq_prev_ = dq;

	std::vector<double> results = {B, res_friction};
    return results;
    // return std::make_pair(B, res_friction);
}
