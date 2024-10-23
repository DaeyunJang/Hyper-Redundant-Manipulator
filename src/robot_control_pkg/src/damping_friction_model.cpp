#include "damping_friction_model.hpp"

#include <cmath>
#include <algorithm> // for std::abs
#include <utility>   // for std::pair

/**
 * @todo Optimization with YG
 */
std::pair<double, double> DampingFrictionModel( const std::vector<double>& q,
                                                const std::vector<double>& dq,
                                                const std::vector<double>& p_q,
                                                const std::vector<double>& p_dq,
                                                int mode)
{
    // 공통
    int n = q.size();
    std::vector<double> Tl(n, 0.0);
    Tl[0] = 2.0;
    std::vector<double> Tr(n, 0.0);
    Tr[0] = 1.5;

    // B segment parameter
    std::vector<double> tau_j(n, 0.0);
    std::vector<double> th_seg_dot(n, 0.0);
    std::vector<double> B_seg(n, 0.0);
    double th_b = 0.451;
    double R = 0.05;

    // Friction parameters
    double mu_s = 0.6;
    double mu_v = 0.3;
    double vs = 0.1;
    double vd = 1.1;
    double del_v_hat = vd - 3 * vs;
    double del_v_u = vd - vs;

    std::vector<double> del_v(n, 0.0);
    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            del_v[i] = dq[i];
        } else {
            del_v[i] = dq[i] - dq[i - 1];
        }
    }

    std::vector<double> friction_l(n, 0.0);
    std::vector<double> friction_r(n, 0.0);
    double c2 = 0.02;

    // segment 당 B와 Friction 계산
    for (int j = 0; j < n; ++j) {
        // Friction 계산
        double c1 = 0.0;
        if (mode == 1) {
            if (del_v[j] < vs) {
                c1 = 0.5 * mu_s * (2 * vs - del_v[j]) * std::pow((del_v[j] + vs), 2) * std::pow(vs, -3) + mu_s;
            } else if (del_v[j] < vd) {
                c1 = mu_v + del_v[j] * (del_v_hat + 2 * del_v[j]) * std::pow((vd - del_v[j]), 2) * std::pow(del_v_u, -3);
            } else {
                c1 = mu_v;
            }
        } else if (mode == 2) {
            c1 = mu_v + (mu_s - mu_v) * exp(-std::pow((del_v[j] / vs), 2));
        } else {
            c1 = mu_v;
        }
        c1 = std::abs(c1);

        // B 계산
        double del_th, del_th_p;
        if (j == 0) {
            del_th = q[j];
            del_th_p = p_q[j];
            friction_l[j] = 0.0;
            friction_r[j] = 0.0;
        } else {
            del_th = q[j] - q[j - 1];
            del_th_p = p_q[j] - p_q[j - 1];

            Tl[j] = Tl[j - 1] * exp(c1 * q[j - 1]);
            Tr[j] = Tr[j - 1] * exp(c1 * q[j - 1]);

            friction_l[j] = Tl[j] - Tl[j - 1];
            friction_r[j] = Tr[j] - Tr[j - 1];
        }

        // tau 계산
        tau_j[j] = R * (cos(del_th / 2) * (Tl[j] * sin(th_b) - Tr[j] * sin(th_b)) - sin(del_th / 2) * (Tl[j] * cos(th_b) + Tr[j] * cos(th_b)));
        th_seg_dot[j] = dq[j] - p_dq[j];

        // B 계산
        B_seg[j] = std::abs(tau_j[j] / th_seg_dot[j]);
    }

    double B = std::accumulate(B_seg.begin(), B_seg.end(), 0.0);
    double res_friction = std::abs(std::accumulate(friction_l.begin(), friction_l.end(), 0.0) + std::accumulate(friction_r.begin(), friction_r.end(), 0.0));

    // NaN 처리
    if (std::isnan(res_friction)) {
        res_friction = 0.0;
    }
    if (std::isnan(B)) {
        B = 0.0;
    }

    return {B, res_friction};
}
