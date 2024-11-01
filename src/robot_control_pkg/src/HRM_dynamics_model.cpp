#include "HRM_dynamics_model.hpp"

// 생성자: 기본 시스템 파라미터 설정
HRMDynamicsModel::HRMDynamicsModel() {}

HRMDynamicsModel::HRMDynamicsModel(double inertia, double damping_coef, double stiffness)
    : inertia_(inertia), damping_coef_(damping_coef), stiffness_(stiffness) {}
HRMDynamicsModel::~HRMDynamicsModel() {}

// 시스템 파라미터를 설정하는 함수
void HRMDynamicsModel::set_parameters(double inertia, double damping_coef, double stiffness) {
    inertia_ = inertia;
    damping_coef_ = damping_coef;
    stiffness_ = stiffness;
}

// 개별적으로 관성 업데이트
void HRMDynamicsModel::update_inertia(double inertia) {
    inertia_ = inertia;
}

// 개별적으로 감쇠 계수 업데이트
void HRMDynamicsModel::update_damping_coefficient(double damping_coef) {
    damping_coef_ = damping_coef;
}

// 개별적으로 스프링 상수 업데이트
void HRMDynamicsModel::update_stiffness(double stiffness) {
    stiffness_ = stiffness;
}


// 동역학 방정식: 토크 계산 (I * acceleration + B * velocity + K * position)
double HRMDynamicsModel::compute_torque(double position, double velocity) {
    // I * acceleration (가속도 계산을 위해 속도 기반)
    double acceleration = velocity / inertia_;  // 단순 모델로 가정

    // 토크 계산: I⋅θ¨ + B⋅θ˙ + K⋅θ + τ_ext + τ_fric = τ_input
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

