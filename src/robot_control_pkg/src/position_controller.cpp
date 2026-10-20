#include "position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PositionController::PositionController()
: x_desired_(Eigen::VectorXd::Zero(6)),
  x_actual_(Eigen::VectorXd::Zero(6)),
  x_err_(Eigen::VectorXd::Zero(6)),
  del_theta_pan_(0.0), del_theta_tilt_(0.0)
{
  initialize();
}

PositionController::~PositionController() = default;

void PositionController::initialize()
{
  pid_controller_pan_.set_PID_gains(
    position_control_params::KP, position_control_params::KI, position_control_params::KD);
  pid_controller_tilt_.set_PID_gains(
    position_control_params::KP, position_control_params::KI, position_control_params::KD);
  surgical_tool_.init_surgical_tool(NUM_OF_JOINT_PAIRS, SEGMENT_ARC,
    SEGMENT_DIAMETER, WIRE_DISTANCE, SHIFT, SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER);
  initialized_ = false;
}

void PositionController::set_limits(double max_speed_deg_s, double derivative_filter_sec)
{
  if (!std::isfinite(max_speed_deg_s) || max_speed_deg_s <= 0.0 ||
      !std::isfinite(derivative_filter_sec) || derivative_filter_sec < 0.0)
  {
    throw std::invalid_argument("Invalid position-controller limits.");
  }
  max_speed_rad_s_ = max_speed_deg_s * std::acos(-1.0) / 180.0;
  derivative_filter_sec_ = derivative_filter_sec;
}

void PositionController::reset(
  const Eigen::VectorXd& x_actual, double pan_rad, double tilt_rad)
{
  const double limit = MAX_BENDING_DEGREE * surgical_tool_.torad();
  if (x_actual.size() != 6 || !x_actual.allFinite() ||
      !std::isfinite(pan_rad) || !std::isfinite(tilt_rad) ||
      std::abs(pan_rad) > limit || std::abs(tilt_rad) > limit)
  {
    throw std::invalid_argument("Invalid position/angle for controller entry.");
  }
  reference_pan_ = pan_rad;
  reference_tilt_ = tilt_rad;
  surgical_tool_.get_IK_result(pan_rad * surgical_tool_.todeg(),
    tilt_rad * surgical_tool_.todeg(), 0.0);
  x_desired_ = x_actual_ = x_actual;
  x_err_.setZero();
  del_theta_pan_ = del_theta_tilt_ = dt_ = 0.0;
  pid_controller_pan_.reset(x_actual(2), x_actual(2));
  pid_controller_tilt_.reset(x_actual(1), x_actual(1));
  initialized_ = true;
}

std::vector<double> PositionController::update(
  const Eigen::VectorXd& x_desired, const Eigen::VectorXd& x_actual, const double& dt)
{
  if (!initialized_) {
    throw std::logic_error("Reset position controller from fresh feedback before update.");
  }
  if (x_desired.size() != 6 || x_actual.size() != 6 ||
      !x_desired.allFinite() || !x_actual.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0)
  {
    throw std::invalid_argument("Invalid position feedback or frame interval; reset required.");
  }
  x_desired_ = x_desired;
  x_actual_ = x_actual;
  x_err_ = x_desired - x_actual;
  dt_ = dt;

  if (dt > position_control_params::FEEDBACK_TIMEOUT_SEC) {
    // Resume after a camera gap without replacing the user's goal or entry
    // angles. Discard only the stale derivative history (position I is zero).
    pid_controller_pan_.reset(x_desired(2), x_actual(2));
    pid_controller_tilt_.reset(x_desired(1), x_actual(1));
  }

  // hrm_base: X passive; Y -> q1 tilt; Z -> q2 pan (near straight pose).
  const double correction_pan = pid_controller_pan_.compute_pd_output(
    x_desired(2), x_actual(2), dt, derivative_filter_sec_);
  const double correction_tilt = pid_controller_tilt_.compute_pd_output(
    x_desired(1), x_actual(1), dt, derivative_filter_sec_);
  if (!std::isfinite(correction_pan) || !std::isfinite(correction_tilt)) {
    throw std::invalid_argument("Non-finite PD correction; reset required.");
  }

  // No repeated addition of the full PD output to the preceding IK command.
  // Without limits this is equivalent to adding only u[k]-u[k-1] each frame.
  const double limit = MAX_BENDING_DEGREE * surgical_tool_.torad();
  // An arbitrarily long image gap must not buy an arbitrarily large step.
  const double max_step = max_speed_rad_s_ *
    std::min(dt, position_control_params::FEEDBACK_TIMEOUT_SEC);
  const double previous_pan = surgical_tool_.pAngle_;
  const double previous_tilt = surgical_tool_.tAngle_;
  const double target_pan = std::clamp(reference_pan_ + correction_pan, -limit, limit);
  const double target_tilt = std::clamp(reference_tilt_ + correction_tilt, -limit, limit);
  const double next_pan = std::clamp(target_pan, previous_pan - max_step, previous_pan + max_step);
  const double next_tilt = std::clamp(target_tilt, previous_tilt - max_step, previous_tilt + max_step);
  auto wire = surgical_tool_.get_IK_result(
    next_pan * surgical_tool_.todeg(), next_tilt * surgical_tool_.todeg(), 0.0);
  del_theta_pan_ = surgical_tool_.pAngle_ - previous_pan;
  del_theta_tilt_ = surgical_tool_.tAngle_ - previous_tilt;
  return wire;
}
