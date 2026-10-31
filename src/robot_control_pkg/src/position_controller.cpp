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
  // IK takes degrees. Invert its own radian conversion to avoid accumulating
  // the legacy float torad()*todeg() round-trip error in the command state.
  surgical_tool_.get_IK_result(pan_rad / surgical_tool_.torad(),
    tilt_rad / surgical_tool_.torad(), 0.0);
  x_desired_ = x_actual_ = x_actual;
  x_err_.setZero();
  del_theta_pan_ = del_theta_tilt_ = dt_ = 0.0;
  pid_controller_pan_.reset(x_actual(2), x_actual(2));
  pid_controller_tilt_.reset(x_actual(1), x_actual(1));
  initialized_ = true;
}

void PositionController::reset_feedback_history(
  const Eigen::VectorXd& x_desired, const Eigen::VectorXd& x_actual)
{
  if (!initialized_) {
    throw std::logic_error("Reset position controller before resuming feedback.");
  }
  if (x_desired.size() != 6 || x_actual.size() != 6 ||
      !x_desired.allFinite() || !x_actual.allFinite())
  {
    throw std::invalid_argument("Invalid position feedback for history reset.");
  }
  x_desired_ = x_desired;
  x_actual_ = x_actual;
  x_err_ = x_desired - x_actual;
  del_theta_pan_ = del_theta_tilt_ = dt_ = 0.0;
  pid_controller_pan_.reset(x_desired(2), x_actual(2));
  pid_controller_tilt_.reset(x_desired(1), x_actual(1));
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
  for (const auto* pid : {&pid_controller_pan_, &pid_controller_tilt_}) {
    if (!std::isfinite(pid->kp_) || pid->kp_ < 0.0 || pid->ki_ != 0.0 ||
        !std::isfinite(pid->kd_) || pid->kd_ < 0.0)
    {
      throw std::invalid_argument("Position PD requires finite nonnegative P/D and I=0.");
    }
  }
  x_desired_ = x_desired;
  x_actual_ = x_actual;
  x_err_ = x_desired - x_actual;
  dt_ = dt;

  if (dt > MAX_CONTROL_FRAME_DT_SEC) {
    // The first fresh frame restores feedback history only. Missing frames
    // provide no motion allowance, even when the freshness timeout is longer.
    reset_feedback_history(x_desired, x_actual);
    dt_ = dt;
    return surgical_tool_.get_IK_result(
      surgical_tool_.pAngle_ / surgical_tool_.torad(),
      surgical_tool_.tAngle_ / surgical_tool_.torad(), 0.0);
  }

  // hrm_base: X passive; Y -> q1 tilt; Z -> q2 pan (near straight pose).
  const double velocity_pan = pid_controller_pan_.compute_output(
    x_desired(2), x_actual(2), dt, derivative_filter_sec_);
  const double velocity_tilt = pid_controller_tilt_.compute_output(
    x_desired(1), x_actual(1), dt, derivative_filter_sec_);
  if (!std::isfinite(velocity_pan) || !std::isfinite(velocity_tilt)) {
    throw std::invalid_argument("Non-finite PD angular velocity; reset required.");
  }

  // Restore rate-PD: output is angular velocity [rad/s]. Limit the velocity,
  // then integrate it exactly once over the source-image interval. PID's own
  // dt computes the error derivative/integral; it does not integrate this rate.
  // A remaining error keeps correcting the command; zero e and de/dt hold it.
  const double increment_pan = std::clamp(
    velocity_pan, -max_speed_rad_s_, max_speed_rad_s_) * dt;
  const double increment_tilt = std::clamp(
    velocity_tilt, -max_speed_rad_s_, max_speed_rad_s_) * dt;
  const double limit = MAX_BENDING_DEGREE * surgical_tool_.torad();
  const double previous_pan = surgical_tool_.pAngle_;
  const double previous_tilt = surgical_tool_.tAngle_;
  const double next_pan = std::clamp(previous_pan + increment_pan, -limit, limit);
  const double next_tilt = std::clamp(previous_tilt + increment_tilt, -limit, limit);
  auto wire = surgical_tool_.get_IK_result(
    next_pan / surgical_tool_.torad(), next_tilt / surgical_tool_.torad(), 0.0);
  del_theta_pan_ = surgical_tool_.pAngle_ - previous_pan;
  del_theta_tilt_ = surgical_tool_.tAngle_ - previous_tilt;
  return wire;
}
