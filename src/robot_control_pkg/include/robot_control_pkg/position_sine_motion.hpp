#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace hrm {

// Absolute hrm_base references. These are positions in metres, not IK angles.
struct PositionSineConfig {
  // center [m], nonnegative amplitude [m], phase [deg].
  std::array<double, 3> y{{0.0, 0.01, 0.0}};
  std::array<double, 3> z{{0.0, 0.01, 90.0}};
  double period = 20.0;
  double speed = 0.010;  // Per-axis reference peak/ramp limit [m/s], not tracking error.

  void validate(double workspace_radius_m) const {
    if (!std::isfinite(workspace_radius_m) || workspace_radius_m <= 0.0 ||
        !std::isfinite(period) || period < 1.0 ||
        !std::isfinite(speed) || speed <= 0.0)
    {
      throw std::invalid_argument("Position sine: period >=1 s and speed/radius >0 must be finite.");
    }
    for (const auto& axis : {y, z}) {
      for (double value : axis) {
        if (!std::isfinite(value)) {
          throw std::invalid_argument("Position sine values must be finite.");
        }
      }
      if (axis[1] < 0.0 || axis[1] * 2.0 * std::acos(-1.0) / period > speed) {
        throw std::invalid_argument(
          "Position sine: amplitude >=0 and 2*pi*amplitude/period <= max_speed_m_s.");
      }
    }
    // Necessary envelope check only: not a proof of coupled Y/Z reachability.
    if (std::hypot(std::abs(y[0]) + y[1], std::abs(z[0]) + z[1]) > workspace_radius_m) {
      throw std::invalid_argument("Position sine envelope exceeds HRM total length.");
    }
  }

  std::array<double, 2> sample(double elapsed) const {
    std::array<double, 2> result;
    const double pi = std::acos(-1.0);
    const auto axes = std::array<std::array<double, 3>, 2>{{y, z}};
    for (unsigned i = 0; i < 2; ++i) {
      result[i] = axes[i][0] + axes[i][1] *
        std::sin(2.0 * pi * (elapsed / period) +
          std::remainder(axes[i][2], 360.0) * pi / 180.0);
    }
    return result;
  }
};

class PositionSineMotion {
public:
  void start(const PositionSineConfig& config, double y, double z, double workspace_radius_m) {
    config.validate(workspace_radius_m);
    if (!std::isfinite(y) || !std::isfinite(z) || std::hypot(y, z) > workspace_radius_m) {
      throw std::invalid_argument("Initial nominal Y/Z must be finite and within HRM total length.");
    }
    config_ = config;
    value_ = {{y, z}};
    elapsed_ = 0.0;
    approaching_ = true;
  }

  std::array<double, 2> advance(double dt, double max_frame_interval = 0.25) {
    // A delayed/invalid frame never advances phase or catches up a missing gap.
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(max_frame_interval) || max_frame_interval <= 0.0 || dt > max_frame_interval)
    {
      return value_;
    }
    if (!approaching_) {elapsed_ = std::fmod(elapsed_ + dt, config_.period);}
    const auto desired = config_.sample(elapsed_);
    bool reached = true;
    for (unsigned i = 0; i < 2; ++i) {
      const double error = desired[i] - value_[i];
      const double step = config_.speed * dt;
      value_[i] += std::clamp(error, -step, step);
      reached = reached && std::abs(error) <= step;
    }
    if (approaching_ && reached) {approaching_ = false;}
    return value_;
  }

private:
  PositionSineConfig config_;
  std::array<double, 2> value_{{0.0, 0.0}};
  double elapsed_ = 0.0;
  bool approaching_ = true;
};
}  // namespace hrm
