#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace hrm {

// Angles are aggregate IK bending commands in degrees, NOT per-joint angles.
struct SineConfig {
  // center, amplitude (nonnegative), phase in degrees; tilt first.
  std::array<double, 3> tilt{{0.0, 45.0, 0.0}};
  std::array<double, 3> pan{{0.0, 45.0, 90.0}};
  double period = 20.0;
  double speed = 15.0;

  void validate() const {
    if (!std::isfinite(period) || period < 1.0 ||
        !std::isfinite(speed) || speed <= 0.0 || speed > 15.0) {
      throw std::invalid_argument("Sine period must be >=1 s; max speed in (0,15] deg/s.");
    }
    for (const auto& axis : {tilt, pan}) {
      for (double value : axis) {
        if (!std::isfinite(value)) {throw std::invalid_argument("Sine values must be finite.");}
      }
      if (axis[1] < 0.0 || std::abs(axis[0]) + axis[1] > 45.0 ||
          axis[1] * 2.0 * std::acos(-1.0) / period > speed) {
        throw std::invalid_argument(
          "Sine amplitude >=0, |center|+amplitude <=45 deg, 2*pi*amplitude/period <= max speed.");
      }
    }
  }

  std::array<double, 2> sample(double elapsed) const {
    std::array<double, 2> result;
    const double pi = std::acos(-1.0);
    const auto axes = std::array<std::array<double, 3>, 2>{{tilt, pan}};
    for (unsigned i = 0; i < 2; ++i) {
      result[i] = axes[i][0] + axes[i][1] *
        std::sin(2.0 * pi * elapsed / period + axes[i][2] * pi / 180.0);
    }
    return result;
  }
};

class SineMotion {
public:
  void start(const SineConfig& config, double tilt, double pan) {
    config.validate();
    if (!std::isfinite(tilt) || !std::isfinite(pan) ||
        std::abs(tilt) > 45.0 || std::abs(pan) > 45.0) {
      throw std::invalid_argument("Initial IK command must be within +/-45 deg.");
    }
    config_ = config;
    value_ = {{tilt, pan}};
    elapsed_ = 0.0;
    approaching_ = true;
  }

  std::array<double, 2> advance(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
      throw std::invalid_argument("Sine timer stalled/invalid dt; start again explicitly.");
    }
    // Approach the first phase without a step. Wave time starts after arrival.
    if (!approaching_) {elapsed_ += dt;}
    const auto desired = config_.sample(elapsed_);
    bool reached = true;
    for (unsigned i = 0; i < 2; ++i) {
      const double error = desired[i] - value_[i];
      const double step = config_.speed * dt;
      value_[i] += std::max(-step, std::min(step, error));
      reached = reached && std::abs(error) <= step;
    }
    if (approaching_ && reached) {approaching_ = false;}
    return value_;
  }

private:
  SineConfig config_;
  std::array<double, 2> value_{{0.0, 0.0}};
  double elapsed_ = 0.0;
  bool approaching_ = true;
};
}  // namespace hrm
