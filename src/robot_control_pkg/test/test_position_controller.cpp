#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "position_controller.hpp"

namespace {
Eigen::VectorXd origin() {return Eigen::VectorXd::Zero(6);}
constexpr double dt = 1.0 / 30.0;
constexpr double angle_tolerance = 1e-8;

// Regression gains are deliberately independent of user-tuned runtime defaults.
class TestPositionController : public PositionController {
public:
  TestPositionController()
  {
    pid_controller_pan_.set_PID_gains(5.0, 0.0, 0.05);
    pid_controller_tilt_.set_PID_gains(5.0, 0.0, 0.05);
    set_limits(position_control_params::MAX_ANGULAR_SPEED_DEG_S, 0.05);
  }
};

// Software-only ideal plant: cable aggregate angles are distributed uniformly
// across their nine active joints, then the real SurgicalTool FK is evaluated.
// Perfect instantaneous actuation is assumed, without friction, lag or noise;
// convergence here is a regression check, NOT hardware validation.
Eigen::VectorXd ideal_pose(const SurgicalTool& tool, double pan, double tilt)
{
  std::vector<double> pan_relative(NUM_OF_BENDING_JOINTS, 0.0);
  std::vector<double> tilt_relative(NUM_OF_BENDING_JOINTS, 0.0);
  for (std::size_t i = 0; i < pan_relative.size(); ++i) {
    if (i % 2 == 0) {tilt_relative[i] = tilt / NUM_OF_TILT_JOINTS;}
    else {pan_relative[i] = pan / NUM_OF_PAN_JOINTS;}
  }
  auto pose = origin();
  pose.head<3>() = tool.computeEndEffectorPosition(
    tool.computeBaseToJointsTransformationMatrices(pan_relative, tilt_relative));
  return pose;
}

Eigen::VectorXd ideal_pose(const PositionController& controller)
{
  return ideal_pose(controller.surgical_tool_, controller.surgical_tool_.pAngle_,
    controller.surgical_tool_.tAngle_);
}
}

TEST(PositionPD, RequiresExplicitModeEntry)
{
  TestPositionController c;
  EXPECT_THROW(c.update(origin(), origin(), dt), std::logic_error);
}

TEST(PositionPD, NonzeroEntryHoldsBothAnglesWithoutNumericalDrift)
{
  TestPositionController c;
  auto actual = origin();
  actual(1) = 0.013;
  actual(2) = -0.008;
  c.reset(actual, 0.2, -0.3);
  for (int i = 0; i < 1000; ++i) {c.update(actual, actual, dt);}
  EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.2, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, -0.3, angle_tolerance);
  EXPECT_NEAR(c.del_theta_pan_, 0.0, 1e-10);
  EXPECT_NEAR(c.del_theta_tilt_, 0.0, 1e-10);
}

TEST(PositionPD, PersistentPositionErrorKeepsMovingCommandWithAxesYZ)
{
  TestPositionController c;
  c.reset(origin(), 0.0, 0.0);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 0.0;
  auto goal = origin();
  goal(0) = 123.0;  // Passive axis must not affect cable angles.
  goal(1) = 0.001;
  goal(2) = -0.002;
  for (int i = 0; i < 60; ++i) {c.update(goal, origin(), dt);}
  // P [rad/(m*s)] times error [m] is an angular rate, integrated over 2s.
  EXPECT_NEAR(c.surgical_tool_.tAngle_, 5.0 * 0.001 * 60 * dt, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -5.0 * 0.002 * 60 * dt, angle_tolerance);
  EXPECT_GT(c.del_theta_tilt_, 0.0);
  EXPECT_LT(c.del_theta_pan_, 0.0);
  EXPECT_DOUBLE_EQ(c.pid_controller_pan_.get_integral(), 0.0);
  EXPECT_DOUBLE_EQ(c.pid_controller_tilt_.get_integral(), 0.0);
}

TEST(PositionPD, ReachedTargetHoldsLatestCommandInsteadOfEntryAngle)
{
  TestPositionController c;
  c.reset(origin(), 0.2, -0.3);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 0.0;
  auto goal = origin();
  goal(1) = 0.001;
  goal(2) = -0.002;
  for (int i = 0; i < 60; ++i) {c.update(goal, origin(), dt);}
  const double pan = c.surgical_tool_.pAngle_;
  const double tilt = c.surgical_tool_.tAngle_;
  for (int i = 0; i < 1000; ++i) {c.update(goal, goal, dt);}
  EXPECT_NEAR(c.surgical_tool_.pAngle_, pan, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, tilt, angle_tolerance);
  EXPECT_NEAR(pan, 0.18, angle_tolerance);
  EXPECT_NEAR(tilt, -0.29, angle_tolerance);
}

TEST(PositionPD, ConventionalPIDUsesErrorIntegralAndErrorDerivative)
{
  PIDController pid;
  pid.set_PID_gains(2.0, 3.0, 4.0);
  pid.reset(0.0, 0.0);
  // e=.5, integral=.05, derivative=5 (the target itself changed).
  EXPECT_NEAR(pid.compute_output(0.5, 0.0, 0.1), 1.0 + 0.15 + 20.0, 1e-12);
  EXPECT_NEAR(pid.get_integral(), 0.05, 1e-12);
  // e=.25, integral=.075, derivative=-2.5.
  EXPECT_NEAR(pid.compute_output(0.5, 0.25, 0.1), 0.5 + 0.225 - 10.0, 1e-12);
}

TEST(PositionPD, OptionalFilterActsOnErrorDerivativeIncludingSetpoint)
{
  PIDController pid;
  pid.set_PID_gains(5.0, 0.0, 0.05);
  pid.reset(0.0, 0.0);
  const double expected = 5.0 * 0.001 + 0.05 * 0.001 / (0.05 + dt);
  EXPECT_NEAR(pid.compute_output(0.001, 0.0, dt, 0.05), expected, 1e-12);
  pid.reset(0.0, 0.0);
  EXPECT_NEAR(pid.compute_pd_output(0.001, 0.0, dt, 0.05), expected, 1e-12);
  EXPECT_DOUBLE_EQ(pid.get_integral(), 0.0);
}

TEST(PositionPD, ErrorDerivativeDampsMeasuredMotionAndResets)
{
  PIDController pid;
  pid.set_PID_gains(0.0, 0.0, 0.05);
  pid.reset(0.0, 0.0);
  EXPECT_NEAR(pid.compute_output(0.0, 0.001, 0.1, 0.05),
    -0.05 * 0.001 / 0.15, 1e-12);
  pid.reset(0.001, 0.001);
  EXPECT_DOUBLE_EQ(pid.compute_output(0.001, 0.001, 0.1, 0.05), 0.0);
}

TEST(PositionPD, ZeroIGainStoresNoHiddenIntegralAndResetClearsLegacyState)
{
  PIDController pid;
  pid.set_PID_gains(5.0, 1.0, 0.05);
  pid.compute_output(1.0, 0.0, dt);
  EXPECT_GT(pid.get_integral(), 0.0);
  EXPECT_THROW(pid.compute_pd_output(1.0, 0.0, dt, 0.05), std::invalid_argument);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.get_integral(), 0.0);
  EXPECT_DOUBLE_EQ(pid.get_previous_error(), 0.0);
  pid.set_PID_gains(5.0, 0.0, 0.05);
  for (int i = 0; i < 1000; ++i) {pid.compute_output(1.0, 0.0, dt, 0.05);}
  EXPECT_DOUBLE_EQ(pid.get_integral(), 0.0);
}

TEST(PositionPD, RateLimitUsesActualFrameIntervalAndBoundsSetpointKick)
{
  constexpr double speed_deg_s = 7.0;
  for (double interval : {0.02, 0.05, 0.1, PositionController::MAX_CONTROL_FRAME_DT_SEC}) {
    TestPositionController c;
    c.set_limits(speed_deg_s, 0.05);
    c.reset(origin(), 0.0, 0.0);
    c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 1000.0;
    auto goal = origin();
    goal(1) = goal(2) = 1.0;
    c.update(goal, origin(), interval);
    const double step = speed_deg_s * std::acos(-1.0) / 180.0 * interval;
    EXPECT_NEAR(c.del_theta_tilt_, step, angle_tolerance);
    EXPECT_NEAR(c.del_theta_pan_, step, angle_tolerance);
  }
}

TEST(PositionPD, AbsoluteLimitHasNoWindupAndOppositeErrorReversesImmediately)
{
  TestPositionController c;
  c.reset(origin(), 0.0, 0.0);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 0.0;
  auto goal = origin();
  goal(1) = 100.0;
  goal(2) = -100.0;
  for (int i = 0; i < 2000; ++i) {c.update(goal, origin(), dt);}
  const double limit = MAX_BENDING_DEGREE * c.surgical_tool_.torad();
  EXPECT_NEAR(c.surgical_tool_.tAngle_, limit, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -limit, angle_tolerance);
  EXPECT_DOUBLE_EQ(c.pid_controller_pan_.get_integral(), 0.0);
  EXPECT_DOUBLE_EQ(c.pid_controller_tilt_.get_integral(), 0.0);
  // Zero error stops further increments; it must NOT reset angle to zero.
  c.update(origin(), origin(), dt);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, limit, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -limit, angle_tolerance);
  c.update(-goal, origin(), dt);
  EXPECT_LT(c.del_theta_tilt_, 0.0);
  EXPECT_GT(c.del_theta_pan_, 0.0);
  EXPECT_LT(c.surgical_tool_.tAngle_, limit);
  EXPECT_GT(c.surgical_tool_.pAngle_, -limit);
}

TEST(PositionPD, RejectsBadFramesAndNonfiniteOutput)
{
  TestPositionController c;
  c.reset(origin(), 0.0, 0.0);
  for (double interval : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(c.update(origin(), origin(), interval), std::invalid_argument);
  }
  EXPECT_THROW(c.update(Eigen::VectorXd::Zero(2), origin(), dt), std::invalid_argument);
  auto bad = origin();
  bad(1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.update(bad, origin(), dt), std::invalid_argument);
  bad(1) = std::numeric_limits<double>::max();
  EXPECT_THROW(c.update(bad, origin(), dt), std::invalid_argument);
}

TEST(PositionPD, ModeEntryDiscardsOldGoalAndDerivativeState)
{
  TestPositionController c;
  c.reset(origin(), 0.0, 0.0);
  auto goal = origin();
  goal(1) = 0.001;
  c.update(goal, origin(), dt);
  auto actual = origin();
  actual(1) = -0.01;
  actual(2) = 0.02;
  c.reset(actual, -0.1, 0.2);
  c.update(actual, actual, dt);
  EXPECT_TRUE(c.x_desired_.isApprox(actual));
  EXPECT_TRUE(c.x_err_.isZero());
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -0.1, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, 0.2, angle_tolerance);
}

TEST(PositionPD, CameraGapKeepsLatestCommandAndResetsOnlyDerivativeHistory)
{
  TestPositionController c;
  c.reset(origin(), 0.2, 0.1);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 1000.0;
  auto goal = origin();
  goal(1) = 0.004;
  goal(2) = 0.006;
  for (int i = 0; i < 30; ++i) {c.update(goal, origin(), dt);}
  const double previous_pan = c.surgical_tool_.pAngle_;
  const double previous_tilt = c.surgical_tool_.tAngle_;
  auto actual = origin();
  actual(1) = 0.001;
  actual(2) = 0.002;
  ASSERT_NO_THROW(c.update(goal, actual, 1.0));
  EXPECT_TRUE(c.x_desired_.isApprox(goal));
  EXPECT_TRUE(c.x_actual_.isApprox(actual));
  EXPECT_DOUBLE_EQ(c.dt_, 1.0);
  EXPECT_DOUBLE_EQ(c.del_theta_pan_, 0.0);
  EXPECT_DOUBLE_EQ(c.del_theta_tilt_, 0.0);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, previous_tilt, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, previous_pan, angle_tolerance);
  // The next normal frame uses only P for the unchanged error, despite the
  // very large D gain and measured-position change during the missing gap.
  c.update(goal, actual, dt);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, previous_tilt + 5.0 * 0.003 * dt,
    angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, previous_pan + 5.0 * 0.004 * dt,
    angle_tolerance);
}

TEST(PositionPD, LongCameraGapDoesNotAccumulateMotionAllowance)
{
  for (double gap : {PositionController::MAX_CONTROL_FRAME_DT_SEC + 0.001, 1.0, 100.0}) {
    TestPositionController c;
    c.reset(origin(), 0.2, -0.1);
    const auto held_wire = c.update(origin(), origin(), dt);
    auto goal = origin();
    goal(1) = 1.0;
    const auto resumed_wire = c.update(goal, origin(), gap);
    EXPECT_DOUBLE_EQ(c.del_theta_pan_, 0.0);
    EXPECT_DOUBLE_EQ(c.del_theta_tilt_, 0.0);
    EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.2, angle_tolerance);
    EXPECT_NEAR(c.surgical_tool_.tAngle_, -0.1, angle_tolerance);
    ASSERT_EQ(resumed_wire.size(), held_wire.size());
    for (std::size_t i = 0; i < held_wire.size(); ++i) {
      EXPECT_NEAR(resumed_wire[i], held_wire[i], 1e-10);
    }
    c.update(goal, origin(), dt);
    EXPECT_NEAR(c.del_theta_tilt_,
      position_control_params::MAX_ANGULAR_SPEED_DEG_S * std::acos(-1.0) / 180.0 * dt,
      angle_tolerance);
  }
}

TEST(PositionPD, ExplicitFeedbackResumeHoldsCommandAndClearsShortPauseDerivative)
{
  TestPositionController c;
  EXPECT_THROW(c.reset_feedback_history(origin(), origin()), std::logic_error);
  c.reset(origin(), 0.2, -0.1);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 1000.0;
  auto goal = origin();
  goal(1) = 0.004;
  goal(2) = 0.006;
  c.update(goal, origin(), dt);
  const double held_pan = c.surgical_tool_.pAngle_;
  const double held_tilt = c.surgical_tool_.tAngle_;
  auto actual = origin();
  actual(1) = 0.001;
  actual(2) = 0.002;
  c.reset_feedback_history(goal, actual);
  EXPECT_DOUBLE_EQ(c.del_theta_pan_, 0.0);
  EXPECT_DOUBLE_EQ(c.del_theta_tilt_, 0.0);
  EXPECT_DOUBLE_EQ(c.dt_, 0.0);
  EXPECT_DOUBLE_EQ(c.surgical_tool_.pAngle_, held_pan);
  EXPECT_DOUBLE_EQ(c.surgical_tool_.tAngle_, held_tilt);
  EXPECT_TRUE(c.x_desired_.isApprox(goal));
  EXPECT_TRUE(c.x_actual_.isApprox(actual));
  EXPECT_TRUE(c.x_err_.isApprox(goal - actual));
  c.update(goal, actual, dt);
  EXPECT_NEAR(c.del_theta_tilt_, 5.0 * 0.003 * dt, angle_tolerance);
  EXPECT_NEAR(c.del_theta_pan_, 5.0 * 0.004 * dt, angle_tolerance);
  EXPECT_THROW(c.reset_feedback_history(Eigen::VectorXd::Zero(2), actual),
    std::invalid_argument);
  actual(1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.reset_feedback_history(goal, actual), std::invalid_argument);
}

TEST(PositionPD, UnsaturatedAngularRateIntegratesActualFrameInterval)
{
  for (double interval : {0.02, 1.0 / 30.0, 0.1}) {
    TestPositionController c;
    c.reset(origin(), 0.0, 0.0);
    c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 0.0;
    auto goal = origin();
    goal(1) = 0.001;
    goal(2) = -0.002;
    for (int i = 0; i < 20; ++i) {
      c.update(goal, origin(), interval);
      EXPECT_NEAR(c.del_theta_tilt_, 0.005 * interval, angle_tolerance);
      EXPECT_NEAR(c.del_theta_pan_, -0.010 * interval, angle_tolerance);
    }
    EXPECT_NEAR(c.surgical_tool_.tAngle_, 0.100 * interval, angle_tolerance);
    EXPECT_NEAR(c.surgical_tool_.pAngle_, -0.200 * interval, angle_tolerance);
  }
}

TEST(PositionPD, ConstantErrorOverSameElapsedTimeIsIndependentOfFrameRate)
{
  constexpr double duration = 2.0;
  for (int frame_rate : {15, 30, 60}) {
    SCOPED_TRACE(::testing::Message() << "Hz=" << frame_rate);
    TestPositionController c;
    c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 0.0;
    c.reset(origin(), 0.2, -0.3);
    auto goal = origin();
    goal(1) = 0.001;
    goal(2) = -0.002;
    const double interval = 1.0 / frame_rate;
    for (int i = 0; i < static_cast<int>(duration * frame_rate); ++i) {
      c.update(goal, origin(), interval);
    }
    EXPECT_NEAR(c.surgical_tool_.tAngle_, -0.3 + 5.0 * 0.001 * duration,
      angle_tolerance);
    EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.2 - 5.0 * 0.002 * duration,
      angle_tolerance);
  }
}

TEST(PositionPD, TunedGainProducesAngularRateRatherThanPerFrameIncrement)
{
  TestPositionController c;
  c.pid_controller_pan_.set_PID_gains(20.55, 0.0, 0.0);
  c.pid_controller_tilt_.set_PID_gains(20.55, 0.0, 0.0);
  c.reset(origin(), 0.0, 0.0);
  auto goal = origin();
  goal(1) = 0.001;
  goal(2) = -0.001;
  c.update(goal, origin(), dt);
  // 20.55 * .001 m = .02055 rad/s, or .000685 rad per 30Hz frame.
  EXPECT_NEAR(c.del_theta_tilt_, 0.000685, angle_tolerance);
  EXPECT_NEAR(c.del_theta_pan_, -0.000685, angle_tolerance);
}

TEST(PositionPD, UnfilteredDerivativeDtCancelsInIntegratedCommandStep)
{
  for (double interval : {1.0 / 15.0, 1.0 / 30.0, 1.0 / 60.0}) {
    TestPositionController c;
    c.set_limits(40.0, 0.0);
    c.pid_controller_pan_.set_PID_gains(2.0, 0.0, 0.001);
    c.pid_controller_tilt_.set_PID_gains(2.0, 0.0, 0.001);
    c.reset(origin(), 0.0, 0.0);
    auto goal = origin();
    goal(1) = 0.001;
    goal(2) = -0.002;
    c.update(goal, origin(), interval);
    // u = Kp*e + Kd*(e-e_prev)/dt, dq = u*dt. With no rate clipping,
    // the first D contribution is Kd*e, not inversely proportional to dt.
    EXPECT_NEAR(c.del_theta_tilt_, 2.0 * 0.001 * interval + 0.001 * 0.001,
      angle_tolerance);
    EXPECT_NEAR(c.del_theta_pan_, -2.0 * 0.002 * interval - 0.001 * 0.002,
      angle_tolerance);
    c.update(goal, origin(), interval);
    EXPECT_NEAR(c.del_theta_tilt_, 2.0 * 0.001 * interval, angle_tolerance);
    EXPECT_NEAR(c.del_theta_pan_, -2.0 * 0.002 * interval, angle_tolerance);
  }
}

TEST(PositionPD, IdealNonlinearFKPlantConvergesToSignedTenMillimetreSteps)
{
  for (double entry_scale : {0.0, 1.0}) {
    for (int axis : {1, 2}) {
      for (double sign : {-1.0, 1.0}) {
        SCOPED_TRACE(::testing::Message() << "axis=" << axis << " sign=" << sign
          << " entry_scale=" << entry_scale);
        TestPositionController c;
        const double pan = 0.10 * entry_scale, tilt = -0.15 * entry_scale;
        const auto entry = ideal_pose(c.surgical_tool_, pan, tilt);
        c.reset(entry, pan, tilt);
        auto goal = entry;
        goal(axis) += sign * 0.010;
        for (int i = 0; i < 1800; ++i) {c.update(goal, ideal_pose(c), dt);}
        const auto reached = ideal_pose(c);
        EXPECT_NEAR(reached(1), goal(1), 1e-6);
        EXPECT_NEAR(reached(2), goal(2), 1e-6);
        EXPECT_DOUBLE_EQ(c.pid_controller_pan_.get_integral(), 0.0);
        EXPECT_DOUBLE_EQ(c.pid_controller_tilt_.get_integral(), 0.0);
      }
    }
  }
}

TEST(PositionPD, IdealNonlinearFKStepConvergenceIsNotTiedToThirtyHertz)
{
  // Compare equal physical durations, not an equal number of frames.
  for (double interval : {1.0 / 15.0, 1.0 / 30.0, 1.0 / 60.0}) {
    TestPositionController c;
    const auto entry = ideal_pose(c.surgical_tool_, 0.0, 0.0);
    c.reset(entry, 0.0, 0.0);
    auto goal = entry;
    goal(1) = 0.010;
    goal(2) = -0.010;
    for (int i = 0; i < std::lround(60.0 / interval); ++i) {
      c.update(goal, ideal_pose(c), interval);
    }
    const auto reached = ideal_pose(c);
    EXPECT_NEAR(reached(1), goal(1), 1e-6);
    EXPECT_NEAR(reached(2), goal(2), 1e-6);
  }
}

TEST(PositionPD, IdealPlantRejectsConstantPositionDisturbanceWithoutI)
{
  TestPositionController c;
  const auto goal = ideal_pose(c.surgical_tool_, 0.0, 0.0);
  c.reset(goal, 0.0, 0.0);
  auto disturbed_pose = [&]() {
    auto measured = ideal_pose(c);
    measured(1) += 0.0025;
    measured(2) -= 0.0015;
    return measured;
  };
  for (int i = 0; i < 1800; ++i) {c.update(goal, disturbed_pose(), dt);}
  const auto reached = disturbed_pose();
  EXPECT_NEAR(reached(1), goal(1), 1e-6);
  EXPECT_NEAR(reached(2), goal(2), 1e-6);
  EXPECT_LT(c.surgical_tool_.tAngle_, 0.0);
  EXPECT_GT(c.surgical_tool_.pAngle_, 0.0);
}

TEST(PositionPD, IdealPlantSlowSinusoidSmokeHasBoundedLagAndCommands)
{
  TestPositionController c;
  const auto entry = ideal_pose(c.surgical_tool_, 0.0, 0.0);
  c.reset(entry, 0.0, 0.0);
  const double pi = std::acos(-1.0);
  double squared_error = 0.0, maximum_error = 0.0, maximum_travel = 0.0;
  int samples = 0;
  for (int i = 0; i < 7200; ++i) {
    const double time = i * dt;
    auto goal = entry;
    goal(1) += 0.002 * std::sin(2.0 * pi * time / 120.0);
    goal(2) += 0.0015 * std::sin(2.0 * pi * time / 120.0);
    c.update(goal, ideal_pose(c), dt);
    const auto reached = ideal_pose(c);
    maximum_travel = std::max(maximum_travel, std::abs(reached(1) - entry(1)));
    const double max_step =
      position_control_params::MAX_ANGULAR_SPEED_DEG_S * pi / 180.0 * dt;
    EXPECT_LE(std::abs(c.del_theta_pan_), max_step + angle_tolerance);
    EXPECT_LE(std::abs(c.del_theta_tilt_), max_step + angle_tolerance);
    if (time >= 120.0) {
      for (int axis : {1, 2}) {
        const double error = goal(axis) - reached(axis);
        squared_error += error * error;
        maximum_error = std::max(maximum_error, std::abs(error));
        ++samples;
      }
    }
  }
  ASSERT_GT(samples, 0);
  EXPECT_GT(maximum_travel, 0.001);
  // Deliberately slow synthetic trajectory; finite P still produces lag.
  // These are not experimentally measured tracking-accuracy bounds.
  EXPECT_LT(std::sqrt(squared_error / samples), 0.0006);
  EXPECT_LT(maximum_error, 0.0011);
}
