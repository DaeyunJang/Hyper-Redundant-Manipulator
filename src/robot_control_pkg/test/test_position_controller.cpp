#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "position_controller.hpp"

namespace {
Eigen::VectorXd origin() {return Eigen::VectorXd::Zero(6);}
constexpr double dt = 1.0 / 30.0;
// Legacy SurgicalTool::torad()/todeg() return float, not double.
constexpr double angle_tolerance = 1e-8;
}

TEST(PositionPD, RequiresExplicitModeEntry)
{
  PositionController c;
  EXPECT_THROW(c.update(origin(), origin(), dt), std::logic_error);
}

TEST(PositionPD, NonzeroEntryHoldsBothAngles)
{
  PositionController c;
  auto actual = origin();
  actual(1) = 0.013;
  actual(2) = -0.008;
  c.reset(actual, 0.2, -0.3);
  for (int i = 0; i < 100; ++i) {c.update(actual, actual, dt);}
  EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.2, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, -0.3, angle_tolerance);
  EXPECT_NEAR(c.del_theta_pan_, 0.0, 1e-10);
  EXPECT_NEAR(c.del_theta_tilt_, 0.0, 1e-10);
}

TEST(PositionPD, SameErrorDoesNotAccumulateAndAxesRemainYZ)
{
  PositionController c;
  c.reset(origin(), 0.0, 0.0);
  auto goal = origin();
  goal(0) = 123.0;  // Passive axis must not affect cable angles.
  goal(1) = 0.001;
  goal(2) = -0.002;
  for (int i = 0; i < 1000; ++i) {c.update(goal, origin(), dt);}
  EXPECT_NEAR(c.surgical_tool_.tAngle_, 0.005, 1e-10);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -0.010, 1e-10);
  EXPECT_DOUBLE_EQ(c.pid_controller_pan_.get_integral(), 0.0);
  EXPECT_DOUBLE_EQ(c.pid_controller_tilt_.get_integral(), 0.0);
}

TEST(PositionPD, SetpointStepHasNoDerivativeKick)
{
  PIDController pid;
  pid.set_PID_gains(5.0, 0.0, 100.0);
  pid.reset(0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute_pd_output(0.001, 0.0, dt, 0.05), 0.005);
}

TEST(PositionPD, DerivativeDampsMeasuredMotionAndResets)
{
  PIDController pid;
  pid.set_PID_gains(0.0, 0.0, 0.05);
  pid.reset(0.0, 0.0);
  EXPECT_NEAR(pid.compute_pd_output(0.0, 0.001, 0.1, 0.05), -0.05 * 0.001 / 0.15, 1e-12);
  pid.reset(0.001, 0.001);
  EXPECT_DOUBLE_EQ(pid.compute_pd_output(0.001, 0.001, 0.1, 0.05), 0.0);
}

TEST(PositionPD, IntegralIsDisabledAndLegacyStateClears)
{
  PIDController pid;
  pid.set_PID_gains(5.0, 1.0, 0.05);
  pid.compute_output(1.0, 0.0, dt);
  EXPECT_GT(pid.get_integral(), 0.0);
  EXPECT_THROW(pid.compute_pd_output(1.0, 0.0, dt, 0.05), std::invalid_argument);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.get_integral(), 0.0);
  EXPECT_DOUBLE_EQ(pid.get_previous_error(), 0.0);
}

TEST(PositionPD, SlewLimitUsesActualFrameInterval)
{
  for (double interval : {0.02, 0.05, 0.1}) {
    PositionController c;
    c.reset(origin(), 0.0, 0.0);
    auto goal = origin();
    goal(1) = goal(2) = 1.0;
    c.update(goal, origin(), interval);
    const double step = 5.0 * std::acos(-1.0) / 180.0 * interval;
    EXPECT_NEAR(c.del_theta_tilt_, step, 1e-10);
    EXPECT_NEAR(c.del_theta_pan_, step, 1e-10);
  }
}

TEST(PositionPD, AbsoluteLimitAndNoWindupAfterSaturation)
{
  PositionController c;
  c.reset(origin(), 0.0, 0.0);
  auto goal = origin();
  goal(1) = 100.0;
  goal(2) = -100.0;
  for (int i = 0; i < 500; ++i) {c.update(goal, origin(), dt);}
  const double limit = MAX_BENDING_DEGREE * std::acos(-1.0) / 180.0;
  EXPECT_NEAR(c.surgical_tool_.tAngle_, limit, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, -limit, angle_tolerance);
  for (int i = 0; i < 500; ++i) {c.update(origin(), origin(), dt);}
  EXPECT_NEAR(c.surgical_tool_.tAngle_, 0.0, 1e-9);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.0, 1e-9);
}

TEST(PositionPD, RejectsBadFramesAndNonfiniteCorrection)
{
  PositionController c;
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
  PositionController c;
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

TEST(PositionPD, CameraGapPreservesGoalAndReferenceButDiscardsOldDerivative)
{
  PositionController c;
  c.reset(origin(), 0.2, 0.1);
  c.pid_controller_pan_.kd_ = c.pid_controller_tilt_.kd_ = 1000.0;
  auto goal = origin();
  goal(1) = 0.004;
  goal(2) = 0.006;
  for (int i = 0; i < 30; ++i) {c.update(goal, origin(), dt);}
  auto actual = origin();
  actual(1) = 0.001;
  actual(2) = 0.002;
  ASSERT_NO_THROW(c.update(goal, actual, 1.0));
  EXPECT_TRUE(c.x_desired_.isApprox(goal));
  EXPECT_DOUBLE_EQ(c.dt_, 1.0);
  EXPECT_NEAR(c.surgical_tool_.tAngle_, 0.1 + 5.0 * 0.003, angle_tolerance);
  EXPECT_NEAR(c.surgical_tool_.pAngle_, 0.2 + 5.0 * 0.004, angle_tolerance);
}

TEST(PositionPD, LongCameraGapDoesNotAccumulateSlewAllowance)
{
  PositionController c;
  c.reset(origin(), 0.0, 0.0);
  auto goal = origin();
  goal(1) = 1.0;
  ASSERT_NO_THROW(c.update(goal, origin(), 100.0));
  const double max_step = 5.0 * std::acos(-1.0) / 180.0 * 0.25;
  EXPECT_NEAR(c.del_theta_tilt_, max_step, angle_tolerance);
  c.update(goal, origin(), dt);
  EXPECT_NEAR(c.del_theta_tilt_, 5.0 * std::acos(-1.0) / 180.0 * dt, angle_tolerance);
}
