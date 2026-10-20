#include <gtest/gtest.h>
#include <limits>
#include "ik_sine_motion.hpp"

TEST(IKSine, DefaultsAreSigned45DegreeQuadratureOver20Seconds)
{
  const hrm::SineConfig c;
  EXPECT_NO_THROW(c.validate());
  EXPECT_DOUBLE_EQ(c.period, 20.);
  EXPECT_DOUBLE_EQ(c.speed, 15.);
  const std::array<std::array<double, 2>, 5> expected{{
    {{0., 45.}}, {{45., 0.}}, {{0., -45.}}, {{-45., 0.}}, {{0., 45.}}
  }};
  for (unsigned i = 0; i < expected.size(); ++i) {
    const auto q = c.sample(5. * i);
    EXPECT_NEAR(q[0], expected[i][0], 1e-10);
    EXPECT_NEAR(q[1], expected[i][1], 1e-10);
  }
}

TEST(IKSine, TiltOnlyZeroTo45AndPanHeld)
{
  hrm::SineConfig c;
  c.period = 30.;
  c.tilt = {{22.5, 22.5, -90.0}};
  c.pan = {{7.0, 0.0, 0.0}};
  c.validate();
  EXPECT_NEAR(c.sample(0)[0], 0.0, 1e-12);
  EXPECT_NEAR(c.sample(15)[0], 45.0, 1e-12);
  EXPECT_NEAR(c.sample(30)[0], 0.0, 1e-12);
  EXPECT_NEAR(c.sample(12)[1], 7.0, 1e-12);
}

TEST(IKSine, PanOnlyAndQuarterPhasePair)
{
  hrm::SineConfig c;
  c.period = 30.;
  c.tilt = {{0., 0., 0.}};
  c.pan = {{22.5, 22.5, -90.}};
  EXPECT_NEAR(c.sample(15)[0], 0., 1e-12);
  EXPECT_NEAR(c.sample(15)[1], 45., 1e-12);
  c.tilt = {{0., 10., 0.}};
  c.pan = {{0., 10., 90.}};
  for (int i = 0; i < 100; ++i) {
    const auto q = c.sample(i * .3);
    EXPECT_NEAR(q[0]*q[0] + q[1]*q[1], 100., 1e-9);
  }
}

TEST(IKSine, RejectUnsafeParameters)
{
  hrm::SineConfig c;
  c.speed = 5.;
  c.tilt = {{30., 20., 0.}};
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c.tilt = {{0., -1., 0.}};
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c.tilt = {{0., 45., 0.}};
  c.period = 30.;
  EXPECT_THROW(c.validate(), std::invalid_argument);  // 45deg/30s exceeds 5deg/s.
  c.period = 60.;
  EXPECT_NO_THROW(c.validate());
  c.pan[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c.pan[2] = 0.;
  c.period = 0.;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c.period = 60.; c.speed = 16.;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(IKSine, ApproachHasNoPhaseJumpAndRespectsSpeed)
{
  hrm::SineConfig c;
  c.period = 30.;
  c.speed = 5.;
  c.tilt = {{22.5, 22.5, 90.}};
  c.pan = {{0., 0., 0.}};
  hrm::SineMotion motion;
  motion.start(c, 0., 0.);
  auto previous = motion.advance(.01);
  EXPECT_NEAR(previous[0], .05, 1e-12);
  for (int i = 0; i < 10000; ++i) {
    auto next = motion.advance(.01);
    EXPECT_LE(std::abs(next[0]-previous[0]), .050000001);
    EXPECT_GE(next[0], -1e-10);
    EXPECT_LE(next[0], 45.000000001);
    EXPECT_DOUBLE_EQ(next[1], 0.);
    previous = next;
  }
  EXPECT_THROW(motion.advance(.3), std::invalid_argument);
  EXPECT_THROW(motion.advance(-1.), std::invalid_argument);
  EXPECT_THROW(motion.start(c, 46., 0.), std::invalid_argument);
}
