#include <gtest/gtest.h>
#include <limits>
#include "position_sine_motion.hpp"

namespace {
constexpr double radius = 19.0 * 4.33e-3;
}

TEST(PositionSine, DefaultsUseMetresQuadratureAndTwentySeconds)
{
  const hrm::PositionSineConfig c;
  EXPECT_NO_THROW(c.validate(radius));
  EXPECT_DOUBLE_EQ(c.period, 20.0);
  EXPECT_DOUBLE_EQ(c.speed, 0.010);
  const std::array<std::array<double, 2>, 5> expected{{
    {{0., .01}}, {{.01, 0.}}, {{0., -.01}}, {{-.01, 0.}}, {{0., .01}}
  }};
  for (unsigned i = 0; i < expected.size(); ++i) {
    const auto q = c.sample(i * 5.0);
    EXPECT_NEAR(q[0], expected[i][0], 1e-12);
    EXPECT_NEAR(q[1], expected[i][1], 1e-12);
  }
}

TEST(PositionSine, CentersAreAbsoluteAndZeroAmplitudeHoldsSelectedAxis)
{
  hrm::PositionSineConfig c;
  c.y = {{-.012, .005, -90.}};
  c.z = {{.003, 0., 90.}};
  c.validate(radius);
  EXPECT_NEAR(c.sample(0)[0], -.017, 1e-12);
  EXPECT_NEAR(c.sample(10)[0], -.007, 1e-12);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(c.sample(i * .2)[1], .003);
  }
  c.y = {{0., 0., 0.}};
  c.z = {{0., .01, 0.}};
  EXPECT_DOUBLE_EQ(c.sample(5)[0], 0.);
  EXPECT_NEAR(c.sample(5)[1], .01, 1e-12);
}

TEST(PositionSine, ApproachHasNoJumpAndWaveStartsAfterArrival)
{
  hrm::PositionSineConfig c;
  c.speed = 0.005;  // A user can still choose a slower approach than the default.
  hrm::PositionSineMotion motion;
  motion.start(c, 0., 0., radius);
  const auto first = motion.advance(.1);
  EXPECT_DOUBLE_EQ(first[0], 0.);
  EXPECT_NEAR(first[1], .0005, 1e-12);
  for (int i = 1; i < 20; ++i) {motion.advance(.1);}
  const auto wave = motion.advance(.1);
  const auto expected = c.sample(.1);
  EXPECT_NEAR(wave[0], expected[0], 1e-12);
  EXPECT_NEAR(wave[1], expected[1], 1e-12);
}

TEST(PositionSine, TenMillimetresPerSecondDefaultAndSlowerOverride)
{
  hrm::PositionSineConfig c;
  c.y[1] = c.z[1] = 0.020;
  EXPECT_NO_THROW(c.validate(radius));  // 6.283 mm/s at T=20 s.
  hrm::PositionSineMotion motion;
  motion.start(c, 0., 0., radius);
  EXPECT_NEAR(motion.advance(.1)[1], .001, 1e-12);
  c.speed = .005;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c.speed = .010;
  c.y[1] = .031;
  EXPECT_NO_THROW(c.validate(radius));
  c.y[1] = .032;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
}

TEST(PositionSine, EveryAxisReferenceStepRespectsMetresPerSecondLimit)
{
  hrm::PositionSineConfig c;
  c.y = {{.005, .004, 25.}};
  c.z = {{-.005, .004, -120.}};
  hrm::PositionSineMotion motion;
  motion.start(c, -.04, .03, radius);
  std::array<double, 2> previous{{-.04, .03}};
  for (int i = 0; i < 5000; ++i) {
    const auto next = motion.advance(.02);
    for (unsigned axis = 0; axis < 2; ++axis) {
      EXPECT_LE(std::abs(next[axis] - previous[axis]), c.speed * .02 + 1e-12);
    }
    previous = next;
  }
}

TEST(PositionSine, MissingAndInvalidIntervalsFreezeWithoutCatchUp)
{
  const hrm::PositionSineConfig c;
  hrm::PositionSineMotion with_gap, uninterrupted;
  with_gap.start(c, 0., .01, radius);
  uninterrupted.start(c, 0., .01, radius);
  with_gap.advance(.1);
  uninterrupted.advance(.1);
  const auto before = with_gap.advance(.1);
  uninterrupted.advance(.1);
  for (double invalid : {0., -.1, .26, 2., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_EQ(with_gap.advance(invalid), before);
  }
  EXPECT_EQ(with_gap.advance(.11, .1), before);
  const auto resumed = with_gap.advance(.1);
  const auto expected = uninterrupted.advance(.1);
  EXPECT_EQ(resumed, expected);
}

TEST(PositionSine, ConfigRejectsNegativeAmplitudeInvalidPeriodAndExcessPeakSpeed)
{
  hrm::PositionSineConfig c;
  c.y[1] = -.001;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c = hrm::PositionSineConfig{}; c.period = .9;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c = hrm::PositionSineConfig{}; c.speed = 0.;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c = hrm::PositionSineConfig{}; c.period = 2.;
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c.period = 20.; c.speed = .004;
  EXPECT_NO_THROW(c.validate(radius));
}

TEST(PositionSine, ConfigRejectsNonfiniteValuesAndOutsideCombinedEnvelope)
{
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    for (unsigned i = 0; i < 3; ++i) {
      auto c = hrm::PositionSineConfig{}; c.y[i] = invalid;
      EXPECT_THROW(c.validate(radius), std::invalid_argument);
      c = hrm::PositionSineConfig{}; c.z[i] = invalid;
      EXPECT_THROW(c.validate(radius), std::invalid_argument);
    }
    auto c = hrm::PositionSineConfig{}; c.period = invalid;
    EXPECT_THROW(c.validate(radius), std::invalid_argument);
    c = hrm::PositionSineConfig{}; c.speed = invalid;
    EXPECT_THROW(c.validate(radius), std::invalid_argument);
  }
  auto c = hrm::PositionSineConfig{};
  c.y = {{.06, .001, 0.}};
  c.z = {{.06, .001, 0.}};
  EXPECT_THROW(c.validate(radius), std::invalid_argument);
  c.z = {{0., .001, 0.}};
  EXPECT_NO_THROW(c.validate(radius));
}

TEST(PositionSine, StartRejectsInvalidNominalPositionAndRestartReinitializesPhase)
{
  const hrm::PositionSineConfig c;
  hrm::PositionSineMotion motion;
  EXPECT_THROW(motion.start(c, .1, 0., radius), std::invalid_argument);
  EXPECT_THROW(motion.start(c, .06, .06, radius), std::invalid_argument);
  EXPECT_THROW(motion.start(c, std::numeric_limits<double>::quiet_NaN(), 0., radius),
    std::invalid_argument);
  motion.start(c, 0., .01, radius);
  motion.advance(.1);
  for (int i = 0; i < 10; ++i) {motion.advance(.1);}
  motion.start(c, 0., 0., radius);
  const auto first_again = motion.advance(.1);
  EXPECT_DOUBLE_EQ(first_again[0], 0.);
  EXPECT_NEAR(first_again[1], c.speed * .1, 1e-12);
}

TEST(PositionSine, VeryLargeFinitePhaseIsWrappedBeforeConversionToRadians)
{
  hrm::PositionSineConfig c;
  c.y[2] = std::numeric_limits<double>::max();
  c.validate(radius);
  EXPECT_TRUE(std::isfinite(c.sample(1.)[0]));
}
