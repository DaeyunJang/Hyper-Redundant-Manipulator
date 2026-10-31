#include <gtest/gtest.h>
#include <cmath>

#include "admittance_controller.hpp"

namespace {

Eigen::VectorXd zero() {return Eigen::VectorXd::Zero(6);}

Eigen::MatrixXd yz(double value) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(6, 6);
  matrix(1, 1) = matrix(2, 2) = value;
  return matrix;
}

void configure(AdmittanceController& controller,
  double mass = 0.05, double damping = 1.0, double spring = 20.0)
{
  auto& filter = controller.admittance_filter_;
  filter.setMassMatrix(yz(mass));
  filter.setDampingMatrix(yz(damping));
  filter.setSpringMatrix(yz(spring));
  filter.reset();
}

struct Gains {double mass, damping, spring;};
// Source settings and two observed/previous settings. These are test cases,
// not a change to defaults or a recommendation for hardware tuning.
const Gains contact_test_gains[] = {{0.05, 1.0, 20.0}, {0.1, 15.0, 50.0}, {0.15, 20.0, 30.0}};

}  // namespace

TEST(AdmittanceController, PhysicalForceDirectionSurvivesLegacySignAdapter) {
  AdmittanceController controller;
  configure(controller);
  auto force = zero();
  force(1) = 0.02;
  force(2) = -0.04;
  const double dt = 0.01;  // Short enough to retain the exact legacy single step.
  const auto displacement = controller.compute(zero(), force, dt);
  EXPECT_NEAR(displacement(1), 0.02 / 0.05 * dt * dt, 1e-12);
  EXPECT_NEAR(displacement(2), -0.04 / 0.05 * dt * dt, 1e-12);
  EXPECT_TRUE(controller.f_env_.isApprox(force));
  EXPECT_TRUE(controller.f_desired_.isZero());
  EXPECT_TRUE(controller.del_f_.isApprox(force));
  EXPECT_TRUE(controller.del_xf_.isApprox(displacement));
  EXPECT_DOUBLE_EQ(controller.dt_, dt);
  // Public force is already mapped to force ON the robot. Converting both
  // references at the legacy boundary yields environment - desired.
  EXPECT_TRUE(controller.admittance_filter_.env_force_.isApprox(-force));
  EXPECT_TRUE(controller.admittance_filter_.desired_force_.isZero());
}

TEST(AdmittanceController, PositiveAndNegativeEnvironmentForcesOnEachActiveAxis) {
  for (const int axis : {1, 2}) {
    for (const double sign : {-1.0, 1.0}) {
      SCOPED_TRACE(::testing::Message() << "axis=" << axis << ", sign=" << sign);
      AdmittanceController controller;
      configure(controller);
      auto force = zero();
      force(axis) = sign * 0.02;
      const double dt = 0.01;
      const auto displacement = controller.compute(zero(), force, dt);
      EXPECT_NEAR(displacement(axis), force(axis) / 0.05 * dt * dt, 1e-12);
      EXPECT_GT(displacement(axis) * force(axis), 0.0);
      EXPECT_DOUBLE_EQ(displacement(axis == 1 ? 2 : 1), 0.0);
      EXPECT_TRUE(controller.del_f_.isApprox(force));
      for (const int inactive : {0, 3, 4, 5}) {
        EXPECT_DOUBLE_EQ(displacement(inactive), 0.0);
      }
    }
  }
}

TEST(AdmittanceController, ZeroForcesAndZeroStatePreserveMinusTenMillimeterNominalY) {
  AdmittanceController controller;
  configure(controller);
  auto nominal = zero();
  nominal(1) = -0.01;
  for (int step = 0; step < 30; ++step) {
    const auto offset = controller.compute(zero(), zero(), 1.0 / 30.0);
    EXPECT_TRUE(offset.isZero());
    const Eigen::VectorXd commanded_position = nominal + offset;
    EXPECT_DOUBLE_EQ(commanded_position(1), -0.01);
    EXPECT_DOUBLE_EQ(commanded_position(2), 0.0);
    EXPECT_TRUE(controller.del_f_.isZero());
    EXPECT_TRUE(controller.admittance_filter_.getXtDot().isZero());
    EXPECT_TRUE(controller.admittance_filter_.getXtDDot().isZero());
  }
}

TEST(AdmittanceController, EqualPhysicalTargetAndEnvironmentHoldZeroState) {
  AdmittanceController controller;
  configure(controller);
  auto force = zero();
  force(1) = -0.15;
  force(2) = 0.2;
  for (int step = 0; step < 30; ++step) {
    EXPECT_TRUE(controller.compute(force, force, 0.03).isZero());
    EXPECT_TRUE(controller.del_f_.isZero());
  }
  EXPECT_TRUE(controller.f_desired_.isApprox(force));
  EXPECT_TRUE(controller.f_env_.isApprox(force));
}

TEST(AdmittanceController, TargetAndEnvironmentHaveOppositeEffects) {
  AdmittanceController controller;
  configure(controller);
  auto force = zero();
  force(1) = 0.2;
  force(2) = -0.1;
  const auto environment_response = controller.compute(zero(), force, 0.03);
  controller.admittance_filter_.reset();
  const auto target_response = controller.compute(force, zero(), 0.03);
  EXPECT_TRUE(target_response.isApprox(-environment_response));
  EXPECT_TRUE(controller.del_f_.isApprox(-force));
  EXPECT_TRUE(controller.admittance_filter_.desired_force_.isApprox(-force));
  EXPECT_TRUE(controller.admittance_filter_.env_force_.isZero());
}

TEST(AdmittanceController, ZeroForcesRetainThenDecayNonzeroState) {
  AdmittanceController controller;
  configure(controller);
  auto force = zero();
  force(1) = 0.2;
  force(2) = -0.1;
  const double dt = 0.01;
  const auto previous_x = controller.compute(zero(), force, dt);
  const auto previous_v = controller.admittance_filter_.getXtDot();
  const Eigen::VectorXd expected_a = (-previous_v - 20.0 * previous_x) / 0.05;
  const Eigen::VectorXd expected_v = previous_v + expected_a * dt;
  const Eigen::VectorXd expected_x = previous_x + expected_v * dt;

  const auto displacement = controller.compute(zero(), zero(), dt);
  EXPECT_FALSE(displacement.isZero());
  EXPECT_TRUE(displacement.isApprox(expected_x, 1e-12));
  EXPECT_TRUE(controller.admittance_filter_.getXtDot().isApprox(expected_v, 1e-12));
  EXPECT_TRUE(controller.admittance_filter_.getXtDDot().isApprox(expected_a, 1e-12));
  EXPECT_TRUE(controller.del_f_.isZero());
  for (int step = 0; step < 300; ++step) {
    const auto residual = controller.compute(zero(), zero(), 1.0 / 30.0);
    ASSERT_TRUE(residual.allFinite());
    EXPECT_LT(residual.norm(), 0.01);
  }
  EXPECT_LT(controller.admittance_filter_.getXt().norm(), 1e-8);
  EXPECT_LT(controller.admittance_filter_.getXtDot().norm(), 1e-8);
}

TEST(AdmittanceController, ConvertedForcesMatchLegacyFilterAcrossMultipleSafeSubsteps) {
  AdmittanceController controller;
  configure(controller);
  AdmittanceFilter legacy(yz(0.05), yz(1.0), yz(20.0));
  const double intervals[] = {0.001, 0.01, 1.0 / 30.0, 0.1, 0.25};
  for (int step = 0; step < 100; ++step) {
    SCOPED_TRACE(::testing::Message() << "step=" << step);
    auto target = zero();
    auto environment = zero();
    // Exercise positive/negative forces and targets, then release the
    // retained state. Larger intervals exercise the legacy safe substeps.
    if (step % 4 != 3) {
      target(1) = step % 2 == 0 ? 0.03 : -0.02;
      target(2) = step % 3 == 0 ? -0.04 : 0.01;
      environment(1) = step % 2 == 0 ? -0.05 : 0.06;
      environment(2) = step % 3 == 0 ? 0.02 : -0.07;
    }
    const double dt = intervals[step % 5];
    const auto expected = legacy.computeAdmittance(-target, -environment, dt);
    const auto actual = controller.compute(target, environment, dt);
    EXPECT_LT((actual - expected).norm(), 1e-12);
    EXPECT_LT((controller.admittance_filter_.getXtDot() - legacy.getXtDot()).norm(), 1e-12);
    EXPECT_LT((controller.admittance_filter_.getXtDDot() - legacy.getXtDDot()).norm(), 1e-12);
    EXPECT_LT((controller.del_f_ - (environment - target)).norm(), 1e-12);
    EXPECT_LT((controller.admittance_filter_.desired_force_ + target).norm(), 1e-12);
    EXPECT_LT((controller.admittance_filter_.env_force_ + environment).norm(), 1e-12);
  }
}

TEST(AdmittanceController, PassiveContactRestoresSignedPerturbationsOnBothAxes) {
  constexpr double environment_stiffness = 100.0;
  constexpr double dt = 1.0 / 30.0;
  for (const auto& gains : contact_test_gains) {
    for (const int axis : {1, 2}) {
      for (const double sign : {-1.0, 1.0}) {
        SCOPED_TRACE(::testing::Message() << "M=" << gains.mass << ", B=" << gains.damping
          << ", K=" << gains.spring << ", axis=" << axis << ", sign=" << sign);
        AdmittanceController controller;
        configure(controller, gains.mass, gains.damping, gains.spring);
        const double initial_offset = sign * 0.001;
        controller.admittance_filter_.xt_(axis) = initial_offset;
        auto nominal = zero();
        nominal(1) = -0.01;
        Eigen::VectorXd measured = nominal + controller.admittance_filter_.getXt();
        for (int step = 0; step < 300; ++step) {
          auto environment = zero();
          environment(axis) = -environment_stiffness * (measured(axis) - nominal(axis));
          const auto offset = controller.compute(zero(), environment, dt);
          ASSERT_TRUE(offset.allFinite());
          ASSERT_TRUE(controller.admittance_filter_.getXtDot().allFinite());
          ASSERT_LT(offset.norm(), 0.01);
          if (step == 0) {
            EXPECT_LT((offset(axis) - initial_offset) * initial_offset, 0.0);
            EXPECT_LT(controller.admittance_filter_.getXtDot()(axis) * initial_offset, 0.0);
          }
          measured = nominal + offset;  // Ideal inner position tracking.
        }
        EXPECT_NEAR(measured(axis), nominal(axis), 1e-8);
        EXPECT_LT(controller.admittance_filter_.getXtDot().norm(), 1e-8);
      }
    }
  }
}

TEST(AdmittanceController, SampledPassiveContactConvergesWithTargetsAndFiftyMilliNewtonBiases) {
  // This exercises the actual C++ controller in an ideal-position-tracking
  // loop. The bilateral spring force is sampled and held once per 30 Hz
  // update, including across the filter's internal integration substeps.
  // ke=0 models no contact. It does not model camera/actuator lag, unilateral
  // contact, estimator dynamics, or establish stability for arbitrary ke.
  constexpr double dt = 1.0 / 30.0;
  for (const auto& gains : contact_test_gains) {
    for (const double environment_stiffness : {0.0, 100.0}) {
      for (const int axis : {1, 2}) {
        for (const double target : {-0.05, 0.0, 0.05}) {
          for (const double bias : {-0.05, 0.0, 0.05}) {
            SCOPED_TRACE(::testing::Message() << "M=" << gains.mass << ", B=" << gains.damping
              << ", K=" << gains.spring << ", ke=" << environment_stiffness
              << ", axis=" << axis << ", target=" << target << ", bias=" << bias);
            AdmittanceController controller;
            configure(controller, gains.mass, gains.damping, gains.spring);
            auto desired = zero();
            desired(axis) = target;
            auto nominal = zero();
            nominal(1) = -0.01;
            Eigen::VectorXd measured = nominal;
            const double equilibrium = (bias - target) / (gains.spring + environment_stiffness);
            for (int step = 0; step < 300; ++step) {
              auto environment = zero();
              environment(axis) = bias - environment_stiffness * (measured(axis) - nominal(axis));
              const auto offset = controller.compute(desired, environment, dt);
              ASSERT_TRUE(offset.allFinite());
              ASSERT_TRUE(controller.admittance_filter_.getXtDot().allFinite());
              ASSERT_TRUE(controller.admittance_filter_.getXtDDot().allFinite());
              // Finite regression bounds for these examples, not output clamps.
              ASSERT_LT(offset.norm(), 0.006);
              ASSERT_LT(controller.admittance_filter_.getXtDot().norm(), 0.06);
              for (int other = 0; other < 6; ++other) {
                if (other != axis) {EXPECT_DOUBLE_EQ(offset(other), 0.0);}
              }
              if (step >= 270) {EXPECT_NEAR(offset(axis), equilibrium, 1e-8);}
              measured = nominal + offset;
            }
            const double final_offset = measured(axis) - nominal(axis);
            const double final_environment = bias - environment_stiffness * final_offset;
            EXPECT_NEAR(final_offset, equilibrium, 1e-8);
            EXPECT_LT(controller.admittance_filter_.getXtDot().norm(), 1e-8);
            // With K>0 a displaced spring carries a force residual: matching
            // the desired force exactly is not generally the equilibrium.
            EXPECT_NEAR(final_environment - target, gains.spring * final_offset, 2e-6);
            if (target == 0.0 && environment_stiffness == 0.0 && bias != 0.0) {
              EXPECT_NEAR(std::abs(final_offset), 0.05 / gains.spring, 1e-8);
            }
          }
        }
      }
    }
  }
}

TEST(AdmittanceController, FullSixDimensionalCoupledMatrixIsPreserved) {
  AdmittanceController controller;
  Eigen::MatrixXd mass = Eigen::MatrixXd::Identity(6, 6);
  mass(0, 1) = mass(1, 0) = 0.25;
  mass(3, 5) = mass(5, 3) = -0.2;
  auto& filter = controller.admittance_filter_;
  filter.setMassMatrix(mass);
  filter.setDampingMatrix(Eigen::MatrixXd::Identity(6, 6) * 2.0);
  filter.setSpringMatrix(Eigen::MatrixXd::Identity(6, 6) * 10.0);
  auto target = zero();
  auto force = zero();
  target << 0.01, -0.02, 0.03, 0.02, -0.01, 0.04;
  force << 0.2, 0.3, -0.4, -0.05, 0.06, 0.07;
  const double dt = 0.02;
  const Eigen::VectorXd expected = mass.ldlt().solve(force - target) * dt * dt;
  const auto displacement = controller.compute(target, force, dt);
  EXPECT_EQ(displacement.size(), 6);
  EXPECT_TRUE(displacement.isApprox(expected, 1e-12));
  EXPECT_TRUE(controller.del_f_.isApprox(force - target));
  EXPECT_NE(displacement(0), 0.0);
  EXPECT_NE(displacement(3), 0.0);
  EXPECT_NE(displacement(4), 0.0);
  EXPECT_NE(displacement(5), 0.0);
}

TEST(AdmittanceController, DoesNotReintroduceOffsetOrVelocitySaturation) {
  AdmittanceController controller;
  auto& filter = controller.admittance_filter_;
  filter.setMassMatrix(yz(0.0));
  filter.setDampingMatrix(yz(1.0));
  filter.setSpringMatrix(yz(0.0));
  auto force = zero();
  force(1) = 0.3;
  force(2) = -0.2;
  const auto displacement = controller.compute(zero(), force, 0.04);
  EXPECT_NEAR(displacement(1), 0.012, 1e-12);
  EXPECT_NEAR(displacement(2), -0.008, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(1), 0.3, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(2), -0.2, 1e-12);
}
