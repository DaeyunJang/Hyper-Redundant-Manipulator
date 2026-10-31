#include <gtest/gtest.h>
#include <limits>
#include "admittance_filter.hpp"
#include "admittance_settings.hpp"
#include "force_feedback.hpp"

namespace {
Eigen::VectorXd zero() {return Eigen::VectorXd::Zero(6);}
Eigen::MatrixXd yz(double value) {
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, 6);
  result(1, 1) = result(2, 2) = value;
  return result;
}
custom_interfaces::msg::ForceIdEstimate tip() {
  custom_interfaces::msg::ForceIdEstimate msg;
  msg.valid = true;
  msg.header.frame_id = "hrm_base";
  msg.header.stamp.sec = msg.published_stamp.sec = 1;
  msg.mode = "class0";
  msg.model_name = "test_model";
  msg.force_scope = "tip";
  msg.load_state = msg.LOADED;
  msg.contact_id = msg.control_contact_id = 18;
  msg.tip_force_valid = true;
  msg.window_samples = msg.samples_in_window = 30;
  return msg;
}
std::string issue(const custom_interfaces::msg::ForceIdEstimate& msg, int64_t now=1000000000LL,
  bool use_contact_id_gate = true)
{
  return hrm::force_feedback_issue(msg, now, 0.25, 2.0, use_contact_id_gate);
}
custom_interfaces::msg::ForceIdEstimate force_ranges(
  const std::string& mode = "masked", int raw_id = 18, bool unloaded = false)
{
  auto msg = tip();
  msg.status = "ready_force_ranges";
  msg.mode = mode;
  msg.contact_id = raw_id;
  msg.probabilities.fill(0.0);
  msg.probabilities[raw_id] = 0.7;
  msg.probabilities[raw_id == 18 ? 17 : 18] = 0.3;
  msg.heatmap_center_id = raw_id == 0 ? 18 : raw_id;
  msg.load_state = unloaded ? msg.UNLOADED : msg.LOADED;
  msg.control_contact_id = unloaded ? 18 : msg.heatmap_center_id;
  msg.tip_force_valid = msg.control_contact_id == 18;
  msg.body_contact_detected = !unloaded && msg.control_contact_id < 18;
  return msg;
}
}

TEST(AdmittanceLegacy, DesiredMinusExternalAndOriginalIntegrationOrder) {
  AdmittanceFilter filter(yz(0.05), yz(1.0), yz(20.0));
  auto force = zero(); force(1) = 0.01; force(2) = -0.02;
  const double dt = 0.001;  // Already small enough: exactly one legacy step.
  auto x = filter.computeAdmittance(zero(), force, dt);
  EXPECT_LT(x(1), 0.0); EXPECT_GT(x(2), 0.0);
  EXPECT_NEAR(x(1), -dt * dt * 0.01 / 0.05, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(1), -dt * 0.01 / 0.05, 1e-12);
  const double previous_v = filter.getXtDot()(1);
  const double expected_a = (-0.01 - previous_v - 20.0 * x(1)) / 0.05;
  filter.computeAdmittance(zero(), force, dt);
  EXPECT_NEAR(filter.getXtDDot()(1), expected_a, 1e-12);
  EXPECT_NEAR(filter.getXt()(1), x(1) + (previous_v + expected_a * dt) * dt, 1e-12);
  filter.reset();
  auto target_x = filter.computeAdmittance(force, zero(), dt);
  EXPECT_TRUE(target_x.isApprox(-x));
}

TEST(AdmittanceLegacy, EntirelyInactiveAxesRemainZeroAndEqualForcesDoNotMove) {
  AdmittanceFilter filter(yz(0.05), yz(1.0), yz(20.0));
  auto force = zero(); force(0) = 99.0; force(3) = 50.0; force(4) = -60.0; force(5) = 70.0;
  for (int i=0; i<100; ++i) {
    EXPECT_TRUE(filter.computeAdmittance(zero(), force, 0.033).isZero());
  }
  force(1) = 0.1; force(2) = -0.2;
  EXPECT_TRUE(filter.computeAdmittance(force, force, 0.033).isZero());
}

TEST(AdmittanceLegacy, HighDampingAtThirtyHzStaysFiniteAndReachesUnclippedEquilibrium) {
  // An unrefined legacy step would have 1-B*dt/M ~= -2.33 and diverge.
  AdmittanceFilter filter(yz(0.05), yz(5.0), yz(2.0));
  auto force = zero(); force(1) = 0.02; force(2) = -0.04;
  for (int i=0; i<1000; ++i) {
    EXPECT_TRUE(filter.computeAdmittance(zero(), force, 1.0 / 30).allFinite());
    EXPECT_LE(filter.getXt().cwiseAbs().maxCoeff(), 0.020001);
  }
  EXPECT_NEAR(filter.xt_(1), -0.01, 1e-7);
  EXPECT_NEAR(filter.xt_(2), 0.02, 1e-7);
  EXPECT_TRUE(filter.xt_.allFinite());
}

TEST(AdmittanceLegacy, PureDamperHasNoFiveMillimeterOrTwoMillimeterPerSecondLimit) {
  AdmittanceFilter filter(yz(0.0), yz(1.0), yz(0.0));
  auto force = zero(); force(1) = 0.2; force(2) = -0.1;
  const auto x = filter.computeAdmittance(zero(), force, 0.1);
  EXPECT_NEAR(x(1), -0.02, 1e-12);
  EXPECT_NEAR(x(2), 0.01, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(1), -0.2, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(2), 0.1, 1e-12);
  const auto stopped = filter.xt_;
  filter.stopVelocity();
  EXPECT_TRUE(filter.xt_.isApprox(stopped));
  EXPECT_TRUE(filter.xtdot_.isZero());
  filter.reset();
  EXPECT_TRUE(filter.xt_.isZero());
}

TEST(AdmittanceLegacy, DamperSpringUsesLegacyFirstOrderEquation) {
  AdmittanceFilter filter(yz(0.0), yz(2.0), yz(3.0));
  auto desired = zero(); desired(1) = 0.02;
  const double dt = 0.01;
  EXPECT_NEAR(filter.computeAdmittance(desired, zero(), dt)(1), 0.0001, 1e-12);
  EXPECT_NEAR(filter.computeAdmittance(desired, zero(), dt)(1), 0.0001985, 1e-12);
  EXPECT_NEAR(filter.getXtDot()(1), 0.00985, 1e-12);
  EXPECT_TRUE(filter.getXtDDot().isZero());
}

TEST(AdmittanceLegacy, SpringOnlyUsesUnclippedStaticDisplacement) {
  AdmittanceFilter filter(yz(0.0), yz(0.0), yz(0.001));
  auto desired = zero(); desired(2) = 0.01;
  EXPECT_NEAR(filter.computeAdmittance(desired, zero(), 0.03)(2), 10.0, 1e-12);
  EXPECT_TRUE(filter.getXtDot().isZero());
  EXPECT_TRUE(filter.getXtDDot().isZero());
}

TEST(AdmittanceLegacy, ZeroMatricesAndOriginalNormThresholdRemainPassive) {
  AdmittanceFilter filter;
  auto desired = Eigen::VectorXd::Ones(6).eval();
  EXPECT_TRUE(filter.computeAdmittance(desired, zero(), 0.03).isZero());
  filter.setSpringMatrix(yz(0.0005));  // Frobenius norm is below legacy eps=1e-3.
  EXPECT_TRUE(filter.computeAdmittance(desired, zero(), 0.03).isZero());
}

TEST(AdmittanceLegacy, AllSixAxesAreSupportedByFullMatrices) {
  AdmittanceFilter filter(Eigen::MatrixXd::Identity(6,6), yz(0.0), yz(0.0));
  const Eigen::VectorXd desired = Eigen::VectorXd::LinSpaced(6, 1.0, 6.0);
  const double dt = 0.01;
  EXPECT_TRUE(filter.computeAdmittance(desired, zero(), dt).isApprox(desired * dt * dt));
  EXPECT_TRUE(filter.getXtDot().isApprox(desired * dt));
  EXPECT_TRUE(filter.getXtDDot().isApprox(desired));
}

TEST(AdmittanceLegacy, CoupledSpringIsSolvedAsOneMatrix) {
  Eigen::MatrixXd spring = Eigen::MatrixXd::Identity(6,6) * 2.0;
  spring(0, 4) = spring(4, 0) = 1.0;
  AdmittanceFilter filter(yz(0.0), yz(0.0), spring);
  auto desired = zero(); desired(0) = 3.0;
  const auto x = filter.computeAdmittance(desired, zero(), 0.03);
  EXPECT_NEAR(x(0), 2.0, 1e-12);
  EXPECT_NEAR(x(4), -1.0, 1e-12);
  EXPECT_TRUE((spring * x).isApprox(desired));
}

TEST(AdmittanceLegacy, SingularCouplingSupportsConsistentForceAndRejectsInconsistentForce) {
  Eigen::MatrixXd spring = yz(1.0);
  spring(1, 2) = spring(2, 1) = 1.0;
  AdmittanceFilter filter(yz(0.0), yz(0.0), spring);
  auto desired = zero(); desired(1) = desired(2) = 0.02;
  const auto x = filter.computeAdmittance(desired, zero(), 0.03);
  EXPECT_NEAR(x(1), 0.01, 1e-12);
  EXPECT_NEAR(x(2), 0.01, 1e-12);
  desired(2) = 0.0;
  EXPECT_THROW(filter.computeAdmittance(desired, zero(), 0.03), std::runtime_error);
  EXPECT_TRUE(filter.getXt().isApprox(x));  // An invalid solve does not partially update state.
}

TEST(AdmittanceLegacy, RejectInvalidParametersAndDt) {
  AdmittanceFilter f(yz(0.05), yz(1.0), yz(20.0));
  EXPECT_THROW(f.setMassMatrix(yz(-1.0)), std::invalid_argument);
  EXPECT_NO_THROW(f.setMassMatrix(Eigen::MatrixXd::Identity(6,6)));
  EXPECT_THROW(f.setMassMatrix(Eigen::MatrixXd::Zero(3,3)), std::invalid_argument);
  auto asymmetric = yz(1.0); asymmetric(1, 2) = 0.1;
  EXPECT_THROW(f.setSpringMatrix(asymmetric), std::invalid_argument);
  auto bad=zero(); bad(2)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(f.computeAdmittance(zero(),bad,0.03), std::invalid_argument);
  for (double dt : {0.0, -0.1, 0.3}) {
    EXPECT_THROW(f.computeAdmittance(zero(),zero(),dt), std::invalid_argument);
  }
  hrm::AdmittanceSettings s;
  EXPECT_FALSE(s.use_contact_id_gate);
  EXPECT_NO_THROW(s.validate());
  s.use_contact_id_gate = true;
  EXPECT_NO_THROW(s.validate());
  s.target_y=3.0; EXPECT_THROW(s.validate(), std::invalid_argument);
  s.target_y=0.0; s.force_sign=0.0; EXPECT_THROW(s.validate(), std::invalid_argument);
}

TEST(ForceFeedback, TipAndUnloadedEligibleButNoFabricatedZero) {
  auto msg=tip(); msg.external_force.y=-0.02;
  EXPECT_TRUE(issue(msg).empty());
  msg.contact_id=0; msg.load_state=msg.UNLOADED;
  EXPECT_TRUE(issue(msg).empty());
  EXPECT_DOUBLE_EQ(msg.external_force.y,-0.02);
}

TEST(ForceFeedback, RejectBodyRegardlessOfMonitorFlag) {
  for (bool enabled : {false,true}) {
    auto msg=tip(); msg.body_contact_enabled=enabled;
    msg.contact_id=16; msg.control_contact_id=16; msg.tip_force_valid=false;
    EXPECT_FALSE(issue(msg).empty());
    msg.control_contact_id=18; msg.tip_force_valid=true; // Contradictory flags cannot authorize it.
    EXPECT_FALSE(issue(msg).empty());
  }
}

TEST(ForceFeedback, RejectMaskedInvalidStaleWrongFrameAndOutOfRange) {
  auto msg=tip(); msg.mode="masked"; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.valid=false; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.header.frame_id="camera"; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.force_scope="body"; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.samples_in_window=29; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.external_force.y=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(issue(msg).empty());
  msg=tip(); msg.external_force.x=3.0; EXPECT_FALSE(issue(msg).empty());
  msg=tip(); EXPECT_FALSE(issue(msg,1400000000LL).empty());
  EXPECT_FALSE(issue(msg,900000000LL).empty());
  msg.header.stamp.sec=0; EXPECT_FALSE(issue(msg).empty());
}

TEST(ForceFeedback, ExplicitForceRangesAllowMaskedUnloadedWithoutChangingRawBodyIdOrForce) {
  auto msg = force_ranges("masked", 6, true);
  msg.external_force.x = .1;
  msg.external_force.y = -.12;
  msg.external_force.z = .03;
  EXPECT_TRUE(issue(msg).empty());
  EXPECT_EQ(msg.contact_id, 6);
  EXPECT_EQ(msg.heatmap_center_id, 6);
  EXPECT_EQ(msg.control_contact_id, 18);
  EXPECT_DOUBLE_EQ(msg.external_force.y, -.12);
  msg = force_ranges("class0", 0, true);
  msg.probabilities.fill(0.); msg.probabilities[0] = 1.;
  msg.heatmap_center_id = 1;  // Positive-class argmax is unused in UNLOADED.
  EXPECT_TRUE(issue(msg).empty());
}

TEST(ForceFeedback, ExplicitForceRangesAllowLoadedTipInBothClassifierModes) {
  for (const auto& mode : {"masked", "class0"}) {
    auto msg = force_ranges(mode, 18);
    msg.external_force.y = .2;
    EXPECT_TRUE(issue(msg).empty());
  }
  // A high class0 probability cannot override the explicit force-range policy.
  auto msg = force_ranges("class0", 0);
  msg.external_force.z = -.2;
  EXPECT_EQ(msg.contact_id, 0);
  EXPECT_EQ(msg.heatmap_center_id, 18);
  EXPECT_TRUE(issue(msg).empty());
}

TEST(ForceFeedback, ForceRangesRejectLoadedBodyRegardlessOfMonitorToggle) {
  for (bool monitor : {false, true}) {
    auto msg = force_ranges("masked", 7);
    msg.body_contact_enabled = monitor;
    EXPECT_FALSE(issue(msg).empty());
    msg.control_contact_id = 18;
    msg.tip_force_valid = true;
    msg.body_contact_detected = false;
    EXPECT_FALSE(issue(msg).empty());
    msg = force_ranges("class0", 0);
    msg.probabilities[18] = 0.0;
    msg.probabilities[7] = .3;
    msg.heatmap_center_id = msg.control_contact_id = 7;
    msg.tip_force_valid = false;
    msg.body_contact_detected = true;
    EXPECT_FALSE(issue(msg).empty());
  }
}

TEST(ForceFeedback, ForceRangesRejectInconsistentPolicyFlagsAndUnknownModes) {
  auto msg = force_ranges();
  msg.load_state = msg.LOAD_UNKNOWN; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.load_state = 99; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.mode = "other"; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.tip_force_valid = false; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.body_contact_detected = true; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.control_contact_id = 17; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges("masked", 6, true);
  msg.control_contact_id = 6; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges("masked", 6, true);
  msg.body_contact_detected = true; EXPECT_FALSE(issue(msg).empty());
}

TEST(ForceFeedback, ForceRangesRejectMismatchedIdsAndMalformedProbabilities) {
  auto msg = force_ranges();
  msg.contact_id = 17; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.contact_id = -1; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.contact_id = 19; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.heatmap_center_id = 17; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.probabilities[0] = .01; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.probabilities[17] = .1; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.probabilities.fill(0.); EXPECT_FALSE(issue(msg).empty());
  for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    msg = force_ranges(); msg.probabilities[5] = invalid;
    EXPECT_FALSE(issue(msg).empty());
  }
  msg = force_ranges("class0", 0);
  msg.probabilities.fill(0.); msg.probabilities[0] = 1.;
  EXPECT_FALSE(issue(msg).empty());  // No positive-ID evidence to support tip.
}

TEST(ForceFeedback, OnlyExplicitForceRangeProvenanceCanAuthorizeMaskedKnownLoad) {
  for (const auto& status : {"", "ready", "ready_force_range", "warming_up", "bad_source"}) {
    auto msg = force_ranges(); msg.status = status;
    EXPECT_FALSE(issue(msg).empty());
  }
  auto msg = tip(); msg.status = "ready";
  EXPECT_TRUE(issue(msg).empty());
  msg.status = "unrecognized_policy";
  EXPECT_FALSE(issue(msg).empty());
}

TEST(ForceFeedback, ForceRangePolicyRetainsFrameFreshnessScopeAndForceGuards) {
  auto msg = force_ranges(); msg.valid = false; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.header.frame_id = "camera"; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.model_name.clear(); EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.force_scope = "body"; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.window_samples = 0; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.samples_in_window = 29; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.external_force.x = 3.; EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); msg.external_force.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(issue(msg).empty());
  msg = force_ranges(); EXPECT_FALSE(issue(msg, 1400000000LL).empty());
  msg = force_ranges(); msg.published_stamp.sec = 0; EXPECT_FALSE(issue(msg).empty());
}

TEST(ForceFeedback, DisplayOnlyIdDoesNotInterruptForceForBodyPredictionOrUnknownLoad) {
  for (const auto& mode : {"masked", "class0"}) {
    auto msg = force_ranges(mode, 9);
    msg.force_scope = "all_single_contact";
    msg.external_force.y = .2;
    EXPECT_TRUE(issue(msg, 1000000000LL, false).empty());
    EXPECT_FALSE(issue(msg, 1000000000LL, true).empty());
    EXPECT_EQ(msg.contact_id, 9);
    EXPECT_DOUBLE_EQ(msg.external_force.y, .2);
  }
  auto msg = force_ranges("masked", 9);
  msg.status = "ready_load_unknown";
  msg.load_state = msg.LOAD_UNKNOWN;
  msg.control_contact_id = -1;
  msg.tip_force_valid = false;
  EXPECT_TRUE(issue(msg, 1000000000LL, false).empty());
  EXPECT_FALSE(issue(msg, 1000000000LL, true).empty());
}

TEST(ForceFeedback, IdGateEnabledAllowsBothEligibleForceModelScopes) {
  for (const auto& scope : {"tip", "all_single_contact"}) {
    auto msg = force_ranges(); msg.force_scope = scope;
    EXPECT_TRUE(issue(msg, 1000000000LL, true).empty());
    msg = tip(); msg.force_scope = scope;
    EXPECT_TRUE(issue(msg, 1000000000LL, true).empty());
  }
}

TEST(ForceFeedback, DisplayOnlyIdStillRequiresValidReadyFiniteFreshForce) {
  auto msg = force_ranges("masked", 9);
  auto check_rejected = [](const auto& estimate) {
    EXPECT_FALSE(issue(estimate, 1000000000LL, false).empty());
  };
  msg.valid = false; check_rejected(msg);
  msg = force_ranges(); msg.status = "warming_up"; check_rejected(msg);
  msg = force_ranges(); msg.mode = "unknown"; check_rejected(msg);
  msg = force_ranges(); msg.model_name.clear(); check_rejected(msg);
  msg = force_ranges(); msg.force_scope = "body"; check_rejected(msg);
  msg = force_ranges(); msg.force_scope.clear(); check_rejected(msg);
  msg = force_ranges(); msg.header.frame_id = "camera"; check_rejected(msg);
  msg = force_ranges(); msg.window_samples = 0; check_rejected(msg);
  msg = force_ranges(); msg.samples_in_window = 29; check_rejected(msg);
  msg = force_ranges(); msg.external_force.z = std::numeric_limits<double>::quiet_NaN();
  check_rejected(msg);
  msg = force_ranges(); msg.external_force.x = std::numeric_limits<double>::infinity();
  check_rejected(msg);
  msg = force_ranges(); msg.external_force.y = 3.; check_rejected(msg);
  msg = force_ranges(); msg.header.stamp.sec = 0; check_rejected(msg);
  msg = force_ranges(); msg.published_stamp.sec = 0; check_rejected(msg);
  msg = force_ranges();
  EXPECT_FALSE(issue(msg, 1400000000LL, false).empty());
  EXPECT_FALSE(issue(msg, 900000000LL, false).empty());
}

TEST(ForceFeedback, MissingOrStaleMeasurementsPauseWithoutAuthorizingTheirForce) {
  auto msg = tip();
  EXPECT_TRUE(hrm::force_feedback_wait_required(msg, "fresh /force_id_estimation required", false));
  EXPECT_FALSE(hrm::force_feedback_wait_required(msg, issue(msg), true));
  for (int64_t now : {1400000000LL, 900000000LL}) {
    const auto stale = issue(msg, now);
    EXPECT_FALSE(stale.empty());
    EXPECT_TRUE(hrm::force_feedback_wait_required(msg, stale, true));
  }
  msg.header.stamp.sec = 0;
  EXPECT_FALSE(issue(msg).empty());
  EXPECT_TRUE(hrm::force_feedback_wait_required(msg, issue(msg), true));
}

TEST(ForceFeedback, RecognizedProducerWarmupAndInputGapsPauseWithoutUsingNaNPlaceholders) {
  for (const auto& status : {"warming_up", "input_time_skew",
      "stale_input:/estimated_segment_angle", "missing_or_invalid_input:/loadcell_state"})
  {
    auto msg = tip();
    msg.valid = false;
    msg.status = status;
    msg.header.stamp.sec = 0;
    msg.samples_in_window = 0;
    msg.external_force.x = std::numeric_limits<double>::quiet_NaN();
    const auto invalid = issue(msg);
    EXPECT_FALSE(invalid.empty());
    EXPECT_TRUE(hrm::force_feedback_wait_required(msg, invalid, true)) << status;
    EXPECT_TRUE(std::isnan(msg.external_force.x));  // Never fabricate a zero force.
    msg.valid = true;
    EXPECT_FALSE(hrm::force_feedback_wait_required(msg, issue(msg), true)) << status;
  }
}

TEST(ForceFeedback, UnknownInvalidAndModelFailureStatusesRemainHardFaults) {
  for (const auto& status : {"", "ready", "no_model_selected", "model_load_failed:bad model",
      "inference_failed:non-finite force", "warming_up_bad", "input_time_skew_bad",
      "stale_input:", "missing_or_invalid_input:", "unrecognized"})
  {
    auto msg = tip();
    msg.valid = false;
    msg.status = status;
    EXPECT_FALSE(hrm::force_feedback_wait_required(msg, issue(msg), true)) << status;
  }
}

TEST(ForceFeedback, StaleStampDoesNotDowngradeStructuralOrForceFailures) {
  auto check_hard = [](const auto& msg) {
    const auto invalid = issue(msg, 1400000000LL, false);
    EXPECT_FALSE(invalid.empty());
    EXPECT_NE(invalid, "force source/publication timestamp stale");
    EXPECT_FALSE(hrm::force_feedback_wait_required(msg, invalid, true));
  };
  auto msg = tip(); msg.header.frame_id = "camera"; check_hard(msg);
  msg = tip(); msg.model_name.clear(); check_hard(msg);
  msg = tip(); msg.force_scope = "body"; check_hard(msg);
  msg = tip(); msg.mode = "other"; check_hard(msg);
  msg = tip(); msg.samples_in_window = 29; check_hard(msg);
  msg = tip(); msg.external_force.y = std::numeric_limits<double>::quiet_NaN(); check_hard(msg);
  msg = tip(); msg.external_force.z = 3.0; check_hard(msg);
}
