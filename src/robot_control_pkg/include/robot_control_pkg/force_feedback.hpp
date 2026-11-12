// AUTHOR: Daeyun
// HRM-specific validity checks for learned force/contact estimates used in control.
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include "custom_interfaces/msg/force_id_estimate.hpp"

namespace hrm {
inline std::string force_feedback_issue(
  const custom_interfaces::msg::ForceIdEstimate& msg, int64_t now_ns,
  double timeout, double max_force, bool use_contact_id_gate = true)
{
  using Estimate = custom_interfaces::msg::ForceIdEstimate;
  if (!msg.valid) {return "force estimate invalid/not ready: " + msg.status;}
  if (msg.header.frame_id != "hrm_base") {return "force frame must be hrm_base";}
  if (msg.model_name.empty() ||
      (msg.force_scope != "tip" && msg.force_scope != "all_single_contact"))
  {
    return "tip or all_single_contact force model required";
  }
  if (msg.mode != "class0" && msg.mode != "masked") {return "unsupported force classifier mode";}
  if (!msg.status.empty() && msg.status != "ready" && msg.status != "ready_force_ranges" &&
      msg.status != "ready_load_unknown")
  {
    return "force estimate status not ready: " + msg.status;
  }
  if (msg.window_samples == 0 || msg.samples_in_window != msg.window_samples) {
    return "force input window incomplete";
  }
  const auto& f = msg.external_force;
  if (!std::isfinite(f.x) || !std::isfinite(f.y) || !std::isfinite(f.z)) {
    return "non-finite force estimate";
  }
  if (std::hypot(f.x, f.y, f.z) > max_force) {return "estimated force exceeds max_force_N";}
  for (const auto& stamp : {msg.header.stamp, msg.published_stamp}) {
    const int64_t ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const double age = (now_ns - ns) * 1e-9;
    if (ns <= 0 || age < -0.05 || age > timeout) {return "force source/publication timestamp stale";}
  }
  // Explicit user-selected tip-contact experiment: ID/load predictions only
  // drive visualization, not feedback authorization. No stale/invalid force
  // is accepted and no force is replaced by zero or by the previous sample.
  if (!use_contact_id_gate) {return "";}
  bool unloaded = false;
  bool tip = false;
  if (msg.status == "ready_force_ranges") {
    // Explicit producer policy: an agreed XYZ force-range detector supplies
    // load_state. The raw classifier remains untouched, including class0=0.
    // Threshold settings are owned by the producer, not inferred here from ID.
    if ((msg.mode != "masked" && msg.mode != "class0") ||
        (msg.load_state != Estimate::UNLOADED && msg.load_state != Estimate::LOADED))
    {
      return "force-range policy requires masked/class0 and a known load state";
    }
    double probability_sum = 0.0;
    for (double probability : msg.probabilities) {
      if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0) {
        return "invalid contact probabilities for force-range policy";
      }
      probability_sum += probability;
    }
    if (std::abs(probability_sum - 1.0) > 1e-4 ||
        (msg.mode == "masked" && msg.probabilities[0] != 0.0))
    {
      return "contact probabilities must be normalized; masked p0 must be zero";
    }
    int positive_winner = 1;
    for (int id = 2; id <= 18; ++id) {
      if (msg.probabilities[id] > msg.probabilities[positive_winner]) {positive_winner = id;}
    }
    int raw_winner = msg.mode == "class0" ? 0 : 1;
    for (int id = raw_winner + 1; id <= 18; ++id) {
      if (msg.probabilities[id] > msg.probabilities[raw_winner]) {raw_winner = id;}
    }
    if ((msg.load_state == Estimate::LOADED && msg.probabilities[positive_winner] <= 0.0) ||
        msg.contact_id != raw_winner || msg.heatmap_center_id != positive_winner)
    {
      return "contact ID/heatmap winner contradict force-range probabilities";
    }
    unloaded = msg.load_state == Estimate::UNLOADED;
    const int control_id = unloaded ? 18 : positive_winner;
    const bool body_contact = !unloaded && control_id < 18;
    if (msg.control_contact_id != control_id || msg.tip_force_valid != (control_id == 18) ||
        msg.body_contact_detected != body_contact)
    {
      return "force-range load/control eligibility flags are inconsistent";
    }
    tip = !unloaded && control_id == 18;
  } else {
    // Native class0 remains compatible with prior ready messages. A masked
    // known-load flag alone must never silently authorize this controller.
    if ((!msg.status.empty() && msg.status != "ready") || msg.mode != "class0" ||
        msg.load_state == Estimate::LOAD_UNKNOWN)
    {
      return "load policy unavailable; require native class0 or explicit ready_force_ranges";
    }
    unloaded = msg.load_state == Estimate::UNLOADED && msg.contact_id == 0;
    tip = msg.load_state == Estimate::LOADED && msg.contact_id == 18;
  }
  if (!(unloaded || tip) || msg.control_contact_id != 18 || !msg.tip_force_valid ||
      msg.body_contact_detected) {
    return "body/non-tip estimate; tip force must not drive admittance";
  }
  return "";
}

// This classifies a pause only. The caller must skip the complete control
// update; invalid producer messages intentionally carry NaN force placeholders.
// Keep the preceding validation issue so stale timestamps cannot hide an
// already-detected bad frame, model, force vector, or force limit.
inline bool force_feedback_wait_required(
  const custom_interfaces::msg::ForceIdEstimate& msg, const std::string& issue,
  bool received_recently)
{
  if (!received_recently || issue == "force source/publication timestamp stale") {
    return true;
  }
  if (msg.valid) {return false;}
  if (msg.status == "warming_up" || msg.status == "input_time_skew") {return true;}
  for (const std::string prefix : {"stale_input:", "missing_or_invalid_input:"}) {
    if (msg.status.size() > prefix.size() && msg.status.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}
}  // namespace hrm
