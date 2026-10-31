# Inference output; the publisher never commands motors or enables admittance.
# robot_control may consume eligible tip force in explicitly selected ADMITTANCE mode.
# header.stamp: newest selected input sample time; header.frame_id: hrm_base.
std_msgs/Header header
builtin_interfaces/Time published_stamp
bool valid
string status
# status=ready_force_ranges identifies the configurable inferred-force box policy.
# ready means native class0; ready_load_unknown means masked without a load detector.
string model_name
string mode
string force_scope

# Inverse-scaled model XYZ target, in N and hrm_base, retaining training sign.
# The control-side -1 sign convention is NOT applied by this publisher.
# NaN while invalid. A tip-only model's output is not a body-force estimate.
geometry_msgs/Vector3 external_force

# Original classifier argmax (0 can mean unloaded for class0; -1 invalid).
int16 contact_id
# Candidate operating ID, not a motor command: 18 for UNLOADED, positive-ID
# candidate for LOADED; -1 if load status is unknown. Raw contact_id is preserved.
# Force-range policy uses positive-ID argmax outside the box, even if raw ID=0.
int16 control_contact_id
uint8 LOAD_UNKNOWN=0
uint8 UNLOADED=1
uint8 LOADED=2
uint8 load_state
# Model-scope/ID-gated eligibility diagnostic only, NOT hardware authorization.
# Controller's visualization-only ID policy does not gate force using this flag.
bool tip_force_valid
bool body_contact_enabled
bool body_contact_detected

# Original probabilities indexed by ID 0..18. In masked mode p0 is unavailable
# and stored as zero; use mode/load_state to distinguish it from class0.
float64[19] probabilities
int16 heatmap_center_id
uint8 id_tol
# Sum of ORIGINAL probabilities inside the displayed band, not accuracy.
float64 band_probability
uint32 samples_in_window
uint32 window_samples
float64 inference_ms
