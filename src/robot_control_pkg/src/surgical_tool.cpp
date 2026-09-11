#include "surgical_tool.hpp"

SurgicalTool::SurgicalTool() {
	init_surgical_tool(
		NUM_OF_JOINT_PAIRS,
		SEGMENT_ARC,
		SEGMENT_DIAMETER,
		WIRE_DISTANCE,
		SHIFT,
		SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER
		);
	std::cout << "Surgical tool is created" << &this->surgicaltool_ << std::endl;
}

SurgicalTool::~SurgicalTool() {

}

void SurgicalTool::init_surgical_tool(int num_joint_pairs,
									 float arc,
									 float diameter,
									 float disWire,
									 float shift,
									 float arc_center_to_seg_center)
{
	this->surgicaltool_.num_joint_pairs = num_joint_pairs;
	this->surgicaltool_.arc 	  =	arc * mm_;
	this->surgicaltool_.diameter  =	diameter * mm_;
	this->surgicaltool_.disWire   =	disWire * mm_;
	this->surgicaltool_.shift	  = shift * torad();
	this->surgicaltool_.arc_center_to_seg_center = arc_center_to_seg_center * mm_;
	this->alpha_ = asin(this->surgicaltool_.disWire / this->surgicaltool_.arc);
}

void SurgicalTool::set_bending_angle(double pAngle, double tAngle) {
	this->pAngle_ = pAngle * torad();
	this->tAngle_ = tAngle * torad();
}

void SurgicalTool::set_forceps_angle(double angle) {	// degree
	this->target_forceps_angle_ = angle;	// non radian
}

std::vector<double> SurgicalTool::get_IK_result(
	double pAngle,
	double tAngle,
	double gAngle)
{
	// Filtering MAX_BENDING_DEGREE (hw_definition.hpp)
	pAngle = std::min(pAngle, MAX_BENDING_DEGREE);
	tAngle = std::min(tAngle, MAX_BENDING_DEGREE);

	pAngle = std::max(pAngle, -MAX_BENDING_DEGREE);
	tAngle = std::max(tAngle, -MAX_BENDING_DEGREE);

	// 1. set angle(degree) of continuum part
	this->set_bending_angle(pAngle, tAngle);
	// 2. set angle(degree) o forceps
	this->set_forceps_angle(gAngle);
	// 3. calculate inverse_kinematics
	this->inverse_kinematics();

	std::vector<double> wire_length_results = {this->wrLengthEast_, this->wrLengthWest_, this->wrLengthSouth_, this->wrLengthNorth_, this->wrLengthGrip};
	return  wire_length_results;
	// return std::tuple<double, double, double, double, double>
	// (this->wrLengthEast_, this->wrLengthWest_, this->wrLengthSouth_, this->wrLengthNorth_, this->wrLengthGrip);
}

void SurgicalTool::inverse_kinematics()
{
	// y = kx (k=SHIFT/SHIFT_THRESHOLD)

	const double twice_joint_pair_count = 2.0 * surgicaltool_.num_joint_pairs;
	this->wrLengthEast_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint_pairs * ( cos(alpha_) - cos(alpha_ - (pAngle_ / twice_joint_pair_count)) + 1 - cos(tAngle_ / twice_joint_pair_count));
	this->wrLengthWest_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint_pairs * ( cos(alpha_) - cos(alpha_ + (pAngle_ / twice_joint_pair_count)) + 1 - cos(tAngle_ / twice_joint_pair_count));
	this->wrLengthSouth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint_pairs * ( cos(alpha_) - cos(alpha_ - (tAngle_ / twice_joint_pair_count)) + 1 - cos(pAngle_ / twice_joint_pair_count));
	this->wrLengthNorth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint_pairs * ( cos(alpha_) - cos(alpha_ + (tAngle_ / twice_joint_pair_count)) + 1 - cos(pAngle_ / twice_joint_pair_count));

	// // Gain for released wire
	// if (this->wrLengthEast_ < 0) { this->wrLengthEast_ = this->wrLengthEast_ * this->release_gain_; }
	// if (this->wrLengthWest_ < 0) { this->wrLengthWest_ = this->wrLengthWest_ * this->release_gain_; }
	// if (this->wrLengthSouth_ < 0) { this->wrLengthSouth_ = this->wrLengthSouth_ * this->release_gain_; }
	// if (this->wrLengthNorth_ < 0) { this->wrLengthNorth_ = this->wrLengthNorth_ * this->release_gain_; }

	this->wrLengthEast_ =  this->wrLengthEast_ / mm_;
	this->wrLengthWest_ =  this->wrLengthWest_ / mm_;
	this->wrLengthSouth_ = this->wrLengthSouth_ / mm_;
	this->wrLengthNorth_ = this->wrLengthNorth_ / mm_;

	// y = -x + 30
	this->wrLengthGrip = ((-1) * this->target_forceps_angle_ + this->max_forceps_deg_) * ( MAX_FORCEPS_RAGNE_MM / MAX_FORCEPS_RAGNE_DEGREE );
}

Eigen::Matrix4d SurgicalTool::computeTransformationMatrix(
	const double& joint_angle,
	const double& twist_angle,
	const double& previous_link_length) const
{
	// Modified D-H convention used by the segment-angle estimator and Table 1:
	// ^(i-1)T_i = Rx(alpha_(i-1)) Tx(r_(i-1)) Rz(q_i), with d_i = 0.
	const double cos_q = std::cos(joint_angle);
	const double sin_q = std::sin(joint_angle);
	const double cos_alpha = std::cos(twist_angle);
	const double sin_alpha = std::sin(twist_angle);

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	transform(0, 0) = cos_q;
	transform(0, 1) = -sin_q;
	transform(1, 0) = cos_alpha * sin_q;
	transform(1, 1) = cos_alpha * cos_q;
	transform(1, 2) = -sin_alpha;
	transform(2, 0) = sin_alpha * sin_q;
	transform(2, 1) = sin_alpha * cos_q;
	transform(2, 2) = cos_alpha;
	transform(0, 3) = previous_link_length;

	return transform;
}

std::vector<Eigen::Matrix4d>
SurgicalTool::computeBaseToJointsTransformationMatrices(
	const std::vector<double>& pan_angles,
	const std::vector<double>& tilt_angles) const
{
	const auto bending_joint_count =
		static_cast<std::size_t>(NUM_OF_BENDING_JOINTS);
	if (pan_angles.size() != bending_joint_count ||
		tilt_angles.size() != bending_joint_count)
	{
		throw std::invalid_argument(
			"Pan and tilt arrays must each contain NUM_OF_BENDING_JOINTS entries.");
	}

	std::vector<Eigen::Matrix4d> transform_matrices;
	transform_matrices.reserve(bending_joint_count);
	Eigen::Matrix4d transform_base_from_previous = Eigen::Matrix4d::Identity();

	for (std::size_t index = 0; index < bending_joint_count; ++index) {
		const bool is_pan_joint = index % 2 == 0;
		const double joint_angle =
			is_pan_joint ? pan_angles[index] : tilt_angles[index];

		double twist_angle = 0.0;
		if (index > 0) {
			twist_angle = index % 2 == 1 ? PI_ / 2.0 : -PI_ / 2.0;
		}

		const double previous_link_length =
			(index == 0 ? PROXIMAL_OFFSET_LENGTH : BENDING_JOINT_SPACING) * mm_;
		const Eigen::Matrix4d transform_previous_from_current =
			computeTransformationMatrix(
				joint_angle, twist_angle, previous_link_length);
		const Eigen::Matrix4d transform_base_from_current =
			transform_base_from_previous * transform_previous_from_current;
		transform_matrices.push_back(transform_base_from_current);
		transform_base_from_previous = transform_base_from_current;
	}

	return transform_matrices;
}

Eigen::Vector3d SurgicalTool::extractXYZfromTransformMatrix(
	const Eigen::Matrix4d& transform) const
{
	return transform.block<3, 1>(0, 3);
}

std::vector<Eigen::Vector3d> SurgicalTool::computeJointPositions(
	const std::vector<Eigen::Matrix4d>& transforms) const
{
	if (transforms.size() !=
		static_cast<std::size_t>(NUM_OF_BENDING_JOINTS))
	{
		throw std::invalid_argument(
			"The FK transform array must contain NUM_OF_BENDING_JOINTS entries.");
	}

	std::vector<Eigen::Vector3d> positions;
	positions.reserve(transforms.size());
	for (const auto& transform : transforms) {
		positions.push_back(extractXYZfromTransformMatrix(transform));
	}
	return positions;
}

Eigen::Matrix4d SurgicalTool::computeEndEffectorTransformation(
	const std::vector<Eigen::Matrix4d>& transforms) const
{
	if (transforms.size() !=
		static_cast<std::size_t>(NUM_OF_BENDING_JOINTS))
	{
		throw std::invalid_argument(
			"The FK transform array must contain NUM_OF_BENDING_JOINTS entries.");
	}

	Eigen::Matrix4d transform_last_joint_from_tip = Eigen::Matrix4d::Identity();
	transform_last_joint_from_tip(0, 3) = DISTAL_OFFSET_LENGTH * mm_;
	return transforms.back() * transform_last_joint_from_tip;
}

Eigen::Vector3d SurgicalTool::computeEndEffectorPosition(
	const std::vector<Eigen::Matrix4d>& transforms) const
{
	return extractXYZfromTransformMatrix(
		computeEndEffectorTransformation(transforms));
}

float SurgicalTool::tomm()
{
	return this->mm_;
}

float SurgicalTool::torad()
{
	return this->deg_;
}

float SurgicalTool::todeg()
{
	return 1.0/this->deg_;
}
