#include "surgical_tool.hpp"

SurgicalTool::SurgicalTool() {
	init_surgical_tool(
		NUM_OF_JOINT,
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

void SurgicalTool::init_surgical_tool(int num_joint,
									 float arc,
									 float diameter,
									 float disWire,
									 float shift,
									 float arc_center_to_seg_center)
{
	this->surgicaltool_.num_joint = num_joint;
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

	this->wrLengthEast_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - (pAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(tAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthWest_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + (pAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(tAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthSouth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - (tAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(pAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthNorth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + (tAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(pAngle_ / (2*surgicaltool_.num_joint)));

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

Eigen::Matrix4d SurgicalTool::computeTransformationMatrix(const double& theta) {
	double R = this->surgicaltool_.arc*3;
	double L = this->surgicaltool_.arc_center_to_seg_center;
	double l_theta = R / std::cos(theta / 2.0) - L;
    double p_x = l_theta * (1.0 + std::cos(theta));
    double p_y = l_theta * std::sin(theta);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = std::cos(theta);
    T(0, 1) = -std::sin(theta);
    T(1, 0) = std::sin(theta);
    T(1, 1) = std::cos(theta);
    T(0, 3) = p_x;
    T(1, 3) = p_y;

    return T;
}

std::vector<Eigen::Matrix4d> SurgicalTool::computeBaseToJointsTransformationMatrices(const std::vector<double>& joint_angles) {
	std::vector<Eigen::Matrix4d> transform_matrices;
	Eigen::Matrix4d T_prev = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();

	for (int i=0; i<this->surgicaltool_.num_joint; i++) {
		Eigen::Matrix4d T_next = computeTransformationMatrix(joint_angles[i]);
		T_current = T_current * T_next;
		transform_matrices.push_back(T_current);
	}
	
	return transform_matrices;
}

Eigen::Vector3d SurgicalTool::extractXYZfromTransformMatrix(const Eigen::Matrix4d& T) {
    return Eigen::Vector3d(T(0, 3), T(1, 3), T(2,3));
}

std::vector<Eigen::Vector3d> SurgicalTool::computeJointPositions(const std::vector<Eigen::Matrix4d>& transforms) {
    std::vector<Eigen::Vector3d> positions;
    for (const auto& T : transforms) {
        positions.push_back(extractXYZfromTransformMatrix(T));
    }
    return positions;
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