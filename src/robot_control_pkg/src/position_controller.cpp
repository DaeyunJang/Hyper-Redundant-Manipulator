#include "position_controller.hpp"

// Constructor to initialize PID gains
PositionController::PositionController()
  : x_desired_(Eigen::VectorXd::Zero(6)),
  x_actual_(Eigen::VectorXd::Zero(6)),
  x_err_(Eigen::VectorXd::Zero(6)),
  del_theta_pan_(0),
  del_theta_tilt_(0) {
    initialize();
  }


PositionController::~PositionController() {};

void PositionController::initialize() {
    pid_controller_pan_.set_PID_gains(position_control_params::KP, position_control_params::KI, position_control_params::KD);
    pid_controller_tilt_.set_PID_gains(position_control_params::KP, position_control_params::KI, position_control_params::KD);
    surgical_tool_.init_surgical_tool(NUM_OF_JOINT, SEGMENT_ARC, SEGMENT_DIAMETER, WIRE_DISTANCE, SHIFT, SEGMENT_ARC_CENTER_TO_SEGMENT_CENTER);
}

std::vector<double> PositionController::update(
  const Eigen::VectorXd& x_desired,
  const Eigen::VectorXd& x_actual,
  const double& dt) {

  this->x_desired_ = x_desired;
  this->x_actual_ = x_actual;
  this->x_err_ = this->x_desired_ - this->x_actual_;
  this->dt_ = dt;

  /**
   * @brief compute PID output for pan and tilt angle
   * @warning Check the coordinate system of HRM
   * Pan  : control(translation) on Y-axis
   * Tilt : control(translation) on Z-axis
   * X-axis value is passive about pan, tilt and external force.
   * If you want to control on X-axis, use robot arm or any other actuation
   * So,
   * pan <- y of x_d : index 1
   * tilt <- z of x_d : index 2
   */
  this->del_theta_pan_ = pid_controller_pan_.compute_output(x_desired(1), x_actual(1), dt);
  this->del_theta_tilt_ =  pid_controller_tilt_.compute_output(x_desired(2), x_actual(2), dt);

  /**
   * @brief input for Inverse-Kinematics (Y.J. Kim)
   * @note  input = (current set angle) + (delta angle)
   */
  double final_theta_input_pan = (this->surgical_tool_.pAngle_ + this->del_theta_pan_) * surgical_tool_.todeg();
  double final_theta_input_tilt = (this->surgical_tool_.tAngle_ + this->del_theta_tilt_) * surgical_tool_.todeg();
  // std::cout << "final_theta_input: " << final_theta_input << std::endl;

  // std::cout << "input_theta(pan): " << final_theta_input_pan << std::endl;
  // std::cout << "input_theta(tilt): " << final_theta_input_tilt << std::endl;
  /**
   * @brief get wire length from IK.
   * @warning If operate 2-DOF manipulation, then substitute the tilt-anle
   */
  auto wire_length_to_move = this->surgical_tool_.get_IK_result(final_theta_input_pan, final_theta_input_tilt, 0);

  return wire_length_to_move;
}
