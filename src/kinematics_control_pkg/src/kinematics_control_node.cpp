#include "kinematics_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>  

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;
using namespace std::chrono_literals;

KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions & node_options)
: Node("KinematicsControlNode", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  //===============================
  // target value publisher
  //===============================
  this->motor_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  this->motor_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  for(int i=0; i<NUM_OF_MOTORS; i++) {
    this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100/10;
  }
  motor_control_publisher_ = this->create_publisher<MotorCommand>("motor_command", QoS_RKL10V);
  RCLCPP_INFO(this->get_logger(), "Publisher 'motor_command' is created.");
  //===============================
  // surgical tool pose(degree) publisher
  //===============================
  surgical_tool_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("surgical_tool_pose", QoS_RKL10V);
  wire_length_publisher_ = 
    this->create_publisher<std_msgs::msg::Float32MultiArray>("wire_length", QoS_RKL10V);
  this->wire_length_.data.resize(NUM_OF_MOTORS);
  //===============================
  // motor status subscriber
  //===============================
  this->motor_state_.actual_position.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_velocity.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_acceleration.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_torque.resize(NUM_OF_MOTORS);
  motor_state_subscriber_ =
    this->create_subscription<MotorState>(
      "motor_state",
      QoS_RKL10V,
      [this] (const MotorState::SharedPtr msg) -> void
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /motor_state.");
        this->op_mode_ = kEnable;
        this->motorstate_op_flag_ = true;
        this->motor_state_.header = msg->header;
        this->motor_state_.actual_position =  msg->actual_position;
        this->motor_state_.actual_velocity =  msg->actual_velocity;
        this->motor_state_.actual_acceleration =  msg->actual_acceleration;
        this->motor_state_.actual_torque =  msg->actual_torque;

        for (int i=0; i<NUM_OF_MOTORS; i++) {
          this->wire_length_.data[i] = this->motor_state_.actual_position[i] * 2 / gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          // this->wire_length_.data.push_back(this->motor_state_.actual_position[i] * 2 / gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION));
        }
        this->wire_length_publisher_->publish(this->wire_length_);
        // if(this->op_mode_ == kEnable) {
        //   this->cal_inverse_kinematics();
        //   this->motor_control_publisher_->publish(this->motor_control_target_val_);
        //   this->surgical_tool_pose_publisher_->publish(this->surgical_tool_pose_);
        // }
      }
    );
  
  //===============================
  // loadcell data subscriber
  //===============================
  loadcell_data_subscriber_ =
    this->create_subscription<custom_interfaces::msg::LoadcellState>(
      "loadcell_state",
      QoS_RKL10V,
      [this] (const custom_interfaces::msg::LoadcellState::SharedPtr msg) -> void
      {
        try {
          this->loadcell_op_flag_ = true;
          this->loadcell_data_.header = msg->header;
          this->loadcell_data_.stress = msg->stress;
          this->loadcell_data_.output_voltage = msg->output_voltage;
          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /loadcell_data.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
        
      }
    );


  /**
   * @brief service
   */
  auto get_target_move_motor_direct = 
  [this](
  const std::shared_ptr<MoveMotorDirect::Request> request,
  std::shared_ptr<MoveMotorDirect::Response> response) -> void
  {
    try {
      int32_t idx = request->index_motor;
      int32_t target_position = request->target_position;
      int32_t target_velocity_profile = request->target_velocity_profile;
      // save the target values
      for (int i=0; i<NUM_OF_MOTORS; i++) {
        if(i == idx) {
          this->motor_control_target_val_.target_position[i] = this->motor_state_.actual_position[i] + target_position;
          this->motor_control_target_val_.target_velocity_profile[i] = target_velocity_profile;
        } else {
          this->motor_control_target_val_.target_position[i] = this->motor_state_.actual_position[i];
          this->motor_control_target_val_.target_velocity_profile[i] = 100;
        }
      }

      // std::cout << target_position << std::endl;
      // std::cout << this->motor_state_.actual_position [0] << std::endl;
      // std::cout << this->motor_control_target_val_.target_position[0] << std::endl;

      // publish and response for service from client
      if(this->op_mode_ == kEnable) {
        this->motor_control_publisher_->publish(this->motor_control_target_val_);
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Service <MoveMotorDirect> accept the request");
      }
      else response->success = false;

    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
    
  };
  move_motor_direct_service_server_ = 
    create_service<MoveMotorDirect>("kinematics/move_motor_direct", get_target_move_motor_direct);

  auto get_target_move_tool_angle = 
  [this](
  const std::shared_ptr<MoveToolAngle::Request> request,
  std::shared_ptr<MoveToolAngle::Response> response) -> void
  {
    try {
      // run
      if(this->op_mode_ == kEnable) {
        if(request->mode == 0) {
          // MOVE ABSOLUTELY
          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, request->tiltangle, request->panangle, request->gripangle);
          this->cal_inverse_kinematics(request->panangle, request->tiltangle, request->gripangle);
          this->motor_control_publisher_->publish(this->motor_control_target_val_);
          this->surgical_tool_pose_publisher_->publish(this->surgical_tool_pose_);
        }
        else if (request->mode == 1) {
          // MOVE RELATIVELY
          double pan_angle = this->current_pan_angle_ + request->panangle;
          double tilt_angle = this->current_tilt_angle_ + request->tiltangle;
          double grip_angle = this->current_grip_angle_ + request->gripangle;

          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, tilt_angle, pan_angle, grip_angle);
          this->cal_inverse_kinematics(pan_angle, tilt_angle, grip_angle);
          this->motor_control_publisher_->publish(this->motor_control_target_val_);
          this->surgical_tool_pose_publisher_->publish(this->surgical_tool_pose_);
        }
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Service <MoveToolAngle> accept the request");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
    
  };
  move_tool_angle_service_server_ = 
    create_service<MoveToolAngle>("kinematics/move_tool_angle", get_target_move_tool_angle);


  auto sine_wave_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // run
      if(request->data) {
        // True --> Start timer
        RCLCPP_INFO(this->get_logger(), "Starting sine wave publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&KinematicsControlNode::publish_sine_wave, this));
        } else {
          RCLCPP_WARN(this->get_logger(), "Error: sine wave motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Sine wave publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Sine wave publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <sine_wave> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  move_sine_wave_server_ = 
    create_service<std_srvs::srv::SetBool>("kinematics/move_sine_wave", sine_wave_callback);


  auto circle_motion_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // run
      if(request->data) {
        // True --> Start timer
        RCLCPP_INFO(this->get_logger(), "Starting Circle-motion publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&KinematicsControlNode::publish_circle_motion, this));
        } else {
          RCLCPP_WARN(this->get_logger(), "Error: circle motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Circle-motion publishing started.";
        
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Circle-motion publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <circle_motion> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  move_circle_motion_server_ = 
    create_service<std_srvs::srv::SetBool>("kinematics/move_circle_motion", circle_motion_callback);

  auto moebius_motion_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // run
      if(request->data) {
        // True --> Start timer
        RCLCPP_INFO(this->get_logger(), "Starting Circle-motion publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&KinematicsControlNode::publish_moebius_motion, this));
        }
        else {
          RCLCPP_WARN(this->get_logger(), "Error: moebius motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Moebius-motion publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Moebius-motion publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <circle_motion> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  move_moebius_motion_server_ = 
    create_service<std_srvs::srv::SetBool>("kinematics/move_moebius_motion", moebius_motion_callback);

  /**
   * @brief homing
   */
  // this->homingthread_ = std::thread(&KinematicsControlNode::homing, this);
}

KinematicsControlNode::~KinematicsControlNode() {

}

void KinematicsControlNode::cal_inverse_kinematics(double pAngle, double tAngle, double gAngle) {
  /* code */
  /* input : actual pos & actual velocity & controller input */
  /* output : target value*/

  this->current_pan_angle_ = pAngle;
  this->current_tilt_angle_ = tAngle;
  this->current_grip_angle_ = gAngle;
  this->surgical_tool_pose_.angular.y = tAngle * M_PI/180;
  this->surgical_tool_pose_.angular.z = pAngle * M_PI/180;
  this->ST_.get_bending_kinematic_result(this->current_pan_angle_, this->current_tilt_angle_, this->current_grip_angle_);

  // std::cout << this->current_pan_angle_ << std::endl;
  // std::cout << this->current_tilt_angle_ << std::endl;
  // std::cout << this->current_grip_angle_ << std::endl;

  double f_val[5];
  f_val[0] = this->ST_.wrLengthEast_;
  f_val[1] = this->ST_.wrLengthWest_;
  f_val[2] = this->ST_.wrLengthSouth_;
  f_val[3] = this->ST_.wrLengthNorth_;
  f_val[4] = this->ST_.wrLengthGrip;

  // std::cout << "--------------------------" << std::endl;
  // std::cout << "East  : " << f_val[0] << " mm" << std::endl;
  // std::cout << "West  : " << f_val[1] << " mm" << std::endl;
  // std::cout << "South : " << f_val[2] << " mm" << std::endl;
  // std::cout << "North : " << f_val[3] << " mm" << std::endl;
  // std::cout << "Grip  : " << f_val[4] << " mm" << std::endl;

  this->motor_control_target_val_.header.stamp = this->now();
  this->motor_control_target_val_.header.frame_id = "kinematics_motor_target_position";
  this->motor_control_target_val_.target_position[0] = DIRECTION_COUPLER * f_val[0] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->motor_control_target_val_.target_position[1] = DIRECTION_COUPLER * f_val[1] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  // this->motor_control_target_val_.target_position[0] = this->motor_state_.actual_position[0] + DIRECTION_COUPLER * f_val[0] * 2 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  // this->motor_control_target_val_.target_position[1] = this->motor_state_.actual_position[1] + DIRECTION_COUPLER * f_val[1] * 2 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  
  // this->motor_control_target_val_.target_position[2] = this->virtual_home_pos_[2]
  //                                                           + DIRECTION_COUPLER * f_val[2] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  // this->motor_control_target_val_.target_position[3] = this->virtual_home_pos_[3]
  //                                                           + DIRECTION_COUPLER * f_val[3] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  // this->motor_control_target_val_.target_position[4] = this->virtual_home_pos_[4]
  //                                                           + DIRECTION_COUPLER * f_val[4] * gear_encoder_ratio_conversion(GEAR_RATIO_3_9, ENCODER_CHANNEL, ENCODER_RESOLUTION);

#if MOTOR_CONTROL_SAME_DURATION
  /**
   * @brief find max value and make it max_velocity_profile 100 (%),
   *        other value have values proportional to 100 (%) each
   */
  static double prev_f_val[NUM_OF_MOTORS];  // for delta length

  std::vector<double> abs_f_val(NUM_OF_MOTORS-1, 0);  // 5th DOF is a forceps
  for (int i=0; i<NUM_OF_MOTORS-1; i++) { abs_f_val[i] = std::abs(this->motor_control_target_val_.target_position[i] - this->motor_state_.actual_position[i]); }

  double max_val = *std::max_element(abs_f_val.begin(), abs_f_val.end()) + 0.00001; // 0.00001 is protection for 0/0 (0 divided by 0)
  int max_val_index = std::max_element(abs_f_val.begin(), abs_f_val.end()) - abs_f_val.begin();
  for (int i=0; i<(NUM_OF_MOTORS-1); i++) { 
    this->motor_control_target_val_.target_velocity_profile[i] = (abs_f_val[i] / max_val) * PERCENT_100 * 0.5;
  }
  // last index means forceps. It doesn't need velocity profile
  this->motor_control_target_val_.target_velocity_profile[NUM_OF_MOTORS-1] = PERCENT_100 * 0.5;
  
#else
  for (int i=0; i<NUM_OF_MOTORS; i++) { 
    this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100 * 0.5;
  }
#endif
  // std::cout << "fin" <<std::endl;
}

double KinematicsControlNode::gear_encoder_ratio_conversion(double gear_ratio, int e_channel, int e_resolution) {
  return gear_ratio * e_channel * e_resolution;
}

void KinematicsControlNode::set_position_zero() {
  for (int i=0; i<NUM_OF_MOTORS; i++) {
    this->virtual_home_pos_[i] = 0;
  }
}

void KinematicsControlNode::publishall()
{

}

void KinematicsControlNode::publish_sine_wave()
{
  double omega = 2.0 * M_PI / period_;
  angle_ = amp_ * std::sin(omega * count_);
  cal_inverse_kinematics(angle_, angle_, 0);
  motor_control_publisher_->publish(motor_control_target_val_);
  surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
  count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
  std::cout << omega << " / " << amp_ << " / " << angle_ << " / " << count_ << " / " << count_add_ << std::endl;
}

void KinematicsControlNode::publish_circle_motion()
{
  double omega = 2.0 * M_PI / period_;
  double pan_deg = amp_ * std::sin(omega * count_);
  double tilt_deg = amp_ * std::cos(omega * count_);
  cal_inverse_kinematics(pan_deg, tilt_deg, 0);
  motor_control_publisher_->publish(motor_control_target_val_);
  surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
  count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
  std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
}

void KinematicsControlNode::publish_moebius_motion()
{
  double omega = 2.0 * M_PI / period_;
  double pan_deg = 0.5 * amp_ * std::sin((omega*2.0) * count_);
  double tilt_deg = amp_ * std::sin(omega * count_);
  cal_inverse_kinematics(pan_deg, tilt_deg, 0);
  motor_control_publisher_->publish(motor_control_target_val_);
  surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
  count_ += count_add_;
  std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
}