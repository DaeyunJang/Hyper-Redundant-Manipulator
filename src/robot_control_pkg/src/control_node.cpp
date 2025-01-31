#include "control_node.hpp"

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;
using namespace std::chrono_literals;

ControlNode::ControlNode(const rclcpp::NodeOptions & node_options)
: Node("ControlNode", node_options),
  control_mode_(ControlMode::kKinematics),
  hrm_controller_enable_(true),
  loop_rate_(SAMPLING_HZ)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  this->declare_parameter("control_mode", "kinematics");
  this->declare_parameter<double>("dynamics/p_gain", KP);
  this->declare_parameter<double>("dynamics/i_gain", KI);
  this->declare_parameter<double>("dynamics/d_gain", KD);
  this->declare_parameter<int>("dynamics/friction_mode", FRICTION_MODE);
  this->declare_parameter<bool>("dynamics/HRM_controller_enable", true);

  // 파라미터 변경 콜백 등록
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ControlNode::parameter_callback, this, std::placeholders::_1)
  );



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
  // dynamics parameters publisher
  //===============================
  // this->motor_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  // this->motor_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  // for(int i=0; i<NUM_OF_MOTORS; i++) {
  //   this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100/10;
  // }
  dynamic_MIMO_values_publisher_ = this->create_publisher<DynamicMIMOValues>("dynamic_MIMO_values", QoS_RKL10V);
  RCLCPP_INFO(this->get_logger(), "Publisher 'dynamic_MIMO_values' is created.");

  //===============================
  // surgical tool pose(degree) publisher
  //===============================
  surgical_tool_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("surgical_tool_pose", QoS_RKL10V);

  wire_length_publisher_ = 
    this->create_publisher<std_msgs::msg::Float32MultiArray>("wire_length", QoS_RKL10V);
  wire_length_velocity_publisher_ = 
    this->create_publisher<std_msgs::msg::Float32MultiArray>("wire_length_velocity", QoS_RKL10V);
  this->wire_length_.data.resize(NUM_OF_MOTORS);
  this->wire_length_velocity_.data.resize(NUM_OF_MOTORS);

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
          this->wire_length_velocity_.data[i] = this->motor_state_.actual_velocity[i] * 2 / gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
        }
        this->wire_length_publisher_->publish(this->wire_length_);
        this->wire_length_velocity_publisher_->publish(this->wire_length_velocity_);
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

  external_force_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Vector3>(
      "estimated_external_force",
      QoS_RKL10V,
      [this] (const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      {
        try {
          this->external_force_op_flag_ = true;
          this->external_force_.x = msg->x;
          this->external_force_.y = msg->y;
          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_external_force.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
      }
    );
  
  segment_angle_subscriber_ =
    this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "estimated_segment_angle/relative",
      QoS_RKL10V,
      [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) -> void
      {
        try {
          this->segment_angle_op_flag_ = true;
          this->segment_angle_.data = msg->data;

          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_segment_angle/relative.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
      }
    );
  segment_angle_absolute_subscriber_ =
    this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "estimated_segment_angle/absolute",
      QoS_RKL10V,
      [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) -> void
      {
        try {
          this->segment_angle_op_flag_ = true;
          this->segment_angle_absolute_.data = msg->data;

          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_segment_angle/absolute.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
      }
    );

  segment_angular_velocity_subscriber_ =
    this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "estimated_segment_angular_velocity/relative",
      QoS_RKL10V,
      [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) -> void
      {
        try {
          this->segment_angular_velocity_op_flag_ = true;
          this->segment_angular_velocity_.data = msg->data;

          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_segment_angle/relative.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
      }
    );

  segment_angular_velocity_absolute_subscriber_ =
    this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "estimated_segment_angular_velocity/absolute",
      QoS_RKL10V,
      [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) -> void
      {
        try {
          this->segment_angular_velocity_op_flag_ = true;
          this->segment_angular_velocity_absolute_.data = msg->data;

          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_segment_angle/absolute.");
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
        }
      }
    );






  /**********************************************************************
   * @brief service
   **********************************************************************/
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
    create_service<MoveMotorDirect>("move_motor_direct", get_target_move_motor_direct);

  auto kinematics_move_tool_angle = 
  [this](
  const std::shared_ptr<MoveToolAngle::Request> request,
  std::shared_ptr<MoveToolAngle::Response> response) -> void
  {
    try {
      // run
      if (control_mode_ != ControlMode::kKinematics) {
        RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
        return;
      }

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
        RCLCPP_INFO(this->get_logger(), "Service <kinematics/move_tool_angle> accept the request");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
    
  };
  kinematics_move_tool_angle_service_server_ = 
    create_service<MoveToolAngle>("kinematics/move_tool_angle", kinematics_move_tool_angle);


  auto sine_wave_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // run
      // if (control_mode_ != ControlMode::kKinematics) {
      //   RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
      //   return;
      // }

      if(request->data) {
        // True --> Start timer
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Starting sine wave publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&ControlNode::publish_sine_wave, this));
        } else {
          RCLCPP_WARN(this->get_logger(), "Error: sine wave motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Sine wave publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        count_ = 0;
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Sine wave publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <motion/move_sine_wave> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  move_sine_wave_server_ = 
    create_service<std_srvs::srv::SetBool>("motion/move_sine_wave", sine_wave_callback);


  auto sine_wave_1time_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // run
      // if (control_mode_ != ControlMode::kKinematics) {
      //   RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
      //   return;
      // }

      if(request->data) {
        // True --> Start timer
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Starting sine wave publishing.");
        if (timer_ == nullptr) {
          count_ = 0;
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&ControlNode::publish_sine_wave_1time, this));
        } else {
          RCLCPP_WARN(this->get_logger(), "Error: sine wave motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Sine wave publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        count_ = 0;
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Sine wave publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <motion/move_sine_wave_1time> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  move_sine_wave_1time_server_ = 
    create_service<std_srvs::srv::SetBool>("motion/move_sine_wave_1time", sine_wave_1time_callback);


  auto circle_motion_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      if (control_mode_ != ControlMode::kKinematics) {
        RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
        return;
      }

      // run
      if(request->data) {
        // True --> Start timer
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Starting Circle-motion publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&ControlNode::publish_circle_motion, this));
        } else {
          RCLCPP_WARN(this->get_logger(), "Error: circle motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Circle-motion publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        count_ = 0;
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Circle-motion publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <kinematics/move_circle_motion> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  kinematics_move_circle_motion_server_ = 
    create_service<std_srvs::srv::SetBool>("kinematics/move_circle_motion", circle_motion_callback);

  auto moebius_motion_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      if (control_mode_ != ControlMode::kKinematics) {
        RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
        return;
      }

      // run
      if(request->data) {
        // True --> Start timer
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Starting Circle-motion publishing.");
        if (timer_ == nullptr) {
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&ControlNode::publish_moebius_motion, this));
        }
        else {
          RCLCPP_WARN(this->get_logger(), "Error: moebius motion is operating. Please stop using [ros2 service call]");
        }
        response->success = true;
        response->message = "Moebius-motion publishing started.";
      } else {
        // 서비스 요청이 False일 때 타이머 중지
        count_ = 0;
        if (timer_ != nullptr) {
            timer_->cancel();
            timer_ = nullptr;
        }
        response->success = true;
        response->message = "Moebius-motion publishing stopped.";
      }

      RCLCPP_INFO(this->get_logger(), "Service <kinematics/move_moebius_motion> accept the request.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  kinematics_move_moebius_motion_server_ = 
    create_service<std_srvs::srv::SetBool>("kinematics/move_moebius_motion", moebius_motion_callback);




  auto control_mode_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
      // requst->data : true-Dynamics, false-Kinematics
      if(request->data) {
        // true : Dynamics
        RCLCPP_INFO(this->get_logger(), "Control mode changed -> Dynamics.");
        this->set_parameter(rclcpp::Parameter("control_mode", "dynamics"));
        response->success = true;
        response->message = "Control mode changed -> Dynamics.";
      } else {
        // false : Kinematics
        RCLCPP_INFO(this->get_logger(), "Control mode changed -> Kinematics.");
        this->set_parameter(rclcpp::Parameter("control_mode", "kinematics"));
        response->success = true;
        response->message = "Control mode changed -> Kinematics.";
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  control_mode_service_server_ = 
    create_service<std_srvs::srv::SetBool>("control/control_mode", control_mode_callback);

  auto dynamics_move_tool_angle = 
  [this](
  const std::shared_ptr<MoveToolAngle::Request> request,
  std::shared_ptr<MoveToolAngle::Response> response) -> void
  {
    try {
      // run
      if (control_mode_ != ControlMode::kDynamics) {
        RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'DYNAMICS\' mode. Change parameter \'control_mode\'.");
        return;
      }
      if(this->op_mode_ == kEnable) {
        if(request->mode == 0) {
          // MOVE ABSOLUTELY
          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, request->tiltangle, request->panangle, request->gripangle);
          double pan_angle =  request->panangle;
          double tilt_angle = request->tiltangle;
          double grip_angle = request->gripangle;
          this->theta_desired_ = pan_angle;
          RCLCPP_INFO(this->get_logger(), "Dynamics mode, target theta_desired: %.2f", this->theta_desired_);
        }
        else if (request->mode == 1) {
          // MOVE RELATIVELY
          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, request->tiltangle, request->panangle, request->gripangle);
          double pan_angle =  this->current_pan_angle_ + request->panangle;
          double tilt_angle = this->current_tilt_angle_ + request->tiltangle;
          double grip_angle = this->current_grip_angle_ + request->gripangle;
          this->theta_desired_ = pan_angle;
          RCLCPP_INFO(this->get_logger(), "Dynamics mode, target theta_desired: %.2f", this->theta_desired_);
        }
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Service <dynamics/move_tool_angle> accept the request");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
    
  };
  dynamics_move_tool_angle_service_server_ = 
    create_service<MoveToolAngle>("dynamics/move_tool_angle", dynamics_move_tool_angle);



  // opertion thread which kinematics and dynamics
  dynamic_control_thread_ = std::thread(&ControlNode::run_dynamic_control_thread, this);
  /**
   * @brief homing
   */
  // this->homingthread_ = std::thread(&ControlNode::homing, this);
}

ControlNode::~ControlNode() {
  if (dynamic_control_thread_.joinable()) {
    dynamic_control_thread_.join();
  }
}

void ControlNode::cal_inverse_kinematics(double pAngle, double tAngle, double gAngle) {
  /* code */
  /* input : actual pos & actual velocity & controller input */
  /* output : target value*/
  this->current_pan_angle_ = pAngle;
  this->current_tilt_angle_ = tAngle;
  this->current_grip_angle_ = gAngle;
  this->surgical_tool_pose_.angular.y = tAngle * M_PI/180;
  this->surgical_tool_pose_.angular.z = pAngle * M_PI/180;
  this->HRM_controller_.surgical_tool_.get_IK_result(this->current_pan_angle_, this->current_tilt_angle_, this->current_grip_angle_);
  // this->ST_.get_IK_result(this->current_pan_angle_, this->current_tilt_angle_, this->current_grip_angle_);

  double f_val[5];
  f_val[0] = this->HRM_controller_.surgical_tool_.wrLengthEast_;
  f_val[1] = this->HRM_controller_.surgical_tool_.wrLengthWest_;
  f_val[2] = this->HRM_controller_.surgical_tool_.wrLengthSouth_;
  f_val[3] = this->HRM_controller_.surgical_tool_.wrLengthNorth_;
  f_val[4] = this->HRM_controller_.surgical_tool_.wrLengthGrip;

  for (int i=0; i<5; i++)
  {
    std::cout<< "f_val" << i << ": " << f_val[i] << std::endl;
  }
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

double ControlNode::gear_encoder_ratio_conversion(double gear_ratio, int e_channel, int e_resolution) {
  return gear_ratio * e_channel * e_resolution;
}

void ControlNode::set_position_zero() {
  for (int i=0; i<NUM_OF_MOTORS; i++) {
    this->virtual_home_pos_[i] = 0;
  }
}

void ControlNode::publishall()
{

}

void ControlNode::publish_sine_wave()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    angle_ = amp_ * std::sin(omega * count_);
    cal_inverse_kinematics(-angle_, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    std::cout << omega << " / " << amp_ << " / " << angle_ << " / " << count_ << " / " << count_add_ << std::endl;
  }
  else if (control_mode_ == ControlMode::kDynamics) {
    double omega = 2.0 * M_PI / period_;
    angle_ = amp_ * std::sin(omega * count_);
    this->theta_desired_ = -angle_;
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_ << " / " << angle_ << " / " << count_ << " / " << count_add_ << std::endl;
  }
}

void ControlNode::publish_sine_wave_1time()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    angle_ = amp_ * std::sin(omega * count_);
    cal_inverse_kinematics(-angle_, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    std::cout << omega << " / " << amp_ << " / " << angle_ << " / " << count_ << " / " << count_add_ << std::endl;
    
    if (count_ >= period_) {
      count_ = 0;
      timer_->cancel();
      timer_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Sine wave cycle completed. Timer stopped.");
    }
  }
  else if (control_mode_ == ControlMode::kDynamics) {
    double omega = 2.0 * M_PI / period_;
    angle_ = amp_ * std::sin(omega * count_);
    this->theta_desired_ = -angle_; // +90 ~ -90
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_ << " / " << angle_ << " / " << count_ << " / " << count_add_ << std::endl;
    
    if (count_ >= period_) {
      count_ = 0;
      timer_->cancel();
      timer_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Sine wave cycle completed. Timer stopped.");
    }
  }
}

void ControlNode::publish_circle_motion()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    double pan_deg = amp_ * std::sin(omega * count_);
    double tilt_deg = amp_ * std::cos(omega * count_);
    cal_inverse_kinematics(pan_deg, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
  }
}

void ControlNode::publish_moebius_motion()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    double pan_deg = 0.5 * amp_ * std::sin((omega*2.0) * count_);
    double tilt_deg = amp_ * std::sin(omega * count_);
    cal_inverse_kinematics(pan_deg, tilt_deg, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;
    std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
  }
}

rcl_interfaces::msg::SetParametersResult ControlNode::parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
  for (const auto &param : parameters) {
    if (param.get_name() == "control_mode") {
      if (param.as_string() == "kinematics") {
        control_mode_ = ControlMode::kKinematics;
        RCLCPP_INFO(this->get_logger(), "Switched to KINEMATICS mode");
      } else if (param.as_string() == "dynamics") {
        control_mode_ = ControlMode::kDynamics;
        RCLCPP_INFO(this->get_logger(), "Switched to DYNAMICS mode");
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown mode. Keeping previous mode.");
      }
    // }
    } else if (param.get_name() == "dynamics/p_gain") {
      double p_gain = param.as_double();
      HRM_controller_.pid_controller_.kp_ = p_gain;
      RCLCPP_INFO(this->get_logger(), "Updated P gain: %f", p_gain);
    } else if (param.get_name() == "dynamics/i_gain") {
      double i_gain = param.as_double();
      HRM_controller_.pid_controller_.ki_ = i_gain;
      RCLCPP_INFO(this->get_logger(), "Updated I gain: %f", i_gain);
    } else if (param.get_name() == "dynamics/d_gain") {
      double d_gain = param.as_double();
      HRM_controller_.pid_controller_.kd_ = d_gain;
      RCLCPP_INFO(this->get_logger(), "Updated D gain: %f", d_gain);
    } else if (param.get_name() == "dynamics/friction_mode") {
      int friction_mode = param.as_int();
      HRM_controller_.damping_friction_model_.mode_ = friction_mode;
      RCLCPP_INFO(this->get_logger(), "Updated friction mode: %d", HRM_controller_.damping_friction_model_.mode_);
    } else if (param.get_name() == "dynamics/HRM_controller_enable") {
      bool enable = param.as_bool();
      HRM_controller_.hrm_controller_enable_ = enable;
      RCLCPP_INFO(this->get_logger(), "Updated friction mode: %d", HRM_controller_.hrm_controller_enable_);
    }


  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void ControlNode::run_dynamic_control_thread() {
  RCLCPP_INFO(this->get_logger(), "control_thread is started on [%s]", this->get_parameter("control_mode").as_string().c_str());

  while (rclcpp::ok()) {
    if (control_mode_ == ControlMode::kDynamics) {
      try {
        // run dynamics() code
        /***
         * @warning
         * optimazation for memory based on avoiding memory copy
         * Y.J. Kim's kinematics is defined that CW direction is positive and CCW is negative.
         * But DY defined opposite.
         * affect to 'theta_desired', 'tension'
         * @param loop_late_
         * loop_late_ is the sampling rate of the controller 
         * loop_late = global variable SAMPLING_HZ @include '../include/dynamics_parameters.hpp' 
         */
        // double theta_desired = this->theta_desired_ * this->HRM_controller_.surgical_tool_.torad();
        double theta_desired = (-1) * this->theta_desired_ * this->HRM_controller_.surgical_tool_.torad();
        
        // if using std::vector<double> a = msg.data
        // The type must be conversion from float(msg.data) to double
        std::vector<double> theta_actual(this->segment_angle_.data.begin(), this->segment_angle_.data.end());
        std::vector<double> omega_actual(this->segment_angular_velocity_.data.begin(), this->segment_angular_velocity_.data.end());

        //************************** */
        // End-effector theta

        // relative
        // double end_effector_theta_actual = std::accumulate(theta_actual.begin(), theta_actual.end(), 0.0);
        // double end_effector_omega_actual = std::accumulate(omega_actual.begin(), omega_actual.end(), 0.0);

        // absolute
        double alpha = 0.5;
        double angle_filtered = 0;
        if (!segment_angle_absolute_.data.empty() && !segment_angle_absolute_prev_.data.empty()) {
          angle_filtered = alpha*segment_angle_absolute_.data.back() + (1-alpha)*segment_angle_absolute_prev_.data.back();
        } else {
          segment_angle_absolute_prev_.data = segment_angle_absolute_.data;
        }
        segment_angle_absolute_prev_.data = segment_angle_absolute_.data;

        double end_effector_theta_actual = angle_filtered;
        double end_effector_omega_actual = segment_angular_velocity_absolute_.data.back();

        //************************** */
        double dt = DT;
        std::vector<double> tension = {this->loadcell_data_.stress[1]*0.001, this->loadcell_data_.stress[0]*0.001}; // g -> kg, Y.J. kiniematics is positive on CW.
        std::vector<double> external_force = {this->external_force_.x*0.001, this->external_force_.y*0.001};

        // test -> no payload
        // external_force[0] = 0;
        // external_force[1] = 0;
        
        double cable_vel_left = this->wire_length_velocity_.data[1] * 0.001;  // mm -> m
        double cable_vel_right = this->wire_length_velocity_.data[0] * 0.001; // mm -> m
        std::vector<double> cable_velocity = {cable_vel_left, cable_vel_right};

        auto wire_length_to_move = this->HRM_controller_.compute(
          theta_desired,
          end_effector_theta_actual,
          end_effector_omega_actual,
          theta_actual,
          omega_actual,
          cable_velocity,
          dt,
          tension,
          external_force,
          this->HRM_controller_.hrm_controller_enable_);

        // publish dynamic MIMO values
        if (this->segment_angle_op_flag_ == true) {
          // 데이터 생성
          dynamic_MIMO_values_.header.stamp = this->get_clock()->now();
          dynamic_MIMO_values_.header.frame_id = "dynamics_MIMO_values";
          dynamic_MIMO_values_.sampling_time = dt; // 초 단위 변환
          dynamic_MIMO_values_.p_gain = HRM_controller_.pid_controller_.kp_;
          dynamic_MIMO_values_.i_gain = HRM_controller_.pid_controller_.ki_;
          dynamic_MIMO_values_.d_gain = HRM_controller_.pid_controller_.kd_;
          dynamic_MIMO_values_.hrm_controller_enable = HRM_controller_.hrm_controller_enable_;
          dynamic_MIMO_values_.theta_desired = theta_desired; // 예제 데이터
          dynamic_MIMO_values_.theta_actual = HRM_controller_.end_effector_theta_actual_;    // 약간의 오차 추가
          dynamic_MIMO_values_.omega_actual = HRM_controller_.end_effector_dtheta_dt_actual_;  // 예제 데이터
          dynamic_MIMO_values_.tension = tension;                          // 예제 데이터
          dynamic_MIMO_values_.cable_velocity_left = cable_velocity[0];
          dynamic_MIMO_values_.cable_velocity_right = cable_velocity[1];
          dynamic_MIMO_values_.torque_input = HRM_controller_.torque_input_;
          dynamic_MIMO_values_.external_force = external_force;                   // 예제 데이터
          dynamic_MIMO_values_.external_torque = HRM_controller_.tau_ext_;
          dynamic_MIMO_values_.friction_mode = HRM_controller_.damping_friction_model_.mode_;
          dynamic_MIMO_values_.friction_torque = HRM_controller_.tau_friction_;
          dynamic_MIMO_values_.damping_coefficient = HRM_controller_.damping_friction_model_.B_;
          dynamic_MIMO_values_.res_friction = HRM_controller_.damping_friction_model_.res_friction_;
          dynamic_MIMO_values_.cmode = HRM_controller_.damping_friction_model_.cmode_;
          dynamic_MIMO_values_.input_alpha = HRM_controller_.theta_ddot_input_;
          dynamic_MIMO_values_.input_omega = HRM_controller_.theta_dot_input_;
          dynamic_MIMO_values_.input_theta = HRM_controller_.theta_input_;

          dynamic_MIMO_values_publisher_->publish(dynamic_MIMO_values_);
          this->segment_angle_op_flag_ = false;
        }

        double f_val[5];
        f_val[0] = wire_length_to_move[0];  // East
        f_val[1] = wire_length_to_move[1];  // West
        f_val[2] = wire_length_to_move[2];  // South
        f_val[3] = wire_length_to_move[3];  // North
        f_val[4] = wire_length_to_move[4];  // grip

        this->motor_control_target_val_.header.stamp = this->now();
        this->motor_control_target_val_.header.frame_id = "motor_target_position";

        if (this->loadcell_data_.stress[0] < TENSION_LIMIT && this->loadcell_data_.stress[1] < TENSION_LIMIT) {
          this->motor_control_target_val_.target_position[0] = DIRECTION_COUPLER * f_val[0] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          this->motor_control_target_val_.target_position[1] = DIRECTION_COUPLER * f_val[1] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
        }
        else {// for prevent wire cut off 
          for (int i=0; i<NUM_OF_MOTORS; i++) {
            this->motor_control_target_val_.target_position[i] = this->motor_state_.actual_position[i];
          }
        }
        

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
          int target_vel_profile = int(std::round(std::abs(HRM_controller_.theta_dot_input_)));
          // prevent velocity 0
          if (target_vel_profile < 20) {
            target_vel_profile = 20;
          }
          for (int i=0; i<NUM_OF_MOTORS; i++) { 
            // this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100 * 0.5;
            this->motor_control_target_val_.target_velocity_profile[i] = std::min(target_vel_profile, 80);
          }
        #endif
        
        this->motor_control_publisher_->publish(this->motor_control_target_val_);

        geometry_msgs::msg::Twist surgical_tool_pose;
        surgical_tool_pose.angular.z = theta_desired;
        // surgical_tool_pose.angular.y = theta_desired;
        this->surgical_tool_pose_publisher_->publish(surgical_tool_pose);

        


        loop_rate_.sleep();
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
      }
    } else {  // kinematics
      //
    }
  }
}