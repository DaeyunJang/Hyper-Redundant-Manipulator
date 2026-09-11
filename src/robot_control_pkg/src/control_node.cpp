#include "control_node.hpp"

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;
using namespace std::chrono_literals;

ControlNode::ControlNode(const rclcpp::NodeOptions & node_options)
: Node("ControlNode", node_options),
  control_mode_(ControlMode::kKinematics),
  hrm_controller_enable_(true),
  del_f_(Eigen::VectorXd::Zero(6)),
  f_desired_(Eigen::VectorXd::Zero(6)),
  f_env_(Eigen::VectorXd::Zero(6)),
  del_xf_(Eigen::VectorXd::Zero(6)),
  x_t_(Eigen::VectorXd::Zero(6)),
  x_desired_(Eigen::VectorXd::Zero(6)),
  x_actual_(Eigen::VectorXd::Zero(6)),
  loop_rate_dynamics_(dynamics_params::SAMPLING_HZ),
  loop_rate_position_with_admittance_(position_control_params::SAMPLING_HZ)
{
  const int qos_depth = std::max(
    1, static_cast<int>(this->declare_parameter<int>("qos_depth", 1)));
  const int initial_control_mode = static_cast<int>(
    this->declare_parameter<int>(
      "control_mode", ControlMode::kKinematics));
  if (initial_control_mode >= ControlMode::kKinematics &&
      initial_control_mode <= ControlMode::kAdmittance)
  {
    control_mode_ = static_cast<ControlMode>(initial_control_mode);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid initial control_mode=%d; using kinematics.",
      initial_control_mode);
  }
  std::cout << "------------------------------------" <<std::endl;
  std::cout << "control_mode_: " << control_mode_ << std::endl;
  std::cout << "------------------------------------" <<std::endl;
  // dynamics controller
  this->declare_parameter<double>("dynamics/p_gain", dynamics_params::KP);
  this->declare_parameter<double>("dynamics/i_gain", dynamics_params::KI);
  this->declare_parameter<double>("dynamics/d_gain", dynamics_params::KD);
  this->declare_parameter<bool>("dynamics/HRM_controller_enable", true);

  // admittance controller
  this->declare_parameter<double>("admittance/M_d", admittance_params::M_d);
  this->declare_parameter<double>("admittance/B_d", admittance_params::B_d);
  this->declare_parameter<double>("admittance/K_d", admittance_params::K_d);

  // position controller
  this->declare_parameter<double>("position_control/pid_controller_pan/p_gain", position_control_params::KP);
  this->declare_parameter<double>("position_control/pid_controller_pan/i_gain", position_control_params::KI);
  this->declare_parameter<double>("position_control/pid_controller_pan/d_gain", position_control_params::KD);

  // 파라미터 변경 콜백 등록
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ControlNode::parameter_callback, this, std::placeholders::_1)
  );

  const auto qos_reliable_latest =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  fk_tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  //===============================
  // target value publisher
  //===============================
  this->motor_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  this->motor_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  for(int i=0; i<NUM_OF_MOTORS; i++) {
    this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100;
  }
  motor_control_publisher_ = this->create_publisher<MotorCommand>("motor_command", qos_reliable_latest);
  RCLCPP_INFO(this->get_logger(), "Publisher 'motor_command' is created.");
  
  //===============================
  // dynamics parameters publisher
  //===============================
  // this->motor_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  // this->motor_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  // for(int i=0; i<NUM_OF_MOTORS; i++) {
  //   this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100/10;
  // }
  dynamic_MIMO_values_publisher_ = this->create_publisher<DynamicMIMOValues>("dynamic_MIMO_values", qos_reliable_latest);
  RCLCPP_INFO(this->get_logger(), "Publisher 'dynamic_MIMO_values' is created.");

  admittance_control_msgs_publisher_ = this->create_publisher<AdmittanceControl>("admittance_controller", qos_reliable_latest);
  RCLCPP_INFO(this->get_logger(), "Publisher 'admittance_controller' is created.");

  position_control_msgs_publisher_ = this->create_publisher<PositionControl>("position_controller", qos_reliable_latest);
  RCLCPP_INFO(this->get_logger(), "Publisher 'position_controller' is created.");

  control_mode_msgs_publisher_ = this->create_publisher<std_msgs::msg::String>("control_mode", qos_reliable_latest);
  RCLCPP_INFO(this->get_logger(), "Publisher 'control_mode' is created.");
  // publish control mode
  control_mode_msgs_.data = ControlModeToString(this->control_mode_);
  control_mode_msgs_publisher_->publish(control_mode_msgs_);
  
  //===============================
  // surgical tool pose(degree) publisher
  //===============================
  surgical_tool_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("surgical_tool_pose", qos_reliable_latest);
  tool_endeffector_pose_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("tool_endeffector_pose", qos_reliable_latest);
  wire_length_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("wire_length", qos_reliable_latest);
  wire_length_velocity_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("wire_length_velocity", qos_reliable_latest);
  this->tool_endeffector_pose_.data.resize(3);
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
      qos_reliable_latest,
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
      qos_reliable_latest,
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
      qos_reliable_latest,
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


    //===============================
  this->motor_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  this->motor_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  for(int i=0; i<NUM_OF_MOTORS; i++) {
    this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100*0.7;
  }


  this->segment_angle_.pan_relative.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.pan_absolute.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.tilt_relative.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.tilt_absolute.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.pan_angular_velocity_relative.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.pan_angular_velocity_absolute.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.tilt_angular_velocity_relative.resize(NUM_OF_BENDING_JOINTS, 0.0);
  this->segment_angle_.tilt_angular_velocity_absolute.resize(NUM_OF_BENDING_JOINTS, 0.0);

  segment_angle_subscriber_ =
    this->create_subscription<SegmentAngle>(
      "estimated_segment_angle",
      qos_reliable_latest,
      [this] (const SegmentAngle::SharedPtr msg) -> void
      {
        try {
          const auto joint_count =
            static_cast<std::size_t>(NUM_OF_BENDING_JOINTS);
          if (msg->pan_relative.size() != joint_count ||
              msg->pan_absolute.size() != joint_count ||
              msg->tilt_relative.size() != joint_count ||
              msg->tilt_absolute.size() != joint_count ||
              msg->pan_angular_velocity_relative.size() != joint_count ||
              msg->pan_angular_velocity_absolute.size() != joint_count ||
              msg->tilt_angular_velocity_relative.size() != joint_count ||
              msg->tilt_angular_velocity_absolute.size() != joint_count)
          {
            RCLCPP_WARN(
              this->get_logger(),
              "Ignoring estimated_segment_angle: every array must contain %d joints.",
              NUM_OF_BENDING_JOINTS);
            return;
          }

          {
            std::lock_guard<std::mutex> lock(this->segment_angle_mutex_);
            this->segment_angle_ = *msg;
            this->segment_angle_op_flag_ = true;
          }
          const auto fk_transforms =
            this->HRM_position_controller_.surgical_tool_.
            computeBaseToJointsTransformationMatrices(
              msg->pan_relative, msg->tilt_relative);
          this->publish_fk_transforms(
            msg->header.stamp,
            msg->header.frame_id.empty() ? "hrm_base" : msg->header.frame_id,
            fk_transforms);
          RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /estimated_segment_angle.");
        } catch (const std::exception & e) {
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

  //
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


  auto set_goal_position = 
  [this](
  const std::shared_ptr<SetGoalPosition::Request> request,
  std::shared_ptr<SetGoalPosition::Response> response) -> void
  {
    try {
      // run
      if(this->op_mode_ == kEnable) {
        if(request->reference_type == "absolute") {
          // MOVE absolute
          RCLCPP_INFO(
            this->get_logger(),
            "Received goal position: x: %.2f, y: %.2f, z: %.2f MODE: Absolute,", 
            request->goal_position.position.x,
            request->goal_position.position.y,
            request->goal_position.position.z);
          this->x_desired_ <<
            request->goal_position.position.x,
            request->goal_position.position.y,
            request->goal_position.position.z,
            0.0, 0.0, 0.0;  // 나머지 값은 기본값 0으로 설정
        } else if (request->reference_type == "relative") {
          // MOVE relative
          RCLCPP_INFO(
            this->get_logger(),
            "Received goal position: x: x+%.2f, y: y+%.2f, z: z+%.2f MODE: Relative,", 
            request->goal_position.position.x,
            request->goal_position.position.y,
            request->goal_position.position.z);
          Eigen::VectorXd delta_x(6);
          delta_x <<
            request->goal_position.position.x,
            request->goal_position.position.y,
            request->goal_position.position.z,
            0.0, 0.0, 0.0;  // 나머지 값은 기본값 0으로 설정
          this->x_desired_ += delta_x;
        }
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Service <position/set_goal_position> accept the request");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
    
  };
  set_goal_position_service_server_ = 
    create_service<SetGoalPosition>("position/set_goal_position", set_goal_position);


  auto sine_wave_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    try {
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

  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "service client DYDDYDYDYDYDY" << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  /**
   * @date 2025.04.10
   * @author DY
   * @brief change control mode
   * @note callback from gui (service call)
   * @param request->data (int8) followed enum 'ControlNode' (control_node.hpp)
   * kinematics=1
   * dynamics=2
   * position=3
   * admittance=4
   * another mode (TBD)
   */
  auto set_control_mode_callback = 
  [this](
  const std::shared_ptr<SetControlMode::Request> request,
        std::shared_ptr<SetControlMode::Response> response) -> void
  {
    try {
      RCLCPP_INFO(this->get_logger(), "Control mode changed -> %d.", request->mode);
      this->set_parameter(rclcpp::Parameter("control_mode", request->mode));
      response->success = true;
      response->message = "Control mode changed -> %d.", request->mode;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
    }
  };
  set_control_mode_service_server_ = 
    create_service<SetControlMode>("control/set_control_mode", set_control_mode_callback);

  // /**
  //  * @brief change mode between kinematics and dynamics
  //  * @note callback from gui (service call)
  //  */
  // auto control_mode_change_between_kinematics_and_dynamics_callback = 
  // [this](
  // const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  //       std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  // {
  //   try {
  //     // requst->data : true-Dynamics, false-Kinematics
  //     if(request->data) {
  //       // true : Dynamics
  //       RCLCPP_INFO(this->get_logger(), "Control mode changed -> Dynamics.");
  //       this->set_parameter(rclcpp::Parameter("control_mode", "dynamics"));
  //       response->success = true;
  //       response->message = "Control mode changed -> Dynamics.";
  //     } else {
  //       // false : Kinematics
  //       RCLCPP_INFO(this->get_logger(), "Control mode changed -> Kinematics.");
  //       this->set_parameter(rclcpp::Parameter("control_mode", "kinematics"));
  //       response->success = true;
  //       response->message = "Control mode changed -> Kinematics.";
  //     }
  //   } catch (const std::exception & e) {
  //     RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
  //   }
  // };
  // control_mode_change_between_kinematics_and_dynamics_service_server_ = 
  //   create_service<std_srvs::srv::SetBool>("control/control_mode_kin_dyn", control_mode_change_between_kinematics_and_dynamics_callback);

  // /**
  //  * @brief change mode between kinematics and admittance
  //  * @note callback from gui (service call)
  //  */
  // auto control_mode_change_between_position_and_admittance_callback = 
  // [this](
  // const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  //       std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  // {
  //   try {
  //     // requst->data : true-Dynamics, false-Kinematics
  //     if(request->data) {
  //       // true : Dynamics
  //       RCLCPP_INFO(this->get_logger(), "Control mode changed -> Admittance.");
  //       this->set_parameter(rclcpp::Parameter("control_mode", "admittance"));
  //       response->success = true;
  //       response->message = "Control mode changed -> Admittance.";
  //     } else {
  //       // false : Kinematics
  //       RCLCPP_INFO(this->get_logger(), "Control mode changed -> Position.");
  //       this->set_parameter(rclcpp::Parameter("control_mode", "position"));
  //       response->success = true;
  //       response->message = "Control mode changed -> Position.";
  //     }
  //   } catch (const std::exception & e) {
  //     RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
  //   }
  // };
  // control_mode_change_between_position_and_admittance_service_server_ = 
  //   create_service<std_srvs::srv::SetBool>("control/control_mode_pos_admit", control_mode_change_between_position_and_admittance_callback);



  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "Threads start..." << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

  // opertion thread which kinematics, dynamics and admittance
  dynamic_control_thread_ = std::thread(&ControlNode::run_dynamic_control_thread, this);
  position_with_admittance_control_thread_ = std::thread(&ControlNode::run_position_with_admittance_control_thread, this);
  /**
   * @brief homing
   */
  // this->homingthread_ = std::thread(&ControlNode::homing, this);
}

ControlNode::~ControlNode() {
  if (dynamic_control_thread_.joinable()) {
    dynamic_control_thread_.join();
  }
  if (position_with_admittance_control_thread_.joinable()) {
    position_with_admittance_control_thread_.join();
  }
}

void ControlNode::publish_fk_transforms(
  const builtin_interfaces::msg::Time& stamp,
  const std::string& base_frame_id,
  const std::vector<Eigen::Matrix4d>& transforms)
{
  std::vector<geometry_msgs::msg::TransformStamped> messages;
  messages.reserve(transforms.size() + 1);

  for (std::size_t index = 0; index < transforms.size(); ++index) {
    const Eigen::Matrix4d& transform = transforms[index];
    Eigen::Quaterniond quaternion(transform.block<3, 3>(0, 0));
    quaternion.normalize();

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_id;
    message.child_frame_id = "hrm_fk_joint_";
    if (index + 1 < 10) {
      message.child_frame_id += "0";
    }
    message.child_frame_id += std::to_string(index + 1);
    message.transform.translation.x = transform(0, 3);
    message.transform.translation.y = transform(1, 3);
    message.transform.translation.z = transform(2, 3);
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    messages.push_back(message);
  }

  const Eigen::Matrix4d tip_transform =
    this->HRM_position_controller_.surgical_tool_.
    computeEndEffectorTransformation(transforms);
  Eigen::Quaterniond tip_quaternion(tip_transform.block<3, 3>(0, 0));
  tip_quaternion.normalize();

  geometry_msgs::msg::TransformStamped tip_message;
  tip_message.header.stamp = stamp;
  tip_message.header.frame_id = base_frame_id;
  tip_message.child_frame_id = "hrm_fk_tip";
  tip_message.transform.translation.x = tip_transform(0, 3);
  tip_message.transform.translation.y = tip_transform(1, 3);
  tip_message.transform.translation.z = tip_transform(2, 3);
  tip_message.transform.rotation.x = tip_quaternion.x();
  tip_message.transform.rotation.y = tip_quaternion.y();
  tip_message.transform.rotation.z = tip_quaternion.z();
  tip_message.transform.rotation.w = tip_quaternion.w();
  messages.push_back(tip_message);

  this->fk_tf_broadcaster_->sendTransform(messages);
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
    // std::cout<< "f_val" << i << ": " << f_val[i] << std::endl;
  }
  this->motor_control_target_val_.header.stamp = this->now();
  this->motor_control_target_val_.header.frame_id = "kinematics_motor_target_position";
  this->motor_control_target_val_.target_position[0] = DIRECTION_COUPLER * f_val[0] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->motor_control_target_val_.target_position[1] = DIRECTION_COUPLER * f_val[1] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->motor_control_target_val_.target_position[2] = DIRECTION_COUPLER * f_val[2] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->motor_control_target_val_.target_position[3] = DIRECTION_COUPLER * f_val[3] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
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
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    cal_inverse_kinematics(trajectory_, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_deg_ << " / " << trajectory_ << " / " << count_ << " / " << count_add_ << std::endl;
  }
  else if (control_mode_ == ControlMode::kDynamics) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    this->theta_desired_ = trajectory_;
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_deg_ << " / " << trajectory_ << " / " << count_ << " / " << count_add_ << std::endl;
  }
  else if (control_mode_ == ControlMode::kPosition || control_mode_ == ControlMode::kAdmittance) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_mm_ * std::sin(omega * count_);
    Eigen::VectorXd x_target = Eigen::VectorXd::Zero(6);
    x_target(1) = trajectory_;
    this->x_desired_ = x_target;
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
  }

}

void ControlNode::publish_sine_wave_1time()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    cal_inverse_kinematics(trajectory_, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_deg_ << " / " << trajectory_ << " / " << count_ << " / " << count_add_ << std::endl;
    
    if (count_ >= period_) {
      count_ = 0;
      timer_->cancel();
      timer_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Sine wave cycle completed. Timer stopped.");
    }
  }
  else if (control_mode_ == ControlMode::kDynamics) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    this->theta_desired_ = trajectory_; // +90 ~ -90
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << omega << " / " << amp_deg_ << " / " << trajectory_ << " / " << count_ << " / " << count_add_ << std::endl;
    
    if (count_ >= period_) {
      count_ = 0;
      timer_->cancel();
      timer_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Sine wave cycle completed. Timer stopped.");
    }
  }
  else if (control_mode_ == ControlMode::kPosition || control_mode_ == ControlMode::kAdmittance) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_mm_ * std::sin(omega * count_);
    Eigen::VectorXd x_target = Eigen::VectorXd::Zero(6);
    x_target(1) = trajectory_;
    this->x_desired_ = x_target;
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    
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
    double pan_deg = amp_deg_ * std::sin(omega * count_);
    double tilt_deg = amp_deg_ * std::cos(omega * count_);
    cal_inverse_kinematics(pan_deg, 0, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;  // 각도를 증가시켜 사인파를 만듦
    // std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
  }
}

void ControlNode::publish_moebius_motion()
{
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    double pan_deg = 0.5 * amp_deg_ * std::sin((omega*2.0) * count_);
    double tilt_deg = amp_deg_ * std::sin(omega * count_);
    cal_inverse_kinematics(pan_deg, tilt_deg, 0);
    motor_control_publisher_->publish(motor_control_target_val_);
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;
    // std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
  }
}

rcl_interfaces::msg::SetParametersResult ControlNode::parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
  for (const auto &param : parameters) {
    if (param.get_name() == "control_mode") {
      if (param.as_int() == ControlMode::kKinematics) {
        control_mode_ = ControlMode::kKinematics;
        RCLCPP_INFO(this->get_logger(), "Switched to KINEMATICS mode");
      } else if (param.as_int() == ControlMode::kDynamics) {
        control_mode_ = ControlMode::kDynamics;
        RCLCPP_INFO(this->get_logger(), "Switched to DYNAMICS mode");
      } else if (param.as_int() == ControlMode::kPosition) {
        control_mode_ = ControlMode::kPosition;
        RCLCPP_INFO(this->get_logger(), "Switched to POSITION mode");
      } else if (param.as_int() == ControlMode::kAdmittance) {
        control_mode_ = ControlMode::kAdmittance;
        RCLCPP_INFO(this->get_logger(), "Switched to ADMITTANCE mode");
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown mode. Keeping previous mode.");
      }
    } 
    // dynamics control
    else if (param.get_name() == "dynamics/p_gain") {
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
    } else if (param.get_name() == "dynamics/HRM_controller_enable") {
      bool enable = param.as_bool();
      HRM_controller_.hrm_controller_enable_ = enable;
      RCLCPP_INFO(this->get_logger(), "Updated friction mode: %d", HRM_controller_.hrm_controller_enable_);
    } 
    // admittance control
    else if (param.get_name() == "admittance/M_d") {
      double M = param.as_double();
      HRM_admittance_controller_.admittance_filter_.M_(1,1) = M;
      RCLCPP_INFO(this->get_logger(), "Updated M_d(1,1): %f", M);
    } else if (param.get_name() == "admittance/B_d") {
      double B = param.as_double();
      HRM_admittance_controller_.admittance_filter_.B_(1,1) = B;
      RCLCPP_INFO(this->get_logger(), "Updated B_d(1,1): %f", B);
    } else if (param.get_name() == "admittance/K_d") {
      double K = param.as_double();
      HRM_admittance_controller_.admittance_filter_.K_(1,1) = K;
      RCLCPP_INFO(this->get_logger(), "Updated K_d(1,1): %f", K);
    }
    // poisiton control
    else if (param.get_name() == "position_control/pid_controller_pan/p_gain") {
      double p_gain = param.as_double();
      HRM_position_controller_.pid_controller_pan_.kp_ = p_gain;
      RCLCPP_INFO(this->get_logger(), "Updated p_gain: %f", p_gain);
    } else if (param.get_name() == "position_control/pid_controller_pan/i_gain") {
      double i_gain = param.as_double();
      HRM_position_controller_.pid_controller_pan_.ki_ = i_gain;
      RCLCPP_INFO(this->get_logger(), "Updated i_gain: %f", i_gain);
    } else if (param.get_name() == "position_control/pid_controller_pan/d_gain") {
      double d_gain = param.as_double();
      HRM_position_controller_.pid_controller_pan_.kd_ = d_gain;
      RCLCPP_INFO(this->get_logger(), "Updated d_gain: %f", d_gain);
    }


  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

std::string ControlNode::ControlModeToString(ControlMode mode) {
  switch(mode) {
    case ControlMode::kKinematics: return "kinematics";
    case ControlMode::kDynamics: return "dynamics";
    case ControlMode::kPosition: return "position";
    case ControlMode::kAdmittance: return "admittance";
    default: return "unknown";
  }
}


void ControlNode::run_dynamic_control_thread() {
  RCLCPP_INFO(this->get_logger(), "dynamic control_thread is started on");

  while (rclcpp::ok()) {
    if (control_mode_ == ControlMode::kDynamics) {
      try {
        SegmentAngle segment_angle_snapshot;
        bool segment_angle_updated = false;
        {
          std::lock_guard<std::mutex> lock(this->segment_angle_mutex_);
          segment_angle_snapshot = this->segment_angle_;
          segment_angle_updated = this->segment_angle_op_flag_;
          if (segment_angle_updated) {
            this->segment_angle_op_flag_ = false;
          }
        }

        // run dynamics() code
        /*** 
         * @warning
         * optimazation for memory based on avoiding memory copy
         * affect to 'theta_desired', 'tension'
         * @param loop_late_
         * loop_late_ is the sampling rate of the controller 
         * loop_late = global variable dynamics_params::SAMPLING_HZ @include '../include/control_parameters.hpp' 
         */
        // double theta_desired = this->theta_desired_ * this->HRM_controller_.surgical_tool_.torad();
        double theta_desired = this->theta_desired_ * this->HRM_controller_.surgical_tool_.torad();
        
        // if using std::vector<double> a = msg.data
        // The type must be conversion from float(msg.data) to double
        std::vector<double> theta_actual = segment_angle_snapshot.pan_relative;
        std::vector<double> omega_actual =
          segment_angle_snapshot.pan_angular_velocity_relative;

        //************************** */
        // End-effector theta

        // relative
        // double end_effector_theta_actual = std::accumulate(theta_actual.begin(), theta_actual.end(), 0.0);
        // double end_effector_omega_actual = std::accumulate(omega_actual.begin(), omega_actual.end(), 0.0);

        // absolute
        double alpha = 0.5;
        double angle_filtered = 0;
        if (!segment_angle_snapshot.pan_absolute.empty() &&
            !segment_angle_pan_absolute_prev_.empty()) {
          angle_filtered = alpha * segment_angle_snapshot.pan_absolute.back() +
            (1 - alpha) * segment_angle_pan_absolute_prev_.back();
        } else {
          angle_filtered = segment_angle_snapshot.pan_absolute.back();
        }
        segment_angle_pan_absolute_prev_ = segment_angle_snapshot.pan_absolute;

        double end_effector_theta_actual = angle_filtered;
        double end_effector_omega_actual =
          segment_angle_snapshot.pan_angular_velocity_absolute.back();

        //************************** */
        double dt = dynamics_params::DT;
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
        if (segment_angle_updated) {
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
          dynamic_MIMO_values_.friction_mode = 0;
          dynamic_MIMO_values_.friction_torque = HRM_controller_.tau_friction_;
          dynamic_MIMO_values_.damping_coefficient = dynamics_params::DAMPING;
          dynamic_MIMO_values_.res_friction = 0.0;
          dynamic_MIMO_values_.cmode = 0;
          dynamic_MIMO_values_.input_alpha = HRM_controller_.theta_ddot_input_;
          dynamic_MIMO_values_.input_omega = HRM_controller_.theta_dot_input_;
          dynamic_MIMO_values_.input_theta = HRM_controller_.theta_input_;

          dynamic_MIMO_values_publisher_->publish(dynamic_MIMO_values_);

          // publish control mode
          control_mode_msgs_.data = ControlModeToString(this->control_mode_);
          control_mode_msgs_publisher_->publish(control_mode_msgs_);
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

      } catch (const std::runtime_error & e) {
        RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
      }
    }
    // Always yield at the configured rate, including inactive control modes.
    // The previous empty kinematics branch busy-spun one CPU core.
    loop_rate_dynamics_.sleep();
  }
}



void ControlNode::run_position_with_admittance_control_thread() {
  RCLCPP_INFO(this->get_logger(), "Position control_thread is started");
  while (rclcpp::ok()) {
    if (control_mode_ == ControlMode::kPosition || control_mode_ == ControlMode::kAdmittance) {
      try {
        SegmentAngle segment_angle_snapshot;
        bool segment_angle_updated = false;
        {
          std::lock_guard<std::mutex> lock(this->segment_angle_mutex_);
          segment_angle_snapshot = this->segment_angle_;
          segment_angle_updated = this->segment_angle_op_flag_;
          if (segment_angle_updated) {
            this->segment_angle_op_flag_ = false;
          }
        }

        /***
         * @note loop_late_
         * loop_late_ is the sampling rate of the controller 
         * loop_late = global variable admittance_params::SAMPLING_HZ @include '../include/control_parameters.hpp' 
         */
        // ================================================================
        // Calculation of admittance control
        // ================================================================

        // std::vector<double> external_force = {this->external_force_.x*0.001, this->external_force_.y*0.001};

        if (control_mode_ == ControlMode::kAdmittance) {
          /**
           * @brief Check the sampling rate of between admittance and position control.
           * @author DY
           * @date 2025.04.14
           * @todo
           * Update sampling time for calculation of admittance
           * At now, joint_angle data is received at 30 Hz from camera vision (from LSTM_force_estimation_pkg)
           * Later, it is necessary to increase the samplig rate from 30 to 60 Hz (intel(R) realsense)
           */
          // calculate admittance
          /**
           * @brief mapping F/T sensor to F_ext of admittance.
           */
          this->f_env_(0) = this->external_force_.y * 0.001;
          this->f_env_(1) = (-1) * this->external_force_.x * 0.001;

          // DEBUG
          // this->f_env_(0) = 0.0; // N
          // this->f_env_(1) = 0.1; // N
          this->f_desired_(0) = 0.02;
          this->f_desired_(1) = 0.02;
          this->del_xf_ = this->HRM_admittance_controller_.compute(this->f_desired_, this->f_env_, admittance_params::DT);
          // calculate admittance - END

          // Print
          auto xt = this->HRM_admittance_controller_.admittance_filter_.getXt();
          auto xt_dot = this->HRM_admittance_controller_.admittance_filter_.getXtDot();
          auto xt_ddot = this->HRM_admittance_controller_.admittance_filter_.getXtDDot();
          
          // std::cout << "--------------- Admittance --------------------" << std::endl;
          // std::cout << "xt" << xt << std::endl;
          // std::cout << "xt_dot" << xt_dot << std::endl;
          // std::cout << "xt_ddot" << xt_ddot << std::endl;
          // std::cout << "xt_ddot" << xt_ddot << std::endl;
          // std::cout << "del_xf_" << del_xf_ << std::endl;
          // compensated desired x
          // x_t = x_d + del_x_f
          this->x_t_ = this->x_desired_ + this->del_xf_;
        } else if (control_mode_ == ControlMode::kPosition) {
          // only position mode
          this->HRM_admittance_controller_.admittance_filter_.xt_.setZero();
          this->HRM_admittance_controller_.admittance_filter_.xtdot_.setZero();
          this->HRM_admittance_controller_.admittance_filter_.xtddot_.setZero();

          this->x_t_ = this->x_desired_;
        }

        // position controller
        double dt = position_control_params::DT;
        /**
         * @brief Get end-effector (x,y) from joint angle
         * if using std::vector<double> a = msg.data
         * The type must be conversion from float(msg.data) to double
         */
        // double alpha = 0.5;
        // double angle_filtered = 0;
        // if (!segment_angle_relative_.data.empty() && !segment_angle_relative_prev_.data.empty()) {
        //   angle_filtered = alpha*segment_angle_relative_.data.back() + (1-alpha)*segment_angle_relative_prev_.data.back();
        // } else {
        //   segment_angle_relative_prev_.data = segment_angle_relative_.data;
        // }
        // segment_angle_relative_prev_.data = segment_angle_relative_.data;

        /**
         * @brief Get the 3D end-effector position from 18-joint D-H FK.
         */
        this->theta_pan_actual_ = segment_angle_snapshot.pan_relative;
        this->theta_tilt_actual_ = segment_angle_snapshot.tilt_relative;
        auto tf_matrices = this->HRM_position_controller_.surgical_tool_.computeBaseToJointsTransformationMatrices(
          this->theta_pan_actual_, this->theta_tilt_actual_);
        Eigen::Vector3d eef_xyz = this->HRM_position_controller_.surgical_tool_.computeEndEffectorPosition(tf_matrices);
        this->x_actual_(0) = eef_xyz.x();
        this->x_actual_(1) = eef_xyz.y();
        this->x_actual_(2) = eef_xyz.z();

        // ************************ Print Values ************************
        // std::cout << "--------------------------" << std::endl;
        // std::cout << "this->f_env_(x): " << this->f_env_(0) << std::endl;
        // std::cout << "this->f_env_(y): " << this->f_env_(1) << std::endl;

        // std::cout << "theta_actual_: ";
        // for (const auto& val : theta_actual_) {
        //   std::cout << val << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "tf_matrices(y): " << tf_matrices[8](1,3) << std::endl;
        // std::cout << "joints_xy: ";
        // for (const auto& joint : joints_xy) {
        //   std::cout << "(" << joint(0) << ", " << joint(1) << ") ";
        // }
        // std::cout << std::endl;
        // std::cout << "x_t(y): " << x_t_(1) << std::endl;
        // std::cout << "x_actual(y): " << x_actual_(1) << std::endl;
        // std::cout << "--------------------------" << std::endl;

        // get wire length to move (PID and Inverse-Kinematics method)
        // @ref Y.J. Kim
        
        auto wire_length_to_move = this->HRM_position_controller_.update(this->x_t_, this->x_actual_, dt);
        // position controller - END_
        // ================================================================
        // Calculation of admittance control - END
        // ================================================================

        // transition to actuator
        double f_val[5];
        f_val[0] = wire_length_to_move[0];  // East
        f_val[1] = wire_length_to_move[1];  // West
        f_val[2] = wire_length_to_move[2];  // South
        f_val[3] = wire_length_to_move[3];  // North
        f_val[4] = wire_length_to_move[4];  // grip

        this->motor_control_target_val_.header.stamp = this->now();
        this->motor_control_target_val_.header.frame_id = "motor_target_position";

        if (
        this->loadcell_data_.stress[0] < TENSION_LIMIT
        && this->loadcell_data_.stress[1] < TENSION_LIMIT
        && this->loadcell_data_.stress[2] < TENSION_LIMIT
        && this->loadcell_data_.stress[3] < TENSION_LIMIT)
        {
          this->motor_control_target_val_.target_position[0] = DIRECTION_COUPLER * f_val[0] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          this->motor_control_target_val_.target_position[1] = DIRECTION_COUPLER * f_val[1] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          this->motor_control_target_val_.target_position[2] = DIRECTION_COUPLER * f_val[2] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          this->motor_control_target_val_.target_position[3] = DIRECTION_COUPLER * f_val[3] * 0.5 * gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
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
          // x_err_(1) : y-axis error
          int target_vel_profile = int(std::round(std::abs(HRM_position_controller_.x_err_(1) * 1000.0 * 10)));
          target_vel_profile = std::min(70, target_vel_profile);
          target_vel_profile = std::max(10, target_vel_profile);
          // std::cout << "target_vel_profile: " << target_vel_profile << std::endl;
          for (int i=0; i<NUM_OF_MOTORS; i++) { 
            // this->motor_control_target_val_.target_velocity_profile[i] = PERCENT_100 * 0.5;
            this->motor_control_target_val_.target_velocity_profile[i] = target_vel_profile;
          }
        #endif
        
        // send motor command
        this->motor_control_publisher_->publish(this->motor_control_target_val_);


        // publish variables
        // std::cout << "publishing position controller data...";
        if (segment_angle_updated) {
          // time
          builtin_interfaces::msg::Time time;
          time = this->get_clock()->now();

          // ----------------------------------------------------
          // Admittance controller
          admittance_control_msgs_.header.stamp = time;
          admittance_control_msgs_.header.frame_id = "admittance_controller";
          admittance_control_msgs_.sampling_time = admittance_params::SAMPLING_HZ;

          // Force message
          admittance_control_msgs_.desired_force.force.x = HRM_admittance_controller_.f_desired_(0);
          admittance_control_msgs_.desired_force.force.y = HRM_admittance_controller_.f_desired_(1);
          admittance_control_msgs_.desired_force.force.z = HRM_admittance_controller_.f_desired_(2);
          admittance_control_msgs_.desired_force.torque.x = HRM_admittance_controller_.f_desired_(3);
          admittance_control_msgs_.desired_force.torque.y = HRM_admittance_controller_.f_desired_(4);
          admittance_control_msgs_.desired_force.torque.z = HRM_admittance_controller_.f_desired_(5);
          
          admittance_control_msgs_.env_force.force.x = HRM_admittance_controller_.f_env_(0);
          admittance_control_msgs_.env_force.force.y = HRM_admittance_controller_.f_env_(1);
          admittance_control_msgs_.env_force.force.z = HRM_admittance_controller_.f_env_(2);
          admittance_control_msgs_.env_force.torque.x = HRM_admittance_controller_.f_env_(3);
          admittance_control_msgs_.env_force.torque.y = HRM_admittance_controller_.f_env_(4);
          admittance_control_msgs_.env_force.torque.z = HRM_admittance_controller_.f_env_(5);

          admittance_control_msgs_.delta_force.force.x = HRM_admittance_controller_.del_f_(0);
          admittance_control_msgs_.delta_force.force.y = HRM_admittance_controller_.del_f_(1);
          admittance_control_msgs_.delta_force.force.z = HRM_admittance_controller_.del_f_(2);
          admittance_control_msgs_.delta_force.torque.x = HRM_admittance_controller_.del_f_(3);
          admittance_control_msgs_.delta_force.torque.y = HRM_admittance_controller_.del_f_(4);
          admittance_control_msgs_.delta_force.torque.z = HRM_admittance_controller_.del_f_(5);

          // Admittance variables
          std::vector<double> m_matrix_vec(
            HRM_admittance_controller_.admittance_filter_.M_.data(),
            HRM_admittance_controller_.admittance_filter_.M_.data() + HRM_admittance_controller_.admittance_filter_.M_.size());
          std::vector<double> b_matrix_vec(
            HRM_admittance_controller_.admittance_filter_.B_.data(),
            HRM_admittance_controller_.admittance_filter_.B_.data() + HRM_admittance_controller_.admittance_filter_.B_.size());
          std::vector<double> k_matrix_vec(
            HRM_admittance_controller_.admittance_filter_.K_.data(),
            HRM_admittance_controller_.admittance_filter_.K_.data() + HRM_admittance_controller_.admittance_filter_.K_.size());
          admittance_control_msgs_.m_matrix = m_matrix_vec;
          admittance_control_msgs_.b_matrix = b_matrix_vec;
          admittance_control_msgs_.k_matrix = k_matrix_vec;

          admittance_control_msgs_.x_ddot.position.x = this->HRM_admittance_controller_.admittance_filter_.getXtDDot()(0);
          admittance_control_msgs_.x_ddot.position.y = this->HRM_admittance_controller_.admittance_filter_.getXtDDot()(1);
          admittance_control_msgs_.x_ddot.position.z = this->HRM_admittance_controller_.admittance_filter_.getXtDDot()(2);
          
          admittance_control_msgs_.x_dot.position.x = this->HRM_admittance_controller_.admittance_filter_.getXtDot()(0);
          admittance_control_msgs_.x_dot.position.y = this->HRM_admittance_controller_.admittance_filter_.getXtDot()(1);
          admittance_control_msgs_.x_dot.position.z = this->HRM_admittance_controller_.admittance_filter_.getXtDot()(2);

          admittance_control_msgs_.x.position.x = this->HRM_admittance_controller_.admittance_filter_.getXt()(0);
          admittance_control_msgs_.x.position.y = this->HRM_admittance_controller_.admittance_filter_.getXt()(1);
          admittance_control_msgs_.x.position.z = this->HRM_admittance_controller_.admittance_filter_.getXt()(2);

          admittance_control_msgs_.dt = this->HRM_admittance_controller_.dt_;

          // ----------------------------------------------------
          // Position controller
          position_control_msgs_.header.stamp = time;
          position_control_msgs_.header.frame_id = "position_controller";

          // gain (pan, tilt same)
          position_control_msgs_.p_gain = HRM_position_controller_.pid_controller_pan_.kp_;
          position_control_msgs_.i_gain = HRM_position_controller_.pid_controller_pan_.ki_;
          position_control_msgs_.d_gain = HRM_position_controller_.pid_controller_pan_.kd_;

          // position message
          /**
           * @note skip orientation (TBD)
           */
          position_control_msgs_.x_desired.position.x = HRM_position_controller_.x_desired_(0);
          position_control_msgs_.x_desired.position.y = HRM_position_controller_.x_desired_(1);
          position_control_msgs_.x_desired.position.z = HRM_position_controller_.x_desired_(2);

          position_control_msgs_.x_actual.position.x = HRM_position_controller_.x_actual_(0);
          position_control_msgs_.x_actual.position.y = HRM_position_controller_.x_actual_(1);
          position_control_msgs_.x_actual.position.z = HRM_position_controller_.x_actual_(2);

          position_control_msgs_.x_error.position.x = HRM_position_controller_.x_err_(0);
          position_control_msgs_.x_error.position.y = HRM_position_controller_.x_err_(1);
          position_control_msgs_.x_error.position.z = HRM_position_controller_.x_err_(2);

          position_control_msgs_.dt = HRM_position_controller_.dt_;
          position_control_msgs_.del_theta_pan = HRM_position_controller_.del_theta_pan_;
          position_control_msgs_.del_theta_tilt = HRM_position_controller_.del_theta_tilt_;

          // joint(segment) angle information
          // Legacy fields carry pan values until PositionControl.msg is extended.
          position_control_msgs_.theta_actual_relative =
            segment_angle_snapshot.pan_relative;
          position_control_msgs_.theta_actual_absolute =
            segment_angle_snapshot.pan_absolute;

          // publish data of controllers
          admittance_control_msgs_publisher_->publish(admittance_control_msgs_);
          position_control_msgs_publisher_->publish(position_control_msgs_);
          
          // publish control mode
          control_mode_msgs_.data = ControlModeToString(this->control_mode_);
          control_mode_msgs_publisher_->publish(control_mode_msgs_);
          /**
           * @brief update tool states (pan and tilt anlge)
           * @warning Check out the coordinate system on paper
           * Pan: rotate about Z-axis
           * Tilt: rotate about Y-axis
           */
          geometry_msgs::msg::Twist surgical_tool_pose;
          surgical_tool_pose.angular.z = HRM_position_controller_.surgical_tool_.pAngle_;
          surgical_tool_pose.angular.y = HRM_position_controller_.surgical_tool_.tAngle_;
          this->surgical_tool_pose_publisher_->publish(surgical_tool_pose);

        }
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
      }
    }

    // Update end-effector pose estimation
    try {
      SegmentAngle segment_angle_snapshot;
      {
        std::lock_guard<std::mutex> lock(this->segment_angle_mutex_);
        segment_angle_snapshot = this->segment_angle_;
      }
      // calculate end-effector pose
      this->theta_pan_actual_ = segment_angle_snapshot.pan_relative;
      this->theta_tilt_actual_ = segment_angle_snapshot.tilt_relative;
      auto tf_matrices = this->HRM_position_controller_.surgical_tool_.computeBaseToJointsTransformationMatrices(
        this->theta_pan_actual_, this->theta_tilt_actual_);
      Eigen::Vector3d eef_xyz = this->HRM_position_controller_.surgical_tool_.computeEndEffectorPosition(tf_matrices);
      this->x_actual_(0) = eef_xyz.x();
      this->x_actual_(1) = eef_xyz.y();
      this->x_actual_(2) = eef_xyz.z();
      tool_endeffector_pose_.data[0] = this->x_actual_(0);
      tool_endeffector_pose_.data[1] = this->x_actual_(1);
      tool_endeffector_pose_.data[2] = this->x_actual_(2);
      
      this->tool_endeffector_pose_publisher_->publish(tool_endeffector_pose_);

    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(this->get_logger(), "[Update end-effector pose] Error: %s", e.what());
    }
    // Exactly one sleep per loop. Position/admittance mode previously slept
    // twice and therefore ran at roughly half the configured frequency.
    loop_rate_position_with_admittance_.sleep();
  }
}
