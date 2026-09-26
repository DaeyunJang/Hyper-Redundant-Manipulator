#include "control_node.hpp"
#include "hardware_metadata.hpp"

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;
using namespace std::chrono_literals;

namespace {
int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
bool finite_values(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}
}  // namespace

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
  loop_rate_dynamics_(dynamics_params::SAMPLING_HZ)
{
  declare_hardware_metadata(*this);
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
  motor_output_enabled_ =
    this->declare_parameter<bool>("motor_output_enabled", false);
  const hrm::SineConfig sine_defaults;
  declare_parameter<std::vector<double>>("motion.sine.tilt",
    std::vector<double>(sine_defaults.tilt.begin(), sine_defaults.tilt.end()));
  declare_parameter<std::vector<double>>("motion.sine.pan",
    std::vector<double>(sine_defaults.pan.begin(), sine_defaults.pan.end()));
  declare_parameter<double>("motion.sine.period_sec", sine_defaults.period);
  declare_parameter<double>("motion.sine.max_speed_deg_s", sine_defaults.speed);
  sine_config().validate();
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
  for (const std::string axis : {"pan", "tilt"}) {
    const std::string prefix = "position_control/pid_controller_" + axis + "/";
    const double p = declare_parameter<double>(prefix + "p_gain", position_control_params::KP);
    const double i = declare_parameter<double>(prefix + "i_gain", position_control_params::KI);
    const double d = declare_parameter<double>(prefix + "d_gain", position_control_params::KD);
    if (!std::isfinite(p) || p < 0.0 || i != 0.0 || !std::isfinite(d) || d < 0.0) {
      throw std::invalid_argument("Position PD requires finite nonnegative P/D and I=0.");
    }
    auto& pid = axis == "pan" ? HRM_position_controller_.pid_controller_pan_ : HRM_position_controller_.pid_controller_tilt_;
    pid.set_PID_gains(p, i, d);
  }
  position_max_speed_deg_s_ = declare_parameter<double>(
    "position_control/max_angular_speed_deg_s", position_control_params::MAX_ANGULAR_SPEED_DEG_S);
  position_derivative_filter_sec_ = declare_parameter<double>(
    "position_control/derivative_filter_sec", position_control_params::DERIVATIVE_FILTER_SEC);
  position_feedback_timeout_sec_ = declare_parameter<double>(
    "position_control/feedback_timeout_sec", position_control_params::FEEDBACK_TIMEOUT_SEC);
  if (!std::isfinite(position_feedback_timeout_sec_) || position_feedback_timeout_sec_ <= 0.0 ||
      position_feedback_timeout_sec_ > position_control_params::FEEDBACK_TIMEOUT_SEC)
  {
    throw std::invalid_argument("Position feedback timeout must be in (0, 0.25] seconds.");
  }
  HRM_position_controller_.set_limits(position_max_speed_deg_s_, position_derivative_filter_sec_);

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
  position_motor_preview_publisher_ = create_publisher<MotorCommand>(
    "position/target_motor_command", qos_reliable_latest);
  position_status_publisher_ = create_publisher<std_msgs::msg::String>(
    "position/control_status", rclcpp::QoS(1).reliable().transient_local());
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
  ik_sine_status_pub_ = create_publisher<std_msgs::msg::String>(
    "kinematics/sine_status", rclcpp::QoS(1).reliable().transient_local());
  stop_ik_sine("STOPPED: explicit Start required");
  
  //===============================
  // surgical tool pose(degree) publisher
  //===============================
  surgical_tool_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("surgical_tool_pose", qos_reliable_latest);
  tool_endeffector_pose_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("tool_endeffector_pose", qos_reliable_latest);
  fk_tip_position_publisher_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>(
      "kinematics/fk_tip_position", qos_reliable_latest);
  wire_length_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("wire_length", qos_reliable_latest);
  wire_length_velocity_publisher_ = 
    this->create_publisher<std_msgs::msg::Float64MultiArray>("wire_length_velocity", qos_reliable_latest);
  target_wire_length_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "kinematics/target_wire_length", qos_reliable_latest);
  this->tool_endeffector_pose_.data.resize(3);
  this->wire_length_.data.resize(NUM_OF_MOTORS);
  this->wire_length_velocity_.data.resize(NUM_OF_MOTORS);
  this->target_wire_length_.data.resize(NUM_OF_MOTORS);
  RCLCPP_WARN(
    this->get_logger(),
    "Motor output is %s. IK preview topic: /kinematics/target_wire_length "
    "([East, West, South, North], mm).",
    motor_output_enabled_ ? "ENABLED" : "DISABLED (dry-run)");

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
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
        if (msg->actual_position.size() != NUM_OF_MOTORS ||
            msg->actual_velocity.size() != NUM_OF_MOTORS)
        {
          valid_motor_feedback_ = false;
          op_mode_ = kStop;
          RCLCPP_WARN(get_logger(), "Ignoring malformed motor_state; expected four positions/velocities.");
          return;
        }
        valid_motor_feedback_ = true;
        motor_received_ = std::chrono::steady_clock::now();
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
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
        if (msg->stress.size() != NUM_OF_MOTORS || !finite_values(msg->stress)) {
          valid_loadcell_feedback_ = false;
          RCLCPP_WARN(get_logger(), "Ignoring malformed loadcell_state; expected four finite stresses.");
          return;
        }
        valid_loadcell_feedback_ = true;
        loadcell_received_ = std::chrono::steady_clock::now();
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
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
        valid_force_feedback_ = std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z);
        if (!valid_force_feedback_) {return;}
        force_received_ = std::chrono::steady_clock::now();
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
            if (msg->header.frame_id != "hrm_base" || stamp_ns(msg->header.stamp) <= 0 ||
                stamp_ns(msg->header.stamp) <= stamp_ns(segment_angle_.header.stamp) ||
                !finite_values(msg->pan_relative) || !finite_values(msg->tilt_relative) ||
                !finite_values(msg->pan_absolute) || !finite_values(msg->tilt_absolute) ||
                !finite_values(msg->pan_angular_velocity_relative) ||
                !finite_values(msg->tilt_angular_velocity_relative) ||
                !finite_values(msg->pan_angular_velocity_absolute) ||
                !finite_values(msg->tilt_angular_velocity_absolute))
            {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Ignoring invalid/duplicate angle frame; require finite hrm_base data and increasing source stamp.");
              return;
            }
            this->segment_angle_ = *msg;
            this->segment_angle_op_flag_ = true;
            ++segment_angle_sequence_;
            segment_angle_received_ = std::chrono::steady_clock::now();
          }
          segment_angle_cv_.notify_one();
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
    std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
    std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    if (ik_sine_active_ || control_mode_ == ControlMode::kPosition || control_mode_ == ControlMode::kAdmittance ||
        !valid_motor_feedback_ || request->index_motor < 0 || request->index_motor >= NUM_OF_MOTORS)
    {
      response->success = false;
      return;
    }
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

      // Direct motor commands never bypass the explicit actuator safety gate.
      if(this->op_mode_ == kEnable) {
        response->success = this->publish_motor_command_if_enabled(
          "move_motor_direct");
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
    std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
    if (ik_sine_active_) {
      response->success = false;
      RCLCPP_WARN(get_logger(), "Stop /kinematics/sine_motion before manual IK commands.");
      return;
    }
    try {
      // run
      if (control_mode_ != ControlMode::kKinematics) {
        RCLCPP_INFO(this->get_logger(), "this motion must be operated on \'KINEMACTICS\' mode. Change parameter \'control_mode\'.");
        return;
      }

      // IK preview is available without a motor connection. Actual actuator
      // output additionally requires motor_output_enabled and motor state.
      if(this->op_mode_ == kEnable || !motor_output_enabled_) {
        if(request->mode == 0) {
          // MOVE ABSOLUTELY
          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, request->tiltangle, request->panangle, request->gripangle);
          this->cal_inverse_kinematics(request->panangle, request->tiltangle, request->gripangle);
          this->publish_motor_command_if_enabled("kinematics/move_tool_angle");
          this->surgical_tool_pose_publisher_->publish(this->surgical_tool_pose_);
        }
        else if (request->mode == 1) {
          // MOVE RELATIVELY
          double pan_angle = this->current_pan_angle_ + request->panangle;
          double tilt_angle = this->current_tilt_angle_ + request->tiltangle;
          double grip_angle = this->current_grip_angle_ + request->gripangle;

          RCLCPP_INFO(this->get_logger(), "MODE: %d, tilt: %.2f, pan: %.2f, grip: %.2f", request->mode, tilt_angle, pan_angle, grip_angle);
          this->cal_inverse_kinematics(pan_angle, tilt_angle, grip_angle);
          this->publish_motor_command_if_enabled("kinematics/move_tool_angle");
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
  [this](const std::shared_ptr<SetGoalPosition::Request> request,
    std::shared_ptr<SetGoalPosition::Response> response) -> void
  {
    std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
    response->success = false;
    if ((control_mode_ != ControlMode::kPosition && control_mode_ != ControlMode::kAdmittance) ||
        !position_ready_ || position_fault_latched_)
    {
      RCLCPP_WARN(get_logger(), "Position goal rejected: wait for fresh-feedback mode entry.");
      return;
    }
    const auto& p = request->goal_position.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        (request->reference_type != "absolute" && request->reference_type != "relative"))
    {
      return;
    }
    // hrm_base metres. X is passive; only Y/Z are controlled.
    const bool absolute = request->reference_type == "absolute";
    const double y = absolute ? p.y : x_desired_(1) + p.y;
    const double z = absolute ? p.z : x_desired_(2) + p.z;
    if (!std::isfinite(y) || !std::isfinite(z)) {return;}
    x_desired_(1) = y;
    x_desired_(2) = z;
    response->success = true;
    RCLCPP_INFO(get_logger(), "Position goal [hrm_base m]: Y=%+.6f Z=%+.6f",
      x_desired_(1), x_desired_(2));
  };
  set_goal_position_service_server_ =
    create_service<SetGoalPosition>("position/set_goal_position", set_goal_position);


  ik_sine_service_ = create_service<std_srvs::srv::SetBool>(
    "kinematics/sine_motion",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
      if (!request->data) {
        stop_ik_sine("STOPPED: no new trajectory targets; no automatic return to zero");
        response->success = true;
        response->message = "Sine stopped; last target retained (not an emergency stop).";
        return;
      }
      try {
        if (control_mode_ != ControlMode::kKinematics || timer_) {
          throw std::runtime_error("Require kinematics mode and no other active motion timer.");
        }
        if (motor_output_enabled_ && !ik_sine_feedback_safe()) {
          throw std::runtime_error("Fresh motor/loadcell data below tension limit required.");
        }
        if (motor_output_enabled_) {
          // A dry-run or direct command can leave IK's last angle different
          // from the hardware. Never treat that angle as a measured start pose.
          const auto lengths = HRM_controller_.surgical_tool_.get_IK_result(
            current_pan_angle_, current_tilt_angle_, current_grip_angle_);
          const double scale = DIRECTION_COUPLER * 0.5 *
            gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
          std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
          for (int i = 0; i < NUM_OF_MOTORS; ++i) {
            if (std::abs(lengths[i] * scale - motor_state_.actual_position[i]) > 1000.0) {
              throw std::runtime_error(
                "Motor positions differ from last IK pose by >1000 counts. "
                "Establish the calibrated starting IK pose before sine Start.");
            }
          }
        }
        ik_sine_.start(sine_config(), current_tilt_angle_, current_pan_angle_);
        ik_sine_tick_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(10ms, std::bind(&ControlNode::publish_ik_sine, this));
        ik_sine_active_ = true;
        std_msgs::msg::String status;
        status.data = motor_output_enabled_ ? "RUNNING: IK sine" : "DRY_RUN: IK sine; motor output blocked";
        ik_sine_status_pub_->publish(status);
        response->success = true;
        response->message = status.data;
      } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
      }
    });

  auto sine_wave_callback = 
  [this](
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
  {
    std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
    if (ik_sine_active_) {
      if (!request->data) {stop_ik_sine("STOPPED by legacy motion stop request");}
      response->success = !request->data;
      response->message = request->data ? "Stop IK sine before another motion." : "IK sine stopped.";
      return;
    }
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
    std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
    if (ik_sine_active_) {
      if (!request->data) {stop_ik_sine("STOPPED by legacy motion stop request");}
      response->success = !request->data;
      response->message = request->data ? "Stop IK sine before another motion." : "IK sine stopped.";
      return;
    }
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
    std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
    if (ik_sine_active_) {
      if (!request->data) {stop_ik_sine("STOPPED by legacy motion stop request");}
      response->success = !request->data;
      response->message = request->data ? "Stop IK sine before another motion." : "IK sine stopped.";
      return;
    }
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
    std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
    if (ik_sine_active_) {
      if (!request->data) {stop_ik_sine("STOPPED by legacy motion stop request");}
      response->success = !request->data;
      response->message = request->data ? "Stop IK sine before another motion." : "IK sine stopped.";
      return;
    }
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
      const auto result = this->set_parameter(rclcpp::Parameter("control_mode", request->mode));
      response->success = result.successful;
      response->message = result.successful ?
        "Control mode changed -> " + std::to_string(request->mode) : result.reason;
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
  // Aggregate command representation at the straight configuration:
  // q1 tilt rotates about Base Z; q2 pan is the orthogonal bending component.
  this->surgical_tool_pose_.angular.z = tAngle * M_PI/180;
  this->surgical_tool_pose_.angular.y = pAngle * M_PI/180;
  this->HRM_controller_.surgical_tool_.get_IK_result(this->current_pan_angle_, this->current_tilt_angle_, this->current_grip_angle_);
  // this->ST_.get_IK_result(this->current_pan_angle_, this->current_tilt_angle_, this->current_grip_angle_);

  double f_val[5];
  f_val[0] = this->HRM_controller_.surgical_tool_.wrLengthEast_;
  f_val[1] = this->HRM_controller_.surgical_tool_.wrLengthWest_;
  f_val[2] = this->HRM_controller_.surgical_tool_.wrLengthSouth_;
  f_val[3] = this->HRM_controller_.surgical_tool_.wrLengthNorth_;
  f_val[4] = this->HRM_controller_.surgical_tool_.wrLengthGrip;

  for (int index = 0; index < NUM_OF_MOTORS; ++index) {
    this->target_wire_length_.data[index] = f_val[index];
  }
  this->target_wire_length_publisher_->publish(this->target_wire_length_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "IK target [mm] East=%+.5f, West=%+.5f, South=%+.5f, North=%+.5f",
    f_val[0], f_val[1], f_val[2], f_val[3]);

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

bool ControlNode::publish_motor_command_if_enabled(const char * command_source)
{
  std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
  // Only the fresh-frame position worker may drive these feedback modes.
  if (control_mode_ == ControlMode::kPosition || control_mode_ == ControlMode::kAdmittance) {
    return false;
  }
  if (!motor_output_enabled_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dry-run: blocked motor_command from %s.", command_source);
    return false;
  }
  if (this->op_mode_ != kEnable) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Blocked motor_command from %s: no valid motor_state received.",
      command_source);
    return false;
  }

  this->motor_control_publisher_->publish(this->motor_control_target_val_);
  return true;
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

hrm::SineConfig ControlNode::sine_config(const std::vector<rclcpp::Parameter>& overrides)
{
  auto get = [&](const std::string& name) {
    for (const auto& parameter : overrides) {
      if (parameter.get_name() == name) {return parameter;}
    }
    return get_parameter(name);
  };
  hrm::SineConfig config;
  auto read_axis = [&](const std::string& name, std::array<double, 3>& target) {
    const auto values = get(name).as_double_array();
    if (values.size() != 3) {
      throw std::invalid_argument("Sine axis must be [center_deg, amplitude_deg, phase_deg].");
    }
    std::copy(values.begin(), values.end(), target.begin());
  };
  read_axis("motion.sine.tilt", config.tilt);
  read_axis("motion.sine.pan", config.pan);
  config.period = get("motion.sine.period_sec").as_double();
  config.speed = get("motion.sine.max_speed_deg_s").as_double();
  config.validate();
  return config;
}

void ControlNode::stop_ik_sine(const std::string& reason)
{
  if (ik_sine_active_ && timer_) {timer_->cancel(); timer_.reset();}
  ik_sine_active_ = false;
  std_msgs::msg::String message;
  message.data = reason;
  ik_sine_status_pub_->publish(message);
}

bool ControlNode::ik_sine_feedback_safe()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const auto steady = std::chrono::steady_clock::now();
  const auto ros_ns = now().nanoseconds();
  auto fresh = [&](std::chrono::steady_clock::time_point received,
                   const builtin_interfaces::msg::Time& stamp) {
    const double age = (ros_ns - stamp_ns(stamp)) * 1e-9;
    return std::chrono::duration<double>(steady - received).count() <= 0.25 &&
           stamp_ns(stamp) > 0 && age >= -0.05 && age <= 0.25;
  };
  return op_mode_ == kEnable && valid_motor_feedback_ && valid_loadcell_feedback_ &&
    fresh(motor_received_, motor_state_.header.stamp) &&
    fresh(loadcell_received_, loadcell_data_.header.stamp) &&
    loadcell_data_.stress.size() == NUM_OF_MOTORS &&
    std::all_of(loadcell_data_.stress.begin(), loadcell_data_.stress.end(),
      [](double value) {return std::isfinite(value) && value < TENSION_LIMIT;});
}

void ControlNode::publish_ik_sine()
{
  std::lock_guard<std::recursive_mutex> lock(position_control_mutex_);
  if (!ik_sine_active_) {return;}
  if (control_mode_ != ControlMode::kKinematics ||
      (motor_output_enabled_ && !ik_sine_feedback_safe())) {
    stop_ik_sine("STOPPED: mode or motor/loadcell safety check failed; explicit restart required");
    return;
  }
  try {
    const auto tick = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(tick - ik_sine_tick_).count();
    ik_sine_tick_ = tick;
    const auto angles = ik_sine_.advance(dt);
    cal_inverse_kinematics(angles[1], angles[0], current_grip_angle_);
    for (auto count : motor_control_target_val_.target_position) {
      if (std::abs(static_cast<double>(count)) > MOTOR_SOFTWARE_LIMIT) {
        throw std::runtime_error("Motor software position limit.");
      }
    }
    publish_motor_command_if_enabled("kinematics/sine_motion");
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
  } catch (const std::exception& error) {
    stop_ik_sine(std::string("STOPPED: ") + error.what());
  }
}

void ControlNode::publish_sine_wave()
{
  std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    cal_inverse_kinematics(trajectory_, 0, 0);
    publish_motor_command_if_enabled("motion/move_sine_wave");
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
  std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
  if (control_mode_ == ControlMode::kKinematics) {
    double omega = 2.0 * M_PI / period_;
    trajectory_ = amp_deg_ * std::sin(omega * count_);
    cal_inverse_kinematics(trajectory_, 0, 0);
    publish_motor_command_if_enabled("motion/move_sine_wave_1time");
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
    publish_motor_command_if_enabled("kinematics/move_circle_motion");
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
    publish_motor_command_if_enabled("kinematics/move_moebius_motion");
    surgical_tool_pose_publisher_->publish(surgical_tool_pose_);
    count_ += count_add_;
    // std::cout << pan_deg <<  " / " << tilt_deg << std::endl;
  }
}

rcl_interfaces::msg::SetParametersResult ControlNode::parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
  std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
  rcl_interfaces::msg::SetParametersResult rejected;
  rejected.successful = false;
  for (const auto& parameter : parameters) {
    if (parameter.get_name().rfind("motion.sine.", 0) == 0) {
      if (ik_sine_active_) {
        rejected.reason = "Stop IK sine before editing waveform settings.";
        return rejected;
      }
      try {sine_config(parameters);} catch (const std::exception& error) {
        rejected.reason = error.what();
        return rejected;
      }
      break;
    }
  }
  // Validate the whole batch before applying any position-control changes.
  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    if (name == "control_mode" && (param.as_int() < 1 || param.as_int() > 4)) {
      rejected.reason = "control_mode must be 1..4.";
      return rejected;
    }
    if (name.rfind("position_control/", 0) == 0) {
      const double value = param.as_double();
      if (!std::isfinite(value) || value < 0.0 ||
          (name.find("/i_gain") != std::string::npos && value != 0.0) ||
          (name == "position_control/max_angular_speed_deg_s" && value <= 0.0) ||
          (name == "position_control/feedback_timeout_sec" &&
            (value <= 0.0 || value > position_control_params::FEEDBACK_TIMEOUT_SEC)))
      {
        rejected.reason = "Position PD: finite nonnegative gains, I=0, speed>0, timeout in (0,0.25].";
        return rejected;
      }
    }
  }
  for (const auto &param : parameters) {
    if (param.get_name() == "control_mode") {
      if (param.as_int() != static_cast<int>(control_mode_.load())) {
        if (ik_sine_active_) {stop_ik_sine("STOPPED: control mode changed");}
        if (position_ready_) {hold_position_from_motor_feedback();}
        if (timer_) {timer_->cancel(); timer_.reset();}
        count_ = 0;
        ++position_entry_revision_;
        position_entry_source_ns_ = now().nanoseconds();
        std::lock_guard<std::mutex> frame_lock(segment_angle_mutex_);
        position_entry_min_sequence_ = segment_angle_sequence_;
        position_ready_ = false;
        position_fault_latched_ = false;
      }
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
    } else if (param.get_name() == "motor_output_enabled") {
      if (param.as_bool() != motor_output_enabled_.load()) {
        if (ik_sine_active_) {stop_ik_sine("STOPPED: motor output gate changed");}
        // Send a current-position hold BEFORE disabling new ROS commands.
        if (position_ready_) {hold_position_from_motor_feedback();}
        if (timer_) {timer_->cancel(); timer_.reset();}
        count_ = 0;
        ++position_entry_revision_;
        position_entry_source_ns_ = now().nanoseconds();
        std::lock_guard<std::mutex> frame_lock(segment_angle_mutex_);
        position_entry_min_sequence_ = segment_angle_sequence_;
        position_ready_ = false;
        position_fault_latched_ = false;
      }
      motor_output_enabled_ = param.as_bool();
      RCLCPP_WARN(
        this->get_logger(), "Motor output %s.",
        motor_output_enabled_ ? "ENABLED" : "DISABLED (dry-run)");
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
    else if (param.get_name() == "position_control/max_angular_speed_deg_s") {
      position_max_speed_deg_s_ = param.as_double();
    } else if (param.get_name() == "position_control/derivative_filter_sec") {
      position_derivative_filter_sec_ = param.as_double();
    } else if (param.get_name() == "position_control/feedback_timeout_sec") {
      position_feedback_timeout_sec_ = param.as_double();
    }
    // position control
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
    } else if (param.get_name() == "position_control/pid_controller_tilt/p_gain") {
      HRM_position_controller_.pid_controller_tilt_.kp_ = param.as_double();
    } else if (param.get_name() == "position_control/pid_controller_tilt/i_gain") {
      HRM_position_controller_.pid_controller_tilt_.ki_ = param.as_double();
    } else if (param.get_name() == "position_control/pid_controller_tilt/d_gain") {
      HRM_position_controller_.pid_controller_tilt_.kd_ = param.as_double();
    }


  }
  HRM_position_controller_.set_limits(position_max_speed_deg_s_, position_derivative_filter_sec_);
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
        
        this->publish_motor_command_if_enabled("dynamics control");

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



void ControlNode::set_position_status(const std::string& status)
{
  if (status == last_position_status_) {return;}
  last_position_status_ = status;
  std_msgs::msg::String message;
  message.data = status;
  position_status_publisher_->publish(message);
  RCLCPP_INFO(get_logger(), "Position control: %s", status.c_str());
}

void ControlNode::publish_position_diagnostics(
  const SegmentAngle& sample, double dt, ControlMode mode)
{
  auto fill_pose = [](geometry_msgs::msg::Pose& pose, const Eigen::VectorXd& value) {
    pose.position.x = value(0);
    pose.position.y = value(1);
    pose.position.z = value(2);
    pose.orientation.w = 1.0;
  };
  auto fill_force = [](geometry_msgs::msg::Wrench& force, const Eigen::VectorXd& value) {
    force.force.x = value(0);
    force.force.y = value(1);
    force.force.z = value(2);
    force.torque.x = value(3);
    force.torque.y = value(4);
    force.torque.z = value(5);
  };
  position_control_msgs_.header = sample.header;
  position_control_msgs_.sampling_time = dt > 0.0 ? 1.0 / dt : 0.0;
  position_control_msgs_.p_gain = HRM_position_controller_.pid_controller_pan_.kp_;
  position_control_msgs_.i_gain = HRM_position_controller_.pid_controller_pan_.ki_;
  position_control_msgs_.d_gain = HRM_position_controller_.pid_controller_pan_.kd_;
  fill_pose(position_control_msgs_.x_desired, HRM_position_controller_.x_desired_);
  fill_pose(position_control_msgs_.x_actual, HRM_position_controller_.x_actual_);
  fill_pose(position_control_msgs_.x_error, HRM_position_controller_.x_err_);
  position_control_msgs_.dt = dt;
  position_control_msgs_.del_theta_pan = HRM_position_controller_.del_theta_pan_;
  position_control_msgs_.del_theta_tilt = HRM_position_controller_.del_theta_tilt_;
  // Legacy message fields are pan-only; full pan/tilt is in SegmentAngle.
  position_control_msgs_.theta_actual_relative = sample.pan_relative;
  position_control_msgs_.theta_actual_absolute = sample.pan_absolute;
  position_control_msgs_publisher_->publish(position_control_msgs_);

  admittance_control_msgs_.header = sample.header;
  admittance_control_msgs_.sampling_time = position_control_msgs_.sampling_time;
  admittance_control_msgs_.dt = dt;
  auto& filter = HRM_admittance_controller_.admittance_filter_;
  fill_force(admittance_control_msgs_.desired_force, HRM_admittance_controller_.f_desired_);
  fill_force(admittance_control_msgs_.env_force, HRM_admittance_controller_.f_env_);
  fill_force(admittance_control_msgs_.delta_force, HRM_admittance_controller_.del_f_);
  admittance_control_msgs_.m_matrix.assign(filter.M_.data(), filter.M_.data() + filter.M_.size());
  admittance_control_msgs_.b_matrix.assign(filter.B_.data(), filter.B_.data() + filter.B_.size());
  admittance_control_msgs_.k_matrix.assign(filter.K_.data(), filter.K_.data() + filter.K_.size());
  fill_pose(admittance_control_msgs_.x, filter.xt_);
  fill_pose(admittance_control_msgs_.x_dot, filter.xtdot_);
  fill_pose(admittance_control_msgs_.x_ddot, filter.xtddot_);
  admittance_control_msgs_publisher_->publish(admittance_control_msgs_);

  control_mode_msgs_.data = ControlModeToString(mode);
  control_mode_msgs_publisher_->publish(control_mode_msgs_);
  geometry_msgs::msg::Twist command_angles;
  command_angles.angular.z = HRM_position_controller_.surgical_tool_.tAngle_;
  command_angles.angular.y = HRM_position_controller_.surgical_tool_.pAngle_;
  surgical_tool_pose_publisher_->publish(command_angles);
}

void ControlNode::hold_position_from_motor_feedback()
{
  MotorCommand hold;
  {
    std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    const double age = (now().nanoseconds() - stamp_ns(motor_state_.header.stamp)) * 1e-9;
    const double receipt_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - motor_received_).count();
    if (!valid_motor_feedback_ || age < -0.05 || age > position_feedback_timeout_sec_ ||
        receipt_age > position_feedback_timeout_sec_ ||
        std::any_of(motor_state_.actual_position.begin(), motor_state_.actual_position.end(),
          [](int32_t value) {return std::abs(static_cast<double>(value)) > MOTOR_SOFTWARE_LIMIT;}))
    {
      RCLCPP_ERROR(get_logger(), "Cannot issue software hold: motor feedback invalid/stale/out of range. Driver stop required.");
      return;
    }
    hold.target_position = motor_state_.actual_position;
  }
  hold.header.stamp = now();
  hold.header.frame_id = "motor_target_position";
  hold.target_velocity_profile.assign(NUM_OF_MOTORS, 10);
  position_motor_preview_publisher_->publish(hold);
  if (motor_output_enabled_) {motor_control_publisher_->publish(hold);}
}

void ControlNode::run_position_with_admittance_control_thread()
{
  RCLCPP_INFO(get_logger(), "Position controller waits for new source-image frames.");
  std::uint64_t seen_sequence = 0;
  std::uint64_t prepared_revision = 0;
  int64_t previous_source_ns = 0;
  while (rclcpp::ok()) {
    SegmentAngle sample;
    std::chrono::steady_clock::time_point image_received;
    std::uint64_t sequence;
    bool new_frame;
    {
      std::unique_lock<std::mutex> frame_lock(segment_angle_mutex_);
      segment_angle_cv_.wait_for(frame_lock, 20ms, [this, &seen_sequence]() {
        return !rclcpp::ok() || segment_angle_sequence_ != seen_sequence;
      });
      if (!rclcpp::ok()) {break;}
      sequence = segment_angle_sequence_;
      new_frame = sequence != seen_sequence;
      seen_sequence = sequence;
      sample = segment_angle_;
      image_received = segment_angle_received_;
    }
    std::lock_guard<std::recursive_mutex> control_lock(position_control_mutex_);
    const ControlMode mode = control_mode_.load();
    const bool active = mode == ControlMode::kPosition || mode == ControlMode::kAdmittance;
    if (prepared_revision != position_entry_revision_) {
      prepared_revision = position_entry_revision_;
      previous_source_ns = 0;
    }

    MotorState motor;
    custom_interfaces::msg::LoadcellState loadcell;
    geometry_msgs::msg::Vector3 force;
    bool motor_fresh, loadcell_fresh, force_fresh;
    const auto steady_now = std::chrono::steady_clock::now();
    const int64_t ros_now_ns = get_clock()->now().nanoseconds();
    auto recent = [&](std::chrono::steady_clock::time_point received) {
      return std::chrono::duration<double>(steady_now - received).count() <= position_feedback_timeout_sec_;
    };
    auto source_recent = [&](const builtin_interfaces::msg::Time& stamp) {
      const int64_t ns = stamp_ns(stamp);
      const double age = (ros_now_ns - ns) * 1e-9;
      return ns > 0 && age >= -0.05 && age <= position_feedback_timeout_sec_;
    };
    {
      std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      motor = motor_state_;
      loadcell = loadcell_data_;
      force = external_force_;
      motor_fresh = valid_motor_feedback_ && recent(motor_received_) && source_recent(motor.header.stamp);
      loadcell_fresh = valid_loadcell_feedback_ && recent(loadcell_received_) && source_recent(loadcell.header.stamp);
      force_fresh = valid_force_feedback_ && recent(force_received_);
    }

    auto fault_or_wait = [&](const std::string& reason) {
      if (position_ready_) {
        // Stop pursuing an old camera target if motor feedback is still valid.
        // This is a software hold, not an EtherCAT/physical emergency stop.
        hold_position_from_motor_feedback();
        position_ready_ = false;
        position_fault_latched_ = true;
      }
      set_position_status((position_fault_latched_ ? "FAULT (re-enter mode or toggle output): " : "WAITING: ") + reason);
    };

    try {
      if (new_frame) {
        const auto transforms = HRM_position_controller_.surgical_tool_.computeBaseToJointsTransformationMatrices(
          sample.pan_relative, sample.tilt_relative);
        const auto tip = HRM_position_controller_.surgical_tool_.computeEndEffectorPosition(transforms);
        if (!tip.allFinite()) {throw std::runtime_error("Non-finite FK tip.");}
        x_actual_.head<3>() = tip;
        tool_endeffector_pose_.data = {tip.x(), tip.y(), tip.z()};
        tool_endeffector_pose_publisher_->publish(tool_endeffector_pose_);
        // Measured-angle FK in metres. Preserve the source image stamp/frame
        // for offline comparison; neither control-loop time nor a held value.
        geometry_msgs::msg::PointStamped fk_tip_position;
        fk_tip_position.header = sample.header;
        fk_tip_position.point.x = tip.x();
        fk_tip_position.point.y = tip.y();
        fk_tip_position.point.z = tip.z();
        fk_tip_position_publisher_->publish(fk_tip_position);
      }
      if (!active) {
        position_ready_ = false;
        set_position_status("INACTIVE");
        continue;
      }
      if (position_fault_latched_) {continue;}
      if (!motor_fresh || !loadcell_fresh) {
        fault_or_wait("fresh four-channel motor and loadcell feedback required");
        continue;
      }
      if (std::any_of(loadcell.stress.begin(), loadcell.stress.end(),
          [](double tension) {return tension >= TENSION_LIMIT;}))
      {
        fault_or_wait("loadcell tension limit");
        continue;
      }
      if (mode == ControlMode::kAdmittance && !force_fresh) {
        fault_or_wait("fresh external force required (legacy admittance model still pending revision)");
        continue;
      }
      const bool image_fresh = sequence > 0 && recent(image_received) && source_recent(sample.header.stamp);
      if (!image_fresh) {
        // Camera gaps skip feedback control, not a latched actuator fault.
        // Keep the goal, IK/encoder references and last motor target unchanged.
        // Motor/loadcell/force guards above remain active while waiting.
        set_position_status("WAITING: fresh image required; auto-resume when available");
        continue;
      }
      // A mode/enable transition may not consume an image captured before it.
      if (!new_frame || sequence <= position_entry_min_sequence_) {continue;}
      const int64_t current_source_ns = stamp_ns(sample.header.stamp);
      if (current_source_ns < position_entry_source_ns_) {continue;}
      double dt = 0.0;
      std::vector<double> wire;
      if (!position_ready_) {
        double pan = 0.0, tilt = 0.0;
        for (std::size_t i = 0; i < NUM_OF_BENDING_JOINTS; ++i) {
          if (i % 2 == 0) {tilt += sample.tilt_relative[i];}
          else {pan += sample.pan_relative[i];}
        }
        // Aggregate active-joint angles initialize the cable IK operating point.
        // They are NOT the Euler orientation of the tip.
        HRM_position_controller_.reset(x_actual_, pan, tilt);
        x_desired_ = x_actual_;
        x_t_ = x_actual_;
        auto& filter = HRM_admittance_controller_.admittance_filter_;
        filter.xt_.setZero();
        filter.xtdot_.setZero();
        filter.xtddot_.setZero();
        HRM_admittance_controller_.f_desired_.setZero();
        HRM_admittance_controller_.f_env_.setZero();
        HRM_admittance_controller_.del_f_.setZero();
        del_xf_.setZero();
        wire = HRM_position_controller_.surgical_tool_.get_IK_result(
          pan * HRM_position_controller_.surgical_tool_.todeg(),
          tilt * HRM_position_controller_.surgical_tool_.todeg(), 0.0);
        for (int i = 0; i < NUM_OF_MOTORS; ++i) {
          position_motor_origin_[i] = motor.actual_position[i];
          position_wire_origin_[i] = wire[i];
        }
        position_ready_ = true;
        previous_source_ns = current_source_ns;
      } else {
        dt = (current_source_ns - previous_source_ns) * 1e-9;
        if (!std::isfinite(dt) || dt <= 0.0) {
          fault_or_wait("invalid source-frame interval");
          continue;
        }
        previous_source_ns = current_source_ns;
        if (mode == ControlMode::kAdmittance) {
          // Frame timing/reset is shared, but the legacy force mapping and
          // Y-only admittance are intentionally NOT redesigned in this task.
          f_env_(0) = force.y * 0.001;
          f_env_(1) = -force.x * 0.001;
          f_desired_(0) = 0.02;
          f_desired_(1) = 0.02;
          // Freeze the force integrator across a camera gap; no catch-up step.
          if (dt <= position_feedback_timeout_sec_) {
            del_xf_ = HRM_admittance_controller_.compute(f_desired_, f_env_, dt);
          }
          x_t_ = x_desired_ + del_xf_;
        } else {
          x_t_ = x_desired_;
        }
        wire = HRM_position_controller_.update(x_t_, x_actual_, dt);
      }

      MotorCommand target;
      target.header = sample.header;
      target.header.frame_id = "motor_target_position";
      target.target_position.resize(NUM_OF_MOTORS);
      // Small bring-up profile; angular slew is separately bounded in PD.
      target.target_velocity_profile.assign(NUM_OF_MOTORS, 10);
      bool target_valid = wire.size() >= NUM_OF_MOTORS;
      for (int i = 0; target_valid && i < NUM_OF_MOTORS; ++i) {
        const double counts = position_motor_origin_[i] +
          DIRECTION_COUPLER * (wire[i] - position_wire_origin_[i]) * 0.5 *
          gear_encoder_ratio_conversion(GEAR_RATIO, ENCODER_CHANNEL, ENCODER_RESOLUTION);
        target_valid = std::isfinite(counts) && std::abs(counts) <= MOTOR_SOFTWARE_LIMIT;
        if (target_valid) {target.target_position[i] = static_cast<int32_t>(std::llround(counts));}
      }
      if (!target_valid) {
        fault_or_wait("non-finite or out-of-range motor target");
        continue;
      }
      position_motor_preview_publisher_->publish(target);
      if (motor_output_enabled_) {motor_control_publisher_->publish(target);}
      publish_position_diagnostics(sample, dt, mode);
      set_position_status(motor_output_enabled_ ? "ACTIVE" : "READY (dry-run)");
    } catch (const std::exception& error) {
      fault_or_wait(error.what());
    }
  }
}
