#include "sentinelx_interceptor_node.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

SentinelXInterceptorNode::SentinelXInterceptorNode() : Node("sentinelx_interceptor_node"),
  interceptor_id_(declare_parameter<std::string>("interceptor_id", "SX-INT-001")),
  mission_id_(""),
  target_id_(""),
  mavlink_sys_id_(declare_parameter<int>("mavlink_sys_id", 1)),
  phase_(Phase::Idle),
  launched_(false),
  healthy_(true),
  has_px4_state_(false),
  has_c2_track_(false),
  seeker_ready_(false),
  seeker_detected_(false),
  seeker_locked_(false)
{
  RCLCPP_INFO(this->get_logger(),"Interceptor ID: %s",interceptor_id_.c_str());
  RCLCPP_INFO(this->get_logger(),"mavlink_sys_id: %d",mavlink_sys_id_);
  
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;  
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),qos_profile);

  c2_command_sub_ = this->create_subscription<cuas_msgs::msg::C2Command>("/cuas/c2/command",qos,std::bind(&SentinelXInterceptorNode::on_c2_command,this,std::placeholders::_1));
  intercept_mission_sub_ = this->create_subscription<cuas_msgs::msg::InterceptMission>("/cuas/c2/mission",qos,std::bind(&SentinelXInterceptorNode::on_c2_missionCallback,this,std::placeholders::_1));
  target_track_sub_ =	this->create_subscription<cuas_msgs::msg::TargetTrack>("/cuas/c2/target_track",qos,std::bind(	&SentinelXInterceptorNode::on_c2_targetTrackCallback,this,std::placeholders::_1));
/*
  c2_command_sub_ = create_subscription<cuas_msgs::msg::C2Command>(
    "/cuas/c2/command", 
    10,
    std::bind(&SentinelXInterceptorNode::on_c2_command, this, std::placeholders::_1));

  c2_track_sub_ = create_subscription<cuas_msgs::msg::TargetTrack>(
    "/cuas/c2/target_track", 
    10,
    std::bind(&SentinelXInterceptorNode::on_c2_target_track, this, std::placeholders::_1));
    */

  px4_state_sub_ = create_subscription<sentinelx::msg::PX4VehicleState>(
    "/sentinelx/px4/state", 
    10,
    std::bind(&SentinelXInterceptorNode::on_px4_state, this, std::placeholders::_1));

  seeker_status_sub_ = create_subscription<sentinelx::msg::SeekerStatus>(
    "/sentinelx/seeker/status", 
    10,
    std::bind(&SentinelXInterceptorNode::on_seeker_status, this, std::placeholders::_1));

  seeker_track_sub_ = create_subscription<sentinelx::msg::SeekerTrack>(
    "/sentinelx/seeker/track", 
    10,
    std::bind(&SentinelXInterceptorNode::on_seeker_track, this, std::placeholders::_1));

  guidance_pub_ = create_publisher<sentinelx::msg::GuidanceCommand>("/sentinelx/guidance/command", 10);
  phase_pub_ = create_publisher<sentinelx::msg::InterceptorPhase>("/sentinelx/interceptor/phase", 10);
  target_estimate_pub_ = create_publisher<sentinelx::msg::InternalTargetEstimate>("/sentinelx/interceptor/target_estimate", 10);
  mission_ack_pub_ = create_publisher<cuas_msgs::msg::MissionAck>("/sentinelx/interceptor/mission_ack", 10);
  status_pub_ = create_publisher<cuas_msgs::msg::InterceptorStatus>("/sentinelx/interceptor/status", 10);
  progress_pub_ = create_publisher<cuas_msgs::msg::InterceptProgress>("/sentinelx/interceptor/progress", 10);
  heartbeat_pub_ = create_publisher<cuas_msgs::msg::InterceptorHeartbeat>("/sentinelx/interceptor/heartbeat", 10);
  result_pub_ = create_publisher<cuas_msgs::msg::EngagementResult>("/sentinelx/interceptor/engagement_result", 10);
  fault_pub_ = create_publisher<cuas_msgs::msg::FaultReport>("/sentinelx/interceptor/fault_report", 10);

  control_timer_ = create_wall_timer(50ms, std::bind(&SentinelXInterceptorNode::control_loop, this));
  RCLCPP_INFO(get_logger(), "SentinelX interceptor node started in simulation-safe mode");
}




void SentinelXInterceptorNode::on_c2_command(const cuas_msgs::msg::C2Command::SharedPtr msg){
  if (launched_ && !c2_command_allowed_after_launch(msg->command_type)) {
    send_ack(*msg, false, cuas_msgs::msg::MissionAck::REJECTED, "C2 command ignored after launch; target updates remain accepted");
    return;
  }
  mission_id_ = msg->mission_id;
  target_id_ = msg->target_id;

  switch (msg->command_type) {
    case cuas_msgs::msg::C2Command::ASSIGN_TARGET:
      phase_ = Phase::MissionLoaded;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "mission loaded");
      break;
    case cuas_msgs::msg::C2Command::PREPARE_INTERCEPT:
      phase_ = Phase::ReadyToLaunch;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "ready to launch");
      break;
    case cuas_msgs::msg::C2Command::AUTHORIZE_LAUNCH:
      launched_ = true;
      phase_ = Phase::InertialMidcourse;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "launch accepted in simulation state machine");
      break;
    case cuas_msgs::msg::C2Command::ABORT:
      phase_ = Phase::ReturnHome;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "safe return requested");
      break;
    case cuas_msgs::msg::C2Command::RETURN_HOME:
      phase_ = Phase::ReturnHome;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "return home requested");
      break;
    case cuas_msgs::msg::C2Command::LAND:
      phase_ = Phase::Idle;
      launched_ = false;
      send_ack(*msg, true, cuas_msgs::msg::MissionAck::OK, "land/idle requested");
      break;
    default:
      send_ack(*msg, false, cuas_msgs::msg::MissionAck::REJECTED, "unsupported command for this simulation kit");
      break;
  }
}

void SentinelXInterceptorNode::on_c2_missionCallback(const cuas_msgs::msg::InterceptMission::SharedPtr msg){
}

void SentinelXInterceptorNode::on_c2_targetTrackCallback(const cuas_msgs::msg::TargetTrack::SharedPtr msg){
  if (!target_id_.empty() && msg->target_id != target_id_) {
    return;
  }
  latest_c2_track_ = *msg;
  has_c2_track_ = true;
}

void SentinelXInterceptorNode::on_px4_state(const sentinelx::msg::PX4VehicleState::SharedPtr msg)
{
  latest_px4_state_ = *msg;
  has_px4_state_ = true;
}

void SentinelXInterceptorNode::on_seeker_status(const sentinelx::msg::SeekerStatus::SharedPtr msg)
{
  seeker_ready_ = msg->ready;
  seeker_detected_ = msg->target_detected;
  seeker_locked_ = msg->target_locked;
  if (!msg->target_id.empty()) {
    target_id_ = msg->target_id;
  }
}

void SentinelXInterceptorNode::on_seeker_track(const sentinelx::msg::SeekerTrack::SharedPtr msg)
{
  latest_seeker_track_ = *msg;
  if (msg->valid && msg->locked) {
    phase_ = Phase::TerminalHoming;
    seeker_locked_ = true;
  } else if (msg->valid && launched_ && phase_ == Phase::InertialMidcourse) {
    phase_ = Phase::SeekerSearch;
    seeker_detected_ = true;
  }
}

void SentinelXInterceptorNode::control_loop()
{
  if (launched_ && seeker_locked_) {
    phase_ = Phase::TerminalHoming;
  } else if (launched_ && seeker_detected_ && phase_ == Phase::InertialMidcourse) {
    phase_ = Phase::SeekerSearch;
  }

  publish_guidance();
  publish_status();
  publish_progress();
  publish_heartbeat();
  publish_phase();
  publish_target_estimate();
}

void SentinelXInterceptorNode::publish_guidance()
{
  sentinelx::msg::GuidanceCommand msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mission_id = mission_id_;
  msg.target_id = target_id_;

  if (phase_ == Phase::TerminalHoming && seeker_locked_) {
    msg.guidance_mode = sentinelx::msg::GuidanceCommand::GUIDANCE_SEEKER_TERMINAL;
    msg.velocity_x_mps = latest_seeker_track_.normalized_x;
    msg.velocity_y_mps = latest_seeker_track_.normalized_y;
    msg.velocity_z_mps = 0.0F;
    msg.use_velocity = true;
    msg.terminal_phase = true;
  } else if (phase_ == Phase::InertialMidcourse || phase_ == Phase::SeekerSearch) {
    msg.guidance_mode = sentinelx::msg::GuidanceCommand::GUIDANCE_GPS_INS;
    msg.use_position = true;
    msg.target_x_m = 0.0F;
    msg.target_y_m = 0.0F;
    msg.target_z_m = -10.0F;
  } else if (phase_ == Phase::ReturnHome) {
    msg.guidance_mode = sentinelx::msg::GuidanceCommand::GUIDANCE_RETURN_HOME;
  } else {
    msg.guidance_mode = sentinelx::msg::GuidanceCommand::GUIDANCE_HOLD;
  }
  guidance_pub_->publish(msg);
}

void SentinelXInterceptorNode::publish_status()
{
  cuas_msgs::msg::InterceptorStatus msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mission_id = mission_id_;
  msg.vehicle_state = to_cuas_state();
  if (has_px4_state_) {
    msg.latitude = latest_px4_state_.latitude_deg;
    msg.longitude = latest_px4_state_.longitude_deg;
    msg.altitude = latest_px4_state_.altitude_m;
    msg.velocity_x = latest_px4_state_.velocity_x_mps;
    msg.velocity_y = latest_px4_state_.velocity_y_mps;
    msg.velocity_z = latest_px4_state_.velocity_z_mps;
    msg.battery_remaining = latest_px4_state_.battery_remaining;
    msg.armed = latest_px4_state_.armed;
    msg.offboard_enabled = latest_px4_state_.offboard_enabled;
  }
  msg.healthy = healthy_;
  status_pub_->publish(msg);
}

void SentinelXInterceptorNode::publish_progress()
{
  cuas_msgs::msg::InterceptProgress msg;
  msg.stamp = now();
  msg.mission_id = mission_id_;
  msg.interceptor_id = interceptor_id_;
  msg.target_id = target_id_;
  msg.phase = to_cuas_progress_phase();
  msg.status_text = "simulation-safe state machine";
  progress_pub_->publish(msg);
}

void SentinelXInterceptorNode::publish_heartbeat()
{
  cuas_msgs::msg::InterceptorHeartbeat msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mavlink_sys_id = mavlink_sys_id_;
  msg.status = launched_ ? cuas_msgs::msg::InterceptorHeartbeat::STATUS_TRACKING : cuas_msgs::msg::InterceptorHeartbeat::STATUS_READY;
  msg.mode = cuas_msgs::msg::InterceptorHeartbeat::MODE_OFFBOARD;
  msg.armed = has_px4_state_ ? latest_px4_state_.armed : false;
  msg.connected = has_px4_state_ ? latest_px4_state_.connected : false;
  msg.offboard_available = true;
  msg.battery_percent = has_px4_state_ ? latest_px4_state_.battery_remaining : 0.0F;
  msg.battery_voltage = has_px4_state_ ? latest_px4_state_.battery_voltage_v : 0.0F;
  msg.mission_id = mission_id_;
  msg.target_id = target_id_;
  msg.message = "simulation-safe heartbeat";
  heartbeat_pub_->publish(msg);
}

void SentinelXInterceptorNode::publish_phase()
{
  sentinelx::msg::InterceptorPhase msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mission_id = mission_id_;
  msg.target_id = target_id_;
  msg.phase = static_cast<uint8_t>(phase_);
  msg.launched = launched_;
  msg.terminal_phase = phase_ == Phase::TerminalHoming;
  msg.c2_command_allowed = !launched_;
  msg.c2_target_update_allowed = true;
  msg.authority = msg.terminal_phase ? sentinelx::msg::InterceptorPhase::AUTHORITY_SEEKER : sentinelx::msg::InterceptorPhase::AUTHORITY_INERTIAL;
  phase_pub_->publish(msg);
}

void SentinelXInterceptorNode::publish_target_estimate()
{
  sentinelx::msg::InternalTargetEstimate msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.target_id = target_id_;
  msg.valid = has_c2_track_ || seeker_detected_;
  msg.source = seeker_locked_ ? sentinelx::msg::InternalTargetEstimate::SOURCE_SEEKER : sentinelx::msg::InternalTargetEstimate::SOURCE_C2_TRACK;
  msg.confidence = seeker_locked_ ? latest_seeker_track_.confidence : latest_c2_track_.confidence;
  target_estimate_pub_->publish(msg);
}

void SentinelXInterceptorNode::send_ack(const cuas_msgs::msg::C2Command & cmd, bool accepted, uint8_t code, const std::string & message)
{
  cuas_msgs::msg::MissionAck ack;
  ack.stamp = now();
  ack.command_id = cmd.command_id;
  ack.mission_id = cmd.mission_id;
  ack.interceptor_id = cmd.interceptor_id.empty() ? interceptor_id_ : cmd.interceptor_id;
  ack.accepted = accepted;
  ack.result_code = code;
  ack.message = message;
  mission_ack_pub_->publish(ack);
}

bool SentinelXInterceptorNode::c2_command_allowed_after_launch(uint8_t command_type) const
{
  return command_type == cuas_msgs::msg::C2Command::ABORT ||
         command_type == cuas_msgs::msg::C2Command::RETURN_HOME ||
         command_type == cuas_msgs::msg::C2Command::LAND;
}

uint8_t SentinelXInterceptorNode::to_cuas_state() const
{
  switch (phase_) {
    case Phase::Idle: return cuas_msgs::msg::InterceptorStatus::IDLE;
    case Phase::MissionLoaded: return cuas_msgs::msg::InterceptorStatus::READY;
    case Phase::ReadyToLaunch: return cuas_msgs::msg::InterceptorStatus::ARMED;
    case Phase::ReturnHome: return cuas_msgs::msg::InterceptorStatus::RETURNING;
    case Phase::Fault: return cuas_msgs::msg::InterceptorStatus::FAULT;
    default: return cuas_msgs::msg::InterceptorStatus::ACTIVE;
  }
}

uint8_t SentinelXInterceptorNode::to_cuas_progress_phase() const
{
  switch (phase_) {
    case Phase::MissionLoaded: return cuas_msgs::msg::InterceptProgress::ASSIGNED;
    case Phase::ReadyToLaunch: return cuas_msgs::msg::InterceptProgress::PREPARING;
    case Phase::Launched: return cuas_msgs::msg::InterceptProgress::LAUNCHED;
    case Phase::InertialMidcourse: return cuas_msgs::msg::InterceptProgress::MIDCOURSE;
    case Phase::SeekerSearch: return cuas_msgs::msg::InterceptProgress::COOPERATIVE_TRACKING;
    case Phase::TerminalHoming: return cuas_msgs::msg::InterceptProgress::TERMINAL_APPROACH_SIM;
    case Phase::InterceptSuccess: return cuas_msgs::msg::InterceptProgress::COMPLETED;
    case Phase::InterceptFailed: return cuas_msgs::msg::InterceptProgress::FAILED;
    default: return cuas_msgs::msg::InterceptProgress::NONE;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SentinelXInterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
