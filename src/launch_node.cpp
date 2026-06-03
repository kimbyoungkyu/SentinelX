#include "launch_node.hpp"
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

inline rclcpp::QoS ReliableControlQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
}

inline rclcpp::QoS BestEffortTelemetryQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
}

LaunchNode::LaunchNode():PX4Proxy("launch_node"),  
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
  RCLCPP_INFO(this->get_logger(), "Interceptor ID: %s", interceptor_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "mavlink_sys_id: %d", mavlink_sys_id_);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  c2_command_sub_    = create_subscription<cuas_msgs::msg::C2Command>("/cuas/c2/command",qos,std::bind(&LaunchNode::on_c2_command, this, std::placeholders::_1));
  target_track_sub_  = create_subscription<cuas_msgs::msg::TargetTrack>("/cuas/c2/target_track",qos,std::bind(&LaunchNode::on_c2_targetTrackCallback, this, std::placeholders::_1));
  px4_state_sub_     = create_subscription<sentinelx::msg::PX4VehicleState>("/sentinelx/px4/state",10,std::bind(&LaunchNode::on_px4_state, this, std::placeholders::_1));
  seeker_status_sub_ = create_subscription<sentinelx::msg::SeekerStatus>("/sentinelx/seeker/status",10,std::bind(&LaunchNode::on_seeker_status, this, std::placeholders::_1));
  seeker_track_sub_  = create_subscription<sentinelx::msg::SeekerTrack>("/sentinelx/seeker/track", 10,std::bind(&LaunchNode::on_seeker_track, this, std::placeholders::_1));

  guidance_pub_ = create_publisher<sentinelx::msg::GuidanceCommand>("/sentinelx/guidance/command", 10);
  phase_pub_ = create_publisher<sentinelx::msg::InterceptorPhase>("/sentinelx/interceptor/phase", 10);
  target_estimate_pub_ = create_publisher<sentinelx::msg::InternalTargetEstimate>("/sentinelx/interceptor/target_estimate", 10);
  mission_ack_pub_ = create_publisher<cuas_msgs::msg::MissionAck>("/cuas/interceptor/ack", ReliableControlQoS());
  result_pub_ = create_publisher<cuas_msgs::msg::EngagementResult>("/cuas/interceptor/result", ReliableControlQoS());
  fault_pub_ = create_publisher<cuas_msgs::msg::FaultReport>("/cuas/interceptor/fault", ReliableControlQoS());
  snapshot_pub_ = create_publisher<cuas_msgs::msg::InterceptorSnapshot>("/cuas/interceptor/snapshot",BestEffortTelemetryQoS());

  control_timer_ = create_wall_timer(50ms,std::bind(&LaunchNode::control_loop, this));
  snapshot_timer_ = create_wall_timer(100ms,std::bind(&LaunchNode::publish_snapshot, this));

  RCLCPP_INFO(get_logger(), "SentinelX interceptor node started in fire-and-forget mode");
}

void LaunchNode::on_c2_command(const cuas_msgs::msg::C2Command::SharedPtr msg)
{
  if (launched_ && !c2_command_allowed_after_launch(msg->command_type)) {
    send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"C2 command ignored after launch; only ABORT is accepted");
    return;
  }

  switch (msg->command_type) {
    case cuas_msgs::msg::C2Command::ASSIGN_TARGET:
    {
      RCLCPP_INFO(get_logger(), "C2Command::ASSIGN_TARGET   received: mission_id=%s, target_id=%s",
        msg->mission_id.c_str(), msg->target_id.c_str());

      if (launched_) {
        send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"cannot assign target after launch");
        return;
      }

      if (msg->mission_id.empty() || msg->target_id.empty()) {
        send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"mission_id or target_id is empty");
        return;
      }

      mission_id_ = msg->mission_id;
      target_id_ = msg->target_id;

      phase_ = Phase::MissionLoaded;
      launched_ = false;
      seeker_detected_ = false;
      seeker_locked_ = false;

      send_ack(*msg,true,cuas_msgs::msg::MissionAck::OK,"target assigned; interceptor is ready for START_INTERCEPT");
      break;
    }

    case cuas_msgs::msg::C2Command::START_INTERCEPT:
    {
      RCLCPP_INFO(get_logger(), "C2Command::START_INTERCEPT received: mission_id=%s, target_id=%s",msg->mission_id.c_str(), msg->target_id.c_str()); 
      if (launched_) {
        send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"interceptor already launched");
        return;
      }

      if (mission_id_.empty() || target_id_.empty()) {
        send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"no assigned mission or target");
        return;
      }

      launched_ = true;
      phase_ = Phase::InertialMidcourse;
      send_ack(*msg,true,cuas_msgs::msg::MissionAck::OK,"START_INTERCEPT accepted; interceptor launched");
      break;
    }
    case cuas_msgs::msg::C2Command::ABORT:
    {
      RCLCPP_INFO(get_logger(), "C2Command::ABORT received: mission_id=%s, target_id=%s", msg->mission_id.c_str(), msg->target_id.c_str()); 
      if (launched_) {
        phase_ = Phase::ReturnHome;
      } else {
        phase_ = Phase::Idle;
        mission_id_.clear();
        target_id_.clear();
      }

      launched_ = false;
      seeker_detected_ = false;
      seeker_locked_ = false;
      send_ack(*msg,true,cuas_msgs::msg::MissionAck::OK,"ABORT accepted; safe action will be selected internally");
      break;
    }

    case cuas_msgs::msg::C2Command::HIT:
    {
      RCLCPP_INFO(get_logger(), "C2Command::HIT received: mission_id=%s, target_id=%s", msg->mission_id.c_str(), msg->target_id.c_str()); 
      if (launched_) {
        phase_ = Phase::ReturnHome;
      } else {
        phase_ = Phase::Idle;
        mission_id_.clear();
        target_id_.clear();
      }

      launched_ = false;
      seeker_detected_ = false;
      seeker_locked_ = false;
      send_ack(*msg,true,cuas_msgs::msg::MissionAck::OK,"HIT accepted; safe action will be selected internally");
      break;
    }

    
    default:
    {
      send_ack(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"unsupported command; allowed commands are ASSIGN_TARGET, START_INTERCEPT, ABORT");
      break;
    }
  }
}

void LaunchNode::on_c2_targetTrackCallback(const cuas_msgs::msg::TargetTrack::SharedPtr msg)
{
  if (!target_id_.empty() && msg->target_id != target_id_) {
    return;
  }
  latest_c2_track_ = *msg;
  has_c2_track_ = true;
}

void LaunchNode::on_px4_state(const sentinelx::msg::PX4VehicleState::SharedPtr msg)
{
  latest_px4_state_ = *msg;
  has_px4_state_ = true;
}

void LaunchNode::on_seeker_status(const sentinelx::msg::SeekerStatus::SharedPtr msg)
{
  seeker_ready_ = msg->ready;
  seeker_detected_ = msg->target_detected;
  seeker_locked_ = msg->target_locked;

  if (!msg->target_id.empty()) {
    target_id_ = msg->target_id;
  }
}

void LaunchNode::on_seeker_track(const sentinelx::msg::SeekerTrack::SharedPtr msg)
{
  latest_seeker_track_ = *msg;

  if (msg->valid && msg->locked && launched_) {
    phase_ = Phase::TerminalHoming;
    seeker_locked_ = true;
  } else if (msg->valid && launched_ && phase_ == Phase::InertialMidcourse) {
    phase_ = Phase::SeekerSearch;
    seeker_detected_ = true;
  }
}

void LaunchNode::control_loop()
{
  if (launched_ && seeker_locked_) {
    phase_ = Phase::TerminalHoming;
  } else if (launched_ && seeker_detected_ && phase_ == Phase::InertialMidcourse) {
    phase_ = Phase::SeekerSearch;
  }
  publish_guidance();
  publish_phase();
  publish_target_estimate();
}

void LaunchNode::onPX4Updated()
{
  if (px4_ready()) {
    healthy_ = true;
    RCLCPP_INFO(this->get_logger(), "Launch Node Ready!!!!");
  } else {
    healthy_ = false;
  }
}
void LaunchNode::publish_guidance()
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
    // ABORT 이후 실제 RTH/LAND/DISARM 정책은 내부 guidance/PX4 계층에서 결정.
    // 필요 시 GuidanceCommand에 ABORT/SAFE_MODE/RETURN_HOME 모드를 추가해서 사용.
  }

  guidance_pub_->publish(msg);
}

void LaunchNode::publish_snapshot()
{
  //px4가 준비된 상태가 아니면 보내지 않는다.
  if (!px4_ready()) return;

  cuas_msgs::msg::InterceptorSnapshot msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mavlink_sys_id = mavlink_sys_id_;

  if (phase_ == Phase::Idle) {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
  } else if (phase_ == Phase::MissionLoaded) {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
  } else {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_TRACKING;
  }

  msg.mode = cuas_msgs::msg::InterceptorSnapshot::MODE_OFFBOARD;
  msg.armed = has_px4_state_ ? latest_px4_state_.armed : false;
  msg.connected = has_px4_state_ ? latest_px4_state_.connected : false;
  msg.offboard_available = true;
  msg.battery_percent = has_px4_state_ ? latest_px4_state_.battery_remaining : 0.0F;
  msg.battery_voltage = has_px4_state_ ? latest_px4_state_.battery_voltage_v : 0.0F;

  msg.mission_id = mission_id_;
  msg.target_id = target_id_;
  msg.message = "fire-and-forget snapshot";

  snapshot_pub_->publish(msg);
}

void LaunchNode::publish_phase()
{
  sentinelx::msg::InterceptorPhase msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.mission_id = mission_id_;
  msg.target_id = target_id_;
  msg.phase = static_cast<uint8_t>(phase_);
  msg.launched = launched_;
  msg.terminal_phase = phase_ == Phase::TerminalHoming;

  // Fire-and-forget:
  // 발사 전에는 ASSIGN_TARGET / START_INTERCEPT 가능.
  // 발사 후에는 ABORT만 가능.
  msg.c2_command_allowed = !launched_;
  msg.c2_target_update_allowed = true;

  msg.authority = msg.terminal_phase
    ? sentinelx::msg::InterceptorPhase::AUTHORITY_SEEKER
    : sentinelx::msg::InterceptorPhase::AUTHORITY_INERTIAL;

  phase_pub_->publish(msg);
}

void LaunchNode::publish_target_estimate()
{
  sentinelx::msg::InternalTargetEstimate msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.target_id = target_id_;
  msg.valid = has_c2_track_ || seeker_detected_;
  msg.source = seeker_locked_
    ? sentinelx::msg::InternalTargetEstimate::SOURCE_SEEKER
    : sentinelx::msg::InternalTargetEstimate::SOURCE_C2_TRACK;
  msg.confidence = seeker_locked_
    ? latest_seeker_track_.confidence
    : latest_c2_track_.confidence;

  target_estimate_pub_->publish(msg);
}

void LaunchNode::send_ack(const cuas_msgs::msg::C2Command & cmd,bool accepted,uint8_t code,const std::string & message)
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

bool LaunchNode::c2_command_allowed_after_launch(uint8_t command_type) const
{
  return command_type == cuas_msgs::msg::C2Command::ABORT;
}

uint8_t LaunchNode::to_cuas_state() const
{
  switch (phase_) {
    case Phase::Idle:
      return cuas_msgs::msg::InterceptorStatus::IDLE;

    case Phase::MissionLoaded:
      return cuas_msgs::msg::InterceptorStatus::READY;

    case Phase::InertialMidcourse:
    case Phase::SeekerSearch:
    case Phase::TerminalHoming:
      return cuas_msgs::msg::InterceptorStatus::ACTIVE;

    case Phase::ReturnHome:
      return cuas_msgs::msg::InterceptorStatus::RETURNING;

    case Phase::Fault:
      return cuas_msgs::msg::InterceptorStatus::FAULT;

    default:
      return cuas_msgs::msg::InterceptorStatus::ACTIVE;
  }
}

uint8_t LaunchNode::to_cuas_progress_phase() const
{
  switch (phase_) {
    case Phase::Idle:
      return cuas_msgs::msg::InterceptProgress::NONE;

    case Phase::MissionLoaded:
      return cuas_msgs::msg::InterceptProgress::ASSIGNED;

    case Phase::Launched:
      return cuas_msgs::msg::InterceptProgress::LAUNCHED;

    case Phase::InertialMidcourse:
      return cuas_msgs::msg::InterceptProgress::MIDCOURSE;

    case Phase::SeekerSearch:
      return cuas_msgs::msg::InterceptProgress::COOPERATIVE_TRACKING;

    case Phase::TerminalHoming:
      return cuas_msgs::msg::InterceptProgress::TERMINAL_APPROACH_SIM;

    case Phase::InterceptSuccess:
      return cuas_msgs::msg::InterceptProgress::COMPLETED;

    case Phase::InterceptFailed:
      return cuas_msgs::msg::InterceptProgress::FAILED;

    case Phase::ReturnHome:
      return cuas_msgs::msg::InterceptProgress::FAILED;

    default:
      return cuas_msgs::msg::InterceptProgress::NONE;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaunchNode>());
  rclcpp::shutdown();
  return 0;
}