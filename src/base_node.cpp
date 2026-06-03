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

BaseNode::BaseNode():PX4Proxy("launch_node"),  
    interceptor_id_(declare_parameter<std::string>("interceptor_id", "SX-INT-001")),
    mission_id_(""),
    target_id_(""),
    mavlink_sys_id_(declare_parameter<int>("mavlink_sys_id", 1))
{
  RCLCPP_INFO(this->get_logger(), "Interceptor ID: %s", interceptor_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "mavlink_sys_id: %d", mavlink_sys_id_);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  c2_command_sub_    = create_subscription<cuas_msgs::msg::C2Command>("/cuas/c2/command",qos,std::bind(&BaseNode::onC2Command, this, std::placeholders::_1));
  target_track_sub_  = create_subscription<cuas_msgs::msg::TargetTrack>("/cuas/c2/target_track",qos,std::bind(&BaseNode::onC2TargetTrack, this, std::placeholders::_1));

  mission_ack_pub_ = create_publisher<cuas_msgs::msg::MissionAck>("/cuas/interceptor/ack", ReliableControlQoS());
  result_pub_ = create_publisher<cuas_msgs::msg::EngagementResult>("/cuas/interceptor/result", ReliableControlQoS());
  fault_pub_ = create_publisher<cuas_msgs::msg::FaultReport>("/cuas/interceptor/fault", ReliableControlQoS());
  snapshot_pub_ = create_publisher<cuas_msgs::msg::InterceptorSnapshot>("/cuas/interceptor/snapshot",BestEffortTelemetryQoS());

  control_timer_ = create_wall_timer(50ms,std::bind(&BaseNode::controlLoop, this));
  snapshot_timer_ = create_wall_timer(100ms,std::bind(&BaseNode::publishSnapshot, this));

  RCLCPP_INFO(get_logger(), "SentinelX interceptor node started in fire-and-forget mode");
}

void BaseNode::onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)
{
  if (launched_ && !c2_command_allowed_after_launch(msg->command_type)) {
    sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"C2 command ignored after launch; only ABORT is accepted");
    return;
  }

  switch (msg->command_type) {
    case cuas_msgs::msg::C2Command::ASSIGN_TARGET:
    {
      RCLCPP_INFO(get_logger(), "C2Command::ASSIGN_TARGET   received: mission_id=%s, target_id=%s",msg->mission_id.c_str(), msg->target_id.c_str());

      if (launched_) {
        sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"cannot assign target after launch");
        return;
      }

      if (msg->mission_id.empty() || msg->target_id.empty()) {
        sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"mission_id or target_id is empty");
        return;
      }

      mission_id_ = msg->mission_id;
      target_id_ = msg->target_id;

      phase_ = Phase::MissionLoaded;
      launched_ = false;
      seeker_detected_ = false;
      seeker_locked_ = false;

      sendAck(*msg,true,cuas_msgs::msg::MissionAck::OK,"target assigned; interceptor is ready for START_INTERCEPT");
      break;
    }

    case cuas_msgs::msg::C2Command::START_INTERCEPT:
    {
      RCLCPP_INFO(get_logger(), "C2Command::START_INTERCEPT received: mission_id=%s, target_id=%s",msg->mission_id.c_str(), msg->target_id.c_str()); 
      if (launched_) {
        sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"interceptor already launched");
        return;
      }

      if (mission_id_.empty() || target_id_.empty()) {
        sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"no assigned mission or target");
        return;
      }

      launched_ = true;
      phase_ = Phase::InertialMidcourse;
      sendAck(*msg,true,cuas_msgs::msg::MissionAck::OK,"START_INTERCEPT accepted; interceptor launched");
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
      sendAck(*msg,true,cuas_msgs::msg::MissionAck::OK,"ABORT accepted; safe action will be selected internally");
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
      sendAck(*msg,true,cuas_msgs::msg::MissionAck::OK,"HIT accepted; safe action will be selected internally");
      break;
    }

    
    default:
    {
      sendAck(*msg,false,cuas_msgs::msg::MissionAck::REJECTED,"unsupported command; allowed commands are ASSIGN_TARGET, START_INTERCEPT, ABORT");
      break;
    }
  }
}

void BaseNode::onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg)
{
  if (!target_id_.empty() && msg->target_id != target_id_) {
    return;
  }
  latest_c2_track_ = *msg;
  has_c2_track_ = true;
}

void BaseNode::controlLoop()
{
  if (launched_ && seeker_locked_) {
    phase_ = Phase::TerminalHoming;
  } else if (launched_ && seeker_detected_ && phase_ == Phase::InertialMidcourse) {
    phase_ = Phase::SeekerSearch;
  }
  //publish_guidance();
  //publish_phase();
  //publish_target_estimate();
}

void BaseNode::onPX4Updated()
{
  if (px4_ready()) {
    healthy_ = true;
    RCLCPP_INFO(this->get_logger(), "Launch Node Ready!!!!");
  } else {
    healthy_ = false;
  }
}

void BaseNode::publishSnapshot()
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

void BaseNode::sendAck(const cuas_msgs::msg::C2Command & cmd,bool accepted,uint8_t code,const std::string & message)
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

void BaseNode::controlLoop() {

}

void BaseNode::publishSnapshot() {

}

