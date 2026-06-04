#include "base_node.hpp"
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

BaseNode::BaseNode(const std::string & node_name,const rclcpp::NodeOptions & options):PX4Proxy(node_name,options),  
    interceptor_id_(declare_parameter<std::string>("interceptor_id", "SXINT001")),
    mission_id_(""),
    target_id_(""),
    mavlink_sys_id_(declare_parameter<int>("mavlink_sys_id", 1))
{
  RCLCPP_INFO(this->get_logger(), "Interceptor ID: %s", interceptor_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "mavlink_sys_id: %d", mavlink_sys_id_);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  c2_command_sub_    = create_subscription<cuas_msgs::msg::C2Command>("/cuas/c2/command",qos,std::bind(&BaseNode::_onC2Command, this, std::placeholders::_1));
  target_track_sub_  = create_subscription<cuas_msgs::msg::TargetTrack>("/cuas/c2/target_track",qos,std::bind(&BaseNode::_onC2TargetTrack, this, std::placeholders::_1));

  mission_ack_pub_ = create_publisher<cuas_msgs::msg::MissionAck>("/cuas/interceptor/ack", ReliableControlQoS());
  result_pub_ = create_publisher<cuas_msgs::msg::EngagementResult>("/cuas/interceptor/result", ReliableControlQoS());
  fault_pub_ = create_publisher<cuas_msgs::msg::FaultReport>("/cuas/interceptor/fault", ReliableControlQoS());
  snapshot_pub_ = create_publisher<cuas_msgs::msg::InterceptorSnapshot>("/cuas/interceptor/snapshot",BestEffortTelemetryQoS());

  control_timer_ = create_wall_timer(10ms,std::bind(&BaseNode::_tick, this));
  //snapshot_timer_ = create_wall_timer(100ms,std::bind(&BaseNode::publishSnapshot, this));

  RCLCPP_INFO(get_logger(), "baseNode initialized");
}

void BaseNode::tick(){
}

void BaseNode::onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)
{
 
}

void BaseNode::onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg)
{
  
}




void BaseNode::_tick(){
  tick();
}
void BaseNode::_onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)
{
 onC2Command(msg);
}

void BaseNode::_onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg)
{
  onC2TargetTrack(msg);
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

