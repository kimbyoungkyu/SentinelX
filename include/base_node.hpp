#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/mission_ack.hpp"
#include "cuas_msgs/msg/target_track.hpp"
#include "cuas_msgs/msg/interceptor_status.hpp"
#include "cuas_msgs/msg/intercept_progress.hpp"
#include "cuas_msgs/msg/interceptor_snapshot.hpp"
#include "cuas_msgs/msg/engagement_result.hpp"
#include "cuas_msgs/msg/fault_report.hpp"
#include "sentinelx/msg/px4_vehicle_state.hpp"
#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
#include "px4_proxy.hpp"

class BaseNode : public PX4Proxy
{
public:
  BaseNode(const std::string & node_name,const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void sendAck(const cuas_msgs::msg::C2Command & cmd, bool accepted, uint8_t code, const std::string & message);

protected:  
  virtual void onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  virtual void onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
  virtual void tick();

protected:  
  std::string interceptor_id_;
  std::string mission_id_;
  std::string target_id_;
  int32_t mavlink_sys_id_;

protected:  
  rclcpp::Subscription<cuas_msgs::msg::C2Command>::SharedPtr c2_command_sub_;
  rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr target_track_sub_;
  rclcpp::Publisher<cuas_msgs::msg::MissionAck>::SharedPtr mission_ack_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<cuas_msgs::msg::EngagementResult>::SharedPtr result_pub_;
  rclcpp::Publisher<cuas_msgs::msg::FaultReport>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;


private:
  void _tick();
  void _onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  void _onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
};

