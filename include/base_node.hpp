#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/mission_ack.hpp"
#include "cuas_msgs/msg/target_track.hpp"
#include "cuas_msgs/msg/interceptor_status.hpp"
#include "cuas_msgs/msg/intercept_progress.hpp"
#include "cuas_msgs/msg/interceptor_snapshot.hpp"
#include "cuas_msgs/msg/engagement_result.hpp"
#include "cuas_msgs/msg/fault_report.hpp"
//#include "cuas_msgs/msg/c2_command.hpp"
//#include "cuas_msgs/msg/intercept_mission.hpp"
//#include "sentinelx/msg/guidance_command.hpp"
#include "sentinelx/msg/px4_vehicle_state.hpp"
#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
//#include "sentinelx/msg/interceptor_phase.hpp"
//#include "sentinelx/msg/internal_target_estimate.hpp"
#include "px4_proxy.hpp"

enum class Phase : uint8_t
{
  Idle = 0,
  MissionLoaded = 1,
  ReadyToLaunch = 2,
  Launched = 3,
  InertialMidcourse = 4,
  SeekerSearch = 5,
  SeekerLocked = 6,
  TerminalHoming = 7,
  InterceptSuccess = 8,
  InterceptFailed = 9,
  ReturnHome = 10,
  Fault = 11
};

class BaseNode : public PX4Proxy
{
public:
  BaseNode();

protected:

  void onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  void onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
  virtual void controlLoop();
  virtual void publishSnapshot();
  void sendAck(const cuas_msgs::msg::C2Command & cmd, bool accepted, uint8_t code, const std::string & message);


  std::string interceptor_id_;
  std::string mission_id_;
  std::string target_id_;
  int32_t mavlink_sys_id_;

  rclcpp::Subscription<cuas_msgs::msg::C2Command>::SharedPtr c2_command_sub_;
  rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr target_track_sub_;
  rclcpp::Publisher<cuas_msgs::msg::MissionAck>::SharedPtr mission_ack_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<cuas_msgs::msg::EngagementResult>::SharedPtr result_pub_;
  rclcpp::Publisher<cuas_msgs::msg::FaultReport>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;
};

