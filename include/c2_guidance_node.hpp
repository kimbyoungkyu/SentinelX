#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/mission_ack.hpp"
#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
#include "sentinelx/msg/interceptor_phase.hpp"
#include "sentinelx/msg/interceptor_health.hpp"
#include "cuas_msgs/msg/target_track.hpp"
#include "cuas_msgs/msg/engagement_result.hpp"
#include "cuas_msgs/msg/fault_report.hpp"
#include "cuas_msgs/msg/interceptor_snapshot.hpp"
#include "px4_proxy.hpp"

class C2GuidanceNode : public PX4Proxy
{
public:
  C2GuidanceNode();

private:
/*
  void onPhase(const sentinelx::msg::InterceptorPhase::SharedPtr msg);
  */
  void onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  void onTargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
  virtual void onPX4Updated() override;

  //void publish_seeker();


  

  void sendAck(const cuas_msgs::msg::C2Command & cmd, bool accepted, uint8_t code, const std::string & message);
  

  std::string interceptor_id_;
  std::string target_id_;
  bool simulate_detection_;
  bool simulate_lock_;
  uint32_t frame_id_;
  uint8_t current_phase_;

  //rclcpp::Subscription<sentinelx::msg::InterceptorPhase>::SharedPtr phase_sub_;
  //rclcpp::Publisher<sentinelx::msg::SeekerStatus>::SharedPtr status_pub_;
  //rclcpp::Publisher<sentinelx::msg::TargetTrack>::SharedPtr track_pub_;
  //rclcpp::Publisher<sentinelx::msg::InterceptorHealth>::SharedPtr health_pub_;

  rclcpp::Subscription<cuas_msgs::msg::C2Command>::SharedPtr c2_command_sub_;
  rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr target_track_sub_;


  rclcpp::Publisher<cuas_msgs::msg::MissionAck>::SharedPtr mission_ack_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<cuas_msgs::msg::EngagementResult>::SharedPtr result_pub_;
  rclcpp::Publisher<cuas_msgs::msg::FaultReport>::SharedPtr fault_pub_;



  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr tHeartbeat_;
};

