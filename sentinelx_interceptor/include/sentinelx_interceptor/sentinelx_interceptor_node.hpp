#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/mission_ack.hpp"
#include "cuas_msgs/msg/target_track.hpp"
#include "cuas_msgs/msg/interceptor_status.hpp"
#include "cuas_msgs/msg/intercept_progress.hpp"
#include "cuas_msgs/msg/interceptor_heartbeat.hpp"
#include "cuas_msgs/msg/engagement_result.hpp"
#include "cuas_msgs/msg/fault_report.hpp"

#include "sentinelx_msgs/msg/guidance_command.hpp"
#include "sentinelx_msgs/msg/px4_vehicle_state.hpp"
#include "sentinelx_msgs/msg/seeker_status.hpp"
#include "sentinelx_msgs/msg/seeker_track.hpp"
#include "sentinelx_msgs/msg/interceptor_phase.hpp"
#include "sentinelx_msgs/msg/internal_target_estimate.hpp"

namespace sentinelx_interceptor
{

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

class SentinelXInterceptorNode : public rclcpp::Node
{
public:
  SentinelXInterceptorNode();

private:
  void on_c2_command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  void on_c2_target_track(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
  void on_px4_state(const sentinelx_msgs::msg::PX4VehicleState::SharedPtr msg);
  void on_seeker_status(const sentinelx_msgs::msg::SeekerStatus::SharedPtr msg);
  void on_seeker_track(const sentinelx_msgs::msg::SeekerTrack::SharedPtr msg);

  void control_loop();
  void publish_guidance();
  void publish_status();
  void publish_progress();
  void publish_heartbeat();
  void publish_phase();
  void publish_target_estimate();
  void send_ack(const cuas_msgs::msg::C2Command & cmd, bool accepted, uint8_t code, const std::string & message);

  bool c2_command_allowed_after_launch(uint8_t command_type) const;
  uint8_t to_cuas_state() const;
  uint8_t to_cuas_progress_phase() const;

  std::string interceptor_id_;
  std::string mission_id_;
  std::string target_id_;
  int32_t mavlink_sys_id_;

  Phase phase_;
  bool launched_;
  bool healthy_;
  bool has_px4_state_;
  bool has_c2_track_;
  bool seeker_ready_;
  bool seeker_detected_;
  bool seeker_locked_;

  cuas_msgs::msg::TargetTrack latest_c2_track_;
  sentinelx_msgs::msg::PX4VehicleState latest_px4_state_;
  sentinelx_msgs::msg::SeekerTrack latest_seeker_track_;

  rclcpp::Subscription<cuas_msgs::msg::C2Command>::SharedPtr c2_command_sub_;
  rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr c2_track_sub_;
  rclcpp::Subscription<sentinelx_msgs::msg::PX4VehicleState>::SharedPtr px4_state_sub_;
  rclcpp::Subscription<sentinelx_msgs::msg::SeekerStatus>::SharedPtr seeker_status_sub_;
  rclcpp::Subscription<sentinelx_msgs::msg::SeekerTrack>::SharedPtr seeker_track_sub_;

  rclcpp::Publisher<sentinelx_msgs::msg::GuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<sentinelx_msgs::msg::InterceptorPhase>::SharedPtr phase_pub_;
  rclcpp::Publisher<sentinelx_msgs::msg::InternalTargetEstimate>::SharedPtr target_estimate_pub_;
  rclcpp::Publisher<cuas_msgs::msg::MissionAck>::SharedPtr mission_ack_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptProgress>::SharedPtr progress_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<cuas_msgs::msg::EngagementResult>::SharedPtr result_pub_;
  rclcpp::Publisher<cuas_msgs::msg::FaultReport>::SharedPtr fault_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace sentinelx_interceptor
