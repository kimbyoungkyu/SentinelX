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
#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/intercept_mission.hpp"



#include "sentinelx/msg/guidance_command.hpp"
#include "sentinelx/msg/px4_vehicle_state.hpp"
#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
#include "sentinelx/msg/interceptor_phase.hpp"
#include "sentinelx/msg/internal_target_estimate.hpp"


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
  //void on_c2_command(const cuas_msgs::msg::C2Command::SharedPtr msg);
  //void on_c2_target_track(const cuas_msgs::msg::TargetTrack::SharedPtr msg);
  void on_px4_state(const sentinelx::msg::PX4VehicleState::SharedPtr msg);
  void on_seeker_status(const sentinelx::msg::SeekerStatus::SharedPtr msg);
  void on_seeker_track(const sentinelx::msg::SeekerTrack::SharedPtr msg);



	void on_c2_command(const cuas_msgs::msg::C2Command::SharedPtr msg);
	/**
	 * @brief 요격 임무 메시지 수신 콜백
	 *
	 * C2가 생성한 요격 임무 정보와 포함된 표적 정보를 출력한다.
	 */
	//void c2MissionCallback(const cuas_msgs::msg::InterceptMission::SharedPtr msg);
  void on_c2_missionCallback(const cuas_msgs::msg::InterceptMission::SharedPtr msg);
	/**
	 * @brief 표적 추적 메시지 수신 콜백
	 *
	 * 센서/레이더 또는 C2에서 전달된 표적 추적 정보를 출력한다.
	 */
	void on_c2_targetTrackCallback(const cuas_msgs::msg::TargetTrack::SharedPtr msg);

























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
  sentinelx::msg::PX4VehicleState latest_px4_state_;
  sentinelx::msg::SeekerTrack latest_seeker_track_;

	rclcpp::Subscription<cuas_msgs::msg::C2Command>::SharedPtr c2_command_sub_;
	rclcpp::Subscription<cuas_msgs::msg::InterceptMission>::SharedPtr intercept_mission_sub_;
	rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr target_track_sub_;














  rclcpp::Subscription<sentinelx::msg::PX4VehicleState>::SharedPtr px4_state_sub_;
  rclcpp::Subscription<sentinelx::msg::SeekerStatus>::SharedPtr seeker_status_sub_;
  rclcpp::Subscription<sentinelx::msg::SeekerTrack>::SharedPtr seeker_track_sub_;

  rclcpp::Publisher<sentinelx::msg::GuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<sentinelx::msg::InterceptorPhase>::SharedPtr phase_pub_;
  rclcpp::Publisher<sentinelx::msg::InternalTargetEstimate>::SharedPtr target_estimate_pub_;


  
  
  
  rclcpp::Publisher<cuas_msgs::msg::MissionAck>::SharedPtr mission_ack_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptProgress>::SharedPtr progress_pub_;
  rclcpp::Publisher<cuas_msgs::msg::InterceptorHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<cuas_msgs::msg::EngagementResult>::SharedPtr result_pub_;
  rclcpp::Publisher<cuas_msgs::msg::FaultReport>::SharedPtr fault_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

