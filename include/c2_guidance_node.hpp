#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
#include "sentinelx/msg/interceptor_phase.hpp"
#include "sentinelx/msg/interceptor_health.hpp"
#include "px4_proxy.hpp"

class C2GuidanceNode : public PX4Proxy
{
public:
  C2GuidanceNode();

private:
  void on_phase(const sentinelx::msg::InterceptorPhase::SharedPtr msg);
  void publish_seeker();


  virtual void onPX4Updated() override;

  std::string interceptor_id_;
  std::string target_id_;
  bool simulate_detection_;
  bool simulate_lock_;
  uint32_t frame_id_;
  uint8_t current_phase_;

  rclcpp::Subscription<sentinelx::msg::InterceptorPhase>::SharedPtr phase_sub_;
  rclcpp::Publisher<sentinelx::msg::SeekerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sentinelx::msg::SeekerTrack>::SharedPtr track_pub_;
  rclcpp::Publisher<sentinelx::msg::InterceptorHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::TimerBase::SharedPtr tHeartbeat_;
  
};

