#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sentinelx_msgs/msg/seeker_status.hpp"
#include "sentinelx_msgs/msg/seeker_track.hpp"
#include "sentinelx_msgs/msg/interceptor_phase.hpp"
#include "sentinelx_msgs/msg/interceptor_health.hpp"

namespace sentinelx_seeker
{

class SentinelXSeekerNode : public rclcpp::Node
{
public:
  SentinelXSeekerNode();

private:
  void on_phase(const sentinelx_msgs::msg::InterceptorPhase::SharedPtr msg);
  void publish_seeker();

  std::string interceptor_id_;
  std::string target_id_;
  bool simulate_detection_;
  bool simulate_lock_;
  uint32_t frame_id_;
  uint8_t current_phase_;

  rclcpp::Subscription<sentinelx_msgs::msg::InterceptorPhase>::SharedPtr phase_sub_;
  rclcpp::Publisher<sentinelx_msgs::msg::SeekerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sentinelx_msgs::msg::SeekerTrack>::SharedPtr track_pub_;
  rclcpp::Publisher<sentinelx_msgs::msg::InterceptorHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentinelx_seeker
