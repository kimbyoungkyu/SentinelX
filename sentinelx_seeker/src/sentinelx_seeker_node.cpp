#include "sentinelx_seeker/sentinelx_seeker_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace sentinelx_seeker
{

SentinelXSeekerNode::SentinelXSeekerNode()
: Node("sentinelx_seeker_node"),
  interceptor_id_(declare_parameter<std::string>("interceptor_id", "SX-INT-001")),
  target_id_(declare_parameter<std::string>("target_id", "")),
  simulate_detection_(declare_parameter<bool>("simulate_detection", false)),
  simulate_lock_(declare_parameter<bool>("simulate_lock", false)),
  frame_id_(0U),
  current_phase_(sentinelx_msgs::msg::InterceptorPhase::PHASE_IDLE)
{
  phase_sub_ = create_subscription<sentinelx_msgs::msg::InterceptorPhase>(
    "/sentinelx/interceptor/phase", 10,
    std::bind(&SentinelXSeekerNode::on_phase, this, std::placeholders::_1));

  status_pub_ = create_publisher<sentinelx_msgs::msg::SeekerStatus>("/sentinelx/seeker/status", 10);
  track_pub_ = create_publisher<sentinelx_msgs::msg::SeekerTrack>("/sentinelx/seeker/track", 10);
  health_pub_ = create_publisher<sentinelx_msgs::msg::InterceptorHealth>("/sentinelx/health", 10);
  timer_ = create_wall_timer(100ms, std::bind(&SentinelXSeekerNode::publish_seeker, this));
  RCLCPP_INFO(get_logger(), "SentinelX seeker simulation stub started");
}

void SentinelXSeekerNode::on_phase(const sentinelx_msgs::msg::InterceptorPhase::SharedPtr msg)
{
  current_phase_ = msg->phase;
  if (!msg->target_id.empty()) {
    target_id_ = msg->target_id;
  }
}

void SentinelXSeekerNode::publish_seeker()
{
  ++frame_id_;

  const bool active_phase = current_phase_ == sentinelx_msgs::msg::InterceptorPhase::PHASE_INERTIAL_MIDCOURSE ||
                            current_phase_ == sentinelx_msgs::msg::InterceptorPhase::PHASE_SEEKER_SEARCH ||
                            current_phase_ == sentinelx_msgs::msg::InterceptorPhase::PHASE_TERMINAL_HOMING;

  sentinelx_msgs::msg::SeekerStatus status;
  status.stamp = now();
  status.interceptor_id = interceptor_id_;
  status.ready = true;
  status.camera_active = active_phase;
  status.processing_active = active_phase;
  status.target_detected = active_phase && simulate_detection_;
  status.target_locked = active_phase && simulate_lock_;
  status.detection_confidence = status.target_detected ? 0.75F : 0.0F;
  status.lock_confidence = status.target_locked ? 0.85F : 0.0F;
  status.frame_id = frame_id_;
  status.target_id = target_id_;
  status_pub_->publish(status);

  sentinelx_msgs::msg::SeekerTrack track;
  track.stamp = now();
  track.interceptor_id = interceptor_id_;
  track.target_id = target_id_;
  track.valid = status.target_detected;
  track.locked = status.target_locked;
  track.azimuth_rad = 0.0F;
  track.elevation_rad = 0.0F;
  track.bearing_rate_radps = 0.0F;
  track.elevation_rate_radps = 0.0F;
  track.normalized_x = 0.0F;
  track.normalized_y = 0.0F;
  track.bbox_width = status.target_detected ? 0.1F : 0.0F;
  track.bbox_height = status.target_detected ? 0.1F : 0.0F;
  track.confidence = status.target_locked ? 0.85F : status.detection_confidence;
  track_pub_->publish(track);
}

}  // namespace sentinelx_seeker
