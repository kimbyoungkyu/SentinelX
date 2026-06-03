#include "terminal_guidance_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

TerminalGuidanceNode::TerminalGuidanceNode():PX4Listener("terminal_guidance_node"),  
  interceptor_id_(declare_parameter<std::string>("interceptor_id", "INT001")),
  target_id_(declare_parameter<std::string>("target_id", "")),
  simulate_detection_(declare_parameter<bool>("simulate_detection", false)),
  simulate_lock_(declare_parameter<bool>("simulate_lock", false)),
  frame_id_(0U),
  current_phase_(sentinelx::msg::InterceptorPhase::PHASE_IDLE)
{
  // 1. Phase Subscriber
  phase_sub_ = create_subscription<sentinelx::msg::InterceptorPhase>(
    "/sentinelx/interceptor/phase", 10,
    std::bind(&TerminalGuidanceNode::on_phase, this, std::placeholders::_1));


  std::string topic_name = "/INT" + interceptor_id_ + "/camera/image_raw";
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::BestEffort);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(  // :: 로 수정
    topic_name,                  // _ 추가
    qos_profile,
    std::bind(&TerminalGuidanceNode::image_callback, this, std::placeholders::_1));    


  // 3. Publishers
  status_pub_ = create_publisher<sentinelx::msg::SeekerStatus>("/sentinelx/seeker/status", 10);
  track_pub_  = create_publisher<sentinelx::msg::SeekerTrack>("/sentinelx/seeker/track", 10);
  health_pub_ = create_publisher<sentinelx::msg::InterceptorHealth>("/sentinelx/health", 10);

  // 4. Timer (chrono_literals 네임스페이스 활성화 상태 기준, 안 된다면 std::chrono::milliseconds(100) 사용)
  using namespace std::chrono_literals; 
  timer_ = create_wall_timer(100ms, std::bind(&TerminalGuidanceNode::publish_seeker, this));
  RCLCPP_INFO(get_logger(), "SentinelX Seeker guidance node started");
}
void TerminalGuidanceNode::on_phase(const sentinelx::msg::InterceptorPhase::SharedPtr msg)
{
  current_phase_ = msg->phase;
  if (!msg->target_id.empty()) {
    target_id_ = msg->target_id;
  }
}

void TerminalGuidanceNode::onPX4Updated(){
    if (px4_ready()) {
      RCLCPP_INFO(this->get_logger(), "Terminal Guidance Node Ready!!!!");
    }
    
}

void TerminalGuidanceNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) const
{
    // 이미지 메타데이터 출력 예시
    RCLCPP_INFO(this->get_logger(), 
        "Received Image -> Header Timestamp: [%d.%d], Resolution: %dx%d, Encoding: %s",
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        msg->width,
        msg->height,
        msg->encoding.c_str()
    );
    // 실제 픽셀 데이터의 크기 확인 (msg->data는 std::vector<uint8_t> 형태입니다)
    // RCLCPP_INFO(this->get_logger(), "Image Data Size: %zu bytes", msg->data.size());
}

void TerminalGuidanceNode::publish_seeker()
{
  ++frame_id_;

  const bool active_phase = current_phase_ == sentinelx::msg::InterceptorPhase::PHASE_INERTIAL_MIDCOURSE ||
                            current_phase_ == sentinelx::msg::InterceptorPhase::PHASE_SEEKER_SEARCH ||
                            current_phase_ == sentinelx::msg::InterceptorPhase::PHASE_TERMINAL_HOMING;

  sentinelx::msg::SeekerStatus status;
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

  sentinelx::msg::SeekerTrack track;
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerminalGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}