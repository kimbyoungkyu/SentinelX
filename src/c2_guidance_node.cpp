/*
  C2에서 보내주는 정보를 바탕으로 유도 명령을 생성하여 발행하는 노드입니다. 또한, C2로부터의 명령을 수신하여 인터셉터의 상태를 관리하고, 내부적으로 시커 트랙과 상태 정보를 바탕으로 타겟 추정치를 생성하여 발행합니다. 주요 기능은 다음과 같습니다:
1. C2 명령 수신 및 처리: C2로부터 ASSIGN_TARGET, START_INTERCEPT, ABORT 명령을 수신하여 인터셉터의 미션과 타겟을 관리합니다. 각 명령에 대해 적절한 ACK 메시지를 발행하여 수신 결과를 C2에 통보합니다.
2. 시커 상태 및 트랙 정보 발행: 시커의 상태(준비, 타겟 탐지, 타�겟 잠금 여부 등)와 타겟 트랙 정보를 주기적으로 발행하여 다른 노드에서 활용할 수 있도록 합니다.
3. 인터셉터 상태 관리: C2 명령과 시커 정보를 바탕으로 인터셉터의 현재 상태(예: Idle, MissionLoaded, InertialMidcourse, SeekerSearch, TerminalHoming 등)를 관리하고, 이를 C2에 발행하여 미션 진행 상황을 통보합니다.
4. 타겟 추정치 생성: 시커 트랙과 C2로부터의 타            
*/
#include "c2_guidance_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

C2GuidanceNode::C2GuidanceNode():PX4Proxy("c2_guidance_node"),
  interceptor_id_(declare_parameter<std::string>("interceptor_id", "SX-INT-001")),
  target_id_(declare_parameter<std::string>("target_id", "")),
  simulate_detection_(declare_parameter<bool>("simulate_detection", false)),
  simulate_lock_(declare_parameter<bool>("simulate_lock", false)),
  frame_id_(0U),
  current_phase_(sentinelx::msg::InterceptorPhase::PHASE_IDLE)
{

  /*
  phase_sub_ = create_subscription<sentinelx::msg::InterceptorPhase>(
    "/sentinelx/interceptor/phase", 10,
    std::bind(&C2GuidanceNode::onPhase, this, std::placeholders::_1));

    */

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


  c2_command_sub_    = create_subscription<cuas_msgs::msg::C2Command>("/cuas/c2/command",qos,std::bind(&C2GuidanceNode::onC2Command, this, std::placeholders::_1));
  target_track_sub_  = create_subscription<cuas_msgs::msg::TargetTrack>("/cuas/c2/target_track",10,std::bind(&C2GuidanceNode::onTargetTrack, this, std::placeholders::_1));



  //status_pub_ = create_publisher<sentinelx::msg::SeekerStatus>("/sentinelx/seeker/status", 10);
  //track_pub_ = create_publisher<sentinelx::msg::SeekerTrack>("/sentinelx/seeker/track", 10);
  //health_pub_ = create_publisher<sentinelx::msg::InterceptorHealth>("/sentinelx/health", 10);
  //timer_ = create_wall_timer(100ms, std::bind(&C2GuidanceNode::publish_seeker, this));
  RCLCPP_INFO(get_logger(), "SentinelX C2 guidance node started");
}

/*
void C2GuidanceNode::onPhase(const sentinelx::msg::InterceptorPhase::SharedPtr msg)
{
  current_phase_ = msg->phase;
  if (!msg->target_id.empty()) {
    target_id_ = msg->target_id;
  }
}
*/


void C2GuidanceNode::onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)
{

}
void C2GuidanceNode::onTargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg){

}

void C2GuidanceNode::onPX4Updated(){
    if (px4_ready()) {
      //RCLCPP_INFO(this->get_logger(), "C2 Node Ready!!!!");
    }
}



void C2GuidanceNode::sendAck(const cuas_msgs::msg::C2Command & cmd,bool accepted,uint8_t code,const std::string & message)
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


#if 0
void C2GuidanceNode::publish_seeker()
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
  //track_pub_->publish(track);
}
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<C2GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}