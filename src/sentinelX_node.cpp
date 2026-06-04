#include "sentinelX_node.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

SentinelXNode::SentinelXNode():BaseNode("sentinelX")
{
  states.push_back(&SentinelXNode::initialize);
  states.push_back(&SentinelXNode::idle);
  states.push_back(&SentinelXNode::ready);
  states.push_back(&SentinelXNode::launch);
  states.push_back(&SentinelXNode::c2Guidance);
  states.push_back(&SentinelXNode::terminalGuidance);
  states.push_back(&SentinelXNode::abort);
  RCLCPP_INFO(get_logger(), "SentinelX node started");
}

void SentinelXNode::tick() {
  BaseNode::tick();
	if (nextStates.size() > 0) {
		ENodeState nextState = nextStates[0];
		(this->*(states[(int)currentState]))(EStateEvent::Leave, 0.0f);
		(this->*(states[(int)nextState]))(EStateEvent::Enter, 0.0f);
		currentState = nextState;
    nextStates.erase(nextStates.begin());
	}
  (this->*(states[(int)currentState]))(EStateEvent::Tick,0);

}


void SentinelXNode::onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)  {
  if (msg->interceptor_id != interceptor_id_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_command = *msg;
    //mission_id_ = msg->mission_id;
    //target_id_ = msg->target_id;

  }
  (this->*(states[(int)currentState]))(EStateEvent::C2Command);
}
void SentinelXNode::onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg) {
  if (msg->target_id != target_id_)return;
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_tracking = *msg;
  }
  (this->*(states[(int)currentState]))(EStateEvent::Tracking);
}


void SentinelXNode::onPX4Updated()
{
    (this->*(states[(int)currentState]))(EStateEvent::PX4Update);
}


void SentinelXNode::changeState(ENodeState nextState){
  nextStates.push_back(nextState);
}
void SentinelXNode::initialize(EStateEvent event,...){
  switch(event){
    case EStateEvent::Enter:
      RCLCPP_INFO(this->get_logger(), "initialize::Enter");
      break;  

    case EStateEvent::Tick:
      if (px4_ready()){
        changeState(ENodeState::idle);
      }
      break;
    case EStateEvent::Leave:
      RCLCPP_INFO(this->get_logger(), "initialize::Leave");
      break;
    default:
      break;
  }
}

/*
  switch (msg->command_type) {
    case cuas_msgs::msg::C2Command::ASSIGN_TARGET:

*/
void SentinelXNode::idle(EStateEvent event,...){
  switch(event){
    case EStateEvent::Enter:
      RCLCPP_INFO(this->get_logger(), "idle::Enter");
      break;  
    case EStateEvent::Leave:
      RCLCPP_INFO(this->get_logger(), "idle::Leave");
      break;
    case EStateEvent::Tick:
      break;

    case EStateEvent::PX4Update:
      publishSnapshot();
      break;

    case EStateEvent::C2Command:{
        switch(latest_command.command_type){
          case cuas_msgs::msg::C2Command::ASSIGN_TARGET: {
              mission_id_ = latest_command.mission_id;
              target_id_ = latest_command.target_id;
              sendAck(latest_command,true,cuas_msgs::msg::MissionAck::OK,"target assigned; interceptor is ready for START_INTERCEPT");
              changeState(ENodeState::ready);
          }break;
          default: {
            sendAck(latest_command,false,cuas_msgs::msg::MissionAck::REJECTED,"interceptor already launched");
          }
          break;
        }
      }
      break;
    case EStateEvent::Tracking:
      break;

    default:
      break;
  }
}

void SentinelXNode::ready(EStateEvent event,...){
  switch(event){
    case EStateEvent::Enter:
      RCLCPP_INFO(this->get_logger(), "ready::Enter");
      break;  
    case EStateEvent::Leave:
      RCLCPP_INFO(this->get_logger(), "ready::Leave");
      break;
    case EStateEvent::Tick:
      break;

    case EStateEvent::PX4Update:
      publishSnapshot();
      break;

    case EStateEvent::C2Command:{
        switch(latest_command.command_type){
          case cuas_msgs::msg::C2Command::START_INTERCEPT: {
              sendAck(latest_command,true,cuas_msgs::msg::MissionAck::OK,"START_INTERCEPT accepted; interceptor launched");
              changeState(ENodeState::launch);
          }break;
          default: {
            sendAck(latest_command,false,cuas_msgs::msg::MissionAck::BUSY,"interceptor already launched");
          }
          break;
        }
      }
      break;

    case EStateEvent::Tracking:
      break;

    default:
      break;
  }

}


void SentinelXNode::launch(EStateEvent event,...) {
  switch(event){
    case EStateEvent::Enter:{
        RCLCPP_INFO(this->get_logger(), "launch::Enter");
        if (px4_ready()){
          arm();
          takeoff(200);
        }
      }
      break;
    case EStateEvent::Leave:
      RCLCPP_INFO(this->get_logger(), "launch::Leave");
      break;
    case EStateEvent::Tick:
      break;
    case EStateEvent::PX4Update:
      publishSnapshot();
      break;
    default:
      break;
  }
}
void SentinelXNode::c2Guidance(EStateEvent event,...) {
  switch(event){
    case EStateEvent::Enter:
      break;
    case EStateEvent::Leave:
      break;
    case EStateEvent::Tick:
      break;
    case EStateEvent::PX4Update:
      publishSnapshot();
      break;
    default:
      break;
  }
}
void SentinelXNode::terminalGuidance(EStateEvent event,...) {
  switch(event){
    case EStateEvent::Enter:
      break;
    case EStateEvent::Leave:
      break;
    case EStateEvent::Tick:
      break;
    case EStateEvent::PX4Update:
      publishSnapshot();
      break;

    default:
      break;
  }
}

void SentinelXNode::abort(EStateEvent event,...) {
  switch(event){
    case EStateEvent::Enter:
      break;
    case EStateEvent::Leave:
      break;
    case EStateEvent::Tick:
      break;
    case EStateEvent::PX4Update:
      publishSnapshot();
      break;

    default:
      break;
  }
}

uint8_t SentinelXNode::convertPX4NavStateToInterceptorMode(uint8_t nav_state) const
{
    
    using Snapshot = cuas_msgs::msg::InterceptorSnapshot;
    using Status = px4_msgs::msg::VehicleStatus;

    switch (nav_state)
    {
    case Status::NAVIGATION_STATE_MANUAL:
        return Snapshot::MODE_MANUAL;

    case Status::NAVIGATION_STATE_STAB:
        return Snapshot::MODE_STABILIZED;

    case Status::NAVIGATION_STATE_POSCTL:
        return Snapshot::MODE_POSITION;

    case Status::NAVIGATION_STATE_AUTO_MISSION:
        return Snapshot::MODE_MISSION;

    case Status::NAVIGATION_STATE_OFFBOARD:
        return Snapshot::MODE_OFFBOARD;

    case Status::NAVIGATION_STATE_AUTO_RTL:
        return Snapshot::MODE_RETURN;

    case Status::NAVIGATION_STATE_AUTO_LAND:
        return Snapshot::MODE_LAND;

    case Status::NAVIGATION_STATE_AUTO_LOITER:
        return Snapshot::MODE_HOLD;

    default:
        return Snapshot::MODE_UNKNOWN;
    }
}

cuas_msgs::msg::InterceptorSnapshot SentinelXNode::makeInterceptorSnapshot(){

   cuas_msgs::msg::InterceptorSnapshot out;

    std::lock_guard<std::mutex> lock(px4_mutex_);

    out.stamp = this->now();


    
    // --------------------------------------------------
    // identity
    // --------------------------------------------------
    out.interceptor_id = interceptor_id_;
    out.mavlink_sys_id = static_cast<int32_t>(latest_status_.system_id);
    //out.vehicle_name = vehicle_name_;

    // --------------------------------------------------
    // PX4 state
    // --------------------------------------------------
    out.connected = true;
    out.armed = latest_status_.arming_state ==  px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

    out.failsafe = latest_status_.failsafe;
    out.landed = latest_land_detected_.landed;

    //out.status = convertPX4StatusToInterceptorStatus();
    out.mode = convertPX4NavStateToInterceptorMode(latest_status_.nav_state);

    out.offboard_available =     latest_control_mode_.flag_control_offboard_enabled ||
        latest_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // --------------------------------------------------
    // global position
    // --------------------------------------------------
    out.position.latitude = latest_global_position_.lat;
    out.position.longitude = latest_global_position_.lon;
    out.position.altitude = latest_global_position_.alt;

    // --------------------------------------------------
    // local NED position
    // --------------------------------------------------
    out.local_x = latest_odometry_.position[0];
    out.local_y = latest_odometry_.position[1];
    out.local_z = latest_odometry_.position[2];

    // --------------------------------------------------
    // velocity NED
    // --------------------------------------------------
    out.velocity.north = latest_odometry_.velocity[0];
    out.velocity.east  = latest_odometry_.velocity[1];
    out.velocity.down  = latest_odometry_.velocity[2];

    // --------------------------------------------------
    // attitude
    // VehicleOdometry q = [w, x, y, z]
    // --------------------------------------------------
    const float qw = latest_odometry_.q[0];
    const float qx = latest_odometry_.q[1];
    const float qy = latest_odometry_.q[2];
    const float qz = latest_odometry_.q[3];

    float roll = std::atan2(2.0f * (qw * qx + qy * qz),1.0f - 2.0f * (qx * qx + qy * qy));

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = std::clamp(sinp, -1.0f, 1.0f);

    float pitch = std::asin(sinp);

    float yaw = std::atan2(2.0f * (qw * qz + qx * qy),1.0f - 2.0f * (qy * qy + qz * qz));

    out.roll = roll;
    out.pitch = pitch;
    out.yaw = yaw;

    // --------------------------------------------------
    // battery
    // --------------------------------------------------
    out.battery_percent = latest_battery_.remaining * 100.0f;
    out.battery_voltage = latest_battery_.voltage_v;
    out.battery_current = latest_battery_.current_a;
    out.battery_warning = latest_failsafe_flags_.battery_warning;


    out.status = (uint8_t)currentState;

    // --------------------------------------------------
    // mission / target
    // --------------------------------------------------
    out.mission_id = mission_id_;
    out.target_id = target_id_;

//    out.target_position.latitude = target_lat_;
//    out.target_position.longitude = target_lon_;
//    out.target_position.altitude = target_alt_;

//    out.target_distance = target_distance_;
//    out.target_bearing = target_bearing_;



    // --------------------------------------------------
    // command / control state
    // --------------------------------------------------
    //out.has_active_command = has_active_command_;
    //out.active_command = active_command_;
    //out.command_timeout_sec = command_timeout_sec_;

    // --------------------------------------------------
    // health
    // --------------------------------------------------

/*
    out.position_valid =
        latest_estimator_flags_.pos_horiz_abs ||
        latest_estimator_flags_.pos_horiz_rel;

    out.velocity_valid =
        latest_estimator_flags_.velocity_horiz ||
        latest_estimator_flags_.velocity_vert;

    out.attitude_valid = latest_estimator_flags_.attitude;
*/
    out.battery_valid = latest_battery_.connected;

    //out.message = buildInterceptorMessage(out);


    return out;

}

void SentinelXNode::publishSnapshot() {
  if (!px4_ready()) return;

  cuas_msgs::msg::InterceptorSnapshot msg = makeInterceptorSnapshot();
  //msg.stamp = now();
  //msg.interceptor_id = interceptor_id_;
  //msg.mavlink_sys_id = mavlink_sys_id_;


  /*

  switch(currentState){
    case ENodeState::initialize:
      msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;  
    break;

    case ENodeState::idle:
      msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
    break;

    case ENodeState::ready:
      msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
    break;


    case ENodeState::launch:
    break;

    case ENodeState::c2Guidance:
    break;

    case ENodeState::terminalGuidance:
    break;

    case ENodeState::abort:
    break;


  }
    */
  //msg.status = (uint8_t)currentState;

  /*
  if (phase_ == Phase::Idle) {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
  } else if (phase_ == Phase::MissionLoaded) {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_READY;
  } else {
    msg.status = cuas_msgs::msg::InterceptorSnapshot::STATUS_TRACKING;
  }
*/
  //msg.connected = true;
  //msg.mode = cuas_msgs::msg::InterceptorSnapshot::MODE_OFFBOARD;
  //msg.armed = has_px4_state_ ? latest_px4_state_.armed : false;
  //msg.connected = has_px4_state_ ? latest_px4_state_.connected : false;
  //msg.offboard_available = true;
  //msg.battery_percent = has_px4_state_ ? latest_px4_state_.battery_remaining : 0.0F;
  //msg.battery_voltage = has_px4_state_ ? latest_px4_state_.battery_voltage_v : 0.0F;

  //msg.mission_id = mission_id_;
  //msg.target_id = target_id_;
  //msg.message = "fire-and-forget snapshot";

  snapshot_pub_->publish(msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SentinelXNode>());
  rclcpp::shutdown();
  return 0;
}

