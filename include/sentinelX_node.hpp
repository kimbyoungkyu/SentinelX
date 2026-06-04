#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/mission_ack.hpp"
#include "cuas_msgs/msg/target_track.hpp"
#include "cuas_msgs/msg/interceptor_status.hpp"
#include "cuas_msgs/msg/intercept_progress.hpp"
#include "cuas_msgs/msg/interceptor_snapshot.hpp"
#include "cuas_msgs/msg/engagement_result.hpp"
#include "cuas_msgs/msg/fault_report.hpp"
#include "sentinelx/msg/px4_vehicle_state.hpp"
#include "sentinelx/msg/seeker_status.hpp"
#include "sentinelx/msg/seeker_track.hpp"
#include "base_node.hpp"


enum class EStateEvent : uint8_t
{
	Enter,
	Tick,
	Leave,
	PX4Update,
	C2Command,
	Tracking
};



enum class ENodeState : uint8_t
{
	//공통
	initialize,
	idle,
	ready,
	launch,
	c2Guidance,
	terminalGuidance,
	abort
};


class SentinelXNode : public BaseNode
{
public:
  explicit SentinelXNode();

protected:  
  virtual void tick() override;
  virtual void onC2Command(const cuas_msgs::msg::C2Command::SharedPtr msg)  override;
  virtual void onC2TargetTrack(const cuas_msgs::msg::TargetTrack::SharedPtr msg) override;
  virtual void onPX4Updated() override;



protected:
	void publishSnapshot();
	
   void changeState(ENodeState nextState);

   void initialize(EStateEvent event,...);  
   void idle(EStateEvent event,...);
   void ready(EStateEvent event,...);  
   void launch(EStateEvent event,...);
   void c2Guidance(EStateEvent event,...);
   void terminalGuidance(EStateEvent event,...);
   void abort(EStateEvent event,...);

   cuas_msgs::msg::InterceptorSnapshot makeInterceptorSnapshot();
private:
  uint8_t convertPX4NavStateToInterceptorMode(uint8_t nav_state) const;
  typedef void (SentinelXNode::* tickFun)(EStateEvent event,...);
  ENodeState currentState = ENodeState::initialize;
  std::vector<tickFun> states;
  std::vector<ENodeState> nextStates;


  cuas_msgs::msg::C2Command  latest_command;
  cuas_msgs::msg::TargetTrack  latest_tracking;

};

