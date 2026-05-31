#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"

#include "sentinelx/msg/guidance_command.hpp"
#include "sentinelx/msg/px4_vehicle_state.hpp"
#include "sentinelx/msg/interceptor_health.hpp"


class SentinelXPX4ControlNode : public rclcpp::Node
{
public:
  SentinelXPX4ControlNode();

private:
  void on_guidance_command(const sentinelx::msg::GuidanceCommand::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void on_battery_status(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void publish_state();
  void publish_health(const std::string & message, bool healthy = true, uint8_t severity = 0U);

  std::string interceptor_id_;
  bool dry_run_;
  bool connected_;
  bool has_status_;
  bool has_local_position_;
  bool has_global_position_;
  bool has_battery_;

  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleLocalPosition local_position_;
  px4_msgs::msg::VehicleGlobalPosition global_position_;
  px4_msgs::msg::BatteryStatus battery_status_;
  sentinelx::msg::GuidanceCommand last_guidance_;

  rclcpp::Subscription<sentinelx::msg::GuidanceCommand>::SharedPtr guidance_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;

  rclcpp::Publisher<sentinelx::msg::PX4VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<sentinelx::msg::InterceptorHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};


