#include "sentinelx_px4_control/sentinelx_px4_control_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace sentinelx_px4_control
{

SentinelXPX4ControlNode::SentinelXPX4ControlNode()
: Node("sentinelx_px4_control_node"),
  interceptor_id_(declare_parameter<std::string>("interceptor_id", "SX-INT-001")),
  dry_run_(declare_parameter<bool>("dry_run", true)),
  connected_(false),
  has_status_(false),
  has_local_position_(false),
  has_global_position_(false),
  has_battery_(false)
{
  guidance_sub_ = create_subscription<sentinelx_msgs::msg::GuidanceCommand>(
    "/sentinelx/guidance/command", 10,
    std::bind(&SentinelXPX4ControlNode::on_guidance_command, this, std::placeholders::_1));

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", 10,
    std::bind(&SentinelXPX4ControlNode::on_vehicle_status, this, std::placeholders::_1));

  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", 10,
    std::bind(&SentinelXPX4ControlNode::on_local_position, this, std::placeholders::_1));

  global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    "/fmu/out/vehicle_global_position", 10,
    std::bind(&SentinelXPX4ControlNode::on_global_position, this, std::placeholders::_1));

  battery_status_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status", 10,
    std::bind(&SentinelXPX4ControlNode::on_battery_status, this, std::placeholders::_1));

  state_pub_ = create_publisher<sentinelx_msgs::msg::PX4VehicleState>("/sentinelx/px4/state", 10);
  health_pub_ = create_publisher<sentinelx_msgs::msg::InterceptorHealth>("/sentinelx/health", 10);
  state_timer_ = create_wall_timer(50ms, std::bind(&SentinelXPX4ControlNode::publish_state, this));

  RCLCPP_WARN(get_logger(), "PX4 control node started in dry_run=%s. No /fmu/in command publishers are created.", dry_run_ ? "true" : "false");
}

void SentinelXPX4ControlNode::on_guidance_command(const sentinelx_msgs::msg::GuidanceCommand::SharedPtr msg)
{
  last_guidance_ = *msg;
  if (dry_run_) {
    RCLCPP_DEBUG(get_logger(), "Guidance command received in dry-run mode: mode=%u", msg->guidance_mode);
    return;
  }
  publish_health("Real PX4 command publishing is intentionally not implemented in this kit", false, sentinelx_msgs::msg::InterceptorHealth::SEVERITY_WARN);
}

void SentinelXPX4ControlNode::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
  has_status_ = true;
  connected_ = true;
}

void SentinelXPX4ControlNode::on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
  has_local_position_ = true;
}

void SentinelXPX4ControlNode::on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  global_position_ = *msg;
  has_global_position_ = true;
}

void SentinelXPX4ControlNode::on_battery_status(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  battery_status_ = *msg;
  has_battery_ = true;
}

void SentinelXPX4ControlNode::publish_state()
{
  sentinelx_msgs::msg::PX4VehicleState msg;
  msg.stamp = now();
  msg.interceptor_id = interceptor_id_;
  msg.connected = connected_;

  if (has_status_) {
    msg.armed = vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    msg.offboard_enabled = vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    msg.failsafe = vehicle_status_.failsafe;
    msg.nav_state = vehicle_status_.nav_state;
    msg.arming_state = vehicle_status_.arming_state;
  }

  if (has_battery_) {
    msg.battery_remaining = battery_status_.remaining;
    msg.battery_voltage_v = battery_status_.voltage_v;
  }

  if (has_global_position_) {
    msg.latitude_deg = global_position_.lat;
    msg.longitude_deg = global_position_.lon;
    msg.altitude_m = global_position_.alt;
  }

  if (has_local_position_) {
    msg.local_x_m = local_position_.x;
    msg.local_y_m = local_position_.y;
    msg.local_z_m = local_position_.z;
    msg.velocity_x_mps = local_position_.vx;
    msg.velocity_y_mps = local_position_.vy;
    msg.velocity_z_mps = local_position_.vz;
    msg.heading_rad = local_position_.heading;
  }

  state_pub_->publish(msg);
}

void SentinelXPX4ControlNode::publish_health(const std::string & message, bool healthy, uint8_t severity)
{
  sentinelx_msgs::msg::InterceptorHealth msg;
  msg.stamp = now();
  msg.node_name = get_name();
  msg.interceptor_id = interceptor_id_;
  msg.healthy = healthy;
  msg.severity = severity;
  msg.message = message;
  health_pub_->publish(msg);
}

}  // namespace sentinelx_px4_control
