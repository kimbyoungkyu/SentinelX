#include "PX4Listener.hpp"

using std::placeholders::_1;

PX4Listener::PX4Listener(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  auto qos = px4TelemetryQoS();

  odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos,
    std::bind(&PX4Listener::onVehicleOdometry, this, _1));

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v4",
    qos,
    std::bind(&PX4Listener::onVehicleStatus, this, _1));

  control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode",
    qos,
    std::bind(&PX4Listener::onVehicleControlMode, this, _1));

  failsafe_flags_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
    "/fmu/out/failsafe_flags",
    qos,
    std::bind(&PX4Listener::onFailsafeFlags, this, _1));

  battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status",
    qos,
    std::bind(&PX4Listener::onBatteryStatus, this, _1));

  global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    "/fmu/out/vehicle_global_position",
    qos,
    std::bind(&PX4Listener::onVehicleGlobalPosition, this, _1));

  land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected",
    qos,
    std::bind(&PX4Listener::onVehicleLandDetected, this, _1));

  command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack_v1",
    qos,
    std::bind(&PX4Listener::onVehicleCommandAck, this, _1));

  estimator_flags_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
    "/fmu/out/estimator_status_flags",
    qos,
    std::bind(&PX4Listener::onEstimatorStatusFlags, this, _1));

  wind_sub_ = create_subscription<px4_msgs::msg::Wind>(
    "/fmu/out/wind",
    qos,
    std::bind(&PX4Listener::onWind, this, _1));

  RCLCPP_INFO(get_logger(), "PX4Listener initialized");
}

rclcpp::QoS PX4Listener::px4TelemetryQoS() const
{
  return rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();
}

void PX4Listener::onVehicleOdometry(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_odometry_ = *msg;
    has_odometry_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onVehicleStatus(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_status_ = *msg;
    has_status_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onVehicleControlMode(
  const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_control_mode_ = *msg;
    has_control_mode_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onFailsafeFlags(
  const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_failsafe_flags_ = *msg;
    has_failsafe_flags_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onBatteryStatus(
  const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_battery_ = *msg;
    has_battery_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onVehicleGlobalPosition(
  const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_global_position_ = *msg;
    has_global_position_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onVehicleLandDetected(
  const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_land_detected_ = *msg;
    has_land_detected_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onVehicleCommandAck(
  const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_command_ack_ = *msg;
    has_command_ack_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onEstimatorStatusFlags(
  const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_estimator_flags_ = *msg;
    has_estimator_flags_ = true;
  }

  onPX4Updated();
}

void PX4Listener::onWind(
  const px4_msgs::msg::Wind::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(px4_mutex_);
    latest_wind_ = *msg;
    has_wind_ = true;
  }

  onPX4Updated();
}