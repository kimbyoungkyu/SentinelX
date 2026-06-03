#include "px4_controller.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <limits>

PX4Controller::PX4Controller(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  auto qos = px4CommandQoS();

  vehicle_command_pub_ =
    create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command",
      qos);

  offboard_control_mode_pub_ =
    create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode",
      qos);

  trajectory_setpoint_pub_ =
    create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint",
      qos);

  attitude_setpoint_pub_ =
    create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint_v1",
      qos);

  rates_setpoint_pub_ =
    create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint",
      qos);

  RCLCPP_INFO(get_logger(), "PX4Controller initialized");
}

rclcpp::QoS PX4Controller::px4CommandQoS() const
{
  return rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();
}

uint64_t PX4Controller::timestamp_us() const
{
  return static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);
}

void PX4Controller::publishVehicleCommand(
  uint16_t command,
  float param1,
  float param2,
  float param3,
  float param4,
  float param5,
  float param6,
  float param7)
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = timestamp_us();
  msg.command = command;

  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;

  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  vehicle_command_pub_->publish(msg);
}

void PX4Controller::arm()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0f);

  RCLCPP_INFO(get_logger(), "PX4 ARM command published");
}

void PX4Controller::disarm()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.0f);

  RCLCPP_INFO(get_logger(), "PX4 DISARM command published");
}

void PX4Controller::takeoff(float altitude_m)
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    altitude_m);

  RCLCPP_INFO(get_logger(), "PX4 TAKEOFF command published altitude=%.2f", altitude_m);
}

void PX4Controller::land()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

  RCLCPP_INFO(get_logger(), "PX4 LAND command published");
}

void PX4Controller::returnToLaunch()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);

  RCLCPP_INFO(get_logger(), "PX4 RTL command published");
}

void PX4Controller::publishOffboardControlModePosition()
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp_us();
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_control_mode_pub_->publish(msg);
}

void PX4Controller::publishOffboardControlModeVelocity()
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp_us();
  msg.position = false;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_control_mode_pub_->publish(msg);
}

void PX4Controller::publishOffboardControlModeAttitude()
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp_us();
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = true;
  msg.body_rate = false;

  offboard_control_mode_pub_->publish(msg);
}

void PX4Controller::publishOffboardControlModeBodyRate()
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp_us();
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = true;

  offboard_control_mode_pub_->publish(msg);
}

void PX4Controller::publishTrajectorySetpointPosition(
  float x_m,
  float y_m,
  float z_m,
  float yaw_rad)
{
  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = timestamp_us();

  msg.position = {x_m, y_m, z_m};
  msg.velocity = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
  msg.acceleration = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };

  msg.yaw = yaw_rad;
  msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

  trajectory_setpoint_pub_->publish(msg);
}

void PX4Controller::publishTrajectorySetpointVelocity(
  float vx_mps,
  float vy_mps,
  float vz_mps,
  float yaw_rad)
{
  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = timestamp_us();

  msg.position = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
  msg.velocity = {vx_mps, vy_mps, vz_mps};
  msg.acceleration = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };

  msg.yaw = yaw_rad;
  msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

  trajectory_setpoint_pub_->publish(msg);
}

void PX4Controller::publishAttitudeSetpoint(
  float qw,
  float qx,
  float qy,
  float qz,
  float thrust_z)
{
  px4_msgs::msg::VehicleAttitudeSetpoint msg{};

  msg.timestamp = timestamp_us();

  msg.q_d[0] = qw;
  msg.q_d[1] = qx;
  msg.q_d[2] = qy;
  msg.q_d[3] = qz;

  msg.thrust_body[0] = 0.0f;
  msg.thrust_body[1] = 0.0f;
  msg.thrust_body[2] = thrust_z;

  attitude_setpoint_pub_->publish(msg);
}

void PX4Controller::publishRatesSetpoint(
  float roll_rate,
  float pitch_rate,
  float yaw_rate,
  float thrust_body_z)
{
  px4_msgs::msg::VehicleRatesSetpoint msg{};

  msg.timestamp = timestamp_us();

  msg.roll = roll_rate;
  msg.pitch = pitch_rate;
  msg.yaw = yaw_rate;

  msg.thrust_body[0] = 0.0f;
  msg.thrust_body[1] = 0.0f;
  msg.thrust_body[2] = thrust_body_z;

  rates_setpoint_pub_->publish(msg);
}