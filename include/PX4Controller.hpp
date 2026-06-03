#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <string>

class PX4Controller : public rclcpp::Node
{
public:
  explicit PX4Controller(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  rclcpp::QoS px4CommandQoS() const;

  void publishVehicleCommand(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f,
    float param5 = 0.0f,
    float param6 = 0.0f,
    float param7 = 0.0f);

  void arm();
  void disarm();
  void takeoff(float altitude_m);
  void land();
  void returnToLaunch();

  void publishOffboardControlModePosition();
  void publishOffboardControlModeVelocity();
  void publishOffboardControlModeAttitude();
  void publishOffboardControlModeBodyRate();

  void publishTrajectorySetpointPosition(
    float x_m,
    float y_m,
    float z_m,
    float yaw_rad = 0.0f);

  void publishTrajectorySetpointVelocity(
    float vx_mps,
    float vy_mps,
    float vz_mps,
    float yaw_rad = 0.0f);

  void publishAttitudeSetpoint(
    float qw,
    float qx,
    float qy,
    float qz,
    float thrust_z);

  void publishRatesSetpoint(
    float roll_rate,
    float pitch_rate,
    float yaw_rate,
    float thrust_body_z);

private:
  uint64_t timestamp_us() const;

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
};