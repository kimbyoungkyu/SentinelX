#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/wind.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <mutex>
#include <string>

class PX4Proxy : public rclcpp::Node
{
public:
  explicit PX4Proxy(const std::string & node_name,const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  rclcpp::QoS px4TelemetryQoS() const;
  rclcpp::QoS px4CommandQoS() const;

  bool has_odometry() const { return has_odometry_; }
  bool has_status() const { return has_status_; }
  bool has_control_mode() const { return has_control_mode_; }
  bool has_failsafe_flags() const { return has_failsafe_flags_; }
  bool has_battery() const { return has_battery_; }
  bool has_global_position() const { return has_global_position_; }
  bool has_land_detected() const { return has_land_detected_; }
  bool has_command_ack() const { return has_command_ack_; }
  bool has_estimator_flags() const { return has_estimator_flags_; }
  bool has_wind() const { return has_wind_; }

  bool px4_ready() const
  {
    return has_odometry_ && has_status_ && has_battery_;
  }

  px4_msgs::msg::VehicleOdometry latest_odometry_;
  px4_msgs::msg::VehicleStatus latest_status_;
  px4_msgs::msg::VehicleControlMode latest_control_mode_;
  px4_msgs::msg::FailsafeFlags latest_failsafe_flags_;
  px4_msgs::msg::BatteryStatus latest_battery_;
  px4_msgs::msg::VehicleGlobalPosition latest_global_position_;
  px4_msgs::msg::VehicleLandDetected latest_land_detected_;
  px4_msgs::msg::VehicleCommandAck latest_command_ack_;
  px4_msgs::msg::EstimatorStatusFlags latest_estimator_flags_;
  px4_msgs::msg::Wind latest_wind_;

  mutable std::mutex px4_mutex_;

  virtual void onPX4Updated() {}

  void publishVehicleCommand(uint16_t command,float param1 = 0.0f,float param2 = 0.0f,float param3 = 0.0f,float param4 = 0.0f,float param5 = 0.0f,float param6 = 0.0f,float param7 = 0.0f);

  void arm();
  void disarm();
  void takeoff(float altitude_m);
  void land();
  void returnToLaunch();

  void publishOffboardControlModePosition();
  void publishOffboardControlModeVelocity();
  void publishOffboardControlModeAttitude();
  void publishOffboardControlModeBodyRate();

  void publishTrajectorySetpointPosition(float x_m,float y_m,float z_m,float yaw_rad = 0.0f);
  void publishTrajectorySetpointVelocity(float vx_mps,float vy_mps,float vz_mps,float yaw_rad = 0.0f);
  void publishAttitudeSetpoint(float qw,float qx,float qy,float qz,float thrust_z);
  void publishRatesSetpoint(float roll_rate,float pitch_rate,float yaw_rate,float thrust_body_z);

private:
  uint64_t timestamp_us() const;

  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onVehicleControlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void onFailsafeFlags(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);
  void onBatteryStatus(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void onVehicleGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void onVehicleLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void onVehicleCommandAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void onEstimatorStatusFlags(const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg);
  void onWind(const px4_msgs::msg::Wind::SharedPtr msg);

private:
  bool has_odometry_ = false;
  bool has_status_ = false;
  bool has_control_mode_ = false;
  bool has_failsafe_flags_ = false;
  bool has_battery_ = false;
  bool has_global_position_ = false;
  bool has_land_detected_ = false;
  bool has_command_ack_ = false;
  bool has_estimator_flags_ = false;
  bool has_wind_ = false;


  //rclcpp::Subscription<cuas_msgs::msg::TargetTrack>::SharedPtr target_track_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_flags_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr estimator_flags_sub_;
  rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr wind_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
};
