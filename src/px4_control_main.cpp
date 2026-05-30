#include <rclcpp/rclcpp.hpp>
#include "sentinelx_px4_control/sentinelx_px4_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentinelx_px4_control::SentinelXPX4ControlNode>());
  rclcpp::shutdown();
  return 0;
}
