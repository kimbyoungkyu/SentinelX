#include <rclcpp/rclcpp.hpp>
#include "sentinelx_interceptor/sentinelx_interceptor_node.hpp"
#include "cuas_msgs/msg/c2_command.hpp"
#include "cuas_msgs/msg/intercept_mission.hpp"
#include "cuas_msgs/msg/target_track.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentinelx_interceptor::SentinelXInterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
