#include <rclcpp/rclcpp.hpp>
#include "sentinelx_seeker/sentinelx_seeker_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentinelx_seeker::SentinelXSeekerNode>());
  rclcpp::shutdown();
  return 0;
}
