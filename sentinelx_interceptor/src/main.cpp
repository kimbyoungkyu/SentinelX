#include <rclcpp/rclcpp.hpp>
#include "sentinelx_interceptor/sentinelx_interceptor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentinelx_interceptor::SentinelXInterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
