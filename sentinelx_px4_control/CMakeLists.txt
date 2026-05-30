cmake_minimum_required(VERSION 3.8)
project(sentinelx_px4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sentinelx_msgs REQUIRED)

include_directories(include)

add_executable(sentinelx_px4_control_node
  src/main.cpp
  src/sentinelx_px4_control_node.cpp
)
ament_target_dependencies(sentinelx_px4_control_node
  rclcpp px4_msgs sentinelx_msgs
)

install(TARGETS sentinelx_px4_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
