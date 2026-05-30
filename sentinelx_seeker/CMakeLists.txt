cmake_minimum_required(VERSION 3.8)
project(sentinelx_seeker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sentinelx_msgs REQUIRED)

include_directories(include)

add_executable(sentinelx_seeker_node
  src/main.cpp
  src/sentinelx_seeker_node.cpp
)
ament_target_dependencies(sentinelx_seeker_node
  rclcpp sentinelx_msgs
)

install(TARGETS sentinelx_seeker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
