cmake_minimum_required(VERSION 3.8)
project(sentinelx_interceptor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cuas_msgs REQUIRED)
find_package(sentinelx_msgs REQUIRED)

include_directories(include)

add_executable(sentinelx_interceptor_node
  src/main.cpp
  src/sentinelx_interceptor_node.cpp
)
ament_target_dependencies(sentinelx_interceptor_node
  rclcpp cuas_msgs sentinelx_msgs
)

install(TARGETS sentinelx_interceptor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
