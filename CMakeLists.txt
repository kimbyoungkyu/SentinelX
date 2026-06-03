cmake_minimum_required(VERSION 3.8)
project(sentinelx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(cuas_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuidanceCommand.msg"
  "msg/InterceptorHealth.msg"
  "msg/InterceptorPhase.msg"
  "msg/InternalTargetEstimate.msg"
  "msg/PX4VehicleState.msg"
  "msg/SeekerStatus.msg"
  "msg/SeekerTrack.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(launch_node src/launch_node.cpp src/px4_proxy.cpp)
ament_target_dependencies(launch_node rclcpp px4_msgs cuas_msgs)
target_link_libraries(launch_node "${cpp_typesupport_target}")

add_executable(c2_guidance_node src/c2_guidance_node.cpp src/px4_proxy.cpp)
ament_target_dependencies(c2_guidance_node px4_msgs rclcpp)
target_link_libraries(c2_guidance_node "${cpp_typesupport_target}")

add_executable(terminal_guidance_node src/terminal_guidance_node.cpp src/px4_proxy.cpp)
ament_target_dependencies(terminal_guidance_node rclcpp px4_msgs sensor_msgs)
target_link_libraries(terminal_guidance_node "${cpp_typesupport_target}")

install(TARGETS
  launch_node
  c2_guidance_node
  terminal_guidance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(
  rclcpp
  rosidl_default_runtime
  cuas_msgs
  px4_msgs
)

ament_package()