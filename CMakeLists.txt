cmake_minimum_required(VERSION 3.8)
project(sentinelx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cuas_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cuas_msgs REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/C2Command.msg"
  "msg/EngagementResult.msg"
  "msg/FaultReport.msg"
  "msg/GeoPosition.msg"
  "msg/GuidanceCommand.msg"
  "msg/InterceptMission.msg"
  "msg/InterceptProgress.msg"
  "msg/InterceptorHealth.msg"
  "msg/InterceptorHeartbeat.msg"
  "msg/InterceptorPhase.msg"
  "msg/InterceptorStatus.msg"
  "msg/InternalTargetEstimate.msg"
  "msg/MissionAck.msg"
  "msg/PX4VehicleState.msg"
  "msg/SeekerStatus.msg"
  "msg/SeekerTrack.msg"
  "msg/TargetTrack.msg"
  "msg/VelocityNED.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sentinelx_interceptor_node
  src/interceptor_main.cpp
  src/sentinelx_interceptor_node.cpp
)
ament_target_dependencies(sentinelx_interceptor_node rclcpp)
target_link_libraries(sentinelx_interceptor_node "${cpp_typesupport_target}")

add_executable(sentinelx_px4_control_node
  src/px4_control_main.cpp
  src/sentinelx_px4_control_node.cpp
)
ament_target_dependencies(sentinelx_px4_control_node rclcpp px4_msgs)
target_link_libraries(sentinelx_px4_control_node "${cpp_typesupport_target}")

add_executable(sentinelx_seeker_node
  src/seeker_main.cpp
  src/sentinelx_seeker_node.cpp
)
ament_target_dependencies(sentinelx_seeker_node rclcpp)
target_link_libraries(sentinelx_seeker_node "${cpp_typesupport_target}")

install(TARGETS
  sentinelx_interceptor_node
  sentinelx_px4_control_node
  sentinelx_seeker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
