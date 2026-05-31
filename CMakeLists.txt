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

# CUASInterceptorCommandListener
# add_executable(CUASInterceptorCommandListener src/CUASInterceptorCommandListener.cpp)
# ament_target_dependencies(CUASInterceptorCommandListener rclcpp px4_msgs cuas_msgs)
# install(TARGETS CUASInterceptorCommandListener DESTINATION lib/${PROJECT_NAME})

# CUASInterceptorReportPublisher
# add_executable(CUASInterceptorReportPublisher src/CUASInterceptorReportPublisher.cpp)
# ament_target_dependencies(CUASInterceptorReportPublisher rclcpp px4_msgs cuas_msgs)
# install(TARGETS CUASInterceptorReportPublisher DESTINATION lib/${PROJECT_NAME})

add_executable(sentinelx_interceptor_node src/sentinelx_interceptor_node.cpp)
ament_target_dependencies(sentinelx_interceptor_node rclcpp cuas_msgs)
target_link_libraries(sentinelx_interceptor_node "${cpp_typesupport_target}")

add_executable(sentinelx_px4_control_node src/sentinelx_px4_control_node.cpp)
ament_target_dependencies(sentinelx_px4_control_node rclcpp px4_msgs)
target_link_libraries(sentinelx_px4_control_node "${cpp_typesupport_target}")

add_executable(sentinelx_c2_guidance_node src/sentinelx_c2_guidance_node.cpp)
ament_target_dependencies(sentinelx_c2_guidance_node rclcpp)
target_link_libraries(sentinelx_c2_guidance_node "${cpp_typesupport_target}")

add_executable(sentinelx_seeker_guidance_node src/sentinelx_seeker_guidance_node.cpp)
ament_target_dependencies(sentinelx_seeker_guidance_node rclcpp sensor_msgs)
target_link_libraries(sentinelx_seeker_guidance_node "${cpp_typesupport_target}")

install(TARGETS
  sentinelx_interceptor_node
  sentinelx_px4_control_node
  sentinelx_c2_guidance_node
  sentinelx_seeker_guidance_node
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