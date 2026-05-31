from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    interceptor_id = "SX-INT-001"
    return LaunchDescription([
        Node(
            package="sentinelx",
            executable="sentinelx_interceptor_node",
            name="sentinelx_interceptor_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "mavlink_sys_id": 1}]
        ),
        Node(
            package="sentinelx",
            executable="sentinelx_px4_control_node",
            name="sentinelx_px4_control_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "dry_run": True}]
        ),
        Node(
            package="sentinelx",
            executable="sentinelx_c2_guidance_node",
            name="sentinelx_c2_guidance_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "simulate_detection": False, "simulate_lock": False}],
        ),
        Node(
            package="sentinelx",
            executable="sentinelx_seeker_guidance_node",
            name="sentinelx_seeker_guidance_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "simulate_detection": False, "simulate_lock": False}],
        ),
#        Node(
#            package="sentinelx",
#            executable="CUASInterceptorReportPublisher",
#            name="CUASInterceptorReportPublisher",
#            output="screen"
#        ),
#        Node(
#            package="sentinelx",
#            executable="CUASInterceptorCommandListener",
#            name="CUASInterceptorCommandListener",
#            output="screen"
#        ),
    ])
