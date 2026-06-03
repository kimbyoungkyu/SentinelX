from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    interceptor_id = "1"
    px4_dir = "/root/PX4-Autopilot"   # 실제 PX4 경로로 수정

    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent","udp4","-p","8888"],
            output="screen"
        ),
        
        ExecuteProcess(
            cmd=["make", "px4_sitl", "sihsim_quadx"],
            cwd=px4_dir,
            output="screen",
            additional_env={
                "PX4_HOME_LAT": "0.0",
                "PX4_HOME_LON": "0.0",
                "PX4_HOME_ALT": "50.0",
             #   "PX4_SYS_AUTOSTART": "4001",
            }
        ),        
        
        Node(
            package="sentinelx",
            executable="launch_node",
            name="launch_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "mavlink_sys_id": 1}]
        ),
        
        Node(
            package="sentinelx",
            executable="c2_guidance_node",
            name="c2_guidance_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "simulate_detection": False, "simulate_lock": False}],
        ),
        Node(
            package="sentinelx",
            executable="terminal_guidance_node",
            name="terminal_guidance_node",
            output="screen",
            parameters=[{"interceptor_id": interceptor_id, "simulate_detection": False, "simulate_lock": False}],
        )
    ])
