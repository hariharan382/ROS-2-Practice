from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    detector_cmd = Node(package="robot_tf2_detector",
                        executable="detector_improved",
                        parameters=[{
                            "use_sim_time": True
                        }],
                        remappings=[
                            ("input_scan", "/scan_now")
                        ])
    
    ld = LaunchDescription()
    ld.add_action(detector_cmd)
    
    return ld
    