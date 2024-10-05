from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vff_avoidance_cmd = Node(
        package="robot_vff_avoidance",
        executable="avoidance_vff",
        parameters=[{
            "use_sim_time":True
        }],
        remappings=[
            ("input_scan", "/scan_raw"),
            ("output_vel", "/cmd_vel")
        ],
        output = "screen"
    )
    
    ld = LaunchDescription()
    ld.add_action(vff_avoidance_cmd)
    
    return ld

