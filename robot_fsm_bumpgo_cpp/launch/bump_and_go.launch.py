from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    bumpgo_cmd = Node(package="robot_fsm_bumpgo_cpp",
                      executable="bump_go_main",
                      output = "screen",
                      parameters=[{
                          "use_sim_time":True
                      }],
                      remappings=[
                          ("input_scan", "/scan_raw"),
                          ("output_vel", "/nav_vel")
                      ])
    
    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)
    
    return ld
    