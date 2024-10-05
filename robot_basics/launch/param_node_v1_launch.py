from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_reader_cmd = Node(
        package="robot_basics",
        executable="param_reader",
        parameters=[{
            "number_particles": 599,
            "topics": ["scan", "image"],
            "topic_types":["sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image"]
        }],
        output="screen"
    )
    
    ld = LaunchDescription()
    ld.add_action(param_reader_cmd)
    
    return ld