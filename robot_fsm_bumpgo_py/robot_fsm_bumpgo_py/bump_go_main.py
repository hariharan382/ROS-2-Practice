from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

class BumpGoNode(Node):
    def __init__(self):
        super().__init__("bump_go")
        
        self.FORWARD = 0
        self.BACK = 1
        self.TURN = 2
        self.STOP = 3
        self.state = self.FORWARD
        self.state_ts = self.get_clock().now()
        
        self.TURNING_TIME = 2.0
        self.BACKING_TIME = 2.0
        self.SCAN_TIMEOUT = 1.0
        
        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3
        self.OBSTACLE_DISTANCE = 1.0
        
        self.last_scan = None
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            "input_scan",
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        self.vel_pub = self.create_publisher(Twist, "output_vel", 10)
        self.timer = self.create_timer(0.05, self.control_cycle)
        
    def scan_callback(self, msg):
        self.last_scan = msg
        
    def control_cycle(self):
        if self.last_scan is None:
            return 
        
        out_vel = Twist()
        
        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR
            
            if self.check_forward_2_stop():
                self.go_state(self.STOP)
            if self.check_forward_2_back():
                self.go_state(self.BACK)
                
        elif self.state == self.BACK:
            out_vel.linear.x = -self.SPEED_LINEAR
            
            if self.check_back_2_turn():
                self.go_state(self.TURN)
                
        elif self.state  == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR
            
            if self.check_turn_2_forwrd():
                self.go_state(self.FORWARD)
                
        elif self.state == self.STOP:
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)
                
        self.vel_pub.publish(out_vel)
        
    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()
        
    def check_forward_2_back(self):
        pos = round(len(self.last_scan.ranges)/2)
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE
    
    def check_forward_2_stop(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)
    
    def check_stop_2_forward(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)

    def check_back_2_turn(self):
        elapsed = self.get_clock().now() - self.state.ts
        return elapsed > Duration(seconds=self.TURNING_TIME)
    
def main(args=None):
    rclpy.init(args=args)
    
    bump_go_node = BumpGoNode()
    
    rclpy.spin(bump_go_node)
    
    bump_go_node.destroy_node()
    
    rclpy.shutdown()
    
if __name__ == "__mina__":
    main()
            
        
        
    