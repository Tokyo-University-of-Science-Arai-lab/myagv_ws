#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_control_topic', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.motor_cont = Float32MultiArray()
    
    def listener_callback(self, twist):
        d = 0.35
        vel_r = (twist.linear.x + d * twist.angular.z) * 240    # 385(0.95)
        vel_l = (twist.linear.x - d * twist.angular.z) * 234    # 232 
        
        if 0 < vel_r < 20:
            vel_r = 20
        elif -20 < vel_r < 0:
            vel_r = -20
        
        if 0 < vel_l < 20:
            vel_l = 20
        elif -20 < vel_l < 0:
            vel_l = -20
        
        '''
        if vel_r < 0:
            vel_r = 0
        elif vel_r > 120:
            vel_r = 120
        
        if vel_l < 0:
            vel_l = 0
        elif vel_l > 120:
            vel_l = 120
        '''
        
        self.get_logger().info("Velocity: right=%f" % vel_r)
        self.get_logger().info("Velocity: left=%f" % vel_l)   

        array = [vel_l, vel_r]
        self.motor_cont = Float32MultiArray(data=array)
        self.publisher.publish(self.motor_cont)

def main(args=None):
    rclpy.init(args=args)
    diff_drive_controller = DiffDriveController()
    rclpy.spin(diff_drive_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()