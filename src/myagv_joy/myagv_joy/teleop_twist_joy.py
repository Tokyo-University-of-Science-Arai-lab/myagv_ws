#! /usr/bin/env python3
# joyからcmd_velへの変換
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopTwistJoy(Node):
    def __init__(self):
        super().__init__('teleop_twist_joy_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.agv_arrival_subscription = self.create_subscription(String, 'agv_arrival', self.agv_arrival_callback, 10)
        self.apriltag_publisher = self.create_publisher(String, 'apriltag_start', 10)
        self.vel = Twist()
        self.publish_cmd_vel = True

    def listener_callback(self, joy_msg):
        if joy_msg.buttons[5] == 1:
            self.publish_cmd_vel = False
            self.apriltag_publisher.publish(String(data="start"))
            self.get_logger().info("Apriltag start signal sent.")
        if self.publish_cmd_vel:
            self.vel.linear.x = 0.2 * joy_msg.axes[1]
            self.vel.angular.z = 0.8 * joy_msg.axes[3]
            self.publisher.publish(self.vel)
            self.get_logger().info("Velocity: Linear=%f" % (self.vel.linear.x))
            self.get_logger().info("Velocity: Angular=%f" % (self.vel.angular.z))

    def agv_arrival_callback(self, msg):
        self.publish_cmd_vel = True
        self.get_logger().info("AGV arrival signal received, resuming cmd_vel publishing.")

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_joy = TeleopTwistJoy()
    rclpy.spin(teleop_twist_joy)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
