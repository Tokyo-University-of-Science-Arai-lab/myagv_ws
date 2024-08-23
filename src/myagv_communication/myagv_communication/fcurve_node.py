import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import time

class FCurveCommandListener(Node):
    def __init__(self, namespace=''):
        super().__init__('fcurve_node')
        self.publisher = self.create_publisher(String, f'{namespace}/move_forward_start', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{namespace}/cmd_vel', 10)
        self.subscription = self.create_subscription(TFMessage,'/tf', self.tf_callback, 10)
        self.fcurve_subscription = self.create_subscription(String, f'{namespace}/fcurve_command', self.fcurve_command_callback, 10)
        self.get_logger().info('fcurve_command_listener node started')
        self.moving_forward = False
        self.tag_detected_count = 0
        self.last_tag_detected_time = 0
        self.tag_value = False

    def tf_callback(self, msg):
        self.count1 = 0
        self.count2 = 0
        if self.tag_value:    
            for transform in msg.transforms:
                if 'tag36h11:3' in transform.child_frame_id:
                    if self.count1== 0:
                        self.get_logger().info('Tag36h11:3 detected.')
                        self.start_turning()
                        self.count1 += 1    
                elif 'tag36h11:4' in transform.child_frame_id:
                    if self.count2 == 0:
                        self.stop_and_publish_move_forward()
                        self.count2 += 1                    
                    

    def fcurve_command_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        self.tag_value = True
        self.get_logger().info('fcurve start')
        self.start_moving_forward()

    def start_moving_forward(self):
        self.get_logger().info('Starting to move forward.')
        self.moving_forward = True
        twist_msg = Twist()
        twist_msg.linear.x = 0.4  # 直進速度
        self.cmd_vel_publisher.publish(twist_msg)

    def start_turning(self):
        self.get_logger().info('Starting to turn and move forward.')
        twist_msg = Twist()
        twist_msg.linear.x = 0.4  # 直進速度
        twist_msg.angular.z = 0.3  # 回転速度
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_and_publish_move_forward(self):
        self.get_logger().info('Stopping and publishing move_forward_start.')
        self.moving_forward = False
        self.tag_value = False
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)  # 停止
        move_forward_start_msg = String()
        move_forward_start_msg.data = "start"
        self.publisher.publish(move_forward_start_msg)
        self.count1 = 0
        self.count2 = 0

def main(args=None):
    rclpy.init(args=args)
    node = FCurveCommandListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
