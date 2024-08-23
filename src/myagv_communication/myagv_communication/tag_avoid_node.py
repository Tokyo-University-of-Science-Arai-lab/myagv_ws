import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TagAvoidNode(Node):

    def __init__(self):
        super().__init__('tag_avoid_node')
        self.subscription_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.subscription_tag_avoid = self.create_subscription(String, '/tag_avoid', self.tag_avoid_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_avoid_complete = self.create_publisher(String, '/avoid_complete', 10)
        self.processing = False
        self.tag_detected = False
        self.avoid_command_received = False
        self.timer = self.create_timer(0.1, self.move_forward)

    def tf_callback(self, msg):
        if self.avoid_command_received:
            self.tag_detected = False
            for transform in msg.transforms:
                if 'tag36h11' in transform.child_frame_id:
                    self.tag_detected = True
                    self.get_logger().info('AprilTag detected')
                    break

    def tag_avoid_callback(self, msg):
        self.get_logger().info('Received tag avoid command')
        self.avoid_command_received = True

    def move_forward(self):
        if self.avoid_command_received:
            if self.tag_detected:
                self.get_logger().info('Moving forward')
                twist = Twist()
                twist.linear.x = 0.1  # Adjust the speed as needed
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
            else:
                self.get_logger().info('Tag not detected, stopping and publishing avoid complete')
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                
                # Publish avoid complete message
                avoid_complete_msg = String()
                avoid_complete_msg.data = 'avoid_complete'
                self.publisher_avoid_complete.publish(avoid_complete_msg)
                
                # Reset state
                self.avoid_command_received = False

def main(args=None):
    rclpy.init(args=args)
    node = TagAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
