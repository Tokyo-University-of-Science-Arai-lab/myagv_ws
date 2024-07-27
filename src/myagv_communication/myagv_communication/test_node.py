import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveForwardStartPublisher(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, '/move_forward_start', 10)
        self.timer = self.create_timer(1.0, self.publish_start_message)  # 1秒間隔でパブリッシュ

    def publish_start_message(self):
        msg = String()
        msg.data = 'start'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "start" to /move_forward_start')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardStartPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
