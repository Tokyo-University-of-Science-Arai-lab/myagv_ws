import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveForwardStartPublisher(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, '/move_forward_start', 10)
        self.get_logger().info('Node is ready to receive input. Type "1" to publish "start", "2" to publish "forward", or "exit" to quit.')

    def input_loop(self):
        while rclpy.ok():
            user_input = input("Enter '1' to publish 'start', '2' to publish 'start', 'exit' to quit: ")
            if user_input == '1':
                self.publish_message('start')
            elif user_input == '2':
                self.publish_message('start')
            elif user_input.lower() == 'exit':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                break

    def publish_message(self, message_content):
        msg = String()
        msg.data = message_content
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{message_content}" to /move_forward_start')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardStartPublisher()
    try:
        node.input_loop()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
