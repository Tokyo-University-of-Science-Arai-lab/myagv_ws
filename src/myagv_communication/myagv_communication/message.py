import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MessageNode(Node):

    def __init__(self):
        super().__init__('message_node')
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        self.subscription = self.create_subscription(String, '/ans/chatter', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publish_message()
        self.received_response = False  # メッセージを受け取ったかどうかのフラグ

    def publish_message(self):
        while True:
            user_input = input("Enter destination (a, b, c, d, e, f): ").strip().lower()
            if user_input in ['a', 'b', 'c', 'd', 'e', 'f']:
                message = String()
                message.data = f"go to {user_input.upper()}"
                self.publisher_.publish(message)
                self.get_logger().info(f"Published: {message.data}")
                break  # 有効な入力を受け取ったらループを抜ける
            else:
                self.get_logger().info("Invalid input. Please enter one of the following: a, b, c, d, e, f.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        self.received_response = True
        rclpy.shutdown()  # メッセージを受け取ったらROS2をシャットダウン

def main(args=None):
    rclpy.init(args=args)
    message_node = MessageNode()

    while rclpy.ok() and not message_node.received_response:
        rclpy.spin_once(message_node)

    message_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
