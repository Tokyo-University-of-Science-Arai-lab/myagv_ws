import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveForwardStartPublisher(Node):
    def __init__(self):
        super().__init__('test_node')
        self.agv_command_publisher = self.create_publisher(String, '/agv_command', 10)
        self.move_forward_publisher = self.create_publisher(String, '/move_forward_start', 10)
        #self.publisher_arrival = self.create_publisher(String, '/agv1/arrival', 10)
        self.subscription = self.create_subscription(
            String, '/agv_arrival', self.arrival_callback, 10)
        self.arrival_count = 0
        self.get_logger().info('Node is ready to receive input. Type "1" to send AGV command, "2" to publish "start" on /move_forward_start, "3" to publish on /arrival, "4" to send AGV to E, or "exit" to quit.')

    def input_loop(self):
        while rclpy.ok():
            user_input = input("Enter '1' to send AGV command, '2' to publish 'start', '3' to publish arrival, '4' to move AGV to E, 'exit' to quit: ")
            if user_input == '1':
                self.publish_agv_command('agv1', 'F')  # 'agv1 move to F'をパブリッシュ
            elif user_input == '2':
                self.publish_move_forward_message('start')  # '/move_forward_start'に'start'をパブリッシュ
            elif user_input == '3':
                self.publish_arrival('agv1', 'B' if self.arrival_count % 2 != 0 else 'C')
                self.arrival_count += 1
            elif user_input == '4':
                self.publish_agv_command('agv1', 'E')  # 'agv1 move to E'をパブリッシュ
            elif user_input.lower() == 'exit':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                break

    def publish_agv_command(self, agv_id, position):
        command_message = String()
        command_message.data = f'{agv_id} move to {position}'
        self.agv_command_publisher.publish(command_message)
        self.get_logger().info(f'Publishing: "{command_message.data}" to /agv_command')

    def publish_move_forward_message(self, message_content):
        msg = String()
        msg.data = message_content
        self.move_forward_publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{message_content}" to /move_forward_start')

    '''
    def publish_arrival(self, agv_id, position):
        msg = String()
        msg.data = f'{agv_id} {position}'
        self.publisher_arrival.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}" to /arrival')
    '''

    def arrival_callback(self, msg):
        self.get_logger().info(f'Received on /agv_arrival: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardStartPublisher()
    try:
        node.input_loop()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
