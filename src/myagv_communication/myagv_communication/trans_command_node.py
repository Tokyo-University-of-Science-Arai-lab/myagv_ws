import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TransCommandNode(Node):
    def __init__(self):
        super().__init__('trans_command_node')
        self.subscription = self.create_subscription(
            String,
            '/agv_command',
            self.listener_callback,
            10
        )
        self.command_publisher = self.create_publisher(String, '/command', 10)
        self.arrival_publisher = self.create_publisher(String, '/agv_arrival', 10)
        
        # Ensure each AGV has a subscriber and publisher
        self.agv_subscribers = self.create_subscription(
            String,
            '/agv_reach',  # Subscribing to /agv_reach
            self.agv_command_callback,
            10
        )
        

    def listener_callback(self, msg):
        parts = msg.data.split()
        agv_id, _, _, position = parts

        command_msg = String()
        command_msg.data = f'go to {position}'
        self.command_publisher.publish(command_msg)  # Publishing to /command
        self.get_logger().info(f'Sending command to : go to {position}')

    def agv_command_callback(self, msg):
        arrival_msg = String()
        arrival_msg.data = f'arrived at {msg.data.split()[-1]}'
        self.arrival_publisher.publish(arrival_msg)
        self.get_logger().info(f'Publishing on /agv_arrival: {arrival_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TransCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
