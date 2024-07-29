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
        self.agv_publishers = {}
        self.agv_subscribers = {}
        self.arrival_publisher = self.create_publisher(String, '/agv_arrival', 10)

    def listener_callback(self, msg):
        parts = msg.data.split()
        agv_id, action, _, position = parts
        # Ensure each AGV has a subscriber and publisher
        if agv_id not in self.agv_subscribers:
            self.agv_subscribers[agv_id] = self.create_subscription(
                String,
                f'/{agv_id}/agv_reach',
                lambda msg: self.agv_command_callback(msg, agv_id),
                10
            )
        if agv_id not in self.agv_publishers:
            self.agv_publishers[agv_id] = self.create_publisher(
                String,
                f'/{agv_id}/agv_command',
                10
            )
        command_msg = String()
        command_msg.data = f'go to {position}'
        self.agv_publishers[agv_id].publish(command_msg)
        self.get_logger().info(f'Sending command to {agv_id}: go to {position}')

    def agv_command_callback(self, msg, agv_id):
        arrival_msg = String()
        arrival_msg.data = f'{agv_id} arrived {msg.data.split()[-1]}'
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
