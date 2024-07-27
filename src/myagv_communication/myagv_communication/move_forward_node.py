import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        self.publisher_ = self.create_publisher(String, '/apriltag_start', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(TFMessage,'/tf',self.tf_callback,10)
        self.move_forward_subscription = self.create_subscription(String,'/move_forward_start',self.move_forward_callback,10)
        self.move_forward_active = False
        self.timer = self.create_timer(0.1, self.control_loop)  # 0.1秒ごとに制御ループを実行

    def tf_callback(self, msg):
        if not self.move_forward_active:
            return
        for transform in msg.transforms:
            if 'tag36h11' in transform.child_frame_id:
                self.publish_start_message()
                self.stop_moving_forward()

    def move_forward_callback(self, msg):
        if msg.data == 'start':
            self.get_logger().info('Received start message, enabling move forward')
            self.move_forward_active = True

    def control_loop(self):
        if not self.move_forward_active:
            return
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info('Publishing cmd_vel: linear.x=0.2')

    def stop_moving_forward(self):
        self.move_forward_active = False
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info('AprilTag recognized, stopping cmd_vel')

    def publish_start_message(self):
        msg = String()
        msg.data = 'start'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "start" to /apriltag_start')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
