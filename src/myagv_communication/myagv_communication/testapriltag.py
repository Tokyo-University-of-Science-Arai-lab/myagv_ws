import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time

class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('testapriltag')
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.processing = False
        self.timer = None

    def tf_callback(self, msg):
        for transform in msg.transforms:
            self.get_logger().info(f'Checking transform: {transform.child_frame_id}')
            time.sleep(3.0)
            if 'tag36h11' in transform.child_frame_id:
                self.get_logger().info('tag detected')
                # 5秒後に再度読み取れるようにする
                self.timer = self.create_timer(5.0, self.reset_processing)
                self.get_logger().info('Timer reset for 5 seconds')
                break

    def reset_processing(self):
        self.get_logger().info('Resetting processing state')
        self.processing = False
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
