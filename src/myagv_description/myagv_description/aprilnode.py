import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import threading

class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('aprilnode')
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.processing = False

    def tf_callback(self, msg):
        if self.processing:
            return
        self.processing = True
        
        for transform in msg.transforms:
            if 'tag36h11' in transform.child_frame_id:
                self.get_logger().info('tag detected')
                # 5秒後に再度読み取れるようにする
                threading.Timer(5.0, self.reset_processing).start()
                break

    def reset_processing(self):
        self.processing = False

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
