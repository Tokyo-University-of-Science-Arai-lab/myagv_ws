import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

class MoveForwardNode(Node):
    def __init__(self, namespace=''):
        super().__init__('move_forward_node', namespace=namespace)
        self.publisher_ = self.create_publisher(String, f'{namespace}/apriltag_start', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{namespace}/cmd_vel', 10)
        self.subscription = self.create_subscription(TFMessage,'/tf', self.tf_callback, 10)
        self.move_forward_subscription = self.create_subscription(String, f'{namespace}/move_forward_start', self.move_forward_callback, 10)
        self.command_publisher = self.create_publisher(String, '/agv1command', 10) #### name change!!!!!!!
        self.move_forward_active = False
        self.timer = self.create_timer(0.1, self.control_loop)  # 0.1秒ごとに制御ループを実行
        self.callback_count = 0 ####実際は0 Eに止まりたいときは1 Dに行きたいときは2

    def tf_callback(self, msg):
        self.count = 0
        #self.get_logger().info(f'callback count = {self.callback_count}')
        if not self.move_forward_active:
            return
        
        for transform in msg.transforms:
            if transform.child_frame_id == 'tag36h11:3':
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.2  # /cmd_velの速度を0.2に変更
                self.cmd_vel_publisher.publish(cmd_vel_msg)
                self.get_logger().info('Detected tag36h11:3, setting cmd_vel speed to 0.2')
                return  # この処理のみを実行し、他の処理に進まない
            
        for transform in msg.transforms:
            if (self.callback_count % 3 == 0 and transform.child_frame_id == 'tag36h11:0') or (self.callback_count % 3 == 1 and transform.child_frame_id == 'tag36h11:2'):
                self.publish_start_message()
                self.stop_moving_forward()
                self.callback_count += 1
            elif (self.callback_count % 3 == 2 and transform.child_frame_id == 'tag36h11:0'):
                self.stop_moving_forward()
                if self.count == 0:
                    command_msg = String()
                    position = 'G'
                    command_msg.data = f'go to {position}'
                    self.command_publisher.publish(command_msg)  # Publishing to /command
                    self.get_logger().info(f'Sending command to : go to {position}')
                self.count += 1
                self.callback_count += 1

    def move_forward_callback(self, msg):
        if msg.data == 'start':
            self.get_logger().info('Received start message, enabling move forward')
            self.move_forward_active = True

    def control_loop(self):
        if not self.move_forward_active:
            return
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.35
        if self.callback_count % 3 == 2 or self.callback_count % 3 == 0:
            cmd_vel_msg.linear.x = 0.23

        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Publishing cmd_vel: linear.x = {cmd_vel_msg.linear.x}')

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
        self.get_logger().info(f'Publishing: "start" to /{self.get_namespace()}/apriltag_start')
    

def main(args=None):
    rclpy.init(args=args)
    namespace = ''  # デフォルト値
    node = MoveForwardNode(namespace=namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
