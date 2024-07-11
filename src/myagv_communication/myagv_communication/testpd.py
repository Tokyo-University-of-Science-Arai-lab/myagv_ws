import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math
from std_msgs.msg import Bool, String

class PDControlNode(Node):
    def __init__(self):
        super().__init__('testpd_node')
        
        self.target_distance = 7.91  # 目標距離
        self.kp = 0.87  # 比例ゲイン
        self.kd = 0.07  # 微分ゲイン

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pd_control_subscription = self.create_subscription(Bool, '/pd_control_active', self.pd_control_callback, 10)
        self.arrival_publisher = self.create_publisher(String, '/pd_control_arrival', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pd_control_active = True  # Flag to control PD loop

    def pd_control_callback(self, msg):
        self.pd_control_active = msg.data
        self.get_logger().info('PD Control activated')

    def control_loop(self):
        if not self.pd_control_active:
            return

        try:
            trans = self.tf_buffer.lookup_transform('default_cam', 'tag36h11:0', rclpy.time.Time())
            #self.get_logger().info(f'Transform: {trans}')  # ここで変換をログに出力
            distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2 + trans.transform.translation.z ** 2)
            error = self.target_distance - distance
            #self.get_logger().info(cmd_vel_msg.linear.x)

            print(distance)
            
            current_time = self.get_clock().now()
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            
            control_signal = -1 * (self.kp * error + self.kd * derivative)
            control_signal = min(0.4, control_signal)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = control_signal
            #self.get_logger().info(cmd_vel)
            if distance <= 0.005 or abs(cmd_vel_msg.linear.x) <= 0.08:
                cmd_vel_msg.linear.x = 0.0
                self.publish_arrival_message()
                #self.pd_control_active = False
            
            self.cmd_vel_publisher.publish(cmd_vel_msg)              
            self.prev_error = error
            self.prev_time = current_time
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def publish_arrival_message(self):
        arrival_msg = String()
        arrival_msg.data = 'Arrived at target'
        self.arrival_publisher.publish(arrival_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PDControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
