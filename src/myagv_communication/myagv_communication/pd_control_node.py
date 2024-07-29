import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math
from std_msgs.msg import String

class PDControlNode(Node):
    def __init__(self, namespace=''):
        super().__init__('pd_control_node', namespace=namespace)
        self.target_distance = 15  # 目標距離 7.91
        self.kp = 0.87  # 比例ゲイン
        self.kd = 0.07  # 微分ゲイン
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        self.pd_control_subscription = self.create_subscription(String, f'/{namespace}/apriltag_start', self.pd_control_callback, 10)
        self.node_start_publisher = self.create_publisher(String, f'/{namespace}/node_start', 10)  # 起動メッセージのためのパブリッシャーを作成
        self.publisher_arrival = self.create_publisher(String, f'/{namespace}/arrival', 10)  # AGV到着メッセージのためのパブリッシャーを作成
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pd_control_active = False  # PDループの制御用フラグ
        self.arrival_count = 0  # AGVの到着回数をカウントする変数
        self.publish_start_message()  # ノードが起動したことを発信

    def pd_control_callback(self, msg):
        self.get_logger().info(f'Received message on {self.get_namespace()}/apriltag_start: {msg.data}')
        if msg.data == 'start':
            self.pd_control_active = True
            self.prev_time = self.get_clock().now()  # 時間をリセット
            self.get_logger().info('PD Control activated by apriltag_start')

    def control_loop(self):
        if not self.pd_control_active:
            return
        try:
            now = rclpy.time.Time()
            namespace = self.get_namespace().strip("/")  # 先頭と末尾の '/' を取り除く
            # 正しくフレーム名を構築
            cam_frame = 'default_cam'
            tag_frame = 'tag36h11:0'
            trans = self.tf_buffer.lookup_transform(cam_frame, tag_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2 + trans.transform.translation.z ** 2)
            self.get_logger().info(f"distance={distance}")
            error = self.target_distance - distance
            current_time = self.get_clock().now()
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            control_signal = -1 * (self.kp * error + self.kd * derivative)
            control_signal = min(0.4, control_signal)
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = control_signal
            
            if distance <= 0.005 or abs(cmd_vel_msg.linear.x) <= 0.08:
                cmd_vel_msg.linear.x = 0.0
                self.arrival_count += 1  # 到着回数をインクリメント
                self.publish_arrival('agv1', 'B' if self.arrival_count % 2 != 0 else 'C')  # 各agvの番号に変える
                self.pd_control_active = False
            
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.prev_error = error
            self.prev_time = current_time
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def publish_start_message(self):
        start_msg = String()
        start_msg.data = 'node started'
        self.node_start_publisher.publish(start_msg)
        self.get_logger().info('Node started message sent.')

    def publish_arrival(self, agv_id, position):
        msg = String()
        msg.data = f'{agv_id} {position}'
        self.publisher_arrival.publish(msg)
        self.get_logger().info(f'{agv_id} {position}')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'agv1'  # 設定するnamespace
    node = PDControlNode(namespace=namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
