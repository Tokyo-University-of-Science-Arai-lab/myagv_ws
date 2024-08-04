import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

class MoveControlNode(Node):

    def __init__(self, namespace=''):
        super().__init__('move_control_node', namespace=namespace)
        self._action_client = ActionClient(self, NavigateToPose, f'{namespace}/navigate_to_pose')
        self.publisher_ = self.create_publisher(String, f'{namespace}/agv_reach', 10)
        self.subscription = self.create_subscription(String, f'{namespace}/command', self.listener_callback, 10)  # 'agv_command'
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # self.cmd_vel_subscription = self.create_subscription(Twist, f'/{namespace}/cmd_vel', self.cmd_vel_callback, 10)
        # self.cmd_vel_publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel_limited', 10)
        self.current_destination = None  # Variable to store the current destination
        self.current_goal_handle = None

        # Define destination coordinates
        self.destinations = {
        
            "A": (-3.205, 4.000, 0.744), 
            "B": (-6.025, 5.111, -0.872),
            "C": (-7.587, 6.615, -0.774),
            "D": (-8.266, 8.698, -2.366),
            "E": (-4.482, 2.490, 3.067),
            "F": (-2.991, 5.177, 2.136),
            "G": (-8.266, 8.698, -2.366),
        }
        '''
        "A": (0.769, 1.214, 1.440),
            "B": (0.774, 0.280, 1.488),
            "C": (-2.695, 0.070, -0.091),
            "D": (-5.418, 0.591, -1.712),
            "E": (-4.482, 2.490, 3.067),
            "F": (0.115, 2.141, 3.086),
            
            "A": (0.769, 1.214, 1.440), 
            "B": (0.774, 0.280, 1.488),
            "C": (-2.695, 0.070, -0.091),
            "D": (-5.418, 0.591, -1.712),
            "E": (-4.482, 2.490, 3.067),
            "F": (0.436, 2.095, 2.705),
            #"F": (0.845, 1.406, 1.470), ##0.845 1.406 1.470
        '''
        # self.pd_control_publisher = self.create_publisher(Bool, f'/{namespace}/pd_control_active', 10)
        self.move_forward_publisher = self.create_publisher(String, f'{namespace}/move_forward_start', 10)
        # self.arrival_subscription = self.create_subscription(String, f'/{namespace}/pd_control_arrival', self.arrival_callback, 10)
        self.arrival_subscription = self.create_subscription(String, f'{namespace}/arrival', self.arrival_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message on {self.get_namespace()}/command: {msg.data}')
        command = msg.data.strip().lower()
        if command.startswith("go to "):
            destination_key = command[6:].upper()
            self.get_logger().info(f'current destination key: {destination_key}')
            if destination_key == "E" or destination_key == "D":
                self.activate_move_forward()
            elif destination_key in self.destinations:
                self.current_destination = destination_key  # Save your current destination
                x, y, z = self.destinations[destination_key]
                self.get_logger().info(f'Navigating to {destination_key} at coordinates ({x}, {y}, {z})')
                self.send_goal(x, y, z)
            else:
                self.get_logger().info(f'Unknown destination: {destination_key}')

    def send_goal(self, x, y, z):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = z

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'goal sent')

    def goal_response_callback(self, future):
        try:
            self.current_goal_handle = future.result()
            self.get_logger().info('Goal handle received :(')
            if not self.current_goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')
            self._get_result_future = self.current_goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().info('Failed to get result :(')

    def get_result_callback(self, future):
        result = future.result().status
        if result == 4:  # 4 is the status code for succeeded
            if not self.current_destination == "F":
                self.publish_goal_reached(self.current_destination)
            else:
                self.activate_move_forward()
        else:
            self.get_logger().info('ゴールに到達できませんでした')

    def publish_goal_reached(self, destination_key):
        agv_id = "agv1"  # ここでAGVのIDを設定する
        message = String()
        if destination_key == 'G':
            destination_key = 'D'
        message.data = f"{agv_id} arrived {destination_key}"
        self.publisher_.publish(message)
        self.get_logger().info(f'Published: {agv_id} arrived {destination_key}!')
        self.cancel_navigation()
        self.get_logger().info(f'cancel navigation ocurred')

    def activate_move_forward(self):
        self.move_forward_publisher.publish(String(data="start"))
        self.get_logger().info('move forward start signal sent.')
        
        '''
        self.get_logger().info('PD Control activated.')
        pd_control_msg = Bool()
        pd_control_msg.data = True
        self.pd_control_publisher.publish(pd_control_msg)
        '''

    def arrival_callback(self, msg):
        #self.current_destination = 'B' ##ここは試験用。消す！！！！！
        self.publish_goal_reached(self.current_destination)
        
    def tf_callback(self, msg):
    	for transform in msg.transforms:
            if 'tag36h11:1' in transform.child_frame_id:
                self.get_logger().info('Tag36h11 detected, triggering publish_goal_reached.')
                self.publish_goal_reached(self.current_destination)

    def cancel_navigation(self):
        self.get_logger().info(f'cancel navigation ocurred2')
        if self.current_goal_handle is not None:
            self.get_logger().info(f'cancelling goal.')
            cancel_goal_future = self.current_goal_handle.cancel_goal_async()
            cancel_goal_future.add_done_callback(self.cancel_done_callback)   
        else:
            self.get_logger().info(f'No active goal to cancel.')

    def cancel_done_callback(self, future):
        self.get_logger().info('Navigaton cancelled.')

    '''
    def cmd_vel_callback(self, msg):
        if self.current_destination == "F":
            # ゴールがFの場合、速度を制限
            limited_msg = Twist()
            limited_msg.linear.x = min(msg.linear.x, 0.1)  # 速度制限を0.1 m/sに設定
            limited_msg.linear.y = msg.linear.y
            limited_msg.angular.z = msg.angular.z
            self.cmd_vel_publisher.publish(limited_msg)
        else:
            self.cmd_vel_publisher.publish(msg)
    '''

def main(args=None):
    rclpy.init(args=args)
    namespace = ''  # 設定するnamespace
    node = MoveControlNode(namespace=namespace)

    # Keep the node spinning and accepting messages
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
