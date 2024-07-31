import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from transforms3d.euler import euler2quat as quaternion_from_euler
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)  # Change to 'agv1/odom'
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 1)  # Create publisher for 'agv1/tf'
        self.subscriber = self.create_subscription(Twist, 'cmd_vel_fb', self.velocity_callback, 1)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.update_odometry)

    def velocity_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        
        # Compute odometry
        dt = (current_time - self.last_time).nanoseconds / 1e9
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        print("*******************************************************************")
        print(f"delta_th : {delta_th}")
        print("*******************************************************************")

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.get_logger().info("x = %f, y = %f, th = %f" % (self.x, self.y, self.th))
        
        # Create quaternion from yaw
        odom_quat = quaternion_from_euler(0, 0, self.th)
        self.get_logger().info("Quaternion: x = %f, y = %f, z = %f, w = %f" % (odom_quat[1], odom_quat[2], odom_quat[3], odom_quat[0]))
        
        # Create TransformStamped message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'  # Keep as 'odom'
        odom_trans.child_frame_id = 'base_link'  # Keep as 'base_link'
        
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = odom_quat[1]
        odom_trans.transform.rotation.y = odom_quat[2]
        odom_trans.transform.rotation.z = odom_quat[3]
        odom_trans.transform.rotation.w  = odom_quat[0]
        
        # Create TFMessage and publish it
        tf_message = TFMessage(transforms=[odom_trans])
        self.tf_pub.publish(tf_message)
        
        # Publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = '/odom'  # Change to 'agv1/odom'
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[1]
        odom.pose.pose.orientation.y = odom_quat[2]
        odom.pose.pose.orientation.z = odom_quat[3]
        odom.pose.pose.orientation.w = odom_quat[0]
        
        # Set the velocity
        odom.child_frame_id = 'base_link'  # Keep as 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
