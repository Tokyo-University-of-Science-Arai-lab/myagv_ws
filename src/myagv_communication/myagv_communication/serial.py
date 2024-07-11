# シリアル通信を改善したバージョンno.2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import serial
import threading
import time

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/motor_control_topic', self.listener_callback, 1)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_fb', 1)
        self.fd = self.open_serial()
        self.serial_lock = threading.Lock()
        self.serial_thread = threading.Thread(target=self.serial_read_thread)
        self.serial_thread_running = True
        self.serial_thread.start()

        self.wheel_radius = 0.0635 # 車輪の半径 [m]
        self.wheel_base = 0.33 # 車輪間の距離 [m]
        self.wheel_right_vel = 0.0
        self.wheel_left_vel = 0.0

    def listener_callback(self, msg):
        if self.fd is not None and self.fd.is_open:
            try:
                a = str(msg.data[0])
                b = str(msg.data[1])
                send_data = a + ',' + b + '\n'
                with self.serial_lock:
                    self.fd.flushInput()  # バッファをクリア
                    # print(send_data)
                    self.fd.write(send_data.encode())
                    self.fd.flushOutput()  # バッファをクリア
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
        else:
            self.get_logger().warn("Serial port not available or not open.")

    def open_serial(self):
        try:
            fd = serial.Serial('/dev/ttyACM0', 9600, write_timeout=0.05, timeout=0.05)
            self.get_logger().info(f"Serial port opened successfully: {fd}")
            return fd
        except Exception as e:
            self.get_logger().error(f"Serial failed: could not open - {e}")
            return None

    def serial_read_thread(self):
        while rclpy.ok() and self.serial_thread_running:
            if self.fd is not None and self.fd.is_open:
                try:
                    with self.serial_lock:
                        data = self.fd.readline().decode().strip()
                    if data:
                        self.get_logger().info(f"Received: {data}")
                        self.process_serial_data(data)
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
            else:
                self.get_logger().warn("Serial port not available or not open.")
            time.sleep(0.03)

    def process_serial_data(self, data):
        try:
            num_values = self.parse_serial_data(data)
            if len(num_values) >= 2:
                self.wheel_left_vel = 2 * 3.14 * self.wheel_radius * num_values[0] / 60
                self.wheel_right_vel = 2 * 3.14 * self.wheel_radius * num_values[1] / 60
                print("********************************")
                print(f"wheel_left :{self.wheel_left_vel}")
                print(f"wheel_right :{self.wheel_right_vel}")
                twist_msg = Twist()
                twist_msg.linear.x = (self.wheel_right_vel + self.wheel_left_vel) / 2
                twist_msg.angular.z = (self.wheel_right_vel - self.wheel_left_vel) / self.wheel_base
                print(f"linear_x")
                print("********************************")
                self.publisher.publish(twist_msg)
                # self.get_logger().info("Velocity: Linear=%f" % (twist_msg.linear.x))
                # self.get_logger().info("Velocity: Angular=%f" % (twist_msg.angular.z))
        except ValueError as e:
            self.get_logger().error(f"Data parsing error: {e}")

    def parse_serial_data(self, data):
        str_values = data.split(',')
        num_values = [float(value) for value in str_values]
        return num_values

    def close_serial(self):
        self.serial_thread_running = False
        if self.fd is not None and self.fd.is_open:
            self.fd.close()
        self.serial_thread.join()

def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    try:
        rclpy.spin(serial_subscriber)
    except KeyboardInterrupt:
        serial_subscriber.get_logger().info('Keyboard Interrupt (SIGINT) received. Exiting...')
    finally:
        serial_subscriber.close_serial()
        rclpy.shutdown()

if __name__ == '__main__':
    main()