#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import math
import time

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('test_continuous_move')

        # Create service client to start servo
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Đang chờ service /servo_node/start_servo...')
        
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Servo Start Response: {future.result().message}")
        else:
            self.get_logger().error("Không thể khởi động Servo!")

        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # Publish at 30 Hz
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dt = 0.0
        self.get_logger().info('Bắt đầu gửi lệnh nội suy Twist đến Servo...')


    def timer_callback(self):
        msg = TwistStamped()
        # Mặc định hệ quy chiếu là base_link để dịch chuyển theo không gian Cartesian tuyệt đối
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Test dịch chuyển: Tiến và lùi theo hình sin trên trục X
        # Biên độ nhỏ để an toàn (e.g. max vận tốc = 0.05 m/s)
        msg.twist.linear.x = 0.03 * math.sin(self.dt)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        # Không sinh vận tốc xoay tĩnh
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_.publish(msg)
        self.dt += 0.05

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()

    try:
        rclpy.spin(twist_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Gửi lệnh stop để robot dừng ngay khi tắt script
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.header.stamp = twist_publisher.get_clock().now().to_msg()
        twist_publisher.publisher_.publish(stop_msg)
        
        twist_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
