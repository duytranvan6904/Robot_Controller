#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ServoTestPublisher(Node):
    def __init__(self):
        super().__init__('test_servo_publisher')
        # Publisher to the moveit_servo Twist topic
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node_main/delta_twist_cmds', 10)
        
        # 30Hz target frequency
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.counter = 0.0
        self.direction = 1.0
        self.get_logger().info('Bắt đầu gửi lệnh Twist liên tục 30Hz cho GP4...')

    def timer_callback(self):
        msg = TwistStamped()
        
        # We need the appropriate planning frame (from gp4_servo.yaml: base_link)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Swap direction every 60 frames (2 seconds)
        if self.counter >= 60:
            self.direction *= -1.0
            self.counter = 0
            
        self.counter += 1

        # Very small increments: 0.02 m/s (2cm/s velocity)
        msg.twist.linear.x = 0.02 * self.direction
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        # Zero rotation
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    test_node = ServoTestPublisher()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
        
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
