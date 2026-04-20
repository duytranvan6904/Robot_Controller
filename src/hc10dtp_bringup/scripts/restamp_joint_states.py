#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRestamper(Node):
    def __init__(self):
        super().__init__('joint_state_restamper')
        # We publish to the main joint_states topic
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # We subscribe to the raw MotoROS2 joints
        self.sub = self.create_subscription(JointState, '/yaskawa/joint_states', self.cb, 10)
        self.get_logger().info('JointState Restamper started to fix Yaskawa clock skew (HC10DTP).')

    def cb(self, msg):
        # Override the potentially out-of-sync Yaskawa clock with the PC's current clock
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRestamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
