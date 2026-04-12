#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from motoros2_interfaces.srv import QueueTrajPoint

class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')
        
        # Subscribe to MoveIt Servo's output
        self.sub_ = self.create_subscription(
            JointTrajectory, 
            '/gp4_arm_controller/joint_trajectory', 
            self.trajectory_callback, 
            10
        )
        
        # Client for MotoROS2 queue
        self.cli_ = self.create_client(QueueTrajPoint, '/yaskawa/queue_traj_point')
        
        self.get_logger().info('MoveIt Servo to MotoROS2 Bridge started.')
        self.joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
        
        # Cache identical points to prevent spam
        self.last_pos = None

    def trajectory_callback(self, msg):
        if not msg.points:
            return
            
        point = msg.points[0]
        
        # Filter if the point hasn't changed (Servo outputs repeats when idle)
        if self.last_pos and all(abs(a - b) < 1e-5 for a, b in zip(self.last_pos, point.positions)):
            return
            
        self.last_pos = point.positions
        
        req = QueueTrajPoint.Request()
        req.joint_names = self.joint_names
        req.point = point
        
        if self.cli_.wait_for_service(timeout_sec=0.1):
            # Async call to avoid blocking the callback
            self.cli_.call_async(req)
        else:
            self.get_logger().warn('Service /yaskawa/queue_traj_point is not available.')

def main(args=None):
    rclpy.init(args=args)
    bridge = ServoBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
