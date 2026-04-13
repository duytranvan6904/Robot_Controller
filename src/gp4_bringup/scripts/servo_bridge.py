#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from motoros2_interfaces.srv import QueueTrajPoint, StartPointQueueMode

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
        
        # Client to activate Point Queue Mode
        self.start_cli_ = self.create_client(StartPointQueueMode, '/yaskawa/start_point_queue_mode')
        self.activate_streaming_mode()
        
        self.get_logger().info('MoveIt Servo to MotoROS2 Bridge started.')
        self.joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
        
        # Cache identical points to prevent spam
        self.last_pos = None
        self.point_count = 0

    def activate_streaming_mode(self):
        self.get_logger().info('Opening Point Queue Mode on robot...')
        while not self.start_cli_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /yaskawa/start_point_queue_mode...')
        
        req = StartPointQueueMode.Request()
        future = self.start_cli_.call_async(req)
        # We don't block here to allow node to startup, 
        # but in a real app you'd check the result.
        future.add_done_callback(self.activation_callback)

    def activation_callback(self, future):
        try:
            response = future.result()
            if response.result_code.value == 1: # READY
                self.get_logger().info('Robot is READY for streaming.')
            else:
                self.get_logger().error(f'Failed to start queue mode: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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
            self.point_count += 1
            if self.point_count % 30 == 0: # Log every 1 second (at 30Hz)
                self.get_logger().info(f'Sent {self.point_count} points to robot.')
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
