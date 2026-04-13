#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

class SceneClearer(Node):
    def __init__(self):
        super().__init__('scene_clearer')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info('Đang gửi lệnh xóa sạch Planning Scene...')

    def timer_callback(self):
        scene = PlanningScene()
        scene.is_diff = True
        
        # Xóa tất cả các vật thể bằng cách gửi lệnh REMOVE cho mọi khả năng
        # Trong thực tế MoveIt cần tên cụ thể, nhưng gửi một tin nhắn rỗng với is_diff=True 
        # và robot_state rỗng có thể giúp reset một số plugin rviz.
        # Tuy nhiên cách chắc chắn hơn là lặp qua các item. 
        # Vì ta không biết tên, ta hy vọng RViz reset khi nhận scene mới.
        
        # Một cách khác: Gửi một đối tượng với operation REMOVE
        # Nhưng MoveIt yêu cầu ID.
        
        # Giải pháp thực tế: Gửi một scene hoàn toàn mới (không phải diff)
        scene.is_diff = False 
        self.publisher.publish(scene)
        
        self.count += 1
        if self.count > 5:
            self.get_logger().info('Đã gửi yêu cầu dọn dẹp xong.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SceneClearer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
