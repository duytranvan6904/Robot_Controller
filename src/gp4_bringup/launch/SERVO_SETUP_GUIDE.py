# ════════════════════════════════════════════════════════════════════
# File 1: src/gp4_moveit_config/config/moveit_servo.yaml
# Đặt file này vào: src/gp4_moveit_config/config/moveit_servo.yaml
# ════════════════════════════════════════════════════════════════════

## MoveIt Servo config cho GP4
## Tham khảo: https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

command_in_type: "speed_units"          # "unitless" hoặc "speed_units"
scale:
  linear:  0.2                          # m/s tối đa
  rotational: 0.4                       # rad/s tối đa

robot_link_command_frame: "base_link"
status_topic: /servo_node/status
command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /gp4_arm_controller/joint_trajectory   # khớp với sim controller

incoming_command_timeout: 0.1
num_outgoing_halt_msgs_to_publish: 4

lower_singularity_threshold: 17.0
hard_stop_singularity_threshold: 30.0
joint_topic: /joint_states
move_group_name: gp4_arm
planning_frame: base_link

check_collisions: true
self_collision_proximity_threshold: 0.01
scene_collision_proximity_threshold: 0.02

cartesian_command_in_topic: /servo_node/delta_twist_cmds
joint_command_in_topic: /servo_node/delta_joint_cmds

publish_period: 0.034                  # ~30 Hz

low_latency_mode: false
use_smoothing: true
smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"


# ════════════════════════════════════════════════════════════════════
# File 2: Thêm servo_node vào sim.launch.py
# Tìm dòng "return LaunchDescription([" và thêm servo_node vào list
# ════════════════════════════════════════════════════════════════════

# PATCH cho src/gp4_bringup/launch/sim.launch.py
# Thêm đoạn này TRƯỚC "return LaunchDescription(["

SERVO_NODE_SNIPPET = """
    # ── MoveIt Servo node (optional: chỉ cần nếu test servo streaming) ──
    import os
    from ament_index_python.packages import get_package_share_directory

    servo_params_file = os.path.join(
        get_package_share_directory('gp4_moveit_config'),
        'config', 'moveit_servo.yaml'
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            servo_params_file,
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
        ],
    )
"""

# Và thêm "servo_node," vào list trong LaunchDescription


# ════════════════════════════════════════════════════════════════════
# File 3: Script test Servo đúng cách (SAU KHI đã patch launch)
# Đặt tại: src/gp4_bringup/test_servo_sine_wave.py  (viết lại)
# ════════════════════════════════════════════════════════════════════

SERVO_SCRIPT = '''#!/usr/bin/env python3
"""
test_servo_sine_wave.py (phiên bản sửa lỗi)
Yêu cầu: servo_node phải đang chạy (xem hướng dẫn bên trên)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


class ServoSineWave(Node):
    def __init__(self):
        super().__init__("servo_sine_wave")

        # Bật servo mode
        self._enable_client = self.create_client(
            Trigger, "/servo_node/start_servo")
        self.get_logger().info("Chờ /servo_node/start_servo ...")

        # Nếu service không có sau 3s → servo_node chưa chạy
        if not self._enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                "LỖII: /servo_node/start_servo không khả dụng!\\n"
                "Hãy đảm bảo servo_node đang chạy trong launch file.\\n"
                "Xem file SERVO_SETUP.md để biết cách thêm servo_node."
            )

        # Gọi start_servo
        future = self._enable_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info("Servo mode bật thành công.")

        self._pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10
        )
        self._t = 0.0
        self._timer = self.create_timer(1.0 / 30.0, self._cb)  # 30 Hz
        self.get_logger().info("Bắt đầu gửi Twist 30Hz (sin wave Z)...")

    def _cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.z = 0.05 * math.sin(self._t)   # ±5cm/s
        self._t += 2.0 * math.pi / 30.0 / 2.0           # 0.5Hz
        self._pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = ServoSineWave()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"\\n{e}")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
