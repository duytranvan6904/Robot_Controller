#!/usr/bin/env python3
"""
test_move_rel_continuous.py
───────────────────────────
Di chuyển robot GP4 bằng MOVE_REL action (sin wave theo trục Z),
tương thích với kiến trúc motion_core → hw_adapter của project.

Chạy sau khi đã launch sim.launch.py hoặc phase9_fake.launch.py:
  python3 src/gp4_bringup/test_move_rel_continuous.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from interfaces.action import ExecuteMotion


# ── Cấu hình chuyển động ─────────────────────────────────────────────
AMPLITUDE_M   = 0.05   # biên độ ±5cm theo trục Z
STEPS         = 8      # số bước mỗi chu kỳ sin
VELOCITY      = 0.10   # 10% tốc độ tối đa
WAIT_BETWEEN  = 0.2    # giây chờ giữa các bước


class MoveRelSineWave(Node):
    def __init__(self):
        super().__init__("move_rel_sine_wave")
        self._client = ActionClient(self, ExecuteMotion, "/execute_motion")
        self.get_logger().info("Chờ action server /execute_motion ...")
        self._client.wait_for_server()
        self.get_logger().info("Sẵn sàng. Bắt đầu sin wave theo trục Z.")

    def send_move_rel(self, dz: float) -> bool:
        """Gửi một bước MOVE_REL và chờ kết quả."""
        goal = ExecuteMotion.Goal()
        goal.primitive_type  = "MOVE_REL"
        goal.delta_x         = 0.0
        goal.delta_y         = 0.0
        goal.delta_z         = dz
        goal.reference_frame = "base_link"
        goal.velocity_scale  = VELOCITY
        goal.acceleration_scale = VELOCITY
        goal.require_approval   = False

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"Goal bị từ chối (dz={dz:.4f}m)")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        result = result_future.result()
        if result and result.result.success:
            self.get_logger().info(
                f"MOVE_REL dz={dz:+.4f}m OK | {result.result.message[:60]}"
            )
            return True

        self.get_logger().warn(
            f"MOVE_REL dz={dz:+.4f}m FAIL | "
            f"{result.result.message[:80] if result else 'no result'}"
        )
        return False

    def run_sine_wave(self, cycles: int = 3):
        """Chạy sin wave trong N chu kỳ."""
        self.get_logger().info(
            f"Sin wave: amplitude=±{AMPLITUDE_M*100:.0f}cm, "
            f"steps={STEPS}/cycle, cycles={cycles}"
        )
        for cycle in range(cycles):
            self.get_logger().info(f"── Chu kỳ {cycle+1}/{cycles} ──")
            for step in range(STEPS):
                angle     = 2.0 * math.pi * step / STEPS
                dz        = AMPLITUDE_M * math.sin(angle + math.pi / STEPS)
                if not self.send_move_rel(dz):
                    self.get_logger().error("Dừng do lỗi MOVE_REL.")
                    return
                time.sleep(WAIT_BETWEEN)

        self.get_logger().info("Hoàn thành sin wave.")


def main():
    rclpy.init()
    node = MoveRelSineWave()
    try:
        node.run_sine_wave(cycles=2)
    except KeyboardInterrupt:
        node.get_logger().info("Người dùng dừng.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
