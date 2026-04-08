#!/usr/bin/env python3
"""
test_continuous_move.py
───────────────────────
Gửi liên tiếp các lệnh MOVE_REL để robot di chuyển theo sine wave trục Z.
Sử dụng /execute_motion action — tương thích kiến trúc motion_core → hw_adapter.

Chạy:
  1) Terminal 1: ros2 launch gp4_bringup sim.launch.py
  2) Terminal 2: python3 src/gp4_bringup/test_continuous_move.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from interfaces.action import ExecuteMotion


# ── Cấu hình ──────────────────────────────────────────────────────────
AMPLITUDE_M   = 0.03     # biên độ ±3 cm theo trục Z
STEPS         = 6        # số bước mỗi chu kỳ sin
VELOCITY      = 0.1     # 10% tốc độ tối đa
ACCEL         = 0.1     # 10% gia tốc tối đa

# Timeouts
SEND_TIMEOUT  = 10.0     # giây chờ goal được accept
RESULT_TIMEOUT = 30.0    # giây chờ execution xong (plan + dispatch + FJT)
PAUSE_BETWEEN = 0.1      # giây chờ giữa các bước


class ContinuousMover(Node):
    def __init__(self):
        super().__init__("continuous_mover")
        self._client = ActionClient(self, ExecuteMotion, "/execute_motion")
        self.get_logger().info("⏳ Chờ action server /execute_motion ...")
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("❌ /execute_motion không khả dụng sau 15s!")
            sys.exit(1)
        self.get_logger().info("✅ Đã kết nối /execute_motion")

    def send_move_rel(self, dx: float, dy: float, dz: float) -> bool:
        """Gửi một bước MOVE_REL đồng bộ, trả True nếu thành công."""
        goal = ExecuteMotion.Goal()
        goal.primitive_type      = "MOVE_REL"
        goal.delta_x             = dx
        goal.delta_y             = dy
        goal.delta_z             = dz
        goal.reference_frame     = "base_link"
        goal.velocity_scale      = VELOCITY
        goal.acceleration_scale  = ACCEL
        goal.require_approval    = False

        t0 = time.monotonic()

        # 1) Gửi goal
        self.get_logger().info(
            f"📤 Gửi MOVE_REL dx={dx:+.4f} dy={dy:+.4f} dz={dz:+.4f}"
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SEND_TIMEOUT)

        if not send_future.done():
            self.get_logger().error("⏰ Timeout khi gửi goal!")
            return False

        goal_handle = send_future.result()
        if not goal_handle:
            self.get_logger().error("❌ Goal handle is None!")
            return False
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal bị từ chối!")
            return False

        self.get_logger().info("✓ Goal accepted, chờ execution...")

        # 2) Chờ kết quả
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=RESULT_TIMEOUT)

        elapsed = time.monotonic() - t0

        if not result_future.done():
            self.get_logger().error(
                f"⏰ Timeout chờ result sau {elapsed:.1f}s! "
                "Pipeline có thể bị kẹt ở dispatch/FJT."
            )
            # Cancel goal để tránh block pipeline
            goal_handle.cancel_goal_async()
            return False

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error(f"❌ Result wrapper is None ({elapsed:.1f}s)")
            return False

        result = wrapped.result
        status = wrapped.status

        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(
                f"✅ OK ({elapsed:.1f}s) | {result.message[:80]}"
            )
            return True

        status_name = {
            1: "ACCEPTED", 2: "EXECUTING", 4: "SUCCEEDED",
            5: "CANCELED", 6: "ABORTED"
        }.get(status, f"UNKNOWN({status})")

        self.get_logger().warn(
            f"⚠️  {status_name} ({elapsed:.1f}s) | "
            f"success={result.success} | {result.message[:100]}"
        )
        return result.success

    def run_sine_wave(self, cycles: int = 2):
        """Chạy sine wave theo trục Z."""
        self.get_logger().info(
            f"🔄 Sine wave: amplitude=±{AMPLITUDE_M*100:.0f}cm, "
            f"steps={STEPS}/cycle, cycles={cycles}, vel={VELOCITY}"
        )
        total_steps = cycles * STEPS
        step_count = 0

        for cycle in range(cycles):
            self.get_logger().info(f"── Chu kỳ {cycle+1}/{cycles} ──")
            for step in range(STEPS):
                # Tính delta Z cho bước tiếp theo
                angle_current = 2.0 * math.pi * step / STEPS
                angle_next    = 2.0 * math.pi * (step + 1) / STEPS

                z_current = AMPLITUDE_M * math.sin(angle_current)
                z_next    = AMPLITUDE_M * math.sin(angle_next)
                dz = z_next - z_current

                if abs(dz) < 1e-6:
                    self.get_logger().info(f"  Step {step}: dz≈0, skip")
                    continue

                step_count += 1
                self.get_logger().info(
                    f"  Step {step_count}/{total_steps}: dz={dz:+.4f}m"
                )

                if not self.send_move_rel(0.0, 0.0, dz):
                    self.get_logger().error("🛑 Dừng do lỗi MOVE_REL!")
                    return False

                time.sleep(PAUSE_BETWEEN)

        self.get_logger().info(f"🎉 Hoàn thành {step_count} bước sine wave!")
        return True

    def run_simple_test(self):
        """Test đơn giản: di chuyển lên 3cm rồi xuống 3cm."""
        self.get_logger().info("=== TEST ĐƠN GIẢN: Lên 3cm → Xuống 3cm ===")

        self.get_logger().info("📤 Bước 1: Di chuyển lên +3cm theo Z")
        ok = self.send_move_rel(0.0, 0.0, 0.03)
        if not ok:
            self.get_logger().error("🛑 Bước 1 thất bại!")
            return False

        time.sleep(0.5)

        self.get_logger().info("📤 Bước 2: Di chuyển xuống -3cm theo Z")
        ok = self.send_move_rel(0.0, 0.0, -0.03)
        if not ok:
            self.get_logger().error("🛑 Bước 2 thất bại!")
            return False

        time.sleep(0.5)

        self.get_logger().info("📤 Bước 3: Di chuyển phải +3cm theo Y")
        ok = self.send_move_rel(0.0, 0.03, 0.0)
        if not ok:
            self.get_logger().error("🛑 Bước 3 thất bại!")
            return False

        time.sleep(0.5)

        self.get_logger().info("📤 Bước 4: Di chuyển trái -3cm theo Y")
        ok = self.send_move_rel(0.0, -0.03, 0.0)
        if not ok:
            self.get_logger().error("🛑 Bước 4 thất bại!")
            return False

        self.get_logger().info("🎉 Test đơn giản hoàn thành!")
        return True


def main():
    rclpy.init()
    node = ContinuousMover()
    try:
        # Chạy test đơn giản trước
        if node.run_simple_test():
            node.get_logger().info("\n=== CHẠY SINE WAVE ===")
            node.run_sine_wave(cycles=2)
    except KeyboardInterrupt:
        node.get_logger().info("⏹ Người dùng dừng (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
