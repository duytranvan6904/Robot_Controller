#include <memory>

#include "motoros2_interfaces/srv/start_traj_mode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>
#include <thread>

/**
 * go_home.cpp
 * ───────────
 * Di chuyển robot GP4 về vị trí HOME đã lưu.
 * Tọa độ góc khớp được đọc từ robot thật ngày 2026-03-20.
 *
 * Cách dùng:
 *   1. Chạy gp4_start.launch.py trước
 *   2. ros2 run hello_moveit go_home
 */

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // ── Bật Trajectory Mode trên MotoROS2 ─────────────────────────────
  auto drive = rclcpp::Node::make_shared("go_home_enable");
  auto enable_client = drive->create_client<motoros2_interfaces::srv::StartTrajMode>(
      "yaskawa/start_traj_mode");

  if (enable_client->wait_for_service(std::chrono::seconds(2))) {
    auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();
    auto future = enable_client->async_send_request(request);
    RCLCPP_INFO(drive->get_logger(), "Sending yaskawa/start_traj_mode request...");
    if (rclcpp::spin_until_future_complete(drive, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->result_code.value == 1) {
        RCLCPP_INFO(drive->get_logger(), "✓ MotoROS2 READY: Trajectory mode enabled.");
      } else {
        RCLCPP_ERROR(drive->get_logger(), "✗ MotoROS2 NOT READY (Code %d): %s", 
                     response->result_code.value, response->message.c_str());
        return 1; // Thoát nếu driver chưa sẵn sàng
      }
    } else {
      RCLCPP_ERROR(drive->get_logger(), "Failed to call service yaskawa/start_traj_mode");
      return 1;
    }
  } else {
    RCLCPP_WARN(drive->get_logger(), "Service yaskawa/start_traj_mode NOT AVAILABLE.");
  }

  // ── MoveIt setup ──────────────────────────────────────────────────
  auto const node = std::make_shared<rclcpp::Node>(
      "go_home",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("go_home");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "gp4_arm");

  // ── Đặt tọa độ HOME (góc khớp đọc từ robot thật 2026-03-20) ─────
  std::vector<double> home_joints = {
      1.5477395698141883,   // joint_1_s
     -0.15883329466662804,  // joint_2_l
     -0.15854787143360877,  // joint_3_u
      0.0,                  // joint_4_r
     -1.6017466450445892,   // joint_5_b
      0.05361262853660316   // joint_6_t
  };

  move_group.setJointValueTarget(home_joints);

  // ── Giới hạn tốc độ & gia tốc (an toàn: 0.1 = 10%) ──────────────────────────
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // ── Planning ──────────────────────────────────────────────────────
  RCLCPP_INFO(logger, "Planning path to HOME position...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Planning OK! Executing with 60s timeout...");
    
    // Đặt timeout dài hơn cho việc thực thi (tránh lỗi timeout reached)
    move_group.move(); // Hoặc dùng execute(plan) nhưng move() kết hợp cả plan và exec
    
    RCLCPP_INFO(logger, "Execution finished.");
  } else {
    RCLCPP_ERROR(logger, "Planning to HOME failed!");
  }

  rclcpp::shutdown();
  return 0;
}
