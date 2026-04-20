"""
hc10dtp_start.launch.py
────────────────────────
Launch file hoàn chỉnh cho Yaskawa HC10DTP với MotoROS2 driver.

Kiến trúc:
  - MotoROS2 driver chạy trên robot (cung cấp /yaskawa/follow_joint_trajectory
    và /yaskawa/joint_states)
  - File này khởi động: move_group + RViz + robot_state_publisher +
    restamp_joint_states + static TF

Cách dùng:
  1. Đảm bảo MotoROS2 driver đang chạy trên robot
  2. ros2 launch hc10dtp_moveit_config hc10dtp_start.launch.py
  3. Dùng RViz để plan & execute, robot thật sẽ chạy theo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    # ── MoveIt config ────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("motoman_hc10dtp", package_name="hc10dtp_moveit_config")
        .robot_description(file_path="config/motoman_hc10dtp.urdf.xacro")
        .robot_description_semantic(file_path="config/motoman_hc10dtp.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "provide_planning_scene_service": True,
    }

    # ── move_group node ──────────────────────────────────────────────────
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
        ],
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_config_path = os.path.join(
        get_package_share_directory("hc10dtp_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ── Robot State Publisher (publish TF từ URDF) ───────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ── Static TF (world -> base_link) ───────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    # ── Restamp Node ─────────────────────────────────────────────────────
    restamp_node = Node(
        package="hc10dtp_bringup",
        executable="restamp_joint_states.py",
        name="restamp_joint_states",
        output="screen",
    )

    return LaunchDescription(
        [
            db_arg,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            rviz_node,
            restamp_node,
        ]
    )
