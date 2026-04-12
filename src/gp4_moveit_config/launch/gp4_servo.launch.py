import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def generate_launch_description():
    # Load MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config")
        .robot_description(file_path="config/motoman_gp4.urdf.xacro")
        .robot_description_semantic(file_path="config/motoman_gp4.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Load servo parameters
    servo_yaml_path = os.path.join(
        get_package_share_directory("gp4_moveit_config"),
        "config",
        "motoman_gp4_servo.yaml",
    )
    with open(servo_yaml_path, "r") as f:
        servo_yaml = yaml.safe_load(f)
    
    # Bundle all parameters
    servo_params = {
        "moveit_servo": servo_yaml,
        **moveit_config.robot_description,
        **moveit_config.robot_description_semantic,
        **moveit_config.robot_description_kinematics,
        **moveit_config.joint_limits,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[servo_params],
        output="screen",
    )

    return LaunchDescription([servo_node])
