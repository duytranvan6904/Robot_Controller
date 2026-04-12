cmake_minimum_required(VERSION 3.8)
project(gp4_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/test_continuous_move.py scripts/servo_bridge.py scripts/restamp_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
