cmake_minimum_required(VERSION 3.8)
project(hc10dtp_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/cartesian_streamer_hc10dtp.py scripts/restamp_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
