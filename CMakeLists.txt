cmake_minimum_required(VERSION 3.8)
project(mr_pinchy_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/test_ntrip.py
  scripts/ntrip_client_py.py
  scripts/nucleus_dvl_bridge.py
  scripts/depth_bridge.py
  scripts/nucleus_imu_bridge.py
  scripts/nucleus_ahrs_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
