cmake_minimum_required(VERSION 3.8)
project(r2_controller)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/r2_main.py
  scripts/serial_bridge.py
  scripts/r2_protocol.py
  scripts/forest_planner.py
  scripts/odometry_printer.py
  scripts/yaw_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
