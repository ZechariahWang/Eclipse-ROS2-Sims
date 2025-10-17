cmake_minimum_required(VERSION 3.8)
project(eclipse_diffdrive)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/drive_forward.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

