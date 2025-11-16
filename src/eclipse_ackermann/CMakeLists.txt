cmake_minimum_required(VERSION 3.8)
project(eclipse_ackermann)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/pure_pursuit_ackermann.py
  src/pose_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
