cmake_minimum_required(VERSION 3.8)
project(ibvs_ur5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

install(DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_printer.py
  scripts/detect_target.py
  scripts/ibvs_math.py
  scripts/ibvs_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
