cmake_minimum_required(VERSION 3.8)
project(ibvs_ur5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
