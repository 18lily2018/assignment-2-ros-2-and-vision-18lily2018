cmake_minimum_required(VERSION 3.5)
project(aisd_vision)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(aisd_msgs REQUIRED)

ament_package()

install(
  DIRECTORY aisd_vision
  DESTINATION lib/python3.10/site-packages
)

