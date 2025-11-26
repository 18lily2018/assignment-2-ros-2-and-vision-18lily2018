cmake_minimum_required(VERSION 3.5)
project(aisd_msgs)

# 默认设置
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口（消息）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hand.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
