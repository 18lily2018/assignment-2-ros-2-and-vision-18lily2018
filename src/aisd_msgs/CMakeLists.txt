cmake_minimum_required(VERSION 3.8)
project(aisd_msgs)

# Use stricter warnings in C++ if available (harmless for us, but standard template)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- Dependencies --------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------- Interface generation (msg + srv) --------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hand.msg"
  "srv/Speak.srv"
)

# Export runtime deps so Python/C++ code can find the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
