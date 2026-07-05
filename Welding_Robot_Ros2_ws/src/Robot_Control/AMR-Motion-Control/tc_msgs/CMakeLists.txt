cmake_minimum_required(VERSION 3.8)
project(tc_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Trimmed to the motor-independent SafetyStatus only (see package.xml).
# SafetyStatus uses primitive types only, so no interface DEPENDENCIES are needed.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
)

ament_package()
