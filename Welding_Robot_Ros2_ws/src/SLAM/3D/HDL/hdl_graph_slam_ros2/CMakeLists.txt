# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 3.8)
project(hdl_graph_slam)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Compiler optimizations
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-O3)
else()
  add_compile_options(-O3 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# External dependencies
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(G2O REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(fast_gicp REQUIRED)

# OpenMP
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Check for CUDA support
find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
  message(STATUS "VGICP CUDA support enabled")
endif()

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloorCoeffs.msg"
  "msg/ScanMatchingStatus.msg"
  "srv/SaveMap.srv"
  "srv/LoadGraph.srv"
  "srv/DumpGraph.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${G2O_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Common dependencies
set(COMMON_DEPS
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  nmea_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  interactive_markers
  ndt_omp_ros2
  fast_gicp
)

###########
## Build ##
###########

# Prefiltering node
add_library(prefiltering_component SHARED
  src/nodes/prefiltering_node.cpp
)
target_link_libraries(prefiltering_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(prefiltering_component ${COMMON_DEPS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(prefiltering_component "${cpp_typesupport_target}")
rclcpp_components_register_node(prefiltering_component
  PLUGIN "hdl_graph_slam::PrefilteringNode"
  EXECUTABLE prefiltering_node
)

# Floor detection node
add_library(floor_detection_component SHARED
  src/nodes/floor_detection_node.cpp
)
target_link_libraries(floor_detection_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(floor_detection_component ${COMMON_DEPS})
target_link_libraries(floor_detection_component "${cpp_typesupport_target}")
rclcpp_components_register_node(floor_detection_component
  PLUGIN "hdl_graph_slam::FloorDetectionNode"
  EXECUTABLE floor_detection_node
)

# Scan matching odometry node
add_library(scan_matching_odometry_component SHARED
  src/nodes/scan_matching_odometry_node.cpp
  src/hdl_graph_slam/registrations.cpp
)
target_link_libraries(scan_matching_odometry_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(scan_matching_odometry_component ${COMMON_DEPS})
target_link_libraries(scan_matching_odometry_component "${cpp_typesupport_target}")
rclcpp_components_register_node(scan_matching_odometry_component
  PLUGIN "hdl_graph_slam::ScanMatchingOdometryNode"
  EXECUTABLE scan_matching_odometry_node
)

# HDL graph slam node
add_library(hdl_graph_slam_component SHARED
  src/nodes/hdl_graph_slam_node.cpp
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/map_cloud_generator.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)
target_link_libraries(hdl_graph_slam_component
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)
ament_target_dependencies(hdl_graph_slam_component ${COMMON_DEPS})
target_link_libraries(hdl_graph_slam_component "${cpp_typesupport_target}")
rclcpp_components_register_node(hdl_graph_slam_component
  PLUGIN "hdl_graph_slam::HdlGraphSlamNode"
  EXECUTABLE hdl_graph_slam_node
)

#############
## Install ##
#############

install(TARGETS
  prefiltering_component
  floor_detection_component
  scan_matching_odometry_component
  hdl_graph_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/map2odom_publisher.py
  scripts/color_to_intensity.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  prefiltering_component
  floor_detection_component
  scan_matching_odometry_component
  hdl_graph_slam_component
)
ament_export_dependencies(${COMMON_DEPS})

ament_package()
