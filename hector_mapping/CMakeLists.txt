cmake_minimum_required(VERSION 3.5)
project(hector_mapping)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(rosidl_default_generators REQUIRED)

### HEADER INCLUDE
include_directories(include)

### MESSAGES

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HectorDebugInfo.msg"
  "msg/HectorIterData.msg"
)


### LIBRARY DEFINITION

add_library(HectorMappingRos SHARED
  src/HectorMappingRos.cpp
  # src/main.cpp
  src/PoseInfoContainer.cpp
)

ament_export_include_directories(include)
ament_export_libraries(HectorMappingRos)
ament_export_dependencies(rclcpp nav_msgs visualization_msgs tf2 message_filters laser_geometry message_runtime eigen3_cmake_module sensor_msgs)
ament_export_dependencies(Eigen3)
# add_dependencies(HectorMappingRos hector_mapping_generate_messages_cpp)
ament_target_dependencies(HectorMappingRos
  rclcpp
  nav_msgs
  visualization_msgs
  tf2
  message_filters
  laser_geometry
  sensor_msgs
  Eigen3
)

### INSTALLATION

install(TARGETS HectorMappingRos
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/hector_slam_lib/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
