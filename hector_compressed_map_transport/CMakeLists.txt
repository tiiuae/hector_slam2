cmake_minimum_required(VERSION 3.5)
project(hector_compressed_map_transport)

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
find_package(cv_bridge REQUIRED)
find_package(hector_map_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(map_to_image_node src/map_to_image_node.cpp)
ament_target_dependencies(map_to_image_node
  rclcpp
  cv_bridge
  hector_map_tools
  sensor_msgs
  nav_msgs
  geometry_msgs
  image_transport
  OpenCV
  Eigen3)

install(TARGETS map_to_image_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
