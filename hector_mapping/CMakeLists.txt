cmake_minimum_required(VERSION 3.5)
project(hector_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3") # because normal release is just -O2

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})
find_package(Boost REQUIRED)

find_package(rosidl_default_generators REQUIRED)

### HEADER INCLUDE
include_directories(
  include
  include/hector_slam_lib
  ${Boost_INCLUDE_DIRS}
  )

### MESSAGES

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HectorDebugInfo.msg"
  "msg/HectorIterData.msg"
)


### LIBRARY DEFINITION

add_library(HectorMappingRos SHARED
  src/HectorMappingRos.cpp
  src/PoseInfoContainer.cpp
)

ament_export_include_directories(include)
ament_export_libraries(HectorMappingRos)
ament_export_dependencies(rclcpp nav_msgs visualization_msgs tf2 tf2_ros message_filters laser_geometry sensor_msgs)
ament_target_dependencies(HectorMappingRos
  rclcpp
  nav_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  message_filters
  laser_geometry
  sensor_msgs
)

target_link_libraries(HectorMappingRos)

rosidl_target_interfaces(HectorMappingRos
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
  )

rclcpp_components_register_nodes(HectorMappingRos "hector_mapping::HectorMappingRos" EXECUTABLE HectorMappingRos)

### INSTALLATION

install(TARGETS HectorMappingRos
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
