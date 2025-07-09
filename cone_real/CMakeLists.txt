cmake_minimum_required(VERSION 3.8)
project(cone_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cone_detector_node src/cone_detector_node.cpp)

ament_target_dependencies(cone_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  geometry_msgs
)

install(TARGETS
  cone_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
