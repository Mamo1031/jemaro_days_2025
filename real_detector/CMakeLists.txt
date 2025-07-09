cmake_minimum_required(VERSION 3.8)
project(real_detector)

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

add_executable(real_detector_node src/real_detector_node.cpp)

ament_target_dependencies(real_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
)

install(TARGETS
  real_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
