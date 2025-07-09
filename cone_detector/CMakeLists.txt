cmake_minimum_required(VERSION 3.8)
project(cone_detector)

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
add_executable(bag_cone_detector_node src/bag_cone_detector_node.cpp)
add_executable(2bag_cone_detector_node src/2bag_cone_detector_node.cpp)
add_executable(3bag_cone_detector_node src/3bag_cone_detector_node.cpp)

ament_target_dependencies(cone_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  geometry_msgs
)

ament_target_dependencies(bag_cone_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  geometry_msgs
)

ament_target_dependencies(2bag_cone_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  geometry_msgs
)

ament_target_dependencies(3bag_cone_detector_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  geometry_msgs
)

install(TARGETS
  cone_detector_node
  bag_cone_detector_node
  2bag_cone_detector_node
  3bag_cone_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
