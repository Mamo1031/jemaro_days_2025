cmake_minimum_required(VERSION 3.5)
project(road_detection)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS script/road_detection.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
