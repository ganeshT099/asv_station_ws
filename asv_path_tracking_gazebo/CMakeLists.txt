cmake_minimum_required(VERSION 3.8)
project(asv_path_tracking_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_executable(gazebo_path_marker_bridge src/gazebo_path_marker_bridge.cpp)
ament_target_dependencies(gazebo_path_marker_bridge
  nav_msgs
  rclcpp
)
target_link_libraries(gazebo_path_marker_bridge
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)

install(TARGETS gazebo_path_marker_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
