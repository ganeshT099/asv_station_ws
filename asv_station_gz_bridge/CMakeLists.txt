cmake_minimum_required(VERSION 3.8)
project(asv_station_gz_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_executable(sea_state_gz_bridge src/sea_state_gz_bridge.cpp)
ament_target_dependencies(sea_state_gz_bridge geometry_msgs nav_msgs visualization_msgs rclcpp)
target_link_libraries(sea_state_gz_bridge
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)

install(TARGETS sea_state_gz_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
