cmake_minimum_required(VERSION 3.8)
project(control_server)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Include directory for httplib.h
include_directories(include)

add_executable(control_server_node src/control_server_node.cpp)
ament_target_dependencies(control_server_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS control_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
