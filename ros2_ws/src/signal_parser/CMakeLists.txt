cmake_minimum_required(VERSION 3.8)
project(signal_parser)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(signal_parser_node src/signal_parser_node.cpp)
ament_target_dependencies(signal_parser_node
  rclcpp
  can_msgs
  nav_msgs
  std_msgs
)

install(TARGETS signal_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
