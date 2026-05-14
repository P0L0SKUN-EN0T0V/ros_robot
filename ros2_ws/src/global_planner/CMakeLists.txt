cmake_minimum_required(VERSION 3.8)
project(global_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node rclcpp)

install(TARGETS global_planner_node DESTINATION lib/${PROJECT_NAME})

ament_package()
