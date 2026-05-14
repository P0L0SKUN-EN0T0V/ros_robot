cmake_minimum_required(VERSION 3.8)
project(global_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node
    rclcpp sensor_msgs nav_msgs tf2 tf2_geometry_msgs)

install(TARGETS global_planner_node DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS src/fake_sim_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
