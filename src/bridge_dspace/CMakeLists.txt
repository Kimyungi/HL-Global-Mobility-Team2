cmake_minimum_required(VERSION 3.8)
project(bridge_dspace)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fma_interfaces REQUIRED)

add_executable(can_bridge_node src/can_bridge_node.cpp)
ament_target_dependencies(can_bridge_node rclcpp fma_interfaces)

add_executable(dummy_ref_publisher src/dummy_ref_publisher.cpp)
ament_target_dependencies(dummy_ref_publisher rclcpp fma_interfaces)

add_executable(dspace_sim_node src/dspace_sim_node.cpp)
ament_target_dependencies(dspace_sim_node rclcpp)

install(TARGETS can_bridge_node dummy_ref_publisher dspace_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch dbc DESTINATION share/${PROJECT_NAME})

ament_package()
