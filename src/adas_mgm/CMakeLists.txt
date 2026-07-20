cmake_minimum_required(VERSION 3.8)
project(adas_mgm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fma_interfaces REQUIRED)

add_executable(mgm_node
  src/mgm_node.cpp
  src/state_machine.cpp
)
ament_target_dependencies(mgm_node rclcpp fma_interfaces)

install(TARGETS mgm_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
