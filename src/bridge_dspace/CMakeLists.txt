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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # dSPACE RX 피드백 CSV 포맷 계약 (2026-08-25). 기록 경로 자체는 CAN 인터페이스가
  # 있어야 돌므로, CAN 없이 검증 가능한 포맷만 고정한다.
  add_executable(vehicle_csv_test test/vehicle_csv_test.cpp)
  target_include_directories(vehicle_csv_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  ament_target_dependencies(vehicle_csv_test fma_interfaces)
  add_test(NAME vehicle_csv_test COMMAND vehicle_csv_test)

  # classic / CAN FD 와이어 포맷 계약 (2026-08-28 FD 이관). socketpair 를 쓰므로
  # CAN 인터페이스 없이 돌고, "FD 로 바꿔도 페이로드는 그대로"를 고정한다.
  add_executable(socketcan_fd_test test/socketcan_fd_test.cpp)
  target_include_directories(socketcan_fd_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME socketcan_fd_test COMMAND socketcan_fd_test)
endif()

ament_package()
