cmake_minimum_required(VERSION 3.8)
project(adas_mgm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fma_interfaces REQUIRED)

# ── MGM 로직 코어 (§5.5) — ROS 의존성 없는 정적 라이브러리.
# 여기에 rclcpp/fma_interfaces를 링크하는 순간 §5.5 위반 — 절대 추가 금지.
add_library(mgm_core STATIC core/mgm_step.cpp)
target_include_directories(mgm_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ── ROS 2 wrapper 노드
add_executable(mgm_node src/mgm_node.cpp)
target_link_libraries(mgm_node mgm_core)
ament_target_dependencies(mgm_node rclcpp fma_interfaces)

# ── back-to-back 하네스 (ROS 무관 — 코어만 링크)
add_executable(core_replay tools/core_replay.cpp)
target_link_libraries(core_replay mgm_core)
add_executable(make_sample_dump tools/make_sample_dump.cpp)
target_link_libraries(make_sample_dump mgm_core)

install(TARGETS mgm_node core_replay make_sample_dump DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
