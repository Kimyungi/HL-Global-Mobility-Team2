cmake_minimum_required(VERSION 3.8)
project(multi_lidar_fusion)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  # 실시간 차량 탑재용 — 기본 최적화 켠다.
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(FUSION_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

# ── 융합 코어 라이브러리 ─────────────────────────────────────────────
# 파이프라인 각 단계는 이 라이브러리에만 있고, ROS 노드(wrapper)는 배선만 한다.
add_library(${PROJECT_NAME}_core SHARED
  src/lidar_converter.cpp
  src/cloud_transformer.cpp
  src/cloud_synchronizer.cpp
  src/motion_compensator.cpp
  src/cloud_merger.cpp
  src/cloud_filter.cpp
  src/virtual_laserscan.cpp
  src/diagnostics.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core ${FUSION_DEPS})

# ── 융합 노드 ────────────────────────────────────────────────────────
add_executable(multi_lidar_fusion_node src/multi_lidar_fusion_node.cpp)
target_link_libraries(multi_lidar_fusion_node ${PROJECT_NAME}_core)
ament_target_dependencies(multi_lidar_fusion_node ${FUSION_DEPS})

# ── 합성 LiDAR 시뮬레이터 (실 센서 없이 전 파이프라인 검증) ──────────
add_executable(test_scan_publisher src/test_scan_publisher.cpp)
ament_target_dependencies(test_scan_publisher ${FUSION_DEPS})

install(TARGETS
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  multi_lidar_fusion_node
  test_scan_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  tools/identify_lidars.sh
  tools/identify_lidar_positions.py
  tools/layout_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright/license 린트는 사내 규약과 충돌하므로 제외
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fusion_core test/test_fusion_core.cpp)
  if(TARGET test_fusion_core)
    target_link_libraries(test_fusion_core ${PROJECT_NAME}_core)
    ament_target_dependencies(test_fusion_core ${FUSION_DEPS})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${FUSION_DEPS})
ament_package()
