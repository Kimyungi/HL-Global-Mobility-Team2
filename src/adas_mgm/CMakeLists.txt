cmake_minimum_required(VERSION 3.8)
project(adas_mgm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

option(
  ADAS_MGM_ENABLE_GENERATED_BACKEND
  "Link the experimental ADAS_MGR2 v1.68 backend into mgm_node"
  OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(fma_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# ── MGM 로직 코어 (§5.5) — ROS 의존성 없는 정적 라이브러리.
# 여기에 rclcpp/fma_interfaces를 링크하는 순간 §5.5 위반 — 절대 추가 금지.
add_library(mgm_core STATIC core/mgm_step.cpp)
target_include_directories(mgm_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ── ROS 2 wrapper 노드
add_executable(mgm_node src/mgm_node.cpp src/decision_backend.cpp)
target_link_libraries(mgm_node mgm_core)
ament_target_dependencies(mgm_node rclcpp rcl_interfaces fma_interfaces std_msgs)

# ── back-to-back 하네스 (ROS 무관 — 코어만 링크)
add_executable(core_replay tools/core_replay.cpp)
target_link_libraries(core_replay mgm_core)
add_executable(make_sample_dump tools/make_sample_dump.cpp)
target_link_libraries(make_sample_dump mgm_core)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ADAS_MGM_PROCESSOR)
set(ADAS_MGM_GENERATED_SUPPORTED FALSE)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
  ADAS_MGM_PROCESSOR MATCHES "^(x86_64|amd64)$" AND
  CMAKE_C_COMPILER_ID MATCHES "^(GNU|Clang)$" AND
  CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang)$")
  set(ADAS_MGM_GENERATED_SUPPORTED TRUE)
endif()

if(ADAS_MGM_ENABLE_GENERATED_BACKEND AND NOT ADAS_MGM_GENERATED_SUPPORTED)
  message(FATAL_ERROR
    "ADAS_MGM_ENABLE_GENERATED_BACKEND=ON requires Linux x86-64 with "
    "GNU/Clang C and C++ compilers; got "
    "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}/"
    "${CMAKE_C_COMPILER_ID}/${CMAKE_CXX_COMPILER_ID}")
endif()

# The generated runtime is available to the parity test without changing the
# production node. Linking it into mgm_node requires the explicit option above.
if(BUILD_TESTING OR ADAS_MGM_ENABLE_GENERATED_BACKEND)
  if(ADAS_MGM_GENERATED_SUPPORTED)
    set(ADAS_MGR2_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/adas_mgr2)
    add_library(adas_mgr2_generated STATIC
      ${ADAS_MGR2_GENERATED_DIR}/ADAS_MGR2.c
      ${ADAS_MGR2_GENERATED_DIR}/rtGetInf.c
      ${ADAS_MGR2_GENERATED_DIR}/rtGetNaN.c
      ${ADAS_MGR2_GENERATED_DIR}/rt_nonfinite.c)
    target_include_directories(adas_mgr2_generated PUBLIC ${ADAS_MGR2_GENERATED_DIR})
    target_compile_options(adas_mgr2_generated PRIVATE -msse2)
    target_link_libraries(adas_mgr2_generated PUBLIC m)

    add_library(mgm_generated_adapter STATIC src/generated_adapter.cpp)
    target_include_directories(mgm_generated_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(mgm_generated_adapter PUBLIC mgm_core adas_mgr2_generated)

    if(ADAS_MGM_ENABLE_GENERATED_BACKEND)
      target_compile_definitions(mgm_node PRIVATE ADAS_MGM_HAS_GENERATED_BACKEND=1)
      target_link_libraries(mgm_node mgm_generated_adapter)
    endif()

    if(BUILD_TESTING)
      add_executable(
        generated_lane_waypoint_parity_test
        test/generated_lane_waypoint_parity_test.cpp)
      target_link_libraries(generated_lane_waypoint_parity_test mgm_generated_adapter)
      add_test(
        NAME generated_lane_waypoint_parity_test
        COMMAND generated_lane_waypoint_parity_test)

      add_executable(
        generated_adapter_safety_test
        test/generated_adapter_safety_test.cpp)
      target_link_libraries(generated_adapter_safety_test mgm_generated_adapter)
      add_test(
        NAME generated_adapter_safety_test
        COMMAND generated_adapter_safety_test)

      add_executable(
        decision_backend_test
        test/decision_backend_test.cpp
        src/decision_backend.cpp)
      target_include_directories(decision_backend_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
      target_compile_definitions(
        decision_backend_test PRIVATE ADAS_MGM_HAS_GENERATED_BACKEND=1)
      target_link_libraries(decision_backend_test mgm_generated_adapter)
      add_test(
        NAME decision_backend_test
        COMMAND decision_backend_test)
    endif()
  else()
    message(STATUS
      "ADAS_MGR2 LANE/WAYPOINT parity disabled: requires Linux x86-64 with GNU/Clang; "
      "got ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}/${CMAKE_C_COMPILER_ID}")
  endif()
endif()

if(BUILD_TESTING)
  # Compile this target without ADAS_MGM_HAS_GENERATED_BACKEND even when the
  # package opt-in is ON. It proves that an unavailable generated request
  # fails at startup instead of silently falling back to the production core.
  add_executable(
    decision_backend_unavailable_test
    test/decision_backend_unavailable_test.cpp
    src/decision_backend.cpp)
  target_include_directories(
    decision_backend_unavailable_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(decision_backend_unavailable_test mgm_core)
  add_test(
    NAME decision_backend_unavailable_test
    COMMAND decision_backend_unavailable_test)
endif()

install(TARGETS mgm_node core_replay make_sample_dump DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/go tools/state DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
