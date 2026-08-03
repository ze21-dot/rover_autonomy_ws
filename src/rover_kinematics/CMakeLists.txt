cmake_minimum_required(VERSION 3.16)
project(rover_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Pure geometry library (header-only, ROS-free) — ADR-0002
add_library(double_ackermann INTERFACE)
target_include_directories(double_ackermann INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS double_ackermann EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_double_ackermann test/test_double_ackermann.cpp)
  target_link_libraries(test_double_ackermann double_ackermann)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
