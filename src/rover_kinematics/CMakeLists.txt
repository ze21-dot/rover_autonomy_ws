cmake_minimum_required(VERSION 3.16)
project(rover_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Pure geometry library (header-only, ROS-free) — ADR-0002
add_library(double_ackermann INTERFACE)
target_include_directories(double_ackermann INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Thin ROS 2 bridge node: Twist -> per-wheel steer/drive commands
add_executable(ackermann_ik_node src/ackermann_ik_node.cpp)
target_link_libraries(ackermann_ik_node double_ackermann)
ament_target_dependencies(ackermann_ik_node rclcpp geometry_msgs std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS double_ackermann EXPORT export_${PROJECT_NAME})
install(TARGETS ackermann_ik_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_double_ackermann test/test_double_ackermann.cpp)
  target_link_libraries(test_double_ackermann double_ackermann)
endif()

ament_package()
